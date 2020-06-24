/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ZedCamera.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/system/cuda_context.hpp"
#include "messages/camera.hpp"
#include "packages/zed/gems/time_offset_calculator.hpp"

namespace isaac {
namespace {

// Factor by which to scale images down when displayed in Sight
constexpr int kSightReduceSize = 4;

// Helper function to retrieve the camera extrinsics
Pose3d GetCameraExtrinsics(const sl::CameraInformation& camera_info) {
  Pose3d out;
  sl::Rotation rot;
  rot.setRotationVector(camera_info.calibration_parameters.R);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  const float* rr = rot.getOrientation().ptr();
  double qq[4] = {rr[0], rr[1], rr[2], rr[3]};
#pragma GCC diagnostic pop
  out.rotation = SO3<double>::FromQuaternion(Quaterniond(qq));
  sl::float3 trs = camera_info.calibration_parameters.T;
  out.translation = Vector3d(trs[0], trs[1], trs[2]);
  return out;
}

// Helper function to copy camera intrinsics to ColorCameraProto
void SetCameraProtoIntrinsics(const sl::CameraParameters& in, ::ColorCameraProto::Builder& out) {
  // Pinhole camera model parameters
  auto pinhole = out.initPinhole();
  ToProto(Vector2d(in.fy, in.fx), pinhole.getFocal());
  ToProto(Vector2d(in.cy, in.cx), pinhole.getCenter());
  pinhole.setCols(in.image_size.width);
  pinhole.setRows(in.image_size.height);

  // Distortion parameters
  auto distortion = out.initDistortion();
  distortion.setModel(DistortionProto::DistortionModel::BROWN);
  Vector5d disto;
  disto << in.disto[0], in.disto[1], in.disto[2], in.disto[3], in.disto[4];
  ToProto(disto, distortion.getCoefficients());
}

// Helper function to change the specified camera parameter if needed.
void SynchronizeCameraSetting(const int64_t userValue, int64_t& storedValue,
                              const sl::CAMERA_SETTINGS settingId, sl::Camera& zed) {
    if (userValue != storedValue) {
      zed.setCameraSettings(settingId, userValue);
      storedValue = userValue;
    }
}

// Gets a view on a ZED image
ImageConstView1ub GrayZedImageView(const sl::Mat& mat) {
  ASSERT(mat.getStepBytes() == mat.getWidth(), "Not yet supported");
  return CreateImageView<uint8_t, 1>(mat.getPtr<sl::uchar1>(), mat.getHeight(), mat.getWidth());
}

// Gets a view on a ZED image
ImageConstView4ub BgraZedImageView(const sl::Mat& mat) {
  return CreateImageView<uint8_t, 4>(mat.getPtr<sl::uchar1>(), mat.getHeight(), mat.getWidth());
}

}  // namespace

void ZedCamera::start() {
  if (!initializeZedCamera()) {
    // reportFailure() was already called in initializeZedCamera() with an error-specific message.
    return;
  }

  show("Zed Camera Model", sl::toString(zed_info_.camera_model).c_str());
  show("Zed Camera Serial Number", zed_info_.serial_number);
  show("Zed Camera Firmware Version", zed_info_.firmware_version);

  // publish the camera extrinsics into the pose tree
  const bool left_T_right_camera_set = node()->pose().trySet(
      get_lhs_camera_frame(), get_rhs_camera_frame(), GetCameraExtrinsics(zed_info_), 0);
  if (!left_T_right_camera_set) {
    reportFailure("Failed to insert the left_T_right ZED camera transform into the Pose Tree.");
    return;
  }

  tickPeriodically();

  sl::Camera& zed_camera{*zed_.get()};
  const std::function<int64_t()> zed_clock_f{[&zed_camera]() {
    return zed_camera.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_CURRENT);
  }};

  const alice::Clock& isaac_clock{*(this->node()->clock())};
  const std::function<int64_t()> isaac_clock_f{[&isaac_clock]() {
    return isaac_clock.timestamp();
  }};

  timestamp_offset_ = zed::TimeOffset(zed_clock_f, isaac_clock_f);
}

void ZedCamera::tick() {
  // A value of 0 disables the auto white balance, while 1 activates it.
  if (get_auto_white_balance()) {
    // Due to a ZED firmware quirk the color temperature must be explicitly set to default for
    // AWB to work.
    if (color_temperature_ != 0) {
      zed_->setCameraSettings(sl::CAMERA_SETTINGS_WHITEBALANCE, 0, true);
      color_temperature_ = 0;
    }
    SynchronizeCameraSetting(1, auto_white_balance_,
                            sl::CAMERA_SETTINGS_AUTO_WHITEBALANCE, *zed_);
  } else {
    SynchronizeCameraSetting(get_color_temperature(), color_temperature_,
                            sl::CAMERA_SETTINGS_WHITEBALANCE, *zed_);
    SynchronizeCameraSetting(0, auto_white_balance_,
                            sl::CAMERA_SETTINGS_AUTO_WHITEBALANCE, *zed_);
  }
  SynchronizeCameraSetting(get_brightness(), brightness_, sl::CAMERA_SETTINGS_BRIGHTNESS, *zed_);
  SynchronizeCameraSetting(get_contrast(), contrast_, sl::CAMERA_SETTINGS_CONTRAST, *zed_);
  SynchronizeCameraSetting(get_exposure(), exposure_, sl::CAMERA_SETTINGS_EXPOSURE, *zed_);
  SynchronizeCameraSetting(get_gain(), gain_, sl::CAMERA_SETTINGS_GAIN, *zed_);

  // If no images are available yet, ERROR_CODE "ERROR_CODE_NOT_A_NEW_FRAME" will be returned.
  // This function is meant to be called frequently in the main loop of your application.
  sl::ERROR_CODE result = zed_->grab(zed_run_params_);
  if (sl::SUCCESS == result) {
    const int64_t isaac_time = zedToIsaacTimestamp(zed_->getTimestamp(
        sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE));

    retriveImages();
    publishImageData(isaac_time);

    publishCameraData(isaac_time);
  } else if (sl::ERROR_CODE_NOT_A_NEW_FRAME != result) {
    // ZED API doesn't guarantee 100% reliability of every call.
    // Skipping a few frames is not a critical problem for video stream consumers
    // like stereo visual odometry
    LOG_WARNING("[ZedCamera] Error capturing images: %s", sl::toString(result).c_str());
  }
}

void ZedCamera::stop() {
  if (zed_.get() != nullptr) {
    zed_->close();
    zed_.reset();
  }
}

bool ZedCamera::initializeZedCamera() {
  sl::InitParameters params = {};
  params.camera_resolution = static_cast<sl::RESOLUTION>(get_resolution());
  // ZED Camera FPS is
  // 1. not tied to a codelet tick rate as the camera has an independent on-board ISP
  // 2. much lower than an IMU poll rate that's equal to the ZedImuReader codelet tick frequency
  params.camera_fps = get_camera_fps();
  params.camera_linux_id = get_device_id();
  params.coordinate_units = sl::UNIT_METER;
  params.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
  params.depth_mode = sl::DEPTH_MODE::DEPTH_MODE_NONE;
  params.sdk_verbose = true;
  params.optional_settings_path = sl::String(get_settings_folder_path().c_str());
  params.sdk_gpu_id = get_gpu_id();
  params.sdk_cuda_ctx = isaac::cuda::GetOrCreateCudaContext(get_gpu_id());
  // This should set for ZedImuReader to read the Zed Mini IMU data
  params.camera_disable_imu = !get_enable_imu();

  // Create the camera object and open the camera
  auto camera = std::make_unique<sl::Camera>();
  sl::ERROR_CODE err = camera->open(params);
  if (err == sl::ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE ||
      err == sl::ERROR_CODE_INVALID_CALIBRATION_FILE) {
    camera->close();
    // Get the device serial number
    const std::vector<sl::DeviceProperties> device_list = camera->getDeviceList();
    const int selected_device_id = get_device_id();
    const auto selected_device = std::find_if(
        device_list.begin(), device_list.end(),
        [selected_device_id](const auto& device) { return device.id == selected_device_id; });

    ASSERT(selected_device != device_list.end(), "Invalid selected zed camera %d", selected_device);
    // The camera is not calibrated or we couldn't find the calibration file.
    // We cannot proceed without calibration therefore we fail this component.
    reportFailure(
        "Calibration file was not found for your zed camera.\n"
        " Please download the factory calibration or calibrate your camera using the zed"
        " calibration utility.\n"
        " The serial number of your zed camera is %d\n"
        " In order to download factory calibration:\n"
        " 1. call engine/build/scripts/download_zed_calibration.sh -s %d\n"
        " 2. copy the downloaded file, SN%d.conf, to the target device and specify the path to the"
        " containing folder using settings_folder_path setting\n",
        selected_device->serial_number, selected_device->serial_number,
        selected_device->serial_number);
    return false;
  } else if (err != sl::SUCCESS) {
    zed_.reset();
    // The camera is unresponsive. We cannot proceed without the camera
    // therefore we fail this component.
    reportFailure("[ZedCamera] Error initializing Zed camera: %s", sl::toString(err).c_str());
    return false;
  }

  zed_resolution_ = camera->getResolution();
  // Retrieve the camera calibration parameters from the ZED firmware
  // Zed Camera recalibrates itself for a selected resolution at sl::Camera::open
  zed_info_ = camera->getCameraInformation(zed_resolution_);

  camera->disableTracking();
  camera->disableSpatialMapping();

  zed_run_params_.sensing_mode = sl::SENSING_MODE_STANDARD;
  zed_run_params_.enable_depth = false;
  zed_run_params_.enable_point_cloud = false;

  zed_.swap(camera);
  return true;
}

void ZedCamera::retriveImages() {
  const bool gray_scale = get_gray_scale();
  const bool rgb = get_rgb();
  const bool enable_factory_rectification = get_enable_factory_rectification();

  // Retrieve left and right images
  if (gray_scale) {
    const sl::VIEW left_view = (enable_factory_rectification)
        ? sl::VIEW_LEFT_GRAY
        : sl::VIEW_LEFT_UNRECTIFIED_GRAY;
    zed_->retrieveImage(left_image_gray_, left_view);

    const sl::VIEW right_view = (enable_factory_rectification)
        ? sl::VIEW_RIGHT_GRAY
        : sl::VIEW_RIGHT_UNRECTIFIED_GRAY;
    zed_->retrieveImage(right_image_gray_, right_view);
  }
  if (rgb) {
    const sl::VIEW left_view = (enable_factory_rectification)
        ? sl::VIEW_LEFT
        : sl::VIEW_LEFT_UNRECTIFIED;
    zed_->retrieveImage(left_image_rgb_, left_view);

    const sl::VIEW right_view = (enable_factory_rectification)
        ? sl::VIEW_RIGHT
        : sl::VIEW_RIGHT_UNRECTIFIED;
    zed_->retrieveImage(right_image_rgb_, right_view);
  }
}

void ZedCamera::publishImageData(const int64_t acq_time) {
  const bool gray_scale = get_gray_scale();
  const bool rgb = get_rgb();

  // Retrieve camera parameters
  // 'raw' specifies if we want the parameters for unrectified images (true)
  // or rectified images (false)
  const bool raw = !get_enable_factory_rectification();
  const auto camera_parameters = raw
    ? zed_info_.calibration_parameters_raw
    : zed_info_.calibration_parameters;

  if (gray_scale) {
    publishGrayData(acq_time, camera_parameters);
  }
  if (rgb) {
    publishRgbData(acq_time, camera_parameters);
  }
}

void ZedCamera::publishCameraData(const int64_t acq_time) {
  // Publish camera extrinsic parameters
  const auto ext = tx_extrinsics().initProto();
  ToProto(GetCameraExtrinsics(zed_info_), ext);
  tx_extrinsics().publish(acq_time);
}

void ZedCamera::publishGrayData(int64_t acq_time, const sl::CalibrationParameters& camera_params) {
  auto l_camera = tx_left_camera_gray().initProto();
  auto r_camera = tx_right_camera_gray().initProto();
  l_camera.setColorSpace(ColorCameraProto::ColorSpace::GRAYSCALE);
  r_camera.setColorSpace(ColorCameraProto::ColorSpace::GRAYSCALE);
  SetCameraProtoIntrinsics(camera_params.left_cam, l_camera);
  SetCameraProtoIntrinsics(camera_params.right_cam, r_camera);

  Image1ub buffer_left_gray(zed_resolution_.height, zed_resolution_.width);
  Copy(GrayZedImageView(left_image_gray_), buffer_left_gray);
  show("left_gray_thumbnail",
        [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_gray)); });
  ToProto(std::move(buffer_left_gray), l_camera.getImage(), tx_left_camera_gray().buffers());

  Image1ub buffer_right_gray(zed_resolution_.height, zed_resolution_.width);
  Copy(GrayZedImageView(right_image_gray_), buffer_right_gray);
  show("right_gray_thumbnail",
        [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_gray)); });
  ToProto(std::move(buffer_right_gray), r_camera.getImage(), tx_right_camera_gray().buffers());

  tx_left_camera_gray().publish(acq_time);
  tx_right_camera_gray().publish(acq_time);
}

void ZedCamera::publishRgbData(int64_t acq_time, const sl::CalibrationParameters& camera_params) {
  auto l_camera = tx_left_camera_rgb().initProto();
  auto r_camera = tx_right_camera_rgb().initProto();
  l_camera.setColorSpace(ColorCameraProto::ColorSpace::RGB);
  r_camera.setColorSpace(ColorCameraProto::ColorSpace::RGB);
  SetCameraProtoIntrinsics(camera_params.left_cam, l_camera);
  SetCameraProtoIntrinsics(camera_params.right_cam, r_camera);

  Image3ub buffer_left_rgb(zed_resolution_.height, zed_resolution_.width);
  ConvertBgraToRgb(BgraZedImageView(left_image_rgb_), buffer_left_rgb);
  show("left_rgb_thumbnail",
        [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_rgb)); });
  show("left_rgb", [&](sight::Sop& sop) { sop.add(buffer_left_rgb); });
  ToProto(std::move(buffer_left_rgb), l_camera.getImage(), tx_left_camera_rgb().buffers());

  Image3ub buffer_right_rgb(zed_resolution_.height, zed_resolution_.width);
  ConvertBgraToRgb(BgraZedImageView(right_image_rgb_), buffer_right_rgb);
  show("right_rgb_thumbnail",
        [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_rgb)); });
  ToProto(std::move(buffer_right_rgb), r_camera.getImage(), tx_right_camera_rgb().buffers());

  tx_left_camera_rgb().publish(acq_time);
  tx_right_camera_rgb().publish(acq_time);
}

}  // namespace isaac
