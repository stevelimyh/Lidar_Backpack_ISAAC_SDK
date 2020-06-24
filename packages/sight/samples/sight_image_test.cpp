/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/tools/parse_command_line.hpp"
#include "engine/alice/tools/websight.hpp"
#include "engine/gems/image/color.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"

namespace isaac {

// Display an image in sight at 50 fps
template <class Image>
class ImageNode : public alice::Codelet {
 public:
  void tick() override {
    show("image", [&](sight::Sop& sop) { sop.add(effect(img_, getTickCount())); });
    const double y =
        static_cast<double>(img_.cols()) * (1.0 + std::cos(getTickCount() * 0.02) * 0.9) * 0.5;
    const double x =
        static_cast<double>(img_.rows()) * (1.0 + std::cos(getTickCount() * 0.0071)) * 0.5;
    show("label", [&](sight::Sop& sop) {
      sop.style = sight::SopStyle{"#f00", false, 20.0};
      sop.add(sight::SopText(get_name(), Vector2d(x, y)));
    });
  }

  void start() override {
    tickPeriodically();
    LoadPng(get_image_filename(), img_);
    img_ = Reduce<2>(img_);
  }

  // Apply some "drunk" effect to avoid having a static image.
  Image effect(const Image& img, int start) {
    Image res(img.rows(), img.cols());
    for (int row = 0; row < img.rows(); row++) {
      const int dec = static_cast<int>(5.0 * std::cos((start + row) * 0.2));
      for (int col = 0; col < img.cols(); col++) {
        int rcol = std::max<int>(0, std::min<int>(img.cols() - 1, dec + col));
        res(row, col) = img(row, rcol);
      }
    }
    return res;
  }

  // Image to be loaded
  ISAAC_PARAM(std::string, image_filename);
  // Name used to render on sight
  ISAAC_PARAM(std::string, name);

 protected:
  Image img_;
};

// Render an image with a projection model
class PinholeImageNode : public alice::Codelet {
 public:
  void tick() override {
    show("image", [&](sight::Sop& sop) {
      geometry::Pinhole<double> pinhole;
      pinhole.dimensions = image_.dimensions();
      pinhole.center = get_center();
      pinhole.focal = get_focal();
      sop.transform = sight::SopTransform(get_world_T_camera(), pinhole);
      sop.add(image_);
    });
    const double angle_start = getTickCount() * 0.1;
    show("path", [&](sight::Sop& sop) {
      std::vector<Vector3d> left, right;
      for (double d = -20.0; d <= 20.0; d += 0.1) {
        const Vector3d pt(d, std::cos(angle_start - d) * 0.5,
                          0.5 * (1.0 + std::sin(angle_start * 0.5)));
        left.push_back(pt + Vector3d(0.0, 0.5, 0.0));
        right.push_back(pt + Vector3d(0.0, -0.5, 0.0));
      }
      sop.add(left, sight::SopStyle{"red", true});
      sop.add(right, sight::SopStyle{"red", true});
    });
    show("bounding_box", [&](sight::Sop& sop) {
      const double offset = 20.0 + 10.0 * std::cos(angle_start);
      const Vector2d bbox_min(offset, offset);
      const Vector2d bbox_max(image_.rows() - offset, image_.cols() - offset);

      sop.transform = sight::SopTransform::CanvasFrame();
      sop.add(geometry::RectangleD::FromOppositeCorners(bbox_min, bbox_max),
              sight::SopStyle("blue", false));
    });
  }

  void start() override {
    tickPeriodically();
    LoadPng(get_path(), image_);
  }

  // Transformation from the camera frame to the world
  ISAAC_PARAM(Pose3d, world_T_camera,
              (Pose3d{SO3d::FromAxisAngle({1.0, 0.0, 0.0}, Pi<double>) *
                          SO3d::FromAxisAngle({0.0, 1.0, 0.0}, Pi<double> / 2) *
                          SO3d::FromAxisAngle({0.0, 0.0, 1.0}, Pi<double> / 2),
                      Vector3d(0.0, 0.0, 1.0)}));
  // Position of the center of the camera in the image
  ISAAC_PARAM(Vector2d, center, Vector2d(270.0, 480.0));
  // Focal of the camera
  ISAAC_PARAM(Vector2d, focal, Vector2d(270.0, 270.0));
  // Image to be laoded
  ISAAC_PARAM(std::string, path);
  // Name used to render on sight
  ISAAC_PARAM(std::string, name);

 protected:
  Image3ub image_;
};

// Render a mesh in 3D and 2D projection view
class MovingMesh : public alice::Codelet {
 public:
  void tick() override {
    const double angle_start = getTickCount() * 0.025;
    show("mesh", [&](sight::Sop& sop) {
      sop.transform = Pose2d::FromXYA(5.0 + 3.5 * std::sin(angle_start),
                                      -3.5 * std::cos(angle_start), angle_start);
      sop.add(sight::SopAsset::FromName("carter"));
    });
  }

  void start() override {
    tickPeriodically();
  }
};

// Display a cloud points in sight at 20 fps (648k pnts/s)
class View3DClouds : public alice::Codelet {
 public:
  void tick() override {
    show(get_name(), [&](sight::Sop& sop) {
      sop.transform = sight::SopTransform(
          {Pose3d::Rotation(Vector3d(0.0, 1.0, 0.0), Pi<double> * (0.5 + get_tilt_angle()))});
      effect(getTickCount(), sop);
    });
  }

  void start() override {
    tickPeriodically();
    LoadPng(get_path() + "depth.png", depth_);
    depth_ = Reduce<8>(depth_);
    LoadPng(get_path() + "right.png", color_);
    color_ = Reduce<8>(color_);
  }

  // Apply some "drunk" effect to avoid having a static image.
  void effect(int start, sight::Sop& sop) {
    SampleCloud3f points(depth_.rows() * depth_.cols());
    SampleCloud3f colors(depth_.rows() * depth_.cols());
    Vector3f center(static_cast<double>(depth_.rows()) * 0.5,
                    static_cast<double>(depth_.cols()) * 0.5, 0.0);
    for (int row = 0; row < depth_.rows(); row++) {
      const int dec = static_cast<int>(5.0 * std::cos((start + row) * 0.2));
      for (int col = 0; col < depth_.cols(); col++) {
        int rcol = std::max<int>(0, std::min<int>(depth_.cols() - 1, dec + col));
        const Vector3f pt = 0.001 * (depth_(row, rcol) * (Vector3f(row, col, 100.0) - center));
        points(row * depth_.cols() + col) = pt;
        const auto pix = color_(row, rcol);
        colors(row * depth_.cols() + col) =
            Vector3f(pix[0] / 255.0f, pix[1] / 255.0f, pix[2] / 255.0f);
      }
    }
    sop.add(sight::SopPointCloud::Create(points, colors));
  }

  // Image to be laoded
  ISAAC_PARAM(std::string, path);
  // Name used to render on sight
  ISAAC_PARAM(std::string, name);
  // Angle to tilt the camera
  ISAAC_PARAM(double, tilt_angle, -0.04);

 protected:
  Image1ub depth_;
  Image3ub color_;
};

void UnrealImageToSight(alice::Application& app) {
  auto left = app.createMessageNode("left")->addComponent<ImageNode<Image3ub>>();
  left->async_set_name("left");
  left->async_set_image_filename("engine/gems/image/data/left.png");
  left->async_set_tick_period("50hz");
  auto right = app.createMessageNode("right")->addComponent<ImageNode<Image3ub>>();
  right->async_set_name("right");
  right->async_set_image_filename("engine/gems/image/data/right.png");
  right->async_set_tick_period("50hz");
  auto depth = app.createMessageNode("depth")->addComponent<ImageNode<Image1ub>>();
  depth->async_set_name("depth");
  depth->async_set_image_filename("engine/gems/image/data/depth.png");
  depth->async_set_tick_period("50hz");
  auto label = app.createMessageNode("label")->addComponent<ImageNode<Image3ub>>();
  label->async_set_name("label");
  label->async_set_image_filename("engine/gems/image/data/label.png");
  label->async_set_tick_period("50hz");

  auto cloud3d = app.createMessageNode("cloud3d")->addComponent<View3DClouds>();
  cloud3d->async_set_name("cloud3d");
  cloud3d->async_set_path("engine/gems/image/data/");
  cloud3d->async_set_tick_period("20hz");

  auto mesh = app.createMessageNode("mesh")->addComponent<MovingMesh>();
  mesh->async_set_tick_period("20hz");

  auto pinhole = app.createMessageNode("pinhole")->addComponent<PinholeImageNode>();
  pinhole->async_set_name("pinhole");
  pinhole->async_set_path("engine/gems/image/data/stairs.png");
  pinhole->async_set_tick_period("50hz");
}

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ImageNode<Image1ub>);
ISAAC_ALICE_REGISTER_CODELET(isaac::ImageNode<Image3ub>);
ISAAC_ALICE_REGISTER_CODELET(isaac::PinholeImageNode);
ISAAC_ALICE_REGISTER_CODELET(isaac::View3DClouds);
ISAAC_ALICE_REGISTER_CODELET(isaac::MovingMesh);

int main(int argc, char** argv) {
  isaac::alice::Application app(isaac::alice::ParseApplicationCommandLine("sight_test"));
  isaac::alice::InitializeSightApi(app);
  app.loadFromText(R"???(
{
  "config": {
    "websight": {
      "WebsightServer": {
        "port": 3000,
        "ui_config": {
          "windows": {
            "3D View": {
              "renderer": "3d",
              "dims": { "width":512, "height":512 },
              "channels": [ { "name": "sight_test/cloud3d/isaac.View3DClouds/cloud3d" } ]
            },
            "Left View": {
              "renderer": "2d",
              "channels": [
                { "name": "sight_test/left/isaac.ImageNode<Image3ub>/image" },
                { "name": "sight_test/left/isaac.ImageNode<Image3ub>/label" }
              ]
            },
            "Right View": {
              "renderer": "2d",
              "channels": [
                { "name": "sight_test/right/isaac.ImageNode<Image3ub>/image" },
                { "name": "sight_test/right/isaac.ImageNode<Image3ub>/label" }
              ]
            },
            "Depth View": {
              "renderer": "2d",
              "channels": [
                { "name": "sight_test/depth/isaac.ImageNode<Image1ub>/image" },
                { "name": "sight_test/depth/isaac.ImageNode<Image1ub>/label" }
              ]
            },
            "Label View": {
              "renderer": "2d",
              "channels": [
                { "name": "sight_test/label/isaac.ImageNode<Image3ub>/image" },
                { "name": "sight_test/label/isaac.ImageNode<Image3ub>/label" }
              ]
            },
            "Pinnole View": {
              "renderer": "2d",
              "channels": [
                { "name": "sight_test/pinhole/isaac.PinholeImageNode/image" },
                { "name": "sight_test/pinhole/isaac.PinholeImageNode/path" },
                { "name": "sight_test/mesh/isaac.MovingMesh/mesh" }
              ]
            }
          },
          "assets": {
            "carter": {
              "obj": "apps/assets/carter.obj",
              "diffuse_map": "apps/assets/carter_albido.png",
              "normal_map": "apps/assets/carter_normal.png",
              "rotation": [-0.5, -0.5, 0.5, 0.5],
              "scale": 0.01
            }
          }
        }
      }
    }
  }
})???");
  isaac::UnrealImageToSight(app);
  app.startWaitStop();
  return 0;
}
