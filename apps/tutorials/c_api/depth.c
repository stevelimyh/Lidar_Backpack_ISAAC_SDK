/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "engine/alice/c_api/isaac_c_api.h"

#define ABORT_ON_ERROR(error)                                                   \
  if (error != isaac_error_success) {                                           \
    fprintf(stderr, "ERROR isaac_c_api: %s\n", isaac_get_error_message(error)); \
    abort();                                                                    \
  }

// An interrupt which will stop the program when Ctr+C is pressed
volatile bool stop_requested = false;
void sigintHandler(int sig_num) {
  stop_requested = true;
  printf("\nStopping program due to SIGINT\n");
}

int main(int argc, char** argv) {
  // Stop the program when Ctrl+C is pressed
  signal(SIGINT, sigintHandler);

  static const int kCenterPixel = 320 * 120 + 160;  // index of center pixel in a 320x240 image
  static const int64_t kMaxBuffersSize = 4;  // maximum amount of buffers we can receive at once
  isaac_buffer_t buffers[kMaxBuffersSize];   // holds pointers to buffers received from messages
  isaac_const_json_t json;                   // holds pointer to JSON payload received from messages
  isaac_uuid_t uuid;                         // message UUID
  isaac_error_t error;                       // holds error codes returned from isaac_c_api

  // create and start the application
  isaac_handle_t app;
  const char* asset_path = "";
  const char* app_file = "apps/tutorials/c_api/depth.app.json";
  error = isaac_create_application(asset_path, app_file, 0, 0, 0, 0, &app);
  ABORT_ON_ERROR(error);
  error = isaac_start_application(app);
  ABORT_ON_ERROR(error);

  while (stop_requested == false) {
    // request latest new message
    error = isaac_receive_latest_new_message(app, "camera_generator", "isaac.CameraGenerator",
                                             "depth", &uuid);

    if (error == isaac_error_success) {
      // we received a new message

      // get the JSON payload
      error = isaac_get_message_json(app, &uuid, &json);
      ABORT_ON_ERROR(error);

      // get the buffer payload, this contains the depth image data as described in the JSON payload
      int64_t buffer_count = kMaxBuffersSize;
      error = isaac_message_get_buffers(app, &uuid, buffers, &buffer_count, isaac_memory_host);
      ABORT_ON_ERROR(error);

      // print information to the terminal about the payload we just received
      printf("received json message: %s\n", json.data);
      for (int i = 0; i < buffer_count; i++) {
        float* image = (float*)(buffers[i].pointer);  // NOLINT
        printf("  buffers[%d] bytes:%ld, center_depth:%f\n", i, buffers[i].size,
               image[kCenterPixel]);
      }

      // release the message so isaac_c_api can reclaim the memory
      error = isaac_release_message(app, &uuid);
      ABORT_ON_ERROR(error);
    } else if (error != isaac_error_no_message_available) {
      // we received an error code other than no message available, abort
      ABORT_ON_ERROR(error);
    }

    // wait a while
    usleep(200000);  // 5 Hz
  }

  // stop and destroy the application
  error = isaac_stop_application(app);
  ABORT_ON_ERROR(error);
  error = isaac_destroy_application(&app);
  ABORT_ON_ERROR(error);

  return 0;
}
