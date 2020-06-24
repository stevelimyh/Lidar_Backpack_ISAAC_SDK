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

  isaac_error_t error;

  // create and start the application
  isaac_handle_t app;
  const char* asset_path = "";
  const char* app_file = "apps/tutorials/c_api/viewer.app.json";
  error = isaac_create_application(asset_path, app_file, 0, 0, 0, 0, &app);
  ABORT_ON_ERROR(error);
  error = isaac_start_application(app);
  ABORT_ON_ERROR(error);

  const int kRows = 540;
  const int kCols = 960;

  // create the JSON object for the dummy camera message
  isaac_const_json_t json;
  json.data = R"%%%(
{
  "colorSpace": "rgb",
  "image": {
    "elementType": "uint8",
    "rows": 540,
    "cols": 960,
    "channels": 3,
    "dataBufferIndex": 0
  }
}
)%%%";
  json.size = strlen(json.data) + 1;  // +1 for null-terminating character

  // create a buffer for the dummy camera message
  isaac_buffer_t buffer;
  buffer.size = kRows * kCols * 3;
  uint8_t* pointer = (uint8_t*)malloc(buffer.size);  // NOLINT
  buffer.pointer = pointer;
  buffer.storage = isaac_memory_host;

  for (int k = 0; stop_requested == false; k++) {
    // write the buffer data
    int index = 0;
    for (int i = 0; i < kRows; i++) {
      for (int j = 0; j < kCols; j++, index += 3) {
        const int ii = i / 16;
        const int jj = j / 16;
        const int base = k % 2 == 0 ? 0 : 255;
        const int value = (ii % 2) == (jj % 2) ? (255 - base) : base;
        pointer[index] = i % 256;
        pointer[index + 1] = j % 256;
        pointer[index + 2] = value;
      }
    }

    // create a new message
    isaac_uuid_t uuid;
    error = isaac_create_message(app, &uuid);
    ABORT_ON_ERROR(error);

    // Set the correct proto ID for the message (take it from the docs)
    error = isaac_set_message_proto_id(app, &uuid, 12905539496848989000llu);
    ABORT_ON_ERROR(error);

    // write the JSON part of the message
    error = isaac_write_message_json(app, &uuid, &json);
    ABORT_ON_ERROR(error);

    // add the buffer to the message
    error = isaac_message_append_buffer(app, &uuid, &buffer, 0);
    ABORT_ON_ERROR(error);

    // enable conversion from JSON to proto
    error = isaac_set_message_auto_convert(app, &uuid, isaac_message_type_proto);
    ABORT_ON_ERROR(error);

    // publish the message
    error = isaac_publish_message(app, "viewer", "camera", "color_listener", &uuid);
    ABORT_ON_ERROR(error);

    // inform the user
    printf("Published a new message with UUID %lx-%lx\n", uuid.upper, uuid.lower);

    // wait a while
    usleep(200000);  // 5 Hz
  }

  // Free up data allocated for the image buffer
  free(pointer);

  // stop and destroy the application
  error = isaac_stop_application(app);
  ABORT_ON_ERROR(error);
  error = isaac_destroy_application(&app);
  ABORT_ON_ERROR(error);

  return 0;
}
