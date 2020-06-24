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
#include <string.h>
#include <unistd.h>

#include "engine/alice/c_api/isaac_c_api.h"

#define ABORT_ON_ERROR(error)                                                   \
  if (error != isaac_error_success) {                                           \
    fprintf(stderr, "ERROR isaac_c_api: %s\n", isaac_get_error_message(error)); \
    abort();                                                                    \
  }

// An interrupt handler which will stop the program when Ctr+C is pressed
volatile bool stop_requested = false;
void sigintHandler(int sig_num) {
  stop_requested = true;
  printf("\nStopping program due to SIGINT\n");
}

// Example stand alone application that uses the Isaac C API to send and outbound message to an
// Isaac node that will echo back any messages it receives. See README.md for more information.
int main(int argc, char** argv) {
  // Stop the program when ctrl+c is pressed
  signal(SIGINT, sigintHandler);

  isaac_error_t error;  // holds error codes returned from isaac_c_api

  // Create and start the application
  isaac_handle_t app;
  error = isaac_create_application("", "c_api_example.app.json", 0, 0, 0, 0, &app);
  ABORT_ON_ERROR(error);
  error = isaac_start_application(app);
  ABORT_ON_ERROR(error);

  // Create outbound message
  isaac_const_json_t out_json;
  out_json.data = "{ \"message\": \"Hello World!\" }";
  out_json.size = strlen(out_json.data) + 1;

  while (stop_requested == false) {
    // Send outbound message to echo nodes in channel
    isaac_uuid_t out_message;
    isaac_create_message(app, &out_message);
    error = isaac_write_message_json(app, &out_message, &out_json);
    ABORT_ON_ERROR(error);
    error = isaac_publish_message(app, "echo", "message_ledger", "in", &out_message);
    ABORT_ON_ERROR(error);

    // Receive inbound message from echo nodes out channel
    isaac_uuid_t in_message;
    error = isaac_receive_latest_new_message(app, "echo", "message_ledger", "out", &in_message);
    ABORT_ON_ERROR(error);
    if (error == isaac_error_success) {
      // Get the pointer to the JSON payload
      isaac_const_json_t const_json = isaac_create_null_const_json();
      error = isaac_get_message_json(app, &in_message, &const_json);
      ABORT_ON_ERROR(error);

      // Print information to the terminal about the payload we just received
      printf("received json message: %s\n", const_json.data);

      // Release message
      error = isaac_release_message(app, &in_message);
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
