/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * @desc converts a Gamepad's state to JSON that can be sent to backend
 *   Currently, only pressed state of buttons and analog values of axes are taken into account
 *   based on current isaac requirements.
 * @param {Object}: Gamepad Object
 * @return JSON object
 */
function gamepadStateToJson(gp) {
  let json = {'buttons': [], 'axes': []};
  // Fill buttons pressed state in JSON
  for (let i = 0; i < gp.buttons.length; i++) {
    json.buttons[i] = gp.buttons[i].pressed;
  }
  // Fill axes values in JSON
  for (let i = 0; i < gp.axes.length; i++) {
    json.axes[i] = gp.axes[i];
  }

  return json;
}

