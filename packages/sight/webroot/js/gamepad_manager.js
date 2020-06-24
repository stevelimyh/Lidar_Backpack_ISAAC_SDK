/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * Manages connected Gamepad Controllers
 * reference: https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API/Using_the_Gamepad_API
 * Only tracks Standard Gamepads. Reference: https://w3c.github.io/gamepad/#remapping
 * Note: The standard gamepad has 4 axes, and up to 17 buttons.
 * @constructor
 */
function GamepadManager() {
  GamepadManager.prototype.SCAN_TIMEOUT_MS = 100;

  /** @private @const duplicate of this to be accessed by private methods */
  const that_ = this;

  /** @private member variables */
  let gamepadsByIDs_ = {};
  let gamepadsByIndices_ = {};
  let numGamepads_ = 0;
  let onScanGamepadsCBs_ = {};  // map for all callbacks to be called during scan cycles
  let setScanIntervalReturnFn_ = null;

  /**
  * @desc sets Callback that is called when connected gamepads are scanned at regular intervals
  */
  this.addOnScanGamepadsCB = function(key, cb) {
    onScanGamepadsCBs_[key] = cb;
  };

  /**
  * @desc removes Callback that is called when connected gamepads are scanned at regular intervals
  */
  this.removeOnScanGamepadsCB = function(key) {
    delete onScanGamepadsCBs_[key];
  };

  /**
  * @desc returns num gamepads tracked by GamepadManager
  */
  this.numGamepads = function() {
    return numGamepads_;
  };

  /**
  * @desc gets gamepad by its ID info.
  *   User should check Gamepad.connected before using the object.
  * @return gamepad object
  */
  this.getGamepadByID = function(id) {
    return gamepadsByIDs_[id];
  };

  /**
  * @desc gets gamepad by its index.
  *   User should check Gamepad.connected before using the object.
  * @return gamepad object
  */
  this.getGamepadByIndex = function(index) {
    return gamepadsByIndices_[index];
  };

  /**
  * @desc gets indices of all gamepads
  * @return array of indices
  */
  this.getGamepadIndices = function() {
    return Object.keys(gamepadsByIndices_);
  };

  /**
  * @desc gets IDs of all gamepads
  * @return array of IDs
  */
  this.getGamepadIDs = function() {
    return Object.keys(gamepadsByIDs_);
  };

  /**
  * @desc starts gamepad scanning cycles
  */
  this.enableGamepadScanning = function() {
    // Regularly scan for newly connected gamepads
    // Why every 100ms? Because trying to keep consistent with input timeout interval in
    // packages/sensors/Joystick.hpp
    setScanIntervalReturnFn_ = setInterval(function() {
      delete gamepadsByIDs_;
      delete gamepadsByIndices_;
      let gamepads = navigator.getGamepads ? navigator.getGamepads() :
          (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
      numGamepads_ = 0;
      for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].mapping === 'standard') {  // Track only standard/remapped GP
          numGamepads_++;
          gamepadsByIDs_[gamepads[i].id] = gamepads[i];
          gamepadsByIndices_[gamepads[i].index] = gamepads[i];
        }
      }
      // Execute all the added callbacks
      for (let cbKey in onScanGamepadsCBs_) {
        let cb = onScanGamepadsCBs_[cbKey];
        if (cb !== null || cb !== undefined) {
          cb();
        }
      }
    }, that_.SCAN_TIMEOUT_MS);
  };

  /**
  * @desc stops gamepad scanning cycles
  */
  this.disableGamepadScanning = function() {
    delete gamepadsByIDs_;
    delete gamepadsByIndices_;
    numGamepads_ = 0;
    if (setScanIntervalReturnFn_ !== null) {
      clearInterval(setScanIntervalReturnFn_);
      setScanIntervalReturnFn_ = null;
    }
  };

  /** Constructor functionality */
  this.enableGamepadScanning();
}

// Create global singleton instance of GamepadManager
let gamepad_manager_ = null;
//  Create a singleton instance
/* Implement a "singleton" */
function GamepadManagerInstance() {
  if (gamepad_manager_ === null) {
    gamepad_manager_ = new GamepadManager();
  }
  return gamepad_manager_;
}
