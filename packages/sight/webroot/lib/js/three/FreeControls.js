/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// This set of controls offers a flying mode while preventing rolling.
THREE.FreeControls = function(camera) {
  this.camera = camera;
  this.keys = { LEFT:  37, UP:   38, RIGHT:  39, BOTTOM:  40,
                LEFT2: 65, UP2:  87, RIGHT2: 68, BOTTOM2: 83,
                HIGH:  82, DOWN: 70, HIGH2:  69, DOWN2: 81};
  // Mouse buttons
  this.mouseButtons = {ORBIT: THREE.MOUSE.LEFT};
  // Linear speed
  this.linear_speed = 5.0;
  // Angular speed
  this.angular_speed = 0.0015;

  // Whether we are currently moving in a direction
  this.moveUp = false;
  this.moveDown = false;
  this.moveRight = false;
  this.moveLeft = false;
  this.moveForward = false;
  this.moveBackward = false;

  // Update the position after a given delta time
  this.update = function(delta_time) {
    if (scope.moveUp) moveUp(this.linear_speed * delta_time);
    if (scope.moveDown) moveUp(-this.linear_speed * delta_time);
    if (scope.moveRight) moveLat(this.linear_speed * delta_time);
    if (scope.moveLeft) moveLat(-this.linear_speed * delta_time);
    if (scope.moveForward) moveForward(-this.linear_speed * delta_time);
    if (scope.moveBackward) moveForward(this.linear_speed * delta_time);
  };

  this.dispose = function() {};

  let scope = this;

  // Move the camera in a given position
  function moveForward(dist) {
    scope.camera.translateZ(dist);
  }
  function moveUp(dist) {
    scope.camera.translateY(dist);
  }
  function moveLat(dist) {
    scope.camera.translateX(dist);
  }

  // Detect mouse motion
  this.onMouseMove = (event) => {
    let dx = event.movementX || event.mozMovementX || event.webkitMovementX || 0;
    let dy = event.movementY || event.mozMovementY || event.webkitMovementY || 0;
    scope.camera.rotateOnWorldAxis(new THREE.Vector3(0.0, 0.0, 1.0), -dx * scope.angular_speed);
    scope.camera.rotateX(-dy * scope.angular_speed);
  };

  // Detect a key is pushed
  this.onKeyDown = (event) => {
    switch (event.keyCode) {
      case scope.keys.UP:
      case scope.keys.UP2:
        scope.moveForward = true;
        break;

      case scope.keys.BOTTOM:
      case scope.keys.BOTTOM2:
        scope.moveBackward = true;
        break;

      case scope.keys.LEFT:
      case scope.keys.LEFT2:
        scope.moveLeft = true;
        break;

      case scope.keys.RIGHT:
      case scope.keys.RIGHT2:
        scope.moveRight = true;
        break;

      case scope.keys.HIGH:
      case scope.keys.HIGH2:
        scope.moveUp = true;
        break;

      case scope.keys.DOWN:
      case scope.keys.DOWN2:
        scope.moveDown = true;
        break;
    }
    event.preventDefault();
  };

  // Detect a key is relased
  this.onKeyUp = (event) => {
    switch (event.keyCode) {
      case scope.keys.UP:
      case scope.keys.UP2:
        scope.moveForward = false;
        break;

      case scope.keys.BOTTOM:
      case scope.keys.BOTTOM2:
        scope.moveBackward = false;
        break;

      case scope.keys.LEFT:
      case scope.keys.LEFT2:
        scope.moveLeft = false;
        break;

      case scope.keys.RIGHT:
      case scope.keys.RIGHT2:
        scope.moveRight = false;
        break;

      case scope.keys.HIGH:
      case scope.keys.HIGH2:
        scope.moveUp = false;
        break;

      case scope.keys.DOWN:
      case scope.keys.DOWN2:
        scope.moveDown = false;
        break;
    }
    event.preventDefault();
  };
};

THREE.FreeControls.prototype = Object.create(THREE.EventDispatcher.prototype);
THREE.FreeControls.prototype.constructor = THREE.OrbitControls;
