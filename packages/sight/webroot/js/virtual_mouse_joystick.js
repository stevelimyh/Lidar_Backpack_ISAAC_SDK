/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * A Virtual Mouse Pad mimicing Joystick Axes
 * Uses Canvas. A button can be moved inside a circular region. Its movement simulates circular
 * motion of a Joystick Axis. Deadzone inside the mousepad is the area which is considered no
 * activity of simulated Joystick zone.
 * @constructor
 */
function VirtualMouseJoystick() {
  /** @private */
  this.canvas_ = document.createElement('canvas');
  /** @private */
  this.ctx_ = this.canvas_.getContext('2d');  // context of 2d canvas
  /** @private */
  this.div_ = document.createElement('div');
  /** @private */
  this.enabled_ = false;
  /** @private */
  this.isLeftButtonPressed_ = false;  // Mouse left pressed on center movable button means actively used
  /** @private @desc cached coordinates of pointer position while mousepad active */
  this.pointerPosFromCenter_ = {x: 0.0, y: 0.0};
  /** @private @desc sub-div for displaying title and state */
  this.disp_ = document.createElement('div');
  /** @private @desc stores return value of setInterval for mousepad scanning */
  this.setScanIntervalReturnFn_ = null;
  // TODO(akshaya): Define Empty function in sight_utils and use here
  /** @private @desc Callback when Mousepad is actively used */
  this.onMousepadScanCB_ = function() {};  // initialize with empty function

  /** Constructor functionality */
  this.div_.align = 'center';

  this.disp_.style.fontStyle = 'italic';
  this.disp_.innerHTML = this.TITLE + ': DISABLED';

  this.canvas_.width = this.canvas_.height = this.PAD_RADIUS * 2;
  this.canvas_.addEventListener('mousedown', this.onMouseDown_.bind(this));
  this.canvas_.addEventListener('mouseup', this.onMouseUp_.bind(this));
  this.canvas_.addEventListener('mouseleave', this.onMouseUp_.bind(this));
  this.canvas_.addEventListener('mousemove', this.onMouseMove_.bind(this));
  this.redraw_();

  this.div_.appendChild(this.canvas_);
  this.div_.appendChild(this.disp_);
}

// Constants
VirtualMouseJoystick.prototype.TITLE = 'Virtual Mousepad-Joystick';  // radius of circular region to move in
VirtualMouseJoystick.prototype.PAD_RADIUS = 75;  // radius of circular region to move in
VirtualMouseJoystick.prototype.PAD_RADIUS_SQUARED =
    VirtualMouseJoystick.prototype.PAD_RADIUS * VirtualMouseJoystick.prototype.PAD_RADIUS;
// radius of the center movable button
VirtualMouseJoystick.prototype.BTN_RADIUS = 0.1 * VirtualMouseJoystick.prototype.PAD_RADIUS;
VirtualMouseJoystick.prototype.BTN_RADIUS_SQUARED =
    VirtualMouseJoystick.prototype.BTN_RADIUS * VirtualMouseJoystick.prototype.BTN_RADIUS;
// radius of the dead zone from center of the pad
// Deadzone at the center of mouse pad: where mousepad considers no simulated Joystick activity
VirtualMouseJoystick.prototype.DEAD_ZONE_RADIUS = 0.35 * VirtualMouseJoystick.prototype.PAD_RADIUS;
VirtualMouseJoystick.prototype.DEAD_ZONE_RADIUS_SQUARED =
    VirtualMouseJoystick.prototype.DEAD_ZONE_RADIUS *
    VirtualMouseJoystick.prototype.DEAD_ZONE_RADIUS;
VirtualMouseJoystick.prototype.DISABLED_CLR = '#CACFC9';
VirtualMouseJoystick.prototype.BTN_CLR = '#217315';  // center movable button color
VirtualMouseJoystick.prototype.SCAN_TIMEOUT_MS = 100;

/** PRIVATE METHODS */
// START

/**
 * @desc draws a circular region at (x,y) with given radius and color
 * @private
 */
VirtualMouseJoystick.prototype.drawCircle_ = function(x, y, radius, color) {
  this.ctx_.beginPath();
  this.ctx_.arc(x, y, radius, 0, 2 * Math.PI);
  this.ctx_.fillStyle = color;
  this.ctx_.fill();
  this.ctx_.closePath();
}

/**
 * @desc redraws mousepad and movable button
 * @private
 */
VirtualMouseJoystick.prototype.redraw_ = function() {
  // clear canvas before redraw
  this.ctx_.clearRect(0, 0, this.canvas_.width, this.canvas_.height);
  // draws circular region of virtual pad to move button circle in
  if (this.enabled_ === false) {
    this.drawCircle_(this.PAD_RADIUS, this.PAD_RADIUS, this.PAD_RADIUS, this.DISABLED_CLR);
  } else {
    this.drawCircle_(this.PAD_RADIUS, this.PAD_RADIUS, this.PAD_RADIUS, SUCCESS_CLR);
  }
  // draws circular deadzone at center of virtual pad
  this.drawCircle_(this.PAD_RADIUS, this.PAD_RADIUS, this.DEAD_ZONE_RADIUS, this.DISABLED_CLR);
  // draws movable button circle
  this.drawCircle_(this.pointerPosFromCenter_.x + this.PAD_RADIUS,
      this.pointerPosFromCenter_.y + this.PAD_RADIUS, this.BTN_RADIUS, this.BTN_CLR);
}

/**
 * @desc on mousedown event CB
 * @private
 */
VirtualMouseJoystick.prototype.onMouseDown_ = function(event) {
  if (this.enabled_ === false) return;
  // do not process if left button not pressed
  if (event.button !== 0) return;
  if (this.isLeftButtonPressed_ === false) {  // not already active
    // if pressed on center movable button then we are active
    let rect = this.canvas_.getBoundingClientRect();
    let x = event.clientX - rect.left - this.PAD_RADIUS;
    let y = event.clientY - rect.top - this.PAD_RADIUS;
    if ((x * x) + (y * y) <= this.BTN_RADIUS_SQUARED) {
      this.isLeftButtonPressed_ = true;
      // cache position for later use
      this.pointerPosFromCenter_.x = x;
      this.pointerPosFromCenter_.y = y;
    }
  }
}

/**
 * @desc on mouseup event CB
 * @private
 */
VirtualMouseJoystick.prototype.onMouseUp_ = function(event) {
  if (this.enabled_ === false) return;
  // do not process if left button not pressed
  if (event.button !== 0) return;
  this.isLeftButtonPressed_ = false;  // reset state
  this.pointerPosFromCenter_.x = this.pointerPosFromCenter_.y = 0.0;
  this.redraw_();
}

/**
 * @desc on mousemove event CB
 * @private
 */
VirtualMouseJoystick.prototype.onMouseMove_ = function(event) {
  if (this.enabled_ === false || this.isLeftButtonPressed_ === false) return;
  // draw center movable button according to new mouse pointer position inside pad
  let rect = this.canvas_.getBoundingClientRect();
  let posInRectX = event.clientX - rect.left;
  let posInRectY = event.clientY - rect.top;
  let posFromCenterX = posInRectX - this.PAD_RADIUS;
  let posFromCenterY = posInRectY - this.PAD_RADIUS;
  if (isPointInCircle(posFromCenterX, posFromCenterY, this.PAD_RADIUS_SQUARED) <= 0) {
    // cache position for later use
    this.pointerPosFromCenter_.x = posFromCenterX;
    this.pointerPosFromCenter_.y = posFromCenterY;

    this.redraw_();
  }
}

// END

/** PUBLIC METHODS */
// START

/**
 * @desc show/hide info/title display
 */
VirtualMouseJoystick.prototype.showInfoDisplay = function(value) {
  if (value === true) {
    this.div_.appendChild(this.disp_);
  } else {
    this.div_.removeChild(this.disp_);
  }
}

/**
 * @desc disables Mousepad and dulls its UI
 */
VirtualMouseJoystick.prototype.disable = function() {
  this.enabled_ = false;
  this.disp_.innerHTML = this.TITLE + ': DISABLED';
  this.redraw_();
  if (this.setScanIntervalReturnFn_ !== null) {
    clearInterval(this.setScanIntervalReturnFn_);
    this.setScanIntervalReturnFn_ = null;
  }
}

/**
 * @desc enables Mousepad and lights up its UI
 */
VirtualMouseJoystick.prototype.enable = function() {
  const that_ = this;

  this.enabled_ = true;
  this.disp_.innerHTML = this.TITLE + ': ENABLED&nbsp;';

  // start the regular mousepad state scanning and execute the associated callbacks
  this.setScanIntervalReturnFn_ = setInterval(function() {
    if (that_.enabled_ === false) return;
    that_.onMousepadScanCB_();
  }, this.SCAN_TIMEOUT_MS);

  this.redraw_();
}

/**
 * @desc set on mouse pad active callback.
 */
VirtualMouseJoystick.prototype.addOnMousepadScanCB = function(cb) {
  this.onMousepadScanCB_ = cb;
}

/**
 * @desc get current simulated Josytick State JSON from mouse pad
 * @return {Object} Json with 4 axes info and deadman button pressed info
 */
VirtualMouseJoystick.prototype.getSimulatedJoystickStateJson = function() {
  let json = {'axes': [0.0, 0.0, 0.0, 0.0], 'deadman-button': false};  // initialize to no activity
  if (this.isLeftButtonPressed_ === true && this.enabled_ === true &&
      isPointInCircle(this.pointerPosFromCenter_.x, this.pointerPosFromCenter_.y,
      this.DEAD_ZONE_RADIUS_SQUARED) >= 0) {  // check if deadzone
    // Fill axes values in JSON
    // Since DifferentialBaseJoysick codelet is just concerned about Axis 1 and 2, we fill others
    // with 0.0
    json.axes[1] = (this.pointerPosFromCenter_.y * 1.0)/this.PAD_RADIUS;
    json.axes[2] = (this.pointerPosFromCenter_.x * 1.0)/this.PAD_RADIUS;
    // Add deadman button pressed state as true
    json['deadman-button'] = true;
  }
  return json;
}

// @desc returns div containing this virtual mousepad
VirtualMouseJoystick.prototype.getElementDiv = function() {
  return this.div_;
}

// END
