/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * A Virtual Keypad simulating Joystick Axes
 * 4 keys for Front, Left, Back, Right
 * Upon keeping motion keys pressed we send a constant Joystick axis value between 0-1.
 * @constructor
 */
function VirtualKeypadJoystick() {
  /** @private */
  this.div_ = document.createElement('div');
  /** @private */
  this.enabled_ = false;
  /** @private */
  this.failsafeButtonState_ = 0;
  /** @private @desc sub-div for displaying title and state */
  this.disp_ = document.createElement('div');
  /** @private @desc keypad buttons map */
  this.keys_ = {
    [this.KEY_RIGHT]: {
      value: 0.0,
      pressed: false,
      uiButton: {}
    },
    [this.KEY_LEFT]: {
      value: 0.0,
      pressed: false,
      uiButton: {}
    },
    [this.KEY_BACK]: {
      value: 0.0,
      pressed: false,
      uiButton: {}
    },
    [this.KEY_FRONT]: {
      value: 0.0,
      pressed: false,
      uiButton: {}
   }
  };
  /** @private @desc stores return value of setInterval for keypad scanning */
  this.setScanIntervalReturnFn_ = null;
  // TODO(akshaya): Define Empty function in sight_utils and use here
  /** @private @desc Callback when Keypadpad is actively used */
  this.onKeypadpadScanCB_ = function() {};  // initialize with empty function

  /** Constructor functionality */
  Object.preventExtensions(this.keys_);

  // Add keyevent listeners
  document.addEventListener('keyup', this.onKeyUp_.bind(this));
  document.addEventListener('keypress', this.onKeyPressed_.bind(this));

  // Initialize ui buttons
  let numKeys = this.KEYS.length;
  for (let i = 0; i < numKeys; i++) {
    this.keys_[this.KEYS[i].key].uiButton =
        renderBtn(this.KEYS[i].btnId, this.KEYS[i].btnLabel, this.BTN_PRIMARY_CLASS);
  }

  // front motion button in next div
  this.div_.appendChild(document.createElement('div').appendChild(
      this.keys_[this.KEY_FRONT].uiButton));
  this.div_.appendChild(this.keys_[this.KEY_FRONT].uiButton);
  // left, right motion buttons in next div
  let div = document.createElement('div');
  div.appendChild(this.keys_[this.KEY_LEFT].uiButton);
  div.appendChild(this.keys_[this.KEY_RIGHT].uiButton);
  this.div_.appendChild(div);
  // back motion button in next div
  this.div_.appendChild(this.keys_[this.KEY_BACK].uiButton);

  this.disp_.style.fontStyle = 'italic';
  this.disp_.innerHTML = this.TITLE + ': DISABLED';
  this.div_.appendChild(this.disp_);
  this.div_.align = 'center';

  this.reColorButtons_();
}

// Constants
VirtualKeypadJoystick.prototype.TITLE = 'Virtual Keypad-Joystick';

VirtualKeypadJoystick.prototype.UI_BTN_FRONT_ID = '__virtual-keypad-front-btn';
VirtualKeypadJoystick.prototype.UI_BTN_BACK_ID = '__virtual-keypad-back-btn';
VirtualKeypadJoystick.prototype.UI_BTN_LEFT_ID = '__virtual-keypad-left-btn';
VirtualKeypadJoystick.prototype.UI_BTN_RIGHT_ID = '__virtual-keypad-right-btn';

VirtualKeypadJoystick.prototype.BTN_PRIMARY_CLASS = 'btn btn-static-isaac';
VirtualKeypadJoystick.prototype.BTN_DISABLE_CLASS =
    VirtualKeypadJoystick.prototype.BTN_PRIMARY_CLASS + ' virtual-keypad-joystick-disabled-btn';
VirtualKeypadJoystick.prototype.BTN_UP_CLASS =
    VirtualKeypadJoystick.prototype.BTN_PRIMARY_CLASS + ' virtual-keypad-joystick-btn-up';
VirtualKeypadJoystick.prototype.BTN_DOWN_CLASS =
    VirtualKeypadJoystick.prototype.BTN_PRIMARY_CLASS + ' virtual-keypad-joystick-btn-down';

VirtualKeypadJoystick.prototype.KEY_LABEL_FRONT = '<i class="material-icons">arrow_upward</i>';
VirtualKeypadJoystick.prototype.KEY_LABEL_BACK = '<i class="material-icons">arrow_downward</i>';
VirtualKeypadJoystick.prototype.KEY_LABEL_LEFT = '<i class="material-icons">arrow_back</i>';
VirtualKeypadJoystick.prototype.KEY_LABEL_RIGHT = '<i class="material-icons">arrow_forward</i>';
VirtualKeypadJoystick.prototype.KEY_FRONT = 'w';
VirtualKeypadJoystick.prototype.KEY_BACK = 's';
VirtualKeypadJoystick.prototype.KEY_LEFT = 'a';
VirtualKeypadJoystick.prototype.KEY_RIGHT = 'd';

VirtualKeypadJoystick.prototype.KEYS = [
  {
    key: VirtualKeypadJoystick.prototype.KEY_LEFT,
    btnId: VirtualKeypadJoystick.prototype.UI_BTN_LEFT_ID,
    btnLabel: VirtualKeypadJoystick.prototype.KEY_LEFT + ' - ' +
      VirtualKeypadJoystick.prototype.KEY_LABEL_LEFT
  },
  {
    key: VirtualKeypadJoystick.prototype.KEY_RIGHT,
    btnId: VirtualKeypadJoystick.prototype.UI_BTN_RIGHT_ID,
    btnLabel: VirtualKeypadJoystick.prototype.KEY_RIGHT + ' - ' +
      VirtualKeypadJoystick.prototype.KEY_LABEL_RIGHT
  },
  {
    key: VirtualKeypadJoystick.prototype.KEY_FRONT,
    btnId: VirtualKeypadJoystick.prototype.UI_BTN_FRONT_ID,
    btnLabel: VirtualKeypadJoystick.prototype.KEY_FRONT + ' - ' +
      VirtualKeypadJoystick.prototype.KEY_LABEL_FRONT
  },
  {
    key: VirtualKeypadJoystick.prototype.KEY_BACK,
    btnId: VirtualKeypadJoystick.prototype.UI_BTN_BACK_ID,
    btnLabel: VirtualKeypadJoystick.prototype.KEY_BACK + ' - ' +
      VirtualKeypadJoystick.prototype.KEY_LABEL_BACK
  }
];
Object.freeze(VirtualKeypadJoystick.prototype.KEYS);

VirtualKeypadJoystick.prototype.KEY_PRESS_VALUE = 0.5;
VirtualKeypadJoystick.prototype.SCAN_TIMEOUT_MS = 100;

/** PRIVATE METHODS */
// START

/**
 * @desc refills all ui buttons' colors
 * @private
 */
VirtualKeypadJoystick.prototype.reColorButtons_ = function() {
  let numKeys = this.KEYS.length;
  if (this.enabled_ === false) {
    for (let i = 0; i < numKeys; i++) {
      this.keys_[this.KEYS[i].key].uiButton.className = this.BTN_DISABLE_CLASS;
    }
  } else {
    for (let i = 0; i < numKeys; i++) {
      if (this.keys_[this.KEYS[i].key].pressed === true) {
        this.keys_[this.KEYS[i].key].uiButton.className = this.BTN_DOWN_CLASS;
      } else {
        this.keys_[this.KEYS[i].key].uiButton.className = this.BTN_UP_CLASS;
      }
    }
  }
}

/**
 * @desc on keypressed event CB
 * @private
 */
VirtualKeypadJoystick.prototype.onKeyPressed_ = function(event) {
  if (this.enabled_ === false) return;
  // if key of interest increase its equivalent motion velocity until 1
  if (this.keys_[event.key] && !this.keys_[event.key].pressed) {  // key is of interest
    this.failsafeButtonState_++;
    this.keys_[event.key].pressed = true;
    this.keys_[event.key].value = this.KEY_PRESS_VALUE;
    // update ui button state
    this.reColorButtons_();
  }
}

/**
 * @desc on keyup event CB
 * @private
 */
VirtualKeypadJoystick.prototype.onKeyUp_ = function(event) {
  if (this.enabled_ === false) return;
  // if key of interest reset its equivalent motion velocity to 0 and pressed state to false
  if (this.keys_[event.key] && this.keys_[event.key].pressed) {  // key is of interest
    this.failsafeButtonState_--;
    this.keys_[event.key].value = 0.0;
    this.keys_[event.key].pressed = false;
    // update ui button state
    this.reColorButtons_();
  }
}

// END

/** PUBLIC METHODS */
// START

/**
 * @desc show/hide info/title display
 */
VirtualKeypadJoystick.prototype.showInfoDisplay = function(value) {
  if (value === true) {
    this.div_.appendChild(this.disp_);
  } else {
    this.div_.removeChild(this.disp_);
  }
}

/**
 * @desc disables Keypad and dulls its UI
 */
VirtualKeypadJoystick.prototype.disable = function() {
  this.enabled_ = false;
  this.disp_.innerHTML = this.TITLE + ': DISABLED';
  this.reColorButtons_();
  if (this.setScanIntervalReturnFn_ !== null) {
    clearInterval(this.setScanIntervalReturnFn_);
    this.setScanIntervalReturnFn_ = null;
  }
}

/**
 * @desc enables Keypad and lights up its UI
 */
VirtualKeypadJoystick.prototype.enable = function() {
  const that_ = this;
  this.enabled_ = true;
  this.disp_.innerHTML = this.TITLE + ': ENABLED&nbsp;';

  // start the regular keypadpad state scanning and execute the associated callbacks
  this.setScanIntervalReturnFn_ = setInterval(function() {
    if (that_.enabled_ === false) return;
    that_.onKeypadpadScanCB_();
  }, this.SCAN_TIMEOUT_MS);

  this.reColorButtons_();
}

/**
 * @desc set on keypad pad active callback.
 */
VirtualKeypadJoystick.prototype.addOnKeypadpadScanCB = function(cb) {
  this.onKeypadpadScanCB_ = cb;
}

/**
 * @desc get current simulated Josytick State JSON from keypad pad
 * @return {Object} Json with 4 axes info and deadman button pressed info
 */
VirtualKeypadJoystick.prototype.getSimulatedJoystickStateJson = function() {
  let json = {'axes': [0.0, 0.0, 0.0, 0.0], 'deadman-button': false};  // initialize to no activity
  if (this.enabled_ === true) {
    // Fill axes values in JSON
    // Since DifferentialBaseJoysick codelet is just concerned about Axis 1 and 2, we fill others
    // with 0.0
    json.axes[1] = this.keys_[this.KEY_BACK].value - this.keys_[this.KEY_FRONT].value;
    json.axes[2] = this.keys_[this.KEY_RIGHT].value - this.keys_[this.KEY_LEFT].value;
    // Add deadman button pressed state
    json['deadman-button'] = this.failsafeButtonState_ > 0;
  }
  return json;
}

// @desc returns div containing this virtual keypadpad
VirtualKeypadJoystick.prototype.getElementDiv = function() {
  return this.div_;
}

// END
