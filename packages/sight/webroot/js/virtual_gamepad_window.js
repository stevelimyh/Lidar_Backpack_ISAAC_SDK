/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * Creates Virtual Gamepad Window UI on WebSight front-end.
 * @constructor
 * -- Uses Websocket to communicate with server(refer to index.html:my_socket)
 * -- Connects wth backend VirtualGamepadBridge on request. NOTE: only 1 such widget can get
 *    connected to the bridge at a time.
 * -- Sends Browser recognized controller state as JSON messages to the bridge
 */
function VirtualGamepadWindowUI() {
  "use strict";

  // Const member variables of VirtualGamepadWindowUI
  VirtualGamepadWindowUI.prototype.ID = '__win-virtual-gamepad';
  VirtualGamepadWindowUI.prototype.ID_BACKEND_CONN_BTN = '__win-virtual-gamepad-backend-conn-btn';
  VirtualGamepadWindowUI.prototype.ID_GP_MODE_BTN = '__win-virtual-gamepad-gp-mode-btn';
  VirtualGamepadWindowUI.prototype.ID_MP_MODE_BTN = '__win-virtual-gamepad-mp-mode-btn';
  VirtualGamepadWindowUI.prototype.ID_MP_MODE_BTN = '__win-virtual-gamepad-kp-mode-btn';

  VirtualGamepadWindowUI.prototype.CONNECT_LABEL =
      TABSPACE + '&nbsp;&nbsp;CONNECT TO BACKEND&nbsp;&nbsp;' + TABSPACE +
      '&nbsp;&nbsp;' + RED_DOT_GLYPHICON;
  VirtualGamepadWindowUI.prototype.DISCONNECT_LABEL =
      'DISCONNECT FROM BACKEND' +
      '&nbsp;&nbsp;' + GREEN_DOT_GLYPHICON;
  VirtualGamepadWindowUI.prototype.BTN_GP_MODE_LABEL = 'Gamepad';
  VirtualGamepadWindowUI.prototype.BTN_MP_MODE_LABEL = 'Mousepad';
  VirtualGamepadWindowUI.prototype.BTN_KP_MODE_LABEL = 'Keypad';
  VirtualGamepadWindowUI.prototype.BACKEND_CONN_DISP_STR = 'Backend Connected to Widget: ';
  VirtualGamepadWindowUI.prototype.GAMEPAD_CONN_DISP_STR =
      'Connected Standard/Remapped Gamepads: ';

  VirtualGamepadWindowUI.prototype.PERSISTENT_MSG_DISP_CLASS =
      'virtual-gamepad-persistent-message-display-div';

  // To avoid misalignment of elements DO NOT use values less that the ones below
  VirtualGamepadWindowUI.prototype.WIDTH_PX_INT = 367;
  VirtualGamepadWindowUI.prototype.HEIGHT_PX_INT = 300;
  VirtualGamepadWindowUI.prototype.WIDTH = this.WIDTH_PX_INT + 'px';
  VirtualGamepadWindowUI.prototype.HEIGHT = this.HEIGHT_PX_INT + 'px';
  VirtualGamepadWindowUI.prototype.MODEPANEL_MAINDIV_HEIGHT = (0.5 * this.HEIGHT_PX_INT) + 'px';
  VirtualGamepadWindowUI.prototype.ALIVE_MSG_INTERVAL_MS = 500;
  VirtualGamepadWindowUI.prototype.GAMEPAD_SCAN_CB_KEY = 'virtual-gamepad';

  VirtualGamepadWindowUI.prototype.GAMEPAD_MODE = 0;
  VirtualGamepadWindowUI.prototype.MOUSEPAD_MODE = 1;
  VirtualGamepadWindowUI.prototype.KEYPAD_MODE = 2;

  /** -------------INSTANCE MEMBER VARIABLES--------------- */

  /** @private @const duplicate of this to be accessed by private methods */
  const that_ = this;
  /** @private displays main widget contents. Direct child of the Widget's Div */
  let mainView_;
  /** @private bool signifying active/inactive state of this widget */
  let isEnabled_ = false;
  /** @private bool to signal if connection to backend started */
  let connectedToBackend_ = false;
  /** @private keeps track of the widget id that the backend is connected to */
  let backendWidgetConnId_ = '';
  /** @private */
  let widgetID_ = null;
  // stores last data update timestamp (DOMHighResTimeStamp: double) of last connected gamepad
  let lastGamepadUpdateTimestamp_ = -1.0;
  /** @private Stores current input mode. 0 = Gamepad, 1 = Mousepad, 2 = Keypad */
  let inputMode_ = this.GAMEPAD_MODE;
  /** @private Stores material-icons type element for Gamepad Icon for Gamepad Mode display */
  let gamepadIcon_;

  /** BUTTONS */
  /** @private */
  let btnBackendConn_;
  /** @private */
  let btnGamepadMode_;
  /** @private */
  let btnMousepadMode_;
  /** @private */
  let btnKeypadMode_;
  /** END -- BUTTONS */

  /** MESSAGE DISPLAYS */
  /** @private */
  let msgOutputDiv_;  // for intermittent popup messages
  /** @private */
  let disableOverlayDiv_;  // overlay div when widget is inactive
  // displays messages persistently to the user
  let backendConnMsgDisp_;
  // displays messages persistently to the user
  let gamepadConnMsgDisp_;
  /** END -- MESSAGE DISPLAYS */

  /** INPUT CONTROLS */
  // Virtual Mouse Joystick
  let vMouseJS_ = new VirtualMouseJoystick();
  // Virtual Keypad Joystick
  let vKeypadJS_ = new VirtualKeypadJoystick();
  /** END -- INPUT CONTROLS */

  // Input Control Mode Panel Div Elements. Allows selection/display of a single mode at a time.
  let divModesPanel_;
  let modesPanelContent_;

  /** END -- INSTANCE MEMBER VARIABLES */

  /** @private @desc creates a div for persistent Msg display */
  let createPersistentMsgDiv_ = function() {
    let disp = document.createElement('div');
    disp.className = that_.PERSISTENT_MSG_DISP_CLASS;
    disp.style.margin = INNER_DIV_MARGINS;
    return disp;
  };

  /** @private @desc creates modes panel buttons */
  let createModesPanelBtn_ = function(title, callback) {
    let btn = document.createElement('button');
    btn.className = 'tablinks';
    btn.innerHTML = title;
    btn.addEventListener('click', callback);
    return btn;
  };

  /** @private @desc deselects/disables unselected modes and enables/selects the new mode */
  let activateNewMode_ = function(mode) {
    // Reset mode state variable
    inputMode_ = mode;
    // Turn all modes off
    btnGamepadMode_.className = btnGamepadMode_.className.replace(" active", "");
    btnMousepadMode_.className = btnMousepadMode_.className.replace(" active", "");
    btnKeypadMode_.className = btnKeypadMode_.className.replace(" active", "");
    while (modesPanelContent_.firstChild) {
      modesPanelContent_.removeChild(modesPanelContent_.firstChild);
    }
    vMouseJS_.disable();
    vKeypadJS_.disable();

    // Activate the new relevant mode
    if (mode === that_.GAMEPAD_MODE) {
      btnGamepadMode_.className += ' active';
      modesPanelContent_.appendChild(gamepadIcon_);
    } else if (mode === that_.MOUSEPAD_MODE) {
      btnMousepadMode_.className += ' active';
      vMouseJS_.enable();
      modesPanelContent_.appendChild(vMouseJS_.getElementDiv());
    } else if (mode === that_.KEYPAD_MODE) {
      btnKeypadMode_.className += ' active';
      vKeypadJS_.enable();
      modesPanelContent_.appendChild(vKeypadJS_.getElementDiv());
    }
  };

  /** -------------BUTTON CALLBACKS------------------------ */

  /** @private @desc connect/disconnect with backend btn callback */
  let cbBtnBackendConn_ = function() {
    if (backendWidgetConnId_ !== '' && backendWidgetConnId_ !== widgetID_) {
      msgPopup(msgOutputDiv_, 'error', 'Backend is connected to another instance. Try later!');
    }
    if (!connectedToBackend_) {
      sendCmdToVGPBridge_('connect', widgetID_);
    } else {
      sendCmdToVGPBridge_('connect', '-1');
    }
  };

  /** @private @desc use gamepad mode btn callback */
  let cbBtnGamepadMode_ = function() {
    activateNewMode_(that_.GAMEPAD_MODE);
  };

  /** @private @desc use virtual mousepad mode btn callback */
  let cbBtnMousepadMode_ = function() {
    activateNewMode_(that_.MOUSEPAD_MODE);
  };

  /** @private @desc use virtual keypad mode btn callback */
  let cbBtnKeypadMode_ = function() {
    activateNewMode_(that_.KEYPAD_MODE);
  };

  /** END -- BUTTON CALLBACKS */

  /** @private @desc func to fill html in virtual_gamepad window */
  let generateUI_ = function() {
    if (widgetID_ === null) widgetID_ = (new Date()).getTime().toString();
    let view = document.createElement('div');
    view.id = that_.ID;
    view.className = 'virtual-gamepad';
    view.setAttribute('align', 'left');
    view.style.width = that_.WIDTH;

    disableOverlayDiv_ = document.createElement('div');
    disableOverlayDiv_.className = 'virtual-gamepad-disable-overlay';
    let disableOverlayText = document.createElement('div');
    disableOverlayText.className = 'virtual-gamepad-disable-overlay-text';
    disableOverlayText.innerHTML = 'Backend\'s VirtualGamepadBridge not Ready yet...';
    disableOverlayDiv_.appendChild(disableOverlayText);

    // div element for unique Widget ID
    let divWidgetID = document.createElement('div');
    divWidgetID.innerHTML = 'WIDGET ID: ' + widgetID_;
    divWidgetID.style.fontWeight = 'bold';
    divWidgetID.style.margin = INNER_DIV_MARGINS;

    // div for backend connection btn
    let divBtns = document.createElement('div');
    btnBackendConn_ = renderBtn(that_.ID_BACKEND_CONN_BTN, that_.CONNECT_LABEL, SECONDARY_BTN_CLASS,
        cbBtnBackendConn_);
    divBtns.appendChild(btnBackendConn_);
    divBtns.style.margin = INNER_DIV_MARGINS;

    // div for the modes panel
    divModesPanel_ = document.createElement('div');
    divModesPanel_.className = 'virtual-gamepad-modes-panel';
    let divBtnsModesPanel = document.createElement('div');
    divBtnsModesPanel.className = 'virtual-gamepad-modes-panel-tab';
    btnGamepadMode_ = createModesPanelBtn_(that_.BTN_GP_MODE_LABEL, cbBtnGamepadMode_);
    btnMousepadMode_ = createModesPanelBtn_(that_.BTN_MP_MODE_LABEL, cbBtnMousepadMode_);
    btnKeypadMode_ = createModesPanelBtn_(that_.BTN_KP_MODE_LABEL, cbBtnKeypadMode_);
    divBtnsModesPanel.appendChild(btnGamepadMode_);
    divBtnsModesPanel.appendChild(btnMousepadMode_);
    divBtnsModesPanel.appendChild(btnKeypadMode_);
    divBtnsModesPanel.style.margin = INNER_DIV_MARGINS;

    modesPanelContent_ = document.createElement('div');
    modesPanelContent_.align = 'center';
    modesPanelContent_.style.height =
        modesPanelContent_.style.minHeight =
        modesPanelContent_.style.maxHeight =
        that_.MODEPANEL_MAINDIV_HEIGHT;

    // instantiate gamepad icon
    gamepadIcon_ = document.createElement('i');
    gamepadIcon_.className = 'material-icons virtual-gamepad-gamepad-icon';
    gamepadIcon_.innerHTML = 'videogame_asset';

    activateNewMode_(inputMode_);

    divModesPanel_.appendChild(modesPanelContent_);
    divModesPanel_.appendChild(divBtnsModesPanel);
    divModesPanel_.style.margin = INNER_DIV_MARGINS;

    // div element for backend connected widget id message display
    backendConnMsgDisp_ = createPersistentMsgDiv_();
    backendConnMsgDisp_.innerHTML = that_.BACKEND_CONN_DISP_STR + 'None';

    // div element for gamepads connected message display
    gamepadConnMsgDisp_ = createPersistentMsgDiv_();
    gamepadConnMsgDisp_.innerHTML = that_.GAMEPAD_CONN_DISP_STR + '0';

    // div element for Message popup
    msgOutputDiv_ = document.createElement('div');
    msgOutputDiv_.id = that_.ID_OUTPUT_BOX;
    msgOutputDiv_.style.margin = '30px 15px 0px';
    msgOutputDiv_.setAttribute('align', 'center');

    mainView_ = document.createElement('div');
    mainView_.className = 'virtual-gamepad-main';
    mainView_.appendChild(disableOverlayDiv_);
    mainView_.appendChild(divWidgetID);
    mainView_.appendChild(divBtns);
    mainView_.appendChild(divModesPanel_);
    mainView_.appendChild(backendConnMsgDisp_);
    mainView_.appendChild(gamepadConnMsgDisp_);
    mainView_.appendChild(msgOutputDiv_);

    view.appendChild(mainView_);

    WindowManager().createWindow(view, 'Virtual Gamepad', {
      width: that_.WIDTH,
      height: that_.HEIGHT,
      hide: true,
    });

    vMouseJS_.showInfoDisplay(false);
    vKeypadJS_.showInfoDisplay(false);
    disable_();  // initially keep disabled
  };

  /** @private @desc makes the widget inactive */
  let disable_ = function() {
    disableOverlayDiv_.style.display = 'inline-block';
  };

  /** @private @desc makes the widget inactive */
  let enable_ = function() {
    disableOverlayDiv_.style.display = 'none';
  };

  /** -------------COMMUNICATION FUNCTIONS------------------------ */

  /**
   * @private @desc Creates JSON for Websight Server containing cmd with
   *   params and sends it over websocket
   * @param {string} cmdStr: cmd type
   * @param {string} paramStr: params for the cmd
   */
  let sendCmdToVGPBridge_ = function(cmdStr, paramStr = null) {
    let dataJson = {
      'cmd': cmdStr,
      'cmd-params': paramStr
    }
    sendToBridge(my_socket, 'virtual_gamepad', dataJson);
  };

  /** @desc called on my_websocket.onmessage if msg was for VirtualGamepadWindowUI */
  this.onWebsocketMessage = function(msg) {
    if (msg['cmd'] === 'connect') {
      onConnectReply_(msg['cmd-data']['value']);
    }
  };

  /** @private @desc call when backend replies to connect signal */
  let onConnectReply_ = function(val) {
    // Receiving this message signifies that the backend is ready.
    // Remove the disable overlay and make the widget active.
    if (isEnabled_ === false) {
      isEnabled_ = true;
      enable_();
    }
    divModesPanel_.className = divModesPanel_.className.replace(" active", "");
    if (val != widgetID_) {
      // (No) other widget connected
      if (val != '-1') {
        backendWidgetConnId_ = val;
      } else {
        backendWidgetConnId_ = '';
      }
      connectedToBackend_ = false;
      btnBackendConn_ = renderBtn(that_.ID_BACKEND_CONN_BTN, that_.CONNECT_LABEL,
          SECONDARY_BTN_CLASS);
      backendConnMsgDisp_.innerHTML = that_.BACKEND_CONN_DISP_STR + ((val == '-1') ? 'None' : val);
    } else {
      // this widget connected
      connectedToBackend_ = true;
      btnBackendConn_ = renderBtn(that_.ID_BACKEND_CONN_BTN, that_.DISCONNECT_LABEL,
          SECONDARY_BTN_CLASS);
      backendConnMsgDisp_.innerHTML = that_.BACKEND_CONN_DISP_STR + val;
      divModesPanel_.className += ' active';
    }
  };

  /** END -- COMMUNICATION FUNCTIONS */

  /** -------------INPUT CONTROLS CALLBACKS------------------------ */

  /** @private @desc callback to be executed every GamepadManager Scan cycle */
  let onGamepadManagerScanCB_ = function() {
    // Regularly update number of gamepads on UI by setting as a Callback with GamepadManager
    gamepadConnMsgDisp_.innerHTML = that_.GAMEPAD_CONN_DISP_STR +
        GamepadManagerInstance().numGamepads();

    // only send state if single gamepad connected. Also reset Gamepad Icon color.
    gamepadIcon_.className = gamepadIcon_.className.replace(" active", "");
    if (!(GamepadManagerInstance().numGamepads() === 1)) {
      return;
    } else {
      gamepadIcon_.className += ' active';
    }

    // Regularly send gamepad's state to the backend
    // START

    // only send if widget connected to backend and gamepad mode is on
    if (!connectedToBackend_ || inputMode_ !== that_.GAMEPAD_MODE) return;
    // Check if the GP is still connected
    let gp = GamepadManagerInstance().getGamepadByIndex(
        GamepadManagerInstance().getGamepadIndices()[0]);
    if (!gp.connected) {  // double check connected state to be sure
      msgPopup(msgOutputDiv_, 'error', 'Gamepad "' + gp.index + '" seems disconnected.');
      return;
    }
    // Check if last timestamp of data update for the gamepad exists and has changed
    let timestamp = gp.timestamp;  // temp storage before processing
    if (timestamp) {
      if (timestamp <= lastGamepadUpdateTimestamp_) return;
      lastGamepadUpdateTimestamp_ = timestamp;
    }

    sendCmdToVGPBridge_('gp-state', JSON.stringify(gamepadStateToJson(gp)));
    // END
  };

  /** @private @desc callback called when virtual mousepad executes its state scan cycles */
  let onMousepadScanCB_ = function() {
    // On virtual mousepad scan iteration, send its simulated Joystick state to the bridge
    // only send if widget connected to backend and mousepad mode in ON
    if (!connectedToBackend_ || inputMode_ !== that_.MOUSEPAD_MODE) return;
    sendCmdToVGPBridge_('vmp-state', JSON.stringify(vMouseJS_.getSimulatedJoystickStateJson()));
  };

  /** @private @desc callback called when virtual keypad executes its state scan cycles */
  let onKeypadScanCB_ = function() {
    // On virtual keypad scan iteration, send its simulated Joystick state to the bridge
    // only send if widget connected to backend and the keypad mode is ON
    if (!connectedToBackend_ || inputMode_ !== that_.KEYPAD_MODE) return;
    sendCmdToVGPBridge_('vkp-state', JSON.stringify(vKeypadJS_.getSimulatedJoystickStateJson()));
  };

  /** END -- INPUT CONTROLS CALLBACKS */


  /** -------------Constructor functionality--------------- */
  // START

  // send regular alive signals to backend
  setInterval(function() {
    if (connectedToBackend_ === true) {
      sendCmdToVGPBridge_('alive', widgetID_);
    }
  }, that_.ALIVE_MSG_INTERVAL_MS);

  generateUI_();
  // add gamepad manager scan cycle callback
  GamepadManagerInstance().addOnScanGamepadsCB(this.GAMEPAD_SCAN_CB_KEY, onGamepadManagerScanCB_);
  // add virtual mousepad scan cycle callback
  vMouseJS_.addOnMousepadScanCB(onMousepadScanCB_);
  // add virtual keypad scan cycle callback
  vKeypadJS_.addOnKeypadpadScanCB(onKeypadScanCB_);
  // END
}

// Create global singleton instance of VirtualGamepadWindowUI
let virtual_gamepad_window_ = null;
//  Create a singleton instance
/* Implement a "singleton" */
function VirtualGamepadWindow() {
  if (virtual_gamepad_window_ === null) {
    virtual_gamepad_window_ = new VirtualGamepadWindowUI();
  }
  return virtual_gamepad_window_;
}

VirtualGamepadWindow();
