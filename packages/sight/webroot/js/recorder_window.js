/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * Creates Record Window UI on WebSight front-end.
 * Is a Singleton Class
 * @constructor
 * Uses Websocket to communicate with server(refer to index.html:my_socket)
 */
function RecordWindowUI() {

  // Const member variables of RecordWindowUI
  RecordWindowUI.prototype.WINDOW_ID             = '__win-record-window';
  RecordWindowUI.prototype.LOGPATH_CONTAINER_ID  = '__win-record-window-logpath-container-box';
  RecordWindowUI.prototype.LOGPATH_NAME_ID       = '__win-record-window-logpath-name-box';
  RecordWindowUI.prototype.BUTTON_ID             = '__win-record-window-button';
  RecordWindowUI.prototype.PREV_REC_BTN_ID       = '__win-record-window-prev-record-btn';
  RecordWindowUI.prototype.INFO_TEXT_ID          = '__win-record-window-info-text';
  RecordWindowUI.prototype.APP_UUID_ID           = '__win-record-window-UUID';
  RecordWindowUI.prototype.CHRONO_ID             = '__win-record-window-chrono';
  RecordWindowUI.prototype.CONTAINER_ID          = '__win-record-window-container';
  RecordWindowUI.prototype.PREV_REC_INFO_ID      = '__win-record-window-prev-record';
  RecordWindowUI.prototype.PREV_REC_PATH         = '__win-record-window-prev-record-path';
  RecordWindowUI.prototype.PREV_REC_PATH_TEXT    = '__win-record-window-prev-record-path-text';
  RecordWindowUI.prototype.PREV_REC_PATH_LABEL   = '__win-record-window-prev-record-path-label';
  RecordWindowUI.prototype.PREV_REC_LENGTH       = '__win-record-window-prev-record-length';
  RecordWindowUI.prototype.WIDTH = '480px';
  RecordWindowUI.prototype.HEIGHT = '300px';
  RecordWindowUI.prototype.STOP_BTN_NAME = '&nbsp&nbsp&nbspSTOP&nbsp&nbsp&nbsp';
  RecordWindowUI.prototype.RECORD_BTN_NAME = 'RECORD';


  /** @private duplicate of this to be accessed by private methods */
  const that_ = this;

  /** @private state of the window */
  let recording_ = false;
  /** @private Initial time at window creation */
  let startTime_ = new Date();
  /** @private timestamp at start record */
  let startRecord_ = new Date(0);
  /** @private timestamp at stop record */
  let stopRecord_ = new Date(0);

  let folderInput_;
  let nameInput_;
  let appID_;
  let chrono_;
  let button_;
  let divPopUp_;
  let prevRecInfo_;
  let socket_;
  let menuHide_;

  /**
   * @private @desc Get the seconds in floating point until two decimals of a date object
   * @param (Date} time
   */
  let getFracScnd_ = function(d) {
    return Math.floor(d / 10) / 100;
  };

  /** @private @desc Update the window UI */
  let updateRecordWindowUI_ = function() {
    let logFolderField = document.getElementById(that_.LOGPATH_CONTAINER_ID);
    let logNameField = document.getElementById(that_.LOGPATH_NAME_ID);

    if (recording_) {
      logFolderField.setAttribute('readOnly', 'true');
      logNameField.setAttribute('readOnly', 'true');

      renderBtn(that_.BUTTON_ID, that_.STOP_BTN_NAME, 'btn btn-success');
    } else {
      logFolderField.removeAttribute('readOnly');
      logNameField.removeAttribute('readOnly');

      renderBtn(that_.BUTTON_ID, that_.RECORD_BTN_NAME, 'btn btn-danger');
    }

    let btn = document.getElementById(that_.PREV_REC_BTN_ID);
    if (menuHide_) {
      document.getElementById(that_.PREV_REC_INFO_ID).style.display = 'none';
      document.getElementById(that_.PREV_REC_INFO_ID).style.visibility = 'hidden';
      btn.innerHTML = '+';
    } else {
      document.getElementById(that_.PREV_REC_INFO_ID).style.display = 'block';
      document.getElementById(that_.PREV_REC_INFO_ID).style.visibility = 'visible';
      btn.innerHTML = '-';
    }
  };

  /** @private @desc btn callback */
  let buttonPress_ = function() {
    if (recording_) {
      // Comunicate with backend
      sendCmdToWebServer_(false);
    } else {
      // Comunicate with backend
      sendCmdToWebServer_(true);
    }
  };

  /** @private @desc hide and show the previous record info */
  let toogleInfoArea_ = function() {
    menuHide_ = !menuHide_;
    updateRecordWindowUI_();
  };

  /** @private @desc func to fill html in record window */
  let updateChrono_ = function (miliseconds) {
    if (recording_) {
      let integerSeconds = Math.trunc(miliseconds / 1000);
      chrono_.innerHTML = hmsFmtToString(secondsToHmsFmt(integerSeconds));
    }
  };

  /** @private @desc update the window with the application UUID
    * @param {string} uuid: The ID for this application
    */
  let setUUID_ = function (uuid) {
    appID_.innerHTML = '<b>Application UUID:</b> ' + uuid;
  };

  /**
   * @private @desc Creates JSON for Websight Server containing cmd with
   *   params and sends it over websocket
   * @param {boolean} recording: If we request to start or stop recording
   */
  let sendCmdToWebServer_ = function(recording) {
    if (socket_ && socket_.readyState == 1) {
      let pathField = document.getElementById(that_.LOGPATH_CONTAINER_ID);
      let nameField = document.getElementById(that_.LOGPATH_NAME_ID);
      let state = recording ? 'true' : 'false';
      let msgData = {
          cmd: 'recording',
          cmdparams: state,
          base_path: pathField.value,
          tag: nameField.value
      }
      let json = {
          type: 'record',
          data: msgData
      }
      socket_.send(JSON.stringify(json));
    } else {
      console.error('Socket is not open!');
      msgPopup(divPopUp_, 'error', 'Cannot communicate with server');
    }
  };

  /** @desc Update the widget to showl the last recorded */
  let updatePrevRecord_ = function(path, length, started) {
    let pathField = document.getElementById(that_.PREV_REC_PATH_TEXT);
    let lengthField = document.getElementById(that_.PREV_REC_LENGTH);
    pathField.value = path;
    lengthField.innerHTML = '<b>Length:</b> ' + length + 's';
  };

  /** @desc called on my_websocket.onmessage if msg was for RecordWindowUI */
  this.onWebsocketMessage = function(msg) {
    // Check if this message is valid
    if (msg.cmd == 'recording') {
      if (recording_) { // We continue recording
        updateChrono_(msg['current_time']);
      } else { //We need to start a recording
        recording_ = true;
        startRecord_ = new Date(msg['start_time']);
        setUUID_(msg['app_uuid']);
        stopRecord_ = new Date(0); // Invalidate
        msgPopup(divPopUp_, 'info', 'Recording started at: '
            + getFracScnd_(startRecord_) + '\n');
        let pathField = document.getElementById(that_.LOGPATH_CONTAINER_ID);
        let nameField = document.getElementById(that_.LOGPATH_NAME_ID);
        pathField.value = msg['base_directory'];
        nameField.value = msg['tag'];
        updateRecordWindowUI_();
      }
      setUUID_(msg['app_uuid']);
    } else if (msg.cmd == 'stopped') {
      if (recording_) { // We need to stop
        recording_ = false;
        stopRecord_ = new Date();
        chrono_.innerHTML = ZERO_TIME_STRING;
        msgPopup(divPopUp_, 'info', 'Recording stoped at '
            + getFracScnd_(stopRecord_ - startTime_) + '<br />'
            + 'Length: ' + getFracScnd_(msg['current_time']) + ' seconds<br />'
            + 'Log Folder:<br />' + msg['absolute_path'] + '<br />saved');
        updatePrevRecord_(msg['absolute_path'], getFracScnd_(msg['current_time']), msg['start_time']);
        updateRecordWindowUI_();
      }
      setUUID_(msg['app_uuid']);
    } else {  // Invalid message!
      msgPopup(divPopUp_, 'error', 'Invalid message received from server');
      console.log('Invalid message received from server');
    }
    // If we have an error message
    if (msg['error_state']) {
      msgPopup(divPopUp_, 'error', msg['error_msg']);
      console.log(msg['error_msg']);
    }
  };

  /** @desc Sets web socket to be used for communication with server
  *   @param (WebSocket} socket
  */
  this.setWebSocket = function(socket) {
    socket_ = socket;
  };

  /** @desc Resets websocket when websocket is closed */
  this.resetWebSocket = function() {
    socket_ = null;
  };

  /** @desc func to fill html and span the record window */
  this.createUI = function() {
    let view = document.createElement('div');
    view.id = that_.WINDOW_ID;
    view.className = 'record-widget';
    view.setAttribute('align', 'left');
    view.style.width = that_.WIDTH;
    // div element for input container folder
    let folderLabel = createLabel('Container folder:', that_.LOGPATH_CONTAINER_ID);
    folderLabel.setAttribute('align', 'left');
    folderInput_ = document.createElement('textarea');
    folderInput_.id = that_.LOGPATH_CONTAINER_ID;
    folderInput_.className = 'input-text';
    folderInput_.setAttribute('rows', '1');
    folderInput_.setAttribute('cols', '25');
    folderInput_.setAttribute('wrap', 'off');
    folderInput_.innerHTML = '/tmp/isaac';
    let divInputFolder = document.createElement('div');
    divInputFolder.className = 'input-text-container';
    divInputFolder.appendChild(folderLabel);
    divInputFolder.appendChild(folderInput_);
    divInputFolder.style.margin = INNER_DIV_MARGINS;
    // div element for input the tag field
    let nameLabel = createLabel('Tag:', that_.LOGPATH_NAME_ID);
    nameLabel.align = 'left';
    nameInput_ = document.createElement('textarea');
    nameInput_.id = that_.LOGPATH_NAME_ID;
    nameInput_.className = 'input-text';
    nameInput_.setAttribute('rows', '1');
    nameInput_.setAttribute('cols', '25');
    nameInput_.setAttribute('wrap', 'off');
    nameInput_.innerHTML = 'test';
    let divInputName = document.createElement('div');
    divInputName.className = 'input-text-container';
    divInputName.appendChild(nameLabel);
    divInputName.appendChild(nameInput_);
    divInputName.style.margin = INNER_DIV_MARGINS;
    // div element for the title label
    appID_ = document.createElement('div');
    appID_.id = that_.APP_UUID_ID;
    appID_.style.margin = INNER_DIV_MARGINS;
    // div element for the chronometer
    chrono_ = document.createElement('div');
    chrono_.id = that_.CHRONO_ID;
    chrono_.className = 'chronometer-label';
    chrono_.innerHTML = ZERO_TIME_STRING;
    // Button element
    button_ = renderBtn(that_.BUTTON_ID, that_.RECORD_BTN_NAME, 'btn btn-danger', buttonPress_);
    // Button for prev record
    let buttonInfo = renderBtn(that_.PREV_REC_BTN_ID, '+', 'btn btn-success', toogleInfoArea_);
    // Container div for the chrono and button
    let container = document.createElement('div');
    container.id = that_.CONTAINER_ID;
    container.appendChild(button_);
    container.appendChild(chrono_);
    container.style.margin = INNER_DIV_MARGINS;
    container.appendChild(buttonInfo);
    // div element for pop up messages
    divPopUp_ = document.createElement('div');
    divPopUp_.id = that_.INFO_TEXT_ID;
    // div element for the previus record info
    prevRecInfo_ = document.createElement('div');
    prevRecInfo_.id = that_.PREV_REC_INFO_ID;
    let pathBox = document.createElement('textarea');
    pathBox.id = that_.PREV_REC_PATH_TEXT;
    pathBox.setAttribute('rows', '1');
    pathBox.setAttribute('cols', '50');
    pathBox.setAttribute('readOnly', 'true');
    let pathBoxLabel = createLabel('Path:', that_.PREV_REC_PATH_TEXT);
    pathBoxLabel.id = that_.PREV_REC_PATH_LABEL;
    let pathDiv = document.createElement('div');
    pathDiv.id = that_.PREV_REC_PATH;
    pathDiv.appendChild(pathBoxLabel);
    pathDiv.appendChild(pathBox);
    let lengthBox = document.createElement('div');
    lengthBox.id = that_.PREV_REC_LENGTH;
    prevRecInfo_.appendChild(pathDiv);
    prevRecInfo_.appendChild(pathBox);
    prevRecInfo_.appendChild(lengthBox);
    prevRecInfo_.style.margin = INNER_DIV_MARGINS;
    menuHide_ = true;
    //Window layout creation
    view.appendChild(appID_);
    view.appendChild(divInputFolder);
    view.appendChild(divInputName);
    view.appendChild(container);
    view.appendChild(prevRecInfo_);
    view.appendChild(divPopUp_);

    WindowManager().createWindow(view, 'Record Control Panel', {
      width: that_.WIDTH,
      height: that_.HEIGHT,
      hide: true,
    });
  };
}

let recorder_window_ = null;
//  Create a singleton instance of map conatiner
/* Implement a "singleton" of MapContainer */
function RecorderWindow() {
  if (recorder_window_ === null) {
    recorder_window_ = new RecordWindowUI();
  }
  return recorder_window_;
}

RecorderWindow().createUI();