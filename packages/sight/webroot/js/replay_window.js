/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/**
 * Constructs a time slider instance
 * inherits from bootstrap-slider
 * @constructor
 * @param {string} id: document id
 */
function ReplayTimeSlider(id) {
  /** @private @const duplicate of this to be accessed by private methods */
  const that_ = this;

  // @private html elements of slider div to be returned from this class
  this.div_ = document.createElement('div');
  this.startLabelElem_ = createLabel(0, null);
  this.endLabelElem_ = createLabel(0, null);
  let sliderElem_ = document.createElement('input');

  sliderElem_.id = id;
  sliderElem_.setAttribute('type', 'text');
  sliderElem_.setAttribute('class', 'span2');

  this.div_.appendChild(this.startLabelElem_);
  this.div_.appendChild(sliderElem_);
  this.div_.appendChild(this.endLabelElem_);

  // Allow adjacent HTML Elements around this div
  this.div_.setAttribute('style', 'display: inline');

  // Call to super class
  Slider.call(this, sliderElem_, {
    id: id,
    value: 0,
    enabled: false,
  });

  // Intialize to start:0; end:0
  this.updateTimeRange(0, 0);

  return this;
}

// Inherit from bootstrap-slider
ReplayTimeSlider.prototype = Object.create(Slider.prototype);
ReplayTimeSlider.prototype.constructor = ReplayTimeSlider;

/**
 * @desc returns HTML Element of ReplayTimeSlider instance
 */
ReplayTimeSlider.prototype.getDivElem = function() {
  return this.div_;
}

/**
 * @desc sets new time range and refreshes on UI
 * @param {integer} start: start time in nanoseconds
 * @param {integer} end: end time in nanoseconds
 */
ReplayTimeSlider.prototype.updateTimeRange = function(start, end) {
  this.setAttribute('min', start);
  this.setAttribute('max', end);
  this.startLabelElem_.innerHTML = TABSPACE + start + TABSPACE;
  this.endLabelElem_.innerHTML = TABSPACE + end + TABSPACE;
}

/**
 * Creates Replay Window UI on WebSight front-end.
 * Is a Singleton Class
 * @constructor
 * Uses Websocket to communicate with server(refer to index.html:my_socket)
 */
function ReplayWindowUI() {
  "use strict";

  // Const member variables of ReplayWindowUI
  ReplayWindowUI.prototype.ID = '__win-replay-window';
  ReplayWindowUI.prototype.ID_LOGPATH_BOX = '__win-replay-window-lopath-box';
  ReplayWindowUI.prototype.ID_START_STOP_BTN = '__win-replay-window-start-stop-btn';
  ReplayWindowUI.prototype.ID_LOAD_BTN = '__win-replay-window-load-btn';
  ReplayWindowUI.prototype.ID_SETTIME_BTN = '__win-replay-window-settime-btn';
  ReplayWindowUI.prototype.ID_SETTIME_BOX = '__win-replay-window-settime-box';
  ReplayWindowUI.prototype.ID_OUTPUT_BOX = '__win-replay-window-output-box';
  ReplayWindowUI.prototype.ID_TIME_SLIDER = '__win-replay-window-time-slider';
  ReplayWindowUI.prototype.START_BTN_NAME = 'START REPLAY';
  ReplayWindowUI.prototype.STOP_BTN_NAME = 'STOP REPLAY&nbsp;&nbsp;';
  ReplayWindowUI.prototype.WIDTH = '700px';
  ReplayWindowUI.prototype.HEIGHT = '225px';

  /** @private @const duplicate of this to be accessed by private methods */
  const that_ = this;
  /** @private bool to signal if input file loaded */
  let fileLoaded_ = false;
  /** @private bool to signal if replay started */
  let replayStarted_ = false;
  /** @private @const time slider ui */
  const slider_ = new ReplayTimeSlider(that_.ID_TIME_SLIDER);
  let boxInputLog_;
  let boxSetTime_;
  let btnLoad_;
  let btnSetTime_;
  let btnStartStop_;
  let msgOutputDiv_;
  let lastLogPathFromBackend_ = '';
  let lastLogPathForBackend_ = '';

  /** @private @desc update UI to not have any log loaded */
  let reset = function() {
    fileLoaded_ = false;
    lastLogPathFromBackend_ = '';
    renderBtn(that_.ID_START_STOP_BTN, that_.START_BTN_NAME,
        GREEN_BTN_CLASS).setAttribute('disabled', 'disabled');
    btnSetTime_.setAttribute('disabled', 'disabled');
    boxSetTime_.value = 0;
    boxSetTime_.setAttribute('readonly', 'true');
    slider_.updateTimeRange(0, 0);
    slider_.setValue(0);
    slider_.disable();
  };

  /** @private @desc update UI if load fails */
  let onLoadFailed_ = function() {
    reset();
    msgPopup(msgOutputDiv_, 'error', 'Load failed!');
  };

  /**
   * @private @desc updates UI when Load succeeds
   * @param {string} path: log path sent from ReplayBridge
   */
  let onLoadSuccessful_ = function(path) {
    fileLoaded_ = true;
    replayStarted_ = false;
    // if Load pressed while replay going on, then render Start Btn
    // Also, since load was successful, start btn gets enabled
    renderBtn(that_.ID_START_STOP_BTN, that_.START_BTN_NAME,
        GREEN_BTN_CLASS).removeAttribute('disabled');
    boxInputLog_.value = path;
    btnSetTime_.removeAttribute('disabled');
    boxSetTime_.value = 0;
    boxSetTime_.removeAttribute('readonly');
    slider_.setValue(0);
    slider_.enable();
    msgPopup(msgOutputDiv_, 'info', 'Load Successful.');
  };

  /** @private @desc load btn callback */
  let onLoadBtnCB_ = function() {
    if (replayStarted_) {
      msgPopup(msgOutputDiv_, 'error', 'Load not allowed while replaying!');
      return;
    }

    let logfilePathStr = boxInputLog_.value;

    if (logfilePathStr === '') {
      lastLogPathFromBackend_ = '';
      onLoadFailed_();
      msgPopup(msgOutputDiv_, 'error', 'Enter folder with recorded log first!');
      return;
    }

    lastLogPathForBackend_ = logfilePathStr;
    sendCmdToReplayBridge_('load', logfilePathStr);
  };

  /**
   * @private @desc called when backend sends response to log cmd
   * @param {string} path: log path sent from ReplayBridge
   */
  let onLoadReply_ = function(path) {
    if (lastLogPathFromBackend_ !== path) {  // change in loaded log path from backend
      lastLogPathFromBackend_ = path;
      if (path === '') {  // empty string from backend means log is not loaded
        onLoadFailed_();
      } else {  // last load succeeded in backend
        onLoadSuccessful_(path);
      }
      return;
    }
    // last load request failed
    if (lastLogPathFromBackend_ !== lastLogPathForBackend_ && lastLogPathForBackend_ !== '') {
      lastLogPathForBackend_ = '';
      onLoadFailed_();
    }
  };

  /**
   * @private @desc called when backend sends log time range udpate
   * @param {integer} startTime: start time in log
   * @param {integer} endTime: end time in log
   */
  let onRangeUpdateReply_ = function(startTime, endTime) {
    if (endTime < 0) {
      slider_.updateTimeRange(0, 0);
    } else {
      slider_.updateTimeRange(startTime, endTime);
      // If thumb position/value is before range min then reset it to range min
      if (slider_.getValue() < startTime) {
        boxSetTime_.value = startTime;
        slider_.setValue(startTime);
      }
    }
  };

  /**
   * @private @desc Creates JSON for Websight Server containing cmd with
   *   params and sends it over websocket
   * @param {string} cmdStr: cmd type
   * @param {string} paramStr: params for the cmd
   */
  let sendCmdToReplayBridge_ = function(cmdStr, paramStr = null) {
    let dataJson = {
      'cmd': cmdStr,
      'cmd-params': paramStr
    }
    sendToBridge(my_socket, 'replay', dataJson);
  };

  /** @private @desc start/stop btn callback */
  let onStartStopBtnCB_ = function() {
    if (!fileLoaded_) {
      msgPopup(msgOutputDiv_, 'error', 'Load should complete before replay can Start/Stop!');
      return;
    }

    if (!replayStarted_) {
      sendCmdToReplayBridge_('start', Math.floor(slider_.getValue()));
    } else {
      sendCmdToReplayBridge_('stop');
    }
  };

  /**
   * @private @desc called when backend sends replay time update
   * @param {num} newTime: new current time in log in nanoseconds
   */
  let onTimeUpdateReply_ = function(newTime) {
    if (newTime < 0) {  // backend not replaying
      if (!replayStarted_) return;
      replayStarted_ = false;

      renderBtn(that_.ID_START_STOP_BTN, that_.START_BTN_NAME, GREEN_BTN_CLASS);
      slider_.enable();
      boxSetTime_.removeAttribute('readonly');
      boxInputLog_.removeAttribute('readonly');
      btnSetTime_.removeAttribute('disabled');
      btnLoad_.removeAttribute('disabled');
      msgPopup(msgOutputDiv_, 'info', 'Replay Stopped');
    } else {
      let sliderMaxValue = slider_.getAttribute('max');
      if (newTime > sliderMaxValue) newTime = sliderMaxValue;
      slider_.setValue(newTime);
      // TODO(akshaya): Add as callback to slider's new func setNewValue
      boxSetTime_.value = newTime;

      if (replayStarted_) return;
      replayStarted_ = true;

      boxSetTime_.value = newTime;
      renderBtn(that_.ID_START_STOP_BTN, that_.STOP_BTN_NAME, RED_BTN_CLASS);
      slider_.disable();
      boxSetTime_.setAttribute('readonly','true');
      boxInputLog_.setAttribute('readonly','true');
      btnSetTime_.setAttribute('disabled', 'disabled');
      btnLoad_.setAttribute('disabled', 'disabled');
      msgPopup(msgOutputDiv_, 'info', 'Replay Started at ' + newTime);
    }
  };

  /** @private @desc set start time btn callback */
  let onSetTimeBtnCB_ = function() {
    if (!fileLoaded_) {
      msgPopup(msgOutputDiv_, 'error', 'Can not SET new start time'
          + ' before Load completes!');
      return;
    }
    if (replayStarted_) {
      msgPopup(msgOutputDiv_, 'error', 'Can not SET new start time'
          + ' while replaying!');
      return;
    }

    if (!Number.isSafeInteger(parseInt(boxSetTime_.value)) || parseInt(boxSetTime_.value) < 0) {
      msgPopup(msgOutputDiv_, 'error', 'Time value must be a non-negative integer');
      return;
    }

    let time = boxSetTime_.value;
    if (time < slider_.getAttribute('min') || time > slider_.getAttribute('max')) {
      msgPopup(msgOutputDiv_, 'error',
          'New Start Time should be between log\'s Start and End Times');
      return;
    }

    slider_.setValue(time);
  };

  /**
   * @private @desc slider on change callback
   * @param {{oldSliderValue: {num}, newSliderValue: {num} param1
   */
  let onSliderChangeCB_ = function(sliderValues) {
    boxSetTime_.value = sliderValues['newValue'];
  };

  /** @desc func to fill html in replay window */
  this.generateUI = function() {
    let view = document.createElement('div');
    view.id = that_.ID;
    view.setAttribute('align', 'left');
    // Prevent content wrapping on window resizing
    view.setAttribute('style', 'width: ' + that_.WIDTH + '; overflow: auto;' +
        ' margin: 0px auto;');

    // div element for input log file
    let divInput = document.createElement('div');
    let inputLabel = createLabel('Input Folder Path:' + TABSPACE + TABSPACE + TABSPACE + TABSPACE +
        TABSPACE, that_.ID_LOGPATH_BOX);
    inputLabel.align = 'left';
    boxInputLog_ = document.createElement('textarea');
    boxInputLog_.id = that_.ID_LOGPATH_BOX;
    boxInputLog_.setAttribute('rows', '1');
    boxInputLog_.setAttribute('cols', '25');
    // make textarea single line horizontal scrollable
    boxInputLog_.setAttribute('wrap', 'off');
    boxInputLog_.setAttribute('style', 'resize: none; vertical-align: middle;');
    boxInputLog_.style.backgroundColor = '#f7f7f7';
    boxInputLog_.style.fontSize = '14px';
    btnLoad_ = renderBtn(that_.ID_LOAD_BTN, 'Load Log', SECONDARY_BTN_CLASS, onLoadBtnCB_);
    btnLoad_.style.color = SUCCESS_CLR;
    divInput.appendChild(inputLabel);
    divInput.appendChild(boxInputLog_);
    divInput.appendChild(btnLoad_);
    divInput.style.margin = INNER_DIV_MARGINS;

    // div for setting a start time before start replay
    let divCurTime = document.createElement('div');
    let setTimeLabel = createLabel('Current Time (nanoseconds):', that_.ID_SETTIME_BOX);
    setTimeLabel.align = 'left';
    boxSetTime_ = document.createElement('textarea');
    boxSetTime_.id = that_.ID_SETTIME_BOX;
    boxSetTime_.setAttribute('rows', '1');
    boxSetTime_.setAttribute('cols', '25');
    boxSetTime_.setAttribute('readonly', 'true');
    boxSetTime_.setAttribute('style', 'resize: none; vertical-align: middle;');
    boxSetTime_.style.backgroundColor = '#f7f7f7';
    boxSetTime_.style.fontSize = '14px';
    boxSetTime_.setAttribute('title', 'SET this as log\'s new start time');
    boxSetTime_.value = 0;
    btnSetTime_ = renderBtn(that_.ID_SETTIME_BTN, 'Set', SECONDARY_BTN_CLASS, onSetTimeBtnCB_);
    btnSetTime_.style.color = SUCCESS_CLR;
    btnSetTime_.setAttribute("disabled", "disabled");
    divCurTime.appendChild(setTimeLabel);
    divCurTime.appendChild(boxSetTime_);
    divCurTime.appendChild(btnSetTime_);
    divCurTime.style.margin = INNER_DIV_MARGINS;

    // div element for timeline slider
    let divSlider = document.createElement('div');
    btnStartStop_ = renderBtn(that_.ID_START_STOP_BTN, that_.START_BTN_NAME,
        GREEN_BTN_CLASS, onStartStopBtnCB_);
    btnStartStop_.setAttribute("disabled", "disabled");
    slider_.on('change', onSliderChangeCB_);
    divSlider.appendChild(slider_.getDivElem());
    divSlider.appendChild(btnStartStop_);
    divSlider.style.margin = INNER_DIV_MARGINS;

    // div element for Message popup
    msgOutputDiv_ = document.createElement('div');
    msgOutputDiv_.id = that_.ID_OUTPUT_BOX;
    msgOutputDiv_.style.margin = '30px 15px 0px';
    msgOutputDiv_.setAttribute('align', 'center');

    view.appendChild(divInput);
    view.appendChild(divCurTime);
    view.appendChild(divSlider);
    view.appendChild(msgOutputDiv_);

    WindowManager().createWindow(view, 'Replay Control Panel', {
      width: that_.WIDTH,
      height: that_.HEIGHT,
      hide: true,
    });
  };

  /** @desc called on my_websocket.onmessage if msg was for ReplayWindowUI */
  this.onWebsocketMessage = function(msg) {
    if (msg['cmd'] === 'load') {
      onLoadReply_(msg['cmd-data']['log-path']);
    } else if (msg['cmd'] === 'range-update') {
      onRangeUpdateReply_(msg['cmd-data']['start-time'], msg['cmd-data']['end-time']);
    } else if (msg['cmd'] === 'time-update') {
      onTimeUpdateReply_(msg['cmd-data']['time']);
    }
  };

  /** @desc called on my_websocket.onclose */
  this.onDisconnect = function() {
    reset();
  };
}

// Create global singleton instance of ReplayWindowUI
let replay_window_ = null;
//  Create a singleton instance of map conatiner
/* Implement a "singleton" of MapContainer */
function ReplayWindow() {
  if (replay_window_ === null) {
    replay_window_ = new ReplayWindowUI();
  }
  return replay_window_;
}

ReplayWindow().generateUI();
