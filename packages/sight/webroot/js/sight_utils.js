/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

const ZERO_TIME_STRING = '00:00:00';
const TABSPACE = '&nbsp;&nbsp;&nbsp;&nbsp;';
const INNER_DIV_MARGINS = '10px 15px 0px';
const GREEN_BTN_CLASS = 'btn btn-sharp-isaac btn-success btn-success-isaac';
const RED_BTN_CLASS = 'btn btn-sharp-isaac btn-danger btn-danger-isaac';
const SECONDARY_BTN_CLASS = 'btn btn-outline-isaac';
const DANGER_CLR = '#f44336';
const SUCCESS_CLR = '#76b900';
const RED_DOT_GLYPHICON =
    '<span class="glyphicon glyphicon-record" style="color:' + DANGER_CLR + '"></span>';
const GREEN_DOT_GLYPHICON =
    '<span class="glyphicon glyphicon-record" style="color:' + SUCCESS_CLR + '"></span>';
const ISAAC_RND_BTN_CLASS = 'btn btn-default btn-round-isaac';

/**
 * @desc convert seconds to {h,m,s}
 * @param (integer} seconds: time in seconds
 * @param {bool} secondsPrecision: loose decimal precision for seconds in output
 * @return array containing hr, mm, seconds
 */
function secondsToHmsFmt(seconds, secondsPrecision = false) {
  let hr = Math.floor(seconds/3600);
  let min = Math.floor((seconds - (hr * 3600))/60);
  let sec = seconds - ((hr * 3600) + (min * 60));

  if (secondsPrecision) sec = Math.round(sec);

  return {'h': hr, 'm': min, 's': sec};
}

/**
 * @desc convert hms fmt to seconds
 * @param (string || Array {h, m, s}} hms: time in hms fmt
 * @return seconds if successful else null
 */
function hmsFmtToSeconds(hms) {
  if (typeof hms === 'string') {
    const timeRe = /^(\d{1,}):(\d{1,2}):(\d{1,2})$/g;
    let time = timeRe.exec(hms);

    if (time === null) {
      return null;
    }

    let timeH = parseInt(time[1]);
    let timeM = parseInt(time[2]);
    let timeS = parseInt(time[3]);

    return  (timeH * 3600) + (timeM * 60) + timeS;
  } else {
    return (hms['h'] * 3600) + (hms['m'] * 60) + hms['s'];
  }
}

/**
 * @desc convert hms fmt Array to string
 * @param (Array {h, m, s}} hms: time in hms fmt
 * @return converted string
 */
function hmsFmtToString(hms) {
  return (hms['h'] < 10 ? '0': '') + hms['h'] + ':' + (hms['m'] < 10 ? '0': '') + hms['m']
    + ':' + (hms['s'] < 10 ? '0': '') + hms['s'];
}

/**
 * @desc func to show msg popup box inside a parent div
 * @param {Object} parentDiv: parent HTML Div to show popup in
 * @param {string} type: 'info' or 'error'
 * @param {string} str: message literal
 */
function msgPopup(parentDiv, type, str) {
  let popupElem = document.createElement('div');
  popupElem.id = 'window-msg-popup';
  let popupClass = 'alert ';

  if (type === 'error') {
    popupClass += 'alert-danger';
  } else {
    popupClass += 'alert-info';
  }

  popupElem.setAttribute('class', popupClass);
  popupElem.setAttribute('style', 'margin: 2px; padding: 0px;' +
      ' font: 12px arial,serif; word-wrap: break-word;' +
      ' word-break: break-all;');
  popupElem.innerHTML = str;
  parentDiv.appendChild(popupElem);
  setTimeout(function() { $('#' + popupElem.id).alert('close'); }, 3000);
}

/**
 * @desc func to create/update a btn
 * @param {string} name: name on the btn
 * @param {string} id: html DOM element id
 * @param {string} btnClass: bootstrap btn class
 * @param {function} clickCB: new onClick event callback
 * @return button object
 */
function renderBtn(id, name, btnClass, clickCB = null) {
  let btn = document.getElementById(id);

  if (btn === null || btn === undefined) {
    // create the btn
    btn = document.createElement('btn');
    btn.id = id;
    btn.type = 'btn';
    btn.setAttribute('style', 'margin-right: 15px; margin-left: 15px; padding: 13px;');
    btn.style.fontWeight = 'Bold';
    btn.style.fontSize = '16px';
  }

  btn.innerHTML = name;
  btn.setAttribute('class', btnClass);
  if (clickCB) btn.addEventListener('click', clickCB);
  return btn;
}

/**
 * @desc func to create a label
 * @param {string} labelStr: title string
 * @param {string} forID: DOM id of the element that_ the label belongs too
 * @return label HTML Element Object
 */
function createLabel(labelStr, forId = null) {
  let label = document.createElement('label');
  if (forId) label.setAttribute('for', forId);
  label.style.fontSize = '14px';
  label.innerHTML = TABSPACE + labelStr + TABSPACE;
  return label;
}

/**
 * @desc sends JSON to WebsightServer via websocket. Should call only after socket is open
 * @param {Object} socket: websocket that is already opened
 * @param {string} bridgeType: Channel name of the destination XXXBridge
 * @param {JSON} dataJson: JSON data for destination
 */
function sendToBridge(socket, bridgeType, dataJson = null) {
  console.assert(socket.readyState == 1, 'Cannot send message before socket for %s is not open!',
      socket.url);
  let json = {
    type: bridgeType,
    data: dataJson
  }
  socket.send(JSON.stringify(json));
}

/**
 * @desc Converts base 10 numbers from [0, 255] to hexadecimal
 * @param {int} value: the base 10 number to be converted
 */
function toHex(value) {
  let hex = Number(value).toString(16);
  if (hex.length < 2) {
    hex = "0" + hex;
  }
  return hex;
}

/**
 * @desc Converts an rgb color to hacedimal color
 * @param {int} r: the red component [0, 255]
 * @param {int} g: the green component [0, 255]
 * @param {int} b: the blue component [0, 255]
 */
function rgbToHex(r, g, b) {
  return "#" + toHex(r) + toHex(g) + toHex(b);
}

/**
 * @desc calculates if a (x, y) coordinate is inside a Circular Region
 * @param (x, y): coordinates of the point
 * @param  squaredCircleRadius: square of radius of circular region of interest
 * @return {int} -1 if inside circle; 0 if on circle; 1 if outside circle
 */
function isPointInCircle(x, y, squaredCircleRadius) {
  const squareX = x * x;
  const squareY = y * y;
  if ((squareX + squareY) < squaredCircleRadius) {
    return -1;
  } else if ((squareX + squareY) > squaredCircleRadius) {
    return 1;
  } else {
    return 0;
  }
}

/**
 * @desc calculates the interpolated pose at time p in [0, 1] assuming at time 0 we are at pose_a
 * at time 1 we are at pose_bs.
 * @param pose_a: pose at time 0
 * @param pose_b: pose at time 1
 * @param p: time beetween [0, 1]
 * @return {THREE.Matrix4} equivalent to (1-p) * pose_a + p * pose_b
 */
function poseInterpolation(pose_a, pose_b, p) {
  let qa = new THREE.Quaternion();
  let ta = new THREE.Vector3();
  let sa = new THREE.Vector3();
  // Decompose into a translation, rotation, and scale
  pose_a.decompose(ta, qa, sa);
  let qb = new THREE.Quaternion();
  let tb = new THREE.Vector3();
  let sb = new THREE.Vector3();
  // Decompose into a translation, rotation, and scale
  pose_b.decompose(tb, qb, sb);
  qa.slerp(qb, p);
  ta.lerp(tb, p);
  sa.lerp(sb, p);
  return new THREE.Matrix4().compose(ta, qa, sa);
}

// Support for requestAnimationFrame for all the browsers
function sightRequestAnimationFrame(callback) {
  let requestAnimationFrame =
      window.requestAnimationFrame        ||
      window.webkitRequestAnimationFrame  ||
      window.mozRequestAnimationFrame     ||
      window.oRequestAnimationFrame       ||
      window.msRequestAnimationFrame      ||
      function(callback) {
        return window.setTimeout(function() {
          callback(Date.now());
        }, 50);
      };
  requestAnimationFrame.call(window, callback);
}

/**
 * @desc Returns the 2D angle equivalent to the quaternion. It assumes the quaternion represents a
 *       2d angle around the z axis.
 * @param {Quaternion} quaternion: the quaterion representing the 2d angle around the Z axis
 */
function quaternionTo2dAngle(quaternion) {
  const angle = 2.0 * Math.acos(quaternion.z < 0.0 ? -quaternion.w : quaternion.w);
  if (angle > Math.PI) {
    return angle - 2.0 * Math.PI;
  }
  return angle;
}
