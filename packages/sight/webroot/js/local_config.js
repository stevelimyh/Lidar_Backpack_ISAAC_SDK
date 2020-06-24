/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Save locally a value.
function saveConfig(cname, cvalue) {
  if (typeof(Storage) !== "undefined") {
    localStorage.setItem(cname, cvalue);
  } else {
    document.cookie = cname.replace(/\s/g, '') + "=" + cvalue;
  }
}

// Get a locally saved value.
function getConfig(cname, def = "") {
  if (typeof(Storage) !== "undefined") {
    let val = localStorage.getItem(cname);
    if (val !== null) return val;
  } else {
    let name = cname.replace(/\s/g, '') + "=";
    let decodedCookie = decodeURIComponent(document.cookie);
    let ca = decodedCookie.split(';');
    for(let i = 0; i <ca.length; i++) {
      let c = ca[i];
      while (c.charAt(0) == ' ') {
          c = c.substring(1);
      }
      if (c.indexOf(name) == 0) {
          return c.substring(name.length, c.length);
      }
    }
  }
  return def;
}

// Delete the config
function deleteConfig() {
  if (typeof(Storage) !== "undefined") {
    localStorage.clear();
  } else {
    let cookies = document.cookie.split(";");
    for (let i = 0; i < cookies.length; i++) {
      let cookie = cookies[i];
      let eqPos = cookie.indexOf("=");
      let name = eqPos > -1 ? cookie.substr(0, eqPos) : cookie;
      document.cookie = name + "=;expires=Thu, 01 Jan 1970 00:00:00 GMT";
    }
  }
}
