/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

"use strict";
// Object containing global variable (meant to avoid to polute all other files.)
let SightConfig_ = {
  new_obj: null,
  accordion: "",
  changed_config: {}
};

// Automatically adjust the config size between 400 to 640px depending on the content.
function ResizeConfig() {
  const isVisible = function(el) { return window.getComputedStyle(el).display !== 'none' };
  let config = document.getElementById("config");
  if (config.currently_hidden) return;
  const width_before = parseInt(config.style.width);
  // Remove transisition and resize to 0 to compute properly the min element size
  config.style.transition = "";
  config.style.width = "0px";
  let width = 400;  // Minimum with for the config
  let accordion = document.getElementById("accordion");
  const kHeaderOffset = 70;
  for (let id = 0; id < accordion.children.length; id++) {
    const title = accordion.children[id].children[0].children[0].children[0];
    width = Math.max(width, title.offsetWidth + kHeaderOffset);
    const panel = accordion.children[id].children[1];
    if (isVisible(panel)) {
      const titles = panel.getElementsByTagName("h4");
      for (let index = 0; index < titles.length; index++) {
        width = Math.max(width, titles[index].scrollWidth + kHeaderOffset);
      }
      const params = panel.getElementsByClassName("__config-param");
      for (let index = 0; index < params.length; index++) {
        width = Math.max(width,  2 * params[index].scrollWidth + kHeaderOffset);
      }
    }
  }
  // Reset original width to start transition from and set a smooth transition
  config.style.width = width_before + "px";
  // If the required width is similar to the current one and smaller, we don't change anything
  const kMinChange = 20;  // Minimum change of the config's size.
  if (width <= config.offsetWidth && width + kMinChange >= config.offsetWidth) {
    return;
  }
  config.style.transition = "width 0.5s ease-in-out";
  config.style.width = Math.min(width, 640) + "px";
}

// Create whole accordion panel, pass the whole object in, loop through every layer of the whole object,
// create header/node/label/input based on each layer key-value pair.
// In the end each input object will be like:
// {
//     input id: "header/node/label"
//     input value: last layer of each value
// }
function GenerateAccordion(new_obj) {
  SightConfig_.new_obj = new_obj;
  if (new_obj !== undefined && new_obj !== null) {
    SightConfig_.accordion = "";
    // loop the first layer of new_obj
    Object.entries(new_obj).forEach(function ([key, value]) {
      // Below we replace all (not just first) spaces and dots with underscore
      // Dots are replaced to support prefixes of subgraphs, e.g.,
      // to handle "navigation.localization.scan_localization".
      const safekey = key.replace(/ /g, '_').replace(/\./g, '_');
      SightConfig_.accordion += `<div class="panel panel-default __config-node">
            <div class="panel-heading" role="tab" id="headingOne">
              <h3 class="panel-title">
              <a role="button" data-toggle="collapse" data-parent="#accordion" href="#${safekey}-tag" aria-expanded="false" aria-controls="collapseOne">` + key + `</a></h3></div>
            <div id="${safekey}-tag" class="panel-collapse collapse" role="tabpanel" aria-labelledby="headingOne">
              <div class="panel-body">`;
      let tag = key;
      // loop the second layer of new_obj
      Object.entries(value).forEach(function ([key, value]) {
        SightConfig_.accordion +='<div class="__config-codelet">';
        tag += `/${key}`;
        if (typeof (value) === 'object' && Object.getOwnPropertyNames(value).length > 1) {
          // let nodeName = value.__type_name;
          SightConfig_.accordion += '<h4>' + key + '</h4>';
          // loop the core layer of new_obj
          Object.entries(value).forEach(function ([key, value]) {
            if (key !== '__type_name') {
              SightConfig_.accordion +='<div class="__config-param">';
              tag += `/${key}`;
              let input;
              if (typeof (value) === 'boolean') {
                if (value) {
                  input = `<input id="${tag}" class="config-input" type="checkbox" checked>`;
                } else {
                  input = `<input id="${tag}" class="config-input" type="checkbox">`;
                }
              } else if (typeof (value) === 'number') {
                input = `<input id='${tag}' type='number' value='${value}' class='config-input form-control' aria-describedby='inputSuccess2Status' step='any'>`;
              } else if (typeof (value) === 'string') {
                input = `<input id='${tag}' type='text' tt='text' value='${value}' class='config-input form-control' aria-describedby='input${tag}Status'>`;
              } else if (Array.isArray(value)) {
                value = JSON.stringify(value).replace("\'", "&rsquo;");
                input = `<input id='${tag}' type='text' tt='array' value='${value}' class='config-input form-control' aria-describedby='input${tag}Status'>`;
              } else {
                value = JSON.stringify(value).replace("\'", "&rsquo;");
                input = `<textarea id='${tag}' tt='object' class='config-input form-control' aria-describedby='input${tag}Status'>${value}</textarea>`;
              }
              SightConfig_.accordion += `
                <div class="form-group has-feedback" id="group+${tag}">
                  <label style="max-width: 285px" class="col-sm-4 control-label" for="${tag}"
                         title="${key}">${key}</label>
                  <div class="col-sm-8">
                    ${input}
                  </div>
                </div>
              `;
              tag = tag.slice(0, - `/${key}`.length);
              SightConfig_.accordion +='</div>'
            }
          });
        }
        tag = tag.slice(0, - `/${key}`.length);
        SightConfig_.accordion +='</div>';
      });
      SightConfig_.accordion += `</div></div></div>`;
    });
    document.getElementById('accordion').innerHTML = SightConfig_.accordion;
    // Override the onchange / onmousewheel event.
    let inputs = document.getElementsByClassName("config-input");
    for (let i = 0; i < inputs.length; i++) {
      inputs[i].onchange = function() { CheckFunc(inputs[i]) };
      inputs[i].onmousewheel = function(e) { this.blur(); };
      if (inputs[i].value !== "" && inputs[i].type !== 'checkbox') inputs[i].required = true;
    }
  }
  let button = document.getElementById("config-submit");
  button.classList.remove("config-pending-change");
  FilterConfig(document.getElementById("config-filter"));
  $('.collapse').on('shown.bs.collapse', function () { ResizeConfig(); });
}

// Filter the config with a given input.
// If the content of input.value can be splits in substring such as each substring appears in
// `node/codelet/param` in the same order, and that the number of substring is minimal compare to
// all the parameters, then the parameter will be displayed, otherwise it will be hidden.
function FilterConfig(input, event) {
  // Erase the input when escape is pressed
  if (event && event.keyCode == 27) {
    input.value = "";
  }
  const filter = input.value;
  let lists = [];
  for (let i = 0; i <= filter.length + 2; i++) {
    lists.push([]);
  }

  // Let's loop through each node
  let nodes = document.getElementById('accordion').getElementsByClassName("__config-node");
  if (nodes.length == 0) return;
  for (let i = 0; i < nodes.length; i++) {
    let render_node = filter.length + 1;
    const node = nodes[i];
    const codelets = node.getElementsByClassName("__config-codelet");
    // Let's loop through each codelet of a given node
    for (let j = 0; j < codelets.length; j++) {
      let render_codelet = filter.length + 1;
      const codelet = codelets[j];
      const params = codelet.getElementsByClassName("__config-param");
      // Let's loop through each parameter of a given codelet
      for (let k = 0; k < params.length; k++) {
        const param = params[k];
        const label = param.getElementsByTagName("label")[0].getAttribute("for");
        const render_param = TextFilter(label, filter, filter.length + 2);
        render_codelet = Math.min(render_codelet, render_param);
        lists[render_param].push(param);
      }
      render_node = Math.min(render_node, render_codelet);
      lists[render_codelet].push(codelet);
    }
    lists[render_node].push(node);
    node.children[1].classList.remove("filter-visible");
  }
  let first = 0;
  let subsections = [];
  // Let's find the first non empty list
  while (lists[first].length == 0 && first < filter.length) first++;
  // Mark all the items inside this list as visible
  for (let i = 0; i < lists[first].length; i++) {
    lists[first][i].classList.remove("filter-hidden");
    if (lists[first][i].classList.contains("__config-node")) {
      subsections.push(lists[first][i].children[1]);
    }
  }
  // Mark all the items as hidden
  for (let j = first+1; j < lists.length; j++) {
    for (let i = 0; i < lists[j].length; i++) {
      lists[j][i].classList.add("filter-hidden");
    }
  }
  // If we have fewer than 3 subsections, we can expand them.
  if (subsections.length <= 3) {
    for (let i = 0; i < subsections.length; i++) {
      subsections[i].classList.add("filter-visible");
    }
  }
  ResizeConfig();
}

// Validate input onChange, pass input object in. If valid,
// save changed value into SightConfig_.changed_config object as single layer
// key-value pair as {"header/node/label": changedValue}
function CheckFunc(obj) {
  let validity = true;
  if (obj.validity.valid) {
    let updateValue;
    let tagKey = obj.id.split('/');
    let node = tagKey[0];
    let component = tagKey[1];
    let label = tagKey[2];
    if (typeof SightConfig_.new_obj[node][component][label] === 'number') {
      if (!Number.isNaN(obj.value)) {
        updateValue = Number(obj.value);
      } else {
        validity = false;
      }
    } else if (typeof SightConfig_.new_obj[node][component][label] === 'boolean') {
      updateValue = obj.checked;
    } else if (obj.getAttribute('tt') === "text") {
      updateValue = obj.value;
    } else if (obj.getAttribute('tt') === "object" || obj.getAttribute('tt') === "array") {
      try {
        updateValue = JSON.parse(obj.value);
      } catch (e) {
        validity = false;
      }
    } else {
      validity = false;
    }
    obj.setAttribute("data-validity", validity);

    if (validity) {
      if (JSON.stringify(SightConfig_.new_obj[node][component][label]) ===
          JSON.stringify(updateValue)) {
        delete SightConfig_.changed_config[obj.id];
      } else {
        SightConfig_.changed_config[obj.id] = updateValue;
      }
      let button = document.getElementById("config-submit");
      if (Object.entries(SightConfig_.changed_config).length > 0) {
        button.classList.add("config-pending-change");
      } else {
        button.classList.remove("config-pending-change");
      }
    }
  }
}

// Generates the final submit object. Create configObj which is the same data structure as source data,
// based on SightConfig_.changed_config, which makes configObj with changed value only. Finally create a submitObj
// which data field is configObj and type field is "setConfig", and send it back to server
function onSubmit() {
  let configJson = {};
  Object.entries(SightConfig_.changed_config).forEach(function ([key, value]) {
    let tagKey = key.split('/');
    let node = tagKey[0];
    let component = tagKey[1];
    let label = tagKey[2];
    if (configJson[node] === undefined) {
      configJson[node] = {};
    }
    if (configJson[node][component] === undefined) {
      configJson[node][component] = {};
    }
    configJson[node][component][label] = value;
    SightConfig_.new_obj[node][component][label] = value;
  });
  SightConfig_.changed_config = {};

  my_socket.send(JSON.stringify({
      type: 'config',
      data: {
        request: 'set',
        config: configJson
      }
    }));
  let button = document.getElementById("config-submit");
  button.classList.remove("config-pending-change");
}

// Send a request for the new config. Reset the variable accordion to be empty once we receive the
// new config.
function LoadConfig() {
  GenerateAccordion([]);
  my_socket.send(JSON.stringify({
      type: 'config',
      data: {
        request: 'get'
      }
    }));
}

// Called when the cancel button is pressed, reload the config.
function onCancel() {
  LoadConfig();
}

// Hide the config on the right side
function hideConfig() {
  let config = document.getElementById("config");
  config.children[0].style.display = "block";
  config.children[1].style.display = "none";
  config.children[2].style.display = "none";
  config.currently_hidden = true;
  config.style.width = "50px";
  config.style.webkitTransform = "translateX(20px)";
  config.style.transition = "0.5s ease-in-out";
  saveConfig("config-status", "hidden");
}

// Show the config on the right side
function showConfig() {
  let config = document.getElementById("config");
  setTimeout(function() {
    for (let i = 0; i < config.children.length; i++) {
      config.children[i].style.display = "";
    }
  }, 450);
  config.currently_hidden = false;
  ResizeConfig();
  config.style.webkitTransform = "";
  saveConfig("config-status", "visible");
}
