/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Customizable Renderer
// This class needs to be extended and some function implemented:
//   - render(op):
//   - getConfigDiv():
// Optional:
//   - getCanvas()
//   - preRendering()
//   - postRendering()
//   - preChannelRendering(channel_id)
//   - postChannelRendering(channel_id)
//   - getLabelTitle()
class SightRenderer {
  constructor(name, new_renderer) {
    // Create the window
    let delete_window = function(name) {
      RendererManager().deleteRenderer(name);
    }
    let options = this.getRendererOptions();
    options.onclose = delete_window;
    options.menu_label = this.getLabelTitle();
    if (new_renderer) {
      options.center = true;
      options.width = 400;
      options.height = 640;
    }
    let canvas = WindowManager().createWindowWithCanvas(name, options);
    this.canvas_ = canvas;
    // Div containing the legend
    this.legend_div = document.getElementById('plot-' + name + '-legend');
    this.legend_div.style.overflow = "hidden";
    this.legend_div.style.display = "";
    // List of channels
    this.channels_ = [];
    // List of markers channels
    this.markers_channels_ = [];
    // Canvas displayed
    // Name of the renderer
    this.name_ = name;
    this.update_scheduled_ = false;
    // Encoder for recording
    this.encoder = null;
    // Div containing the config;
    this.setting_div_ = document.createElement("div");
    this.loaded_ = false;
    this.custom_frame_ = false;
    // The referential frame to be rendered
    this.base_referential_frame_ = PoseTree().defaultFrame();
    // The referential frame to be rendered
    this.widget_base_ = null;
    // The delay in rendering. It adds a little bit of lag but provide a more accurate rendering
    this.render_delay_ = 0.2;

    // Whether or not the window should fit the content
    this.window_fit_content_ = getConfig(name+"_fit_content", "false") === "true";
    if (this.window_fit_content_) {
      this.fixWindowSize();
    } else {
      this.setWindowResizable();
    }

    // Whether or not the legend should be hidden
    this.hide_legend_ = getConfig(name+"_hide_legend", "false") === "true";
    if (this.hide_legend_) {
      this.hideLegend();
    } else {
      this.showLegend();
    }

    // A static frame used for time synchronization
    this.static_frame_ = this.base_referential_frame_;
    // The current time we are rendering the scene
    this.current_time_ = 0.0;

    // Add mouse event.
    this.mousewheel_ = this.mousewheel_.bind(this);
    this.mousemove_ = this.mousemove_.bind(this);
    this.mouseout_ = this.mouseout_.bind(this);
    this.mousedown_ = this.mousedown_.bind(this);
    this.mouseup_ = this.mouseup_.bind(this);
    this.setupCanvasProperties();
  }

  setupCanvasProperties() {
    let binder = this;
    let mousewheelevt=(/Firefox/i.test(navigator.userAgent))? "DOMMouseScroll" : "mousewheel";
    if (this.canvas_.attachEvent) {
      // if IE (and Opera depending on user setting)
      this.canvas_.attachEvent("on"+mousewheelevt, this.mousewheel_);
      this.canvas_.attachEvent("onmousemove", this.mousemove_);
      this.canvas_.attachEvent("onmouseout", this.mouseout_);
      this.canvas_.attachEvent("onmousedown", this.mousedown_);
      this.canvas_.attachEvent("onmouseup", this.mouseup_);
    } else if (this.canvas_.addEventListener) {
      // WC3 browsers
      this.canvas_.addEventListener(mousewheelevt, this.mousewheel_, true);
      this.canvas_.addEventListener("mousemove", this.mousemove_);
      this.canvas_.addEventListener("mouseout", this.mouseout_);
      this.canvas_.addEventListener("mousedown", this.mousedown_);
      this.canvas_.addEventListener("mouseup", this.mouseup_);
    }

    {
      this.menu_ = new ContextMenu([
        {
          text: "Enable all channels",
          callback: function() {
            binder.enableAllChannels();
          }
        },
        {
          text: "Check all channels",
          callback: function() {
            binder.checkAllChannels();
          }
        },
        {
          text: "Uncheck all channels",
          callback: function() {
            binder.uncheckAllChannels();
          }
        },
        {
          text: this.hide_legend_ ? "Show channels" : "Hide channels",
          callback: function() {
            if (binder.hide_legend_) {
              binder.showLegend();
              this.updateTitle("Hide channels");
            } else {
              binder.hideLegend();
              this.updateTitle("Show channels");
            }
            binder.getWindow().click();
          }
        },
        {
          text: "",  // Separator
          enable: false
        },
        {
          text: "Save Canvas",
          callback: function() {
            let name = prompt("Name of the image", binder.name_ + ".png");
            if (name !== null) {
              binder.saveImage(name);
            }
          }
        },
        {
          text: "Start Recording",
          callback: function() {
            if (binder.encoder !== null) {
              this.updateTitle("Start Recording");
              try {
                binder.stopRecording();
              } catch (e) {
                console.error(e);
              }
            } else {
              this.updateTitle("Stop Recording");
              try {
                binder.startRecording();
              } catch (e) {
                console.error(e);
              }
            }
          }
        },
        {
          text: "",  // Separator
          enable: false
        },
        {
          text: this.window_fit_content_ ? "Allow resizing window" : "Window fit content",
          callback: function() {
            const parent = binder.getWindow();
            if (binder.window_fit_content_ === false) {
              binder.fixWindowSize();
              this.updateTitle("Allow resizing window");
            } else {
              binder.setWindowResizable();
              this.updateTitle("Window fit content");
            }
            binder.update();
          }
        },
        {
          text: "Settings",
          callback: function() {
            binder.showConfig();
          }
        },
        {
          text: "Change delay: ",
          submenu: [
            {
              slider: {min: 0.0, max: 2.0, value: binder.render_delay_},
              callback: function(value) {
                binder.render_delay_ = value;
              }
            },
          ]
        }
      ]);
      this.menu_.attachTo(this.canvas_);
    }
  }

  // Hide the legend
  hideLegend() {
    this.hide_legend_ = true;
    this.legend_div.classList.add("hide-element");
    saveConfig(this.name_+"_hide_legend", true);
  }
  // Show the legend
  showLegend() {
    this.hide_legend_ = false;
    this.legend_div.classList.remove("hide-element");
    saveConfig(this.name_+"_hide_legend", false);
  }

  // Make th windows size fit the inside content
  fixWindowSize() {
    this.window_fit_content_ = true;
    this.getWindow().classList.add("sight-floating-window-div-disable-resize");
    saveConfig(this.name_+"_fit_content", true);
  }
  // Allowed the windows to be resized
  setWindowResizable() {
    this.window_fit_content_ = false;
    this.getWindow().classList.remove("sight-floating-window-div-disable-resize");
    saveConfig(this.name_+"_fit_content", false);
  }

  // Returns the option specific to the renderer
  getRendererOptions() {
    return {resize: false, close: true};
  }

  // Returns the window
  getWindow() {
    return this.canvas_.parentNode.parentNode;
  }

  // Returns the name of the renderer
  getName() {
    return this.name_;
  }

  // Returns the default label
  getLabelTitle() {
    return "Others";
  }

  // Enable all channels
  enableAllChannels() {
    for (let i = 0; i < this.channels_.length; i++) {
      const node = tree_menu_.findNode(this.channels_[i].name);
      if (!node) continue;
      if (!node.checkbox.checked) node.checkbox.click();
    }
  }

  // Enable all channels
  checkAllChannels() {
    for (let i = 0; i < this.channels_.length; i++) {
      let check = this.channels_[i].label.cb;
      if (check.checked === false) {
        check.click();
      }
    }
  }
  // Disable all channels
  uncheckAllChannels() {
    for (let i = 0; i < this.channels_.length; i++) {
      let check = this.channels_[i].label.cb;
      if (check.checked === true) {
        check.click();
      }
    }
  }

  // Notify of a change (Enable/Disable) for a given channel
  updateChannelStatus(channel) {
    let chan = this.channels_.find(x => x.c === channel);
    if (chan !== null) {
      if (channel.status_ == false) {
        chan.label.classList.remove("renderer-label-enable");
        chan.label.classList.remove("renderer-label-invalid");
        chan.label.classList.add("renderer-label-disable");
        chan.label.title = "Channel is disabled in the Channel menu on the left! You will see" +
            " the last data received on this channel before it was disabled.";
      } else if (channel.empty()) {
        chan.label.classList.remove("renderer-label-enable");
        chan.label.classList.remove("renderer-label-invalid");
        chan.label.classList.add("renderer-label-disable");
        chan.label.title = "This channel is currently empty. Check the name is correct and that " +
            "the codelet is ticking properly";
      } else {
        const incompatible_frames = channel.findIncompatibleFrames(this.base_referential_frame_);
        if (Object.keys(incompatible_frames).length > 0) {
          chan.label.classList.remove("renderer-label-enable");
          chan.label.classList.remove("renderer-label-disable");
          chan.label.classList.add("renderer-label-invalid");
          let list_missing_transformation = "[";
          for (let frame in incompatible_frames) {
            list_missing_transformation += this.base_referential_frame_ + "_T_" + frame + ", ";
          }
          chan.label.title =
              "Channel is disabled because the coordinate frames are not connected: " +
              list_missing_transformation + "] is not defined";
        } else {
          chan.label.classList.remove("renderer-label-disable");
          chan.label.classList.remove("renderer-label-invalid");
          chan.label.classList.add("renderer-label-enable");
          chan.label.title = "";
        }
      }
    }
  }

  // Set max frame rate
  sortLabels() {
    let div = this.legend_div;
    let elements = [].slice.call(div.childNodes);
    [].slice.call(elements).sort(function(a, b) {
      return a.channel.z - b.channel.z;
    }).forEach(function(val, index) {
      div.appendChild(val);
    });
  }

  // Create the legend of a given channel
  createLegend_(channel, visibilityChanged, isMarker) {
    // Create the legend for the channel
    let legend = document.createElement('div');
    let binder = this;

    let holder = document.createElement('div');
    holder.style.display = "flex";
    legend.appendChild(holder);
    if (this.getLabelEdit instanceof Function) {
      let edit_img = document.createElement('img');
      edit_img.classList.add("unselectable");
      edit_img.src = "map-assets/edit_objects.png";
      edit_img.width = 20;
      edit_img.height = 20;
      edit_img.edit_mode = false;
      edit_img.channel = channel;
      holder.appendChild(edit_img);
      edit_img.addEventListener('click', function() {
        let channel = this.channel;
        if (this.edit_mode) {
          this.src = "map-assets/edit_objects.png";
          while (channel.edit.firstChild) channel.edit.removeChild(channel.edit.firstChild);
        } else {
          this.src = "map-assets/stop_recording.png";
          channel.edit.appendChild(binder.getLabelEdit(channel.config));
          channel.edit.style.width = "max-content";
        }
        this.edit_mode = !this.edit_mode;
      });
    }

    let label = document.createElement('label');
    let check = document.createElement('input');
    let edit = document.createElement('div');
    check.type = "checkbox";
    check.checked = channel.a;
    label.appendChild(check);
    if (isMarker) {
      label.appendChild(document.createTextNode('Marker: ' + channel.c));
    } else {
      label.appendChild(document.createTextNode(channel.c.name_));
    }
    holder.appendChild(label);
    legend.channel = channel.c;
    legend.appendChild(edit);
    this.legend_div.appendChild(legend);
    if (isMarker) {
      check.channel = channel;
    } else {
      check.channel = this.channels_[this.channels_.length - 1];
    }
    check.renderer = this;

    check.addEventListener('change', function() {
      check.channel.a = this.checked;
      if (visibilityChanged) {
        visibilityChanged(this.checked);
      }
      this.renderer.update(null);
    });
    channel.edit = edit;
    channel.label = label;
    label.cb = check
    if (isMarker) {
      this.updateMarkerLabel(channel.c, 'enable');
    } else {
      this.updateChannelStatus(channel.c);
    }
    this.getWindow().click();
  }

  searchMarkerLabel(markerName) {
    let label = null;
    for (let i = 0; i < this.legend_div.childNodes.length; i++) {
      if (this.legend_div.childNodes[i].childNodes[0].childNodes[1].innerText ===
            'Marker: ' + markerName) {
        label = this.legend_div.childNodes[i].childNodes[0].childNodes[1];
        break;
      }
    }
    return label;
  }

  updateMarkerLabel(markerName, status) {
    let label = this.searchMarkerLabel(markerName);

    if (label !== null) {
      switch(status) {
        case 'enable': {
          label.classList.remove("renderer-label-disable");
          label.classList.remove("renderer-label-invalid");
          label.classList.add("renderer-label-enable");
          label.title = "";
        }
        break;
        case 'disable': {
          label.classList.remove("renderer-label-enable");
          label.classList.remove("renderer-label-invalid");
          label.classList.add("renderer-label-disable");
          label.title = "Marker not found in PoseTree";
        }
        break;
        case 'invalid': {
          label.classList.remove("renderer-label-disable");
          label.classList.remove("renderer-label-enable");
          label.classList.add("renderer-label-invalid");
          label.title = "Marker is not registered as an interactive marker";
        }
        break;
      }
    }
  }

  // get the list of potential markers for this renderer
  getPotentialMarkers() {
    let potentialMarkers = [];
    const interactiveMarkers = InteractiveMarkersManager().getMarkers(this.base_referential_frame_);
    for (let i in interactiveMarkers) {
      const edge = {
          lhs: interactiveMarkers[i].lhs,
          rhs: interactiveMarkers[i].rhs,
          interactive: true
      }
      potentialMarkers.push(edge);
    }

    // Obtain the list of all edges
    const all_edges = PoseTree().getEdgesNames();
    for (let i in all_edges) {
      const distA =  PoseTree().getPathLength(this.base_referential_frame_, all_edges[i].lhs);
      const distB =  PoseTree().getPathLength(this.base_referential_frame_, all_edges[i].rhs);
      if (distA == -1 || distB == -1) {
        // The edge is not reachable from this.base_referential_frame_
        continue;
      }
      let edge;
      // We need to order the edge, since we want to root at this.base_referential_frame_
      if (distA <= distB) {
        edge = {
            lhs: all_edges[i].lhs,
            rhs: all_edges[i].rhs,
            interactive: false
          };
      } else { // Swap the edge
        edge = {
            lhs: all_edges[i].rhs,
            rhs: all_edges[i].lhs,
            interactive: false
          };
      }
      const found = potentialMarkers.find(m => m.lhs === edge.lhs && m.rhs === edge.rhs);
      if (typeof found === 'undefined') {
        // This should be a non interactive marker
        potentialMarkers.push(edge);
      }
    }

    return potentialMarkers;
  }

  // Add a channel
  addChannel(channel, zIndex, active, refresh, config, visiblityChanged) {
    let chan = this.channels_.find(x => x.c === channel);
    if (chan == null) {
      // zIndex, channel, active
      if (zIndex === undefined) zIndex = 10 * (1 + this.channels_.length);
      if (active === undefined) active = true;
      if (refresh === undefined) refresh = true;
      if (config === undefined) config = {};
      chan = {z: zIndex, c: channel, a: active, r: refresh, config: config, name: channel.name_};
      this.channels_.push(chan);
      channel.addRenderer(this);
      this.createLegend_(chan, visiblityChanged, false);
    } else {
      if (zIndex !== undefined) chan.z = zIndex;
      if (refresh !== undefined) chan.r = refresh;
      // Do not change the active status as we should keep the user choice
    }
    // Sort by z-index
    this.channels_.sort(function(a, b) { return a.z - b.z; });
    this.sortLabels();
  }

  // Add a markers channel
  addMarkerChannel(channel, zIndex, active, interactive, config, visiblityChanged) {
    let chan = this.markers_channels_.find(x => x.c === channel);
    if (typeof chan === 'undefined') {
      // zIndex, channel, active
      if (zIndex === undefined) zIndex = 10 * (1 + this.markers_channels_.length);
      if (active === undefined) active = true;
      if (interactive === undefined) interactive = true;
      if (config === undefined) config = {};
      chan = {z: zIndex, c: channel, a: active, i: interactive, config: config};
      this.markers_channels_.push(chan);
    } else {
      if (zIndex !== undefined) chan.z = zIndex;
      if (interactive !== undefined) chan.i = interactive;
      // Do not change the active status as we should keep the user choice
    }
    this.createLegend_(chan, visiblityChanged, true);
    // Sort by z-index
    this.markers_channels_.sort(function(a, b) { return a.z - b.z; });
    this.sortLabels();
  }

  // Default special config
  mergeSpecialConfig(config) {}

  // Create the config object.
  toConfig() {
    let config = {
      renderer: this.getRendererType(),
      dims: {
        width: this.getCanvas().width,
        height: this.getCanvas().height
      },
      channels: [],
      markers: []
    };
    for (let name in this.channels_) {
      config.channels.push({
        name: this.channels_[name].c.name_,
        active: this.channels_[name].a,
        refresh: this.channels_[name].r,
        zIndex: this.channels_[name].z,
        config: this.channels_[name].config || {}
      });
    }

    config.custom_frame = this.custom_frame_;
    config.base_frame = this.base_referential_frame_;
    config.static_frame = this.static_frame_;

    for (let name in this.markers_channels_) {
      config.markers.push({
        name: this.markers_channels_[name].c,
        active: this.markers_channels_[name].a,
        interactive: this.markers_channels_[name].i,
        zIndex: this.markers_channels_[name].z,
        config: this.markers_channels_[name].config || {}
      });
    }
    this.mergeSpecialConfig(config);
    return config;
  }

  // Default special config
  parseSpecialConfig(config) {}

  // Parse from a config.
  parseFromConfig(config) {
    if (console.hasOwnProperty("delay")) {
      this.render_delay_ = console.delay;
    }
    if (config.hasOwnProperty("dims")) {
      this.setDimensions(config.dims);
    }
    if (config.hasOwnProperty("base_frame")) {
      this.base_referential_frame_ = config.base_frame;
    }
    this.custom_frame_ = config.custom_frame === true;
    this.custom_frame_ = config.custom_frame === true;
    this.static_frame_ = config.static_frame;
    if (config.hasOwnProperty("channels")) {
      while (this.legend_div.firstChild) {
        this.legend_div.removeChild(this.legend_div.firstChild);
      }
      for (let c in this.channels_) {
        this.channels_[c].c.removeRenderer(this);
      }
      this.channels_ = [];
      for (let c in config.channels) {
        let conf = config.channels[c];
        this.addChannel(RendererManager().channels_[conf.name], conf.zIndex, conf.active,
                        conf.refresh, conf.config);
      }
    }

    if (config.hasOwnProperty("markers")) {
      this.markers_channels_ = [];
      for (let i in config.markers) {
        if (typeof config.markers[i].name === 'undefined') {
          continue;
        }
        this.addMarkerChannel(config.markers[i].name, config.markers[i].zIndex,
                              config.markers[i].active, config.markers[i].interactive,
                              config.markers[i].config);
      }
    }

    this.parseSpecialConfig(config);
    this.getWindow().click();
  }

  // Call at the beginning of the rendering section
  preRendering() {}
  // Call at the end of the rendering section
  postRendering() {}
  // Call before rendering the channel
  preChannelRendering(channel_id) {}
  // Call after rendering the channel
  postChannelRendering(channel_id) {}

  // Handle mouse event
  mousewheel_(evt) {}
  // Handle mouse move
  mousemove_(evt) {}
  // Handle mouse over
  mouseout_(evt) {}
  // Handle mouse down
  mousedown_(evt) {}
  // Handle mouse up
  mouseup_(evt) {}

  update(channel) {
    if (this.window_fit_content_ && this.setting_div_.parentNode === null) {
      const parent = this.getWindow();
      const width = this.canvas_.width;
      const height = this.canvas_.height + this.legend_div.clientHeight +
                     parent.children[0].clientHeight;
      parent.style.width = width + "px";
      parent.style.height = height + "px";
    }
    for (let i = 0; i < this.channels_.length; i++) {
      let cha = this.channels_[i];
      if (cha.c === channel && !cha.a) return;
    }
    this.update_scheduled_ = true;
  }

  applyInput() {}
  postApplyInput() {}
  preRenderFrame() {}
  resetBaseFrame() {}
  renderFrame() {
    // Do not render if not visible
    if (!this.canvas_.isVisible()) return;
    if (!this.loaded_) return;
    if (this.update_scheduled_) {
      this.updateImpl(null);
    }
    this.annimate();
  }
  annimate() {}

  // An update is beeing requested, if possible we render.
  updateImpl(channel) {
    if (this.channels_.length == 0) return;
    for (let i = 0; i < this.channels_.length; i++) {
      let cha = this.channels_[i];
      // Do not update for not active channel or if the channel does not require refresh
      if (cha.c === channel && (!cha.a || !cha.r)) return;
    }
    this.resetBaseFrame();
    if(this.widget_base_ != this.base_referential_frame_) {
      InteractiveMarkersManager().setWidgetBaseFrame(
          this.markersWidgetName_, this.base_referential_frame_);
      this.widget_base_ = this.base_referential_frame_;
    }
    if (this.custom_frame_ !== true ||
        !PoseTree().areConnected(this.static_frame_, this.base_referential_frame_)) {
      this.base_referential_frame_ = this.channels_[0].c.getFrame();
      this.static_frame_ = PoseTree().areConnected("world", this.base_referential_frame_)
          ? "world"
          : this.base_referential_frame_;
    }
    this.current_time_ = PoseTree().now() - this.render_delay_;
    this.preRendering();
    for (let i = 0; i < this.channels_.length; i++) {
      let cha = this.channels_[i];
      if (this.hide_legend_ === false) {
        this.updateChannelStatus(cha.c);
      }
      if (cha.a) {
        const incompatible_frames = cha.c.findIncompatibleFrames(this.base_referential_frame_,
                                                                 this.current_time_);
        if (Object.keys(incompatible_frames).length > 0) {
          continue;
        }
        this.preChannelRendering(i);
        try {
          this.channels_[i].c.render(this, this.current_time_);
        } catch (e) {
          console.error(e);
        }
        this.postChannelRendering(i);
      }
    }
    this.postRendering();

    this.update_scheduled_ = false;
  }

  // Returns the drawing canvas
  getCanvas() {
    return this.canvas_;
  }

  // Start the recording of the canvas
  startRecording() {
    var options = {
      mimeType: 'video/webm;codecs=h264',
      audioBitsPerSecond : 64000,
      videoBitsPerSecond : 5000000
    }
    this.encoder = new MediaRecorder(this.getCanvas().captureStream(), options);
    this.encoder.chunks = [];
    this.encoder.ondataavailable = function(e) {
      this.chunks.push(e.data);
    }
    let that = this;
    this.encoder.onstop = function() {
      let name = prompt("Name of the video", that.name_ + ".webm");
      if (name !== null) {
        let templink = document.createElement("a");
        templink.download = name;
        const blob = new Blob(this.chunks, { 'type' : 'video/webm' });
        templink.href = URL.createObjectURL(blob);
        document.body.appendChild(templink);
        templink.click();
        document.body.removeChild(templink);
      }
    }
    this.encoder.start();
  }

  // Stop recording
  stopRecording() {
    if (this.encoder == null) {
      console.error("Cannot stop recording as it never started");
      return;
    }
    this.encoder.stop();
    this.encoder = null;
  }

  // Save the canvas as an image
  saveImage(file) {
    let templink = document.createElement("a");
    templink.download = file;
    if (file.endsWith(".jpg") || file.endsWith(".jpeg")) {
      templink.href =
          this.getDataImage("image/jpeg").replace("image/jpeg", "image/octet-stream");
    } else {
      templink.href =
          this.getDataImage("image/png").replace("image/png", "image/octet-stream");
    }
    document.body.appendChild(templink);
    templink.click();
    document.body.removeChild(templink);
  }

  // Set the dimensions
  setDimensions(dims) {
    this.getCanvas().width = dims.width;
    this.getCanvas().height = dims.height;
  }

  // Returns the config window
  hideConfig() {
    let parent = this.setting_div_.parentNode;
    if (parent != null) {
      parent.removeChild(this.setting_div_);
    }
    for (let i = 0; i < parent.children.length; i++) {
      parent.children[i].style.display = "";
    }
    if (this.window_fit_content_) {
      this.getWindow().classList.add("sight-floating-window-div-disable-resize");
    }
  }

  // Returns the config window
  showConfig() {
    if (this.window_fit_content_) {
      this.getWindow().classList.remove("sight-floating-window-div-disable-resize");
    }
    let parent = this.legend_div.parentNode;
    for (let i = 0; i < parent.children.length; i++) {
      parent.children[i].style.display = "none";
    }
    this.setting_div_ = this.getConfigDiv();
    parent.appendChild(this.setting_div_);
  }
}
