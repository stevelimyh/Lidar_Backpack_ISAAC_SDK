/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// 3D Renderer, implement a SightRenderer.
// To control the camera, hold down the left mouse button and then:
// - Move the mouse to rotate the camera
// - Move forward/backward: UP/DOWN/W/S
// - Move laterallly: LEFT/RIGHT/A/D
// - Move up/down: Q/E/R/F
// Double click to enter fullscreen mode, hit escape to leave it.
class Renderer3D extends SightRenderer {
  constructor(name, new_renderer = true) {
    super(name, new_renderer);
    this.sceneSetup_(name);
    this.objects_ = [];
    this.cached_sphere_ = new THREE.SphereBufferGeometry(1.0, 16, 16);
    this.cached_sphere_.shared = true;
    this.cached_box_ = new THREE.BoxGeometry(1, 1, 1);
    this.cached_box_.shared = true;

    // Default style
    this.current_color_ = "#000";
    this.override_color = false;
    this.current_fill_ = false;
    this.current_size_ = 1.0;
    this.current_alpha_ = 1.0;

    this.clock_ = new THREE.Clock();

    const that = this;
    this.canvas_.resize = function(dims) {
      that.setCanvasDimensions(dims);
    };
    // If we lose the context, let's recreate a canvas with all the valid property
    this.canvas_.addEventListener('webglcontextlost', function(event) {
      let canvas = document.createElement("canvas");
      canvas.width = that.canvas_.width;
      canvas.height = that.canvas_.height;
      canvas.resize = function(dims) {
        that.setCanvasDimensions(dims);
      };
      canvas.isVisible = that.canvas_.isVisible;
      that.canvas_.parentNode.insertBefore(canvas, that.canvas_);
      that.canvas_.parentNode.removeChild(that.canvas_);

      that.unregisterGlobalEventListeners();
      that.canvas_ = canvas;
      that.setupCanvasProperties();
      that.sceneSetup_(name);
      that.registerGlobalEventListeners();
    });
    this.loaded_ = true;
    // Whether fullscreen mode is on
    this.fullscreen_ = false;

    // Enter or leave the fullscreen mode
    const onDoubleClick = (event) => {
      if (this.fullscreen_) {
        if (document.exitFullscreen) {
          document.exitFullscreen();
        } else if (document.webkitExitFullscreen) {
          document.webkitExitFullscreen();
        } else if (document.mozCancelFullScreen) {
          document.mozCancelFullScreen();
        } else if (document.msExitFullscreen) {
          document.msExitFullscreen();
        }
      } else {
        if (that.canvas_.requestFullscreen) {
          that.canvas_.requestFullscreen();
        } else if (that.canvas_.webkitRequestFullscreen) {
          that.canvas_.webkitRequestFullscreen(Element.ALLOW_KEYBOARD_INPUT);
        } else if (that.canvas_.mozRequestFullScreen) {
          that.canvas_.mozRequestFullScreen();
        } else if (that.canvas_.msRequestFullscreen) {
          that.canvas_.msRequestFullscreen();
        }
      }
    }

    // Whether the pointer is hidden
    this.pointerLocked = false;
    this.controllingCamera = false;

    // Lock the pointer
    const lockPointer = () => {
      this.cursorPosition_canvas = undefined;
      if (this.pointerLocked) return;
      this.pointerLocked = true;
      document.body.requestPointerLock = document.body.requestPointerLock ||
          document.body.mozRequestPointerLock ||
          document.body.webkitRequestPointerLock;
      document.body.requestPointerLock();
    }

    // Unlock the pointer
    const unlockPointer = (event) => {
      this.cursorPosition_canvas =
        new THREE.Vector2(
          (event.offsetX - this.canvas_.clientWidth/2)/this.canvas_.clientWidth,
          (this.canvas_.clientHeight/2 - event.offsetY)/this.canvas_.clientHeight
        );
      if (!this.pointerLocked) return;
      this.pointerLocked = false;
      document.exitPointerLock();
    }

    // Used to detect double click
    this.lastMouseDown = Date.now();

    this.onCanvasMouseDown = (event) => {
      if (event.button == 0) {
        this.markersWidget_.selectionEvent(this.cursorPosition_canvas);
      }
      let markersWidgetSelected = this.markersWidget_.isSelected();
      if (markersWidgetSelected) {
        return;
      }
      switch (event.button) {
      case 0: // left
        let now = Date.now();
        if (now - this.lastMouseDown < 250) {
          onDoubleClick();
        }
        this.lastMouseDown = now;
        lockPointer();
        this.markersWidgetDoMouseRotation_ = true;
        this.controllingCamera = !markersWidgetSelected;
        break;
      case 1: // middle
        lockPointer();
        this.markersWidgetDoMouseTranslation_ = true;
        break;
      }
    };

    this.mouseMotion_ = new THREE.Vector2(0,0);
    this.onDocumentMouseMove = (event) => {
      this.mouseMotion_.add(new THREE.Vector2(event.movementX, event.movementY));
      if (this.controllingCamera) {
        this.controls_.onMouseMove(event);
      }
    };

    this.onDocumentMouseUp = (event) => {
      switch(event.button) {
        case 0: // left
          this.controllingCamera = false;
          this.markersWidgetDoMouseRotation_ = false;
          break;
        case 1: // middle
          this.markersWidgetDoMouseTranslation_ = false;
          break;
      }

      if (!this.controllingCamera &&
          !this.markersWidgetDoMouseRotation_ &&
          !this.markersWidgetDoMouseTranslation_) {
        unlockPointer(event);
      }
    };

    this.keyboardInputState_ = {};
    this.onWindowKeyDown = (event) => {
      if (event.code === "AltLeft") {
        this.markersWidget_.setRotationMode();
      }
      if (!event.repeat) {
        this.keyboardInputState_[event.code] = true;
      }
      if (this.controllingCamera) {
        this.controls_.onKeyDown(event);
      }
    };

    this.onWindowKeyUp = (event) => {
      if (event.code === "AltLeft") {
        this.markersWidget_.setTranslationMode();
      }
      if (!event.repeat) {
        this.keyboardInputState_[event.code] = false;
      }
      this.controls_.onKeyUp(event);
    };

    this.onCanvasContextMenu = (event) => {
      if (this.controllingCamera === false) return;
      event.preventDefault();
    };

    // When there is a change of fullscreen, we need to resize the canvas accordingly
    this.onFullscreenChange = (event) => {
      if (event.target != this.canvas_) return;
      if (this.fullscreen_) {
        this.fullscreen_ = false;
        if (document.webkitIsFullScreen || document.mozFullScreen ||
            document.msFullscreenElement !== null) {
          this.canvas_.resize({width: this.prevWidth, height: this.prevHeight });
        }
      } else {
        this.prevWidth = this.canvas_.width;
        this.prevHeight = this.canvas_.height;
        this.fullscreen_ = true;
        this.canvas_.resize({width: window.outerWidth, height: window.outerHeight });
      }
    };

    this.onCanvasMouseMove = (event) => {
      this.cursorPosition_canvas =
        new THREE.Vector2(
          (event.offsetX - this.canvas_.clientWidth/2)/this.canvas_.clientWidth,
          (this.canvas_.clientHeight/2 - event.offsetY)/this.canvas_.clientHeight
        );
    }

    this.registerGlobalEventListeners();
  }

  // register the events on the canvas
  registerGlobalEventListeners() {
    this.canvas_.addEventListener('mousedown', this.onCanvasMouseDown, false);
    this.canvas_.addEventListener('contextmenu', this.onCanvasContextMenu, false);
    this.canvas_.addEventListener('mousemove', this.onCanvasMouseMove, false);
    document.addEventListener('mousemove', this.onDocumentMouseMove, false);
    document.addEventListener('mouseup', this.onDocumentMouseUp, false);
    window.addEventListener('keydown', this.onWindowKeyDown, false);
    window.addEventListener('keyup', this.onWindowKeyUp, false);
    document.addEventListener('webkitfullscreenchange', this.onFullscreenChange, false);
    document.addEventListener('mozfullscreenchange', this.onFullscreenChange, false);
    document.addEventListener('fullscreenchange', this.onFullscreenChange, false);
    document.addEventListener('MSFullscreenChange', this.onFullscreenChange, false);
  }

  // unregister the events on the canvas
  unregisterGlobalEventListeners() {
    this.canvas_.removeEventListener('mousedown', this.onCanvasMouseDown, false);
    this.canvas_.removeEventListener('contextmenu', this.onCanvasContextMenu, false);
    this.canvas_.removeEventListener('mousemove', this.onCanvasMouseMove, false);
    document.removeEventListener('mousemove', this.onDocumentMouseMove, false);
    document.removeEventListener('mouseup', this.onDocumentMouseUp, false);
    window.removeEventListener('keydown', this.onWindowKeyDown, false);
    window.removeEventListener('keyup', this.onWindowKeyUp, false);
    document.removeEventListener('webkitfullscreenchange', this.onFullscreenChange, false);
    document.removeEventListener('mozfullscreenchange', this.onFullscreenChange, false);
    document.removeEventListener('fullscreenchange', this.onFullscreenChange, false);
    document.removeEventListener('MSFullscreenChange', this.onFullscreenChange, false);
  }

  // Returns the option specific to the renderer
  getRendererOptions() {
    const that = this;
    let onresize = function(dims) {
      if (that.getCanvas() && dims.width && dims.height) {
        that.setCanvasDimensions(dims);
      }
    };
    return {resize: true, close: true, onresize: onresize};
  }

  // Returns the default label
  getLabelTitle() {  // override
    return "3D Drawing";
  }

  // Returns the Renderer type
  getRendererType() {
    return "3d";
  }

  // Get the context of the canvas
  getDataImage(type) {
    this.renderer_.render(this.scene_, this.camera_);
    return this.canvas_.toDataURL(type);
  }

  // Default special config
  mergeSpecialConfig(config) {
    const camera_pos = this.camera_.position;
    const camera_rot = this.camera_.quaternion;
    config.camera = {
      position: [camera_pos.x, camera_pos.y, camera_pos.z],
      rotation: [camera_rot.w, camera_rot.x, camera_rot.y, camera_rot.z]
    }
  }

  // Default special config
  parseSpecialConfig(config) {
    if (config.camera) {
      const camera_pos = config.camera.position;
      const camera_rot = config.camera.rotation;
      this.camera_.position.set(camera_pos[0], camera_pos[1], camera_pos[2]);
      this.camera_.quaternion.set(camera_rot[1], camera_rot[2], camera_rot[3], camera_rot[0]);
    }
  }

  applyInput () {
    this.update(null);
    this.controls_.update(this.clock_.getDelta());

    // Validate the status of the markers channels
    for (let i in this.markers_channels_) {
      const markerName = this.markers_channels_[i].c;
      let marker = InteractiveMarkersManager().getMarker(markerName);
      if (marker !== null) {
        this.updateMarkerLabel(markerName, 'enable');
      } else {
        this.updateMarkerLabel(markerName, 'invalid');
      }
      if (!this.markersWidget_.containsMarker(markerName)) {
        try {
          const transform =
              PoseTree().get(this.base_referential_frame_, markerName, this.current_time_);
          if (transform == null) continue;
          // Decompose matrix in rotation and translation
          let trans = new THREE.Vector3();
          let quat = new THREE.Quaternion();
          let scale = new THREE.Vector3();
          transform.decompose(trans, quat, scale);
          this.markersWidget_.addMarker(markerName, trans, quat, this.markers_channels_[i].i);
        } catch(e) {
          console.warn(e);
        }
      }
    }
    this.markersWidget_.syncWithChannels(this.markers_channels_);
    this.markersWidget_.updateVisibilityStatus(this.markers_channels_);
  }

  feedBackActiveMarkers() {
    const names = this.markersWidget_.getMarkersNames();
    for (let i in names) {
      const path = names[i];
      if (this.markersWidget_.markerIsActive(path)) {
        // Get the current (as the user is editing) pose
        let v = this.markersWidget_.transform(path).position;
        let q = this.markersWidget_.transform(path).orientation;
        const baseReferentialFrame_T_rhs = new THREE.Matrix4().makeRotationFromQuaternion(q)
              .setPosition(v);
        let marker = InteractiveMarkersManager().getMarker(path);
        // This is a non interactive markers, we cannot feedback
        let lhs = marker.lhs;
        let rhs = marker.rhs;
        const lhs_T_baseReferentialFrame = PoseTree().get(lhs, this.base_referential_frame_);
        const lhs_T_rhs =
              new THREE.Matrix4().multiplyMatrices(
                  lhs_T_baseReferentialFrame, baseReferentialFrame_T_rhs);
        // Let the Front end know that we just update our Pose
        const timestamp = InteractiveMarkersManager().toBackendTime(Date.now());
        PoseTree().set(lhs, rhs, lhs_T_rhs, timestamp);
        // Decompose matrix in rotation and translation
        let tmpTrans = new THREE.Vector3();
        let tmpQuat = new THREE.Quaternion();
        let tmpScale = new THREE.Vector3();
        lhs_T_rhs.decompose(tmpTrans, tmpQuat, tmpScale);
        // Let the Back end know that we just update our Pose
        InteractiveMarkersManager().notifyEdgeChange(lhs, rhs, tmpQuat, tmpTrans, timestamp);
      }
    }
  }

  updateInactiveMarkers() {
    try {
      let updatedMarkers = InteractiveMarkersManager().getMarkers(this.base_referential_frame_);
      for (let path in updatedMarkers) {
        const channels = this.markers_channels_.filter(x => x.c == path);
        if (channels.length == 0) {
          delete updatedMarkers[path];
          continue;
        }
        if (this.markersWidget_.interactionEnabled(path)) {
          // Disable interaction.
          if (this.markersWidget_.markerIsActive(path)) {
            delete updatedMarkers[path];
          } else {
            // User is not interacting at the moment. Disable interaction.
            // This is also not great UX: the user may be on their way to interact with the widget and
            // we just move it. Works for now, though.
            this.markersWidget_.disableInteraction(path);
          }
        }
        if (channels[0].config && channels[0].config.size) {
          updatedMarkers[path].scale = config.size;
        }
      }
      this.markersWidget_.set(updatedMarkers);
      for (let path in updatedMarkers) {
        this.markersWidget_.enableInteraction(path);
      }
    } catch (e) {
      console.warn(e);
    }
  }

  postApplyInput() {
    this.feedBackActiveMarkers();
  }

  preRenderFrame() {
    this.updateInactiveMarkers();
  }

  // Request an animation from the WebGL Renderer
  annimate() {
    try {
      this.setCanvasDimensions(this.getCanvas());
      this.renderer_.render(this.scene_, this.camera_);
      this.mouseMotion_.set(0, 0);
    } catch (e) {
      console.error(e);
    }
  }

  // Create a div to edit the config
  getLabelEdit(config) {
    let div = document.createElement('div');
    div.className = "label-edit"

    // Add default color pick
    let color_group = document.createElement('div');
    let color_chooser = document.createElement('input');
    let color_check_label = document.createElement('label');
    let color_check = document.createElement('input');
    color_check.type = "checkbox";
    color_check.checked = config.color === null || config.color === undefined;
    color_check_label.innerHTML = " &nbsp; use default &nbsp; ";
    color_check_label.appendChild(color_check);
    color_chooser.type = "color";
    color_chooser.value = config.color || "#000000";
    color_chooser.onchange = function() {
      config.color = this.value;
      binder.update_scheduled_ = true;
      color_check.checked = false;
    }
    color_check.onchange = function() {
      if (color_check.checked === true) {
        delete config.color;
      } else {
        config.color = color_chooser.value;
      }
      binder.update_scheduled_ = true;
    }
    color_group.appendChild(document.createTextNode("Color: "));
    color_group.appendChild(color_chooser);
    color_group.appendChild(color_check_label);

    // Add slider for size
    let size_group = document.createElement('label');
    size_group.innerHTML = "Default size: ";
    let size_holder = document.createElement('div');
    size_holder.className = "edit-slider";
    let size = document.createElement('input');
    size.type = "range";
    size.className = "slider-2d-rendering";
    size.min = 0;
    size.max = 1000;
    size.value = (50 * (config.size || 1.0)) | 0;
    size_holder.appendChild(size)
    size_group.appendChild(size_holder);
    let size_text = document.createElement('div');
    size_text.innerHTML = parseFloat(size.value) / 50.0;
    size_group.appendChild(size_text);
    size.oninput = function() {
      config.size = parseFloat(this.value) / 50.0;
      size_text.innerHTML = config.size;
    }

    // Add default color pick
    let z_group = document.createElement('div');
    let z_chooser = document.createElement('input');
    z_chooser.type = "number";
    z_chooser.value = config.z || 0.5;
    z_chooser.onchange = function() {
      config.z = parseFloat(this.value);
    }
    z_group.appendChild(document.createTextNode("Default Z: "));
    z_group.appendChild(z_chooser);

    div.appendChild(color_group);
    div.appendChild(size_group);
    div.appendChild(z_group);
    return div;
  }

  // Render the list of operations using a callback function
  render(op, time) {
    if (!op.cached_obj_) {
      op.cached_obj_ = {};
    }
    let that = this;
    if (!op.cached_obj_[this.getName()]) {
      let renderImpl = function(op) {
        const backup_color = that.current_color_;
        const backup_fill = that.current_fill_;
        const backup_size = that.current_size_;
        const backup_alpha = that.current_alpha_;
        let transformation = null;
        let frame = null;
        let has_frame = false;
        if (op.p) {
          if (op.p.t === "c") return null;
          if (op.p.t !== "f") {
            transformation = SightChannel.GetTransform(op, that.static_frame_, time);
          } else {
            frame = op.p.f;
            has_frame = true;
          }
        }
        if (op.s) {
          if (op.s.c && !that.override_color) that.current_color_ = op.s.c;
          if (op.s.f) that.current_fill_ = op.s.f;
          if (op.s.s) that.current_size_ = op.s.s * that.current_size_factor_;
          if (op.s.a) that.current_alpha_ = op.s.a;
        }
        let obj = null;
        if (op.t === "sop") {
          obj = new THREE.Group();
          for (let i in op.d) {
            let item = renderImpl(op.d[i]);
            if (item) {
              obj.add(item);
              has_frame = has_frame || item.has_frame;
            }
          }
          // Don't create empty group
          if (obj.children.length == 0) obj = null;
        } else if (op.t == "img") {
          obj = that.drawImage_(op);
        } else if (op.t == "point_cloud") {
          obj = that.drawPointCloud_(op);
        } else if (op.t == "line") {  // line
          obj = that.drawLine_(op);
        } else if (op.t == "sphr") {  // sphere (aka circle)
          obj = that.drawCircle_(op);
        } else if (op.t == "text") {  // text
          obj = that.drawText_(op);
        } else if (op.t == "cube") {  // cuboid (aka rectangle)
          obj = that.drawRect_(op);
        } else if (op.t == "pnts") {  // point
          obj = that.drawPoint_(op);
        } else if (op.t == "asset") {  // point
          obj = that.drawAsset_(op);
        }
        if (obj) {
          if (transformation) {
            obj.applyMatrix(transformation);
          } else if (frame) {
            obj.frame = frame;
            obj.frame_scale = op.p.s || 1.0;
          }
          obj.has_frame = has_frame;
        }

        if (obj) {
          obj.castShadow = true;
          obj.receiveShadow = true;
        }

        that.current_fill_ = backup_fill;
        that.current_color_ = backup_color;
        that.current_size_ = backup_size;
        that.current_alpha_ = backup_alpha;
        return obj;
      }
      try {
        const keys = Object.keys(op.cached_obj_);
        if (keys.length > 0) {
          op.cached_obj_[this.getName()] = op.cached_obj_[keys[0]].clone();
        } else {
          const obj = renderImpl(op);
          if (obj) {
            op.cached_obj_[this.getName()] = obj;
          }
        }
        op.delete_callback.push(function(op) {
          that.deleteOperation(op.cached_obj_[that.getName()]);
          delete op.cached_obj_[that.getName()];
        });
      } catch (e) {
        console.error(e);
        console.error(op);
      }
    }
    let cached_obj = op.cached_obj_[this.getName()];
    if (cached_obj) {
      if (cached_obj.has_frame) {
        // If the transformation is set with the PoseTree, we need to update the transformation
        let updateFrame = function(obj) {
          if (!obj.has_frame) return;
          if (obj.frame) {
            let parent_T_world = obj.parent ?
                new THREE.Matrix4().getInverse(obj.parent.matrixWorld) :
                new THREE.Matrix4();
            obj.matrix.copy(
                parent_T_world.multiply(
                    PoseTree().get(that.base_referential_frame_, obj.frame, that.current_time_)));
            obj.matrix.scale(new THREE.Vector3(obj.frame_scale, obj.frame_scale, obj.frame_scale));
            // Force update internal member
            obj.applyMatrix(new THREE.Matrix4());
          }
          for (let child in obj.children) {
            updateFrame(obj.children[child]);
          }
        }
        updateFrame(cached_obj);
      }
      this.objects_.push(cached_obj);
    }
  }

  // Remove the operation from the scene if present, and destruct it
  deleteOperation(obj) {
    if (obj === null) return;
    if (obj === undefined) return;
    if (obj.shared === true) return;
    if (obj.children !== undefined) {
      while (obj.children.length > 0) {
        this.deleteOperation(obj.children[0]);
        obj.remove(obj.children[0]);
      }
    }
    if (obj.geometry && obj.geometry.shared !== true) {
      obj.geometry.dispose();
      delete obj.geometry;
    }
    if (obj.material) {
      obj.material.dispose();
      delete obj.material;
    }
    if (obj.dispose instanceof Function) {
      obj.dispose();
    }
  }

  // Set the dimensions
  setDimensions(dimensions) {
    let win_style = this.getWindow().style;
    win_style.width = dimensions.width +"px";
    win_style.height = dimensions.height +"px";
  }

  // Resize the canvas and update the view
  setCanvasDimensions(dims) {  // override
    // Make sure we don't override the width/height in fullscreen mode
    if (this.fullscreen_) {
      dims = {width: window.outerWidth, height: window.outerHeight };
    }
    const width = dims.width;
    const height = dims.height;
    let canvas = this.getCanvas();
    canvas.width = width;
    canvas.height = height;
    const aspect = width / height;
    if (this.camera_ == null) {
      this.camera_ = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000.0);
      this.camera_.position.set(0.0, 0.0, 2.0);
      this.controls_ = new THREE.FreeControls(this.camera_);
      this.controls_.enableZoom = true;
    } else {
      this.camera_.aspect = aspect;
      this.camera_.updateProjectionMatrix();
    }
    this.renderer_.setSize(width, height);
  }

  // Before rendering store the state to compute the diff
  preRendering() {  // override
    this.old_objects_ = this.objects_;
    this.objects_ = [];
    // keeps track on how many time a mesh has been loaded
    this.asset_counter_ = {};
    // Keep track of the current channel
    this.current_channel_ = "";
  }

  // After the rendering update the list of objects
  postRendering() {  // override
    for (let i in this.old_objects_) {
      if (!this.objects_.includes(this.old_objects_[i])) {
        this.scene_.remove(this.old_objects_[i]);
      }
    }
    for (let i in this.objects_) {
      if (!this.old_objects_.includes(this.objects_[i])) {
        this.scene_.add(this.objects_[i]);
      }
    }
    delete this.old_objects_;
    delete this.asset_counter_;
    delete this.current_channel_;
  }

  // Set up the default options (TOOD)
  preChannelRendering(channel_id) {  // override
    const config = this.channels_[channel_id].config;
    this.current_channel_ = this.name_ + "/" + this.channels_[channel_id].c.getName();
    this.channel_options_ = {
      default_z: config.z || 0.5
    };
    if (config.color) {
      this.current_color_ = config.color;
      this.override_color = true;
    } else {
      this.current_color_ = "#000";
      this.override_color = false;
    }
    this.channel_id_ = channel_id
    this.current_fill_ = config.fill || false;
    this.current_size_ = config.size || 1;
    this.current_size_factor_ = config.size || 1;
  }

  // TODO Provide a custom config specific to the 3D
  // Returns the div containing the config
  getConfigDiv() {
    // TODO customize this view
    const that = this;
    // Create a new div to hold all the data
    let div = document.createElement("div");
    div.className = "win-manager-div";

    // Helper to create a div that keep the display property
    let createDiv = function() {
      let d = document.createElement("div");
      d.style.display = "inherit";
      return d;
    }

    // Helper to append an element with a label on the left size
    let append = function(title, elem) {
      let d = document.createElement("div");
      d.className += " win-manager-inside-div";
      let label = document.createElement("label");
      label.appendChild(document.createTextNode(title));
      d.append(label);
      d.append(elem);
      div.appendChild(d);
      let sep = document.createElement("hr");
      div.appendChild(sep);
    }

    // Create the Window Name component
    let name_input = document.createElement("input");
    name_input.type = "text";
    name_input.value = this.name_;
    name_input.disabled = true;
    append("Window name", name_input);

    // Create the dimensions of the window component
    let dims_holder = createDiv();
    let dims_width = document.createElement("input");
    let dims_height = document.createElement("input");
    dims_width.type = "number";
    dims_height.type = "number";
    dims_width.value = parseInt(this.getWindow().style.width);
    dims_height.value = parseInt(this.getWindow().style.height);
    dims_holder.appendChild(dims_height);
    dims_holder.appendChild(document.createTextNode(" x "));
    dims_holder.appendChild(dims_width);
    append("Window dimensions", dims_holder);

    // Create the Frame of the window component
    let frame_holder = createDiv();
    let frame_base_sel = document.createElement("select");
    let frame_static_sel = document.createElement("select");
    let frame_names = PoseTree().getNodeNames();
    let idx = 0;
    for (let name in frame_names) {
      {
        let opt = document.createElement("option");
        opt.value = frame_names[name];
        opt.text = frame_names[name];
        frame_base_sel.add(opt);
        if (frame_names[name] === this.base_referential_frame_) {
          frame_base_sel.selectedIndex = idx;
        }
      }
      {
        let opt = document.createElement("option");
        opt.value = frame_names[name];
        opt.text = frame_names[name];
        frame_static_sel.add(opt);
        if (frame_names[name] === this.static_frame_) {
          frame_static_sel.selectedIndex = idx;
        }
      }
      idx++;
    }
    let frame_lock = document.createElement("input");
    frame_lock.type = "checkbox";
    frame_lock.checked = this.custom_frame_ === true;
    let frame_lock_label = document.createElement("label");
    frame_lock_label.appendChild(frame_lock);
    frame_lock_label.appendChild(document.createTextNode("Automatic"));
    frame_lock_label.style.fontWeight = "normal";
    frame_lock_label.style.color = "#7f7f7f";
    frame_lock.addEventListener("click", function() {
      frame_base_sel.disabled = frame_lock.checked;
      frame_static_sel.disabled = frame_lock.checked;
    });
    frame_lock.click();
    frame_base_sel.style.display = "block";
    frame_static_sel.style.display = "block";
    frame_lock_label.style.display = "block";
    frame_holder.appendChild(frame_base_sel);
    frame_holder.appendChild(frame_static_sel);
    frame_holder.appendChild(frame_lock_label);
    append("", frame_holder);

    // Will hold the current list of added channel
    // TODO make it possible to reorganized the order and remove some
    let list_holder = document.createElement("ul");
    list_holder.className += " channel-list";
    // Select the channel component
    let channels_holder = createDiv();
    channels_holder.style.display = "inline-block";
    let channel_sel = document.createElement("select");
    {
      let opt = document.createElement("option");
      opt.value = "";
      opt.text = "Select channel";
      opt.disabled = true;
      channel_sel.add(opt);
      channel_sel.selectedIndex = 0;
    }
    const channels = RendererManager().getActiveChannelNames();
    for (let c in channels) {
      const name = channels[c];
      let opt = document.createElement("option");
      opt.value = name;
      opt.text = name;
      channel_sel.add(opt);
    }

    // Remove a channel from the list
    let removeFromList = function(elem) {
      channel_sel.options[elem.pos].disabled = false;
      list_holder.removeChild(elem);
    }

    // Remove a marker from the list
    let removeFromMarkerList = function(elem) {
      marker_sel.options[elem.pos].disabled = false;
      marker_list_holder.removeChild(elem);
    }

    // Find the index of an element.
    let findIndex = function(elem) {
      for (let i = 0; i < elem.parentNode.children.length; i++) {
        if (elem === elem.parentNode.children[i]) {
          return i;
        }
      }
      return -1;
    }
    // Helper to add a new channel;
    let addToList = function(index, active = true, refresh = true) {
      let holder = document.createElement("channel");
      // Button to move channels around
      let left_div = document.createElement("div");
      let up = document.createElement('i');
      up.className += " material-icons";
      up.innerHTML = "keyboard_arrow_up";
      up.style.fontSize = "18px";
      up.addEventListener('click', function() {
        let index = findIndex(holder);
        if (index > 0 && index < list_holder.children.length) {
          list_holder.insertBefore(holder, list_holder.children[index - 1]);
        }
      });
      left_div.appendChild(up);
      let down = document.createElement('i');
      down.className += " material-icons";
      down.innerHTML = "keyboard_arrow_down";
      down.style.fontSize = "18px";
      down.addEventListener('click', function() {
        let index = findIndex(holder);
        if (index >= 0 && index + 1 < list_holder.children.length) {
          list_holder.insertBefore(list_holder.children[index + 1], holder);
        }
      });
      left_div.appendChild(down);
      // Remove button
      let right_div = document.createElement("div");
      let cancel = document.createElement('i');
      cancel.className += " material-icons";
      cancel.innerHTML = "cancel";
      cancel.style.fontSize = "22px";
      cancel.title = "Remove channels"
      cancel.addEventListener('click', function() {
        removeFromList(holder);
      });
      right_div.appendChild(cancel);
      let d = document.createElement("chanel-name");
      d.appendChild(document.createTextNode(channel_sel.options[index].value));
      holder.conf = {
        name: channel_sel.options[index].value,
        active: active,
        refresh: refresh
      }
      holder.pos = index;
      holder.appendChild(left_div);
      holder.appendChild(d);
      holder.appendChild(right_div);
      list_holder.appendChild(holder);
      // TODO: Remove from the option
      channel_sel.options[index].disabled = true;
      channel_sel.selectedIndex = 0;
    }

    // Helper to add a new marker
    let addToMarkersList = function(index, active = true, interactive = false) {
      let holder = document.createElement("channel");
      // Remove button
      let right_div = document.createElement("div");
      let cancel = document.createElement('i');
      cancel.className += " material-icons";
      cancel.innerHTML = "cancel";
      cancel.style.fontSize = "22px";
      cancel.title = "Remove markers"
      cancel.addEventListener('click', function() {
        removeFromMarkerList(holder);
      });
      right_div.appendChild(cancel);
      let d = document.createElement("chanel-name");
      d.appendChild(document.createTextNode(marker_sel.options[index].value));
      holder.conf = {
        name: marker_sel.options[index].value,
        active: active,
        interactive: interactive,
        zIndex: 0,
        config: {}
      }
      holder.pos = index;
      holder.appendChild(d);
      holder.appendChild(right_div);
      marker_list_holder.appendChild(holder);
      marker_sel.options[index].disabled = true;
      marker_sel.selectedIndex = 0;
    }

    let add = document.createElement("button");
    add.style.display = "block";
    add.appendChild(document.createTextNode("Add channel"));
    add.addEventListener('click', function() {
      if (channel_sel.selectedIndex == 0) return;
      addToList(channel_sel.selectedIndex);
    });
    channels_holder.appendChild(channel_sel);
    channels_holder.appendChild(add);
    append("", channels_holder);
    // Add current channels to the list
    for (let cha in this.channels_) {
      for (let i = 1; i < channel_sel.options.length; i++) {
        if (this.channels_[cha].c.name_ === channel_sel.options[i].value) {
          addToList(i, this.channels_[cha].a, this.channels_[cha].r);
          break;
        }
      }
    }
    // Interactive markers manage logic
    let marker_list_holder = document.createElement("ul");
    marker_list_holder.className += " channel-list";
    let markers_holder = createDiv();
    let marker_sel = document.createElement("select");
    {
      let opt = document.createElement("option");
      opt.value = "";
      opt.text = "Select marker";
      opt.disabled = true;
      marker_sel.add(opt);
      marker_sel.selectedIndex = 0;
    }
    const potentialMarkers = that.getPotentialMarkers();
    for (let i in potentialMarkers) {
      const name = potentialMarkers[i].rhs;
      let opt = document.createElement("option");
      opt.value = name;
      opt.text = name;
      opt.interactive = potentialMarkers[i].interactive;
      marker_sel.add(opt);
    }
    let addMarker = document.createElement("button");
    addMarker.style.display = "block";
    addMarker.appendChild(document.createTextNode("Add marker"));
    addMarker.addEventListener('click', function() {
      if (marker_sel.selectedIndex == 0) return;
      const interactivity = marker_sel.options[marker_sel.selectedIndex].interactive;
      addToMarkersList(marker_sel.selectedIndex, true, interactivity);
    });
    markers_holder.appendChild(marker_sel);
    markers_holder.appendChild(addMarker);
    append("", markers_holder);
    // Add current selected markers to list
    for (let marker in this.markers_channels_) {
      for (let i = 1; i < marker_sel.options.length; i++) {
        if (this.markers_channels_[marker].c === marker_sel.options[i].value) {
          addToMarkersList(i, this.markers_channels_[marker].a, this.markers_channels_[marker].i);
          break;
        }
      }
    }
    // Add the button to create the window.
    let create = document.createElement("button");
    create.appendChild(document.createTextNode("Update"));
    create.addEventListener('click', function() {
      let config = {};
      config.channels = [];
      config.markers = [];
      config.renderer = "3d";
      config.dims = {width: dims_width.value, height: dims_height.value};
      config.custom_frame = frame_lock.checked !== true;
      config.base_frame = frame_base_sel.options[frame_base_sel.selectedIndex].value;
      config.static_frame = frame_static_sel.options[frame_static_sel.selectedIndex].value;

      for (let i = 0; i < list_holder.children.length; i++) {
        config.channels.push(list_holder.children[i].conf);
      }

      for (let i = 0; i < marker_list_holder.children.length; i++) {
        config.markers.push({
          name: marker_list_holder.children[i].conf.name,
          zIndex: marker_list_holder.children[i].conf.z,
          active: marker_list_holder.children[i].conf.active,
          interactive: marker_list_holder.children[i].conf.interactive,
          config: marker_list_holder.children[i].conf.config
        });
      }

      that.parseFromConfig(config);
      that.hideConfig();
      // Add non-interactive markers to markers_widget
      for (let i in that.markers_channels_) {
        if (that.markers_channels_[i].i) continue;
        if (that.markersWidget_.containsMarker(that.markers_channels_[i].c)) continue;
        if (typeof that.base_referential_frame_ === 'undefined') continue;

        const matrix = PoseTree().get(that.base_referential_frame_, that.markers_channels_[i].c,
                                         that.current_time_);
        let trans = new THREE.Vector3();
        let quat = new THREE.Quaternion();
        let scale = new THREE.Vector3();
        matrix.decompose(trans, quat, scale);
        that.markersWidget_.addMarker(that.markers_channels_[i].c, trans, quat, false);
      }
      that.update(null);
    });
    append("", create);

    div.append(list_holder);
    div.append(marker_list_holder);
    return div;
  }

  /// private members

  // Set up the scene environment.
  sceneSetup_(name) {
    // basic 3JS utilities
    this.renderer_ = new THREE.WebGLRenderer({ canvas: this.canvas_, antialias: true });
    this.renderer_.shadowMap.enabled = true;
    this.renderer_.shadowMap.type = THREE.PCFSoftShadowMap;
    this.scene_ = new THREE.Scene();
    // Transform from Three.js coordinates system (ZXY) to Isaac coordinates system.
    let background_color = new THREE.Color(0x182944);
    this.scene_.background = background_color;
    this.scene_.fog = new THREE.Fog(background_color, 2, 100);
    this.canvas_ = this.renderer_.domElement;
    this.setCanvasDimensions(this.canvas_);

    // create env lights
    let ambient = new THREE.AmbientLight(0x555555);
    this.scene_.add(ambient);

    let directionalLight = new THREE.DirectionalLight(0xffffff);
    directionalLight.position.set(1, 0, 1);
    directionalLight.castShadow = true;
    directionalLight.shadowMapSoft = true;
    directionalLight.shadow.mapSize.width = 1024;
    directionalLight.shadow.mapSize.height = 1024
    directionalLight.shadow.camera.near = 0.5;
    directionalLight.shadow.camera.far = 500;
    directionalLight.shadow.soft = true;
    this.scene_.add(directionalLight);

    let spotLight = new THREE.SpotLight(0xffffff);
    spotLight.position.set(0, 0, 50);
    spotLight.castShadow = true;
    spotLight.shadowMapSoft = true;
    spotLight.shadow.mapSize.width = 4096;
    spotLight.shadow.mapSize.height = 4096;
    spotLight.shadow.camera.near = 0.5;
    spotLight.shadow.camera.far = 500;
    spotLight.shadow.soft = true;
    this.scene_.add(spotLight);

    let grid = new THREE.GridHelper(100.0, 10, new THREE.Color(0xbbbbbb), new THREE.Color(0xbbbbbb));
    grid.rotation.set(-Math.PI / 2, 0, 0);
    this.scene_.add(grid);

    this.raycaster_ = new THREE.Raycaster();
    if (typeof this.markersWidget_ !== 'undefined' && this.markersWidget_ !== null) {
      this.markersWidget_.reset();
    }
    this.markersWidget_ = Create3DMarkersWidget(this.scene_, this.camera_, this.renderer_);
    this.markersWidgetName_ = name + "-3D";
    InteractiveMarkersManager().addMarkersWidget(this.markersWidgetName_, {
      "reset": () => {
        this.markersWidget_.reset();
      },
      "enableInteraction": this.markersWidget_.enableInteraction,
      "disableInteraction": this.markersWidget_.disableInteraction,
      "set": this.markersWidget_.set,
      "containsMarker": this.markersWidget_.containsMarker,
      "addMarker": (channel, path, pathTransformTranslation, pathTransformOrientation) => {
        this.markersWidget_.addMarker(path, pathTransformTranslation,
            pathTransformOrientation, true);
      }
    }, this.base_referential_frame_);

    let cross = CreateIsaacAxes(2.0, 1.0);
    grid.rotation.set(-Math.PI / 2, 0, 0);
    this.scene_.add(cross);
  }

  // Draw an image on the canvas
  drawImage_(op) {
    // Add a small epsilon height to prevent map to be drawn on exactly the same plan.
    return CreateMap(op.img, 1.0, 20.0, this.channel_id_ * 1e-2 /* floor_z */);
  }

  // Renders a 3D point cloud
  drawPointCloud_(op) {
    let dim = op.dim;
    if (dim != 3) {
      // TODO: add 2d point cloud support.
      return;
    }

    let material = null;

    let geometry = new THREE.BufferGeometry();

    let coordinates = new Float32Array(Base64Binary.decodeArrayBuffer(op.points));
    geometry.addAttribute('position', new THREE.BufferAttribute(coordinates, 3));

    let colors = new Float32Array(Base64Binary.decodeArrayBuffer(op.colors));
    let has_colors = colors.length == coordinates.length;
    if (has_colors && !this.override_color) {
      material = new THREE.PointsMaterial({
          size: 2.5 * this.current_size_,
          vertexColors: THREE.VertexColors,
          sizeAttenuation: false,
          transparent: this.current_alpha_ < 1,
          opacity: this.current_alpha_
      });
      geometry.addAttribute('color', new THREE.BufferAttribute(colors, 3));
    } else {
      if (!op.cached_pt_material_) {
        op.cached_pt_material_ = new THREE.PointsMaterial({
            size: 2.5 * this.current_size_,
            color: this.current_color_,
            sizeAttenuation: false,
            transparent: this.current_alpha_ < 1,
            opacity: this.current_alpha_
        });
      }
      material = op.cached_pt_material_;
    }

    return new THREE.Points(geometry, material);
  }

  // Draw a circle
  drawCircle_(op) {
    if (!op.cached_mesh_material_) {
      op.cached_mesh_material_ = new THREE.MeshPhongMaterial({
          color: this.current_color_,
          wireframeLinewidth: this.current_size_,
          wireframe: !this.current_fill_,
          transparent: this.current_alpha_ < 1,
          opacity: this.current_alpha_
      });
    }
    const material = op.cached_mesh_material_;
    let sphere = new THREE.Mesh(this.cached_sphere_, material);
    sphere.scale.set(op.r, op.r, op.r);
    sphere.position.copy(this.getThreeJsPosition_(op.c));
    return sphere;
  }

  // Draw a line
  drawLine_(op) {
    if (!op.cached_line_material_) {
      op.cached_line_material_ = new THREE.LineBasicMaterial({
          color: this.current_color_,
          linewidth: this.current_size_,
          transparent: this.current_alpha_ < 1,
          opacity: this.current_alpha_
      });
    }
    const material = op.cached_line_material_;
    let geometry = new THREE.Geometry();
    for (let i = 0; i < op.p.length; i++) {
      geometry.vertices.push(this.getThreeJsPosition_(op.p[i]));
    }
    return new THREE.Line(geometry, material);
  }

  // Draw text
  drawText_(op) {}

  // Draw a point
  drawPoint_(op) {
    if (this.current_fill_) {
      return this.drawLine_(op);
    }
    if (!op.cached_pt_material_) {
      op.cached_pt_material_ = new THREE.PointsMaterial({
          size: 2.5 * this.current_size_,
          color: this.current_color_,
          sizeAttenuation: false,
          transparent: this.current_alpha_ < 1,
          opacity: this.current_alpha_
      });
    }
    const material = op.cached_pt_material_;
    let geometry = new THREE.Geometry();
    for (let i = 0; i < op.p.length; i++) {
      geometry.vertices.push(this.getThreeJsPosition_(op.p[i]));
    }
    return new THREE.Points(geometry, material);
  }

  // Draw a rectangle
  drawRect_(op) {
    if (!op.cached_mesh_material_) {
      op.cached_mesh_material_ = new THREE.MeshPhongMaterial({
          color: this.current_color_,
          wireframeLinewidth: this.current_size_,
          wireframe: !this.current_fill_,
          transparent: this.current_alpha_ < 1,
          opacity: this.current_alpha_
      });
    }
    const material = op.cached_mesh_material_;
    let cube = new THREE.Mesh(this.cached_box_, material);
    cube.scale.copy(this.getThreeJsScale_(op.a, op.b));
    cube.position.addVectors(this.getThreeJsPosition_(op.a), this.getThreeJsPosition_(op.b))
        .divideScalar(2);
    return cube;
  }

  // Draw Asset
  drawAsset_(op) {
    let name = this.current_channel_;
    if (name in this.asset_counter_) {
      this.asset_counter_[name]++;
    } else {
      this.asset_counter_[name] = 1;
    }
    let asset = AssetManager().getAsset3D(op.n, name + "/" + this.asset_counter_[name]);
    // If the material is just one color, then we apply the current color
    if (asset && asset.default_material && asset.default_material.isMeshPhongMaterial) {
      asset.default_material.copy(asset.backup_material);
      if (this.current_color_ && this.current_color_[0] == "#" && this.current_color_.length == 7) {
        asset.default_material.color.set(parseInt(this.current_color_.substr(1, 6), 16));
      }
    }
    return asset;
  }

  // Return position vector from array of numbers.
  // For 2 element array, use default z value. For 3 element array use provided z value.
  // Adjust coordinate system from z-up (input array) to y-up (returned THREE.Vector3)
  getThreeJsPosition_(position) {
    const x = position[0];
    const y = position[1];
    const z = position.length > 2 ? position[2] : this.channel_options_.default_z;
    return new THREE.Vector3(x, y, z);
  }

  // Return vector with absolute difference between each element of two arrays.
  // For 2 element arrays, set difference to 0. For 3 element array use provided z values.
  // Adjust coordinate system from z-up (input array) to y-up (returned THREE.Vector3)
  getThreeJsScale_(position_0, position_1) {
    const x = Math.abs(position_0[0] - position_1[0]);
    const y = Math.abs(position_0[1] - position_1[1]);
    const z = (position_0.length > 2 && position_1.length > 2) ?
        Math.abs(position_0[2] - position_1[2]) : 0.0;
    return new THREE.Vector3(x, y, z);
  }
}
