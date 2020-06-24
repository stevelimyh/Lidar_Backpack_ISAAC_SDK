/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// 2D Renderer, implement a SightRenderer.
class Renderer2D extends SightRenderer {
  constructor(name, new_renderer = true) {
    super(name, new_renderer);
    // Zoom
    this.zoom = 1.0;

    // Canvas we pre-render
    this.render_canvas_ = document.createElement('canvas');
    this.render_canvas_.width = this.canvas_.width;
    this.render_canvas_.height = this.canvas_.height;

    this.markers_canvas_ = document.createElement('canvas');

    // Whether showing the tooltip
    this.show_tooltip_ = false;
    let binder = this;
    this.menu_.addItem({
      text: "Show pixel information",
      callback: function() {
        if (binder.show_tooltip_) {
          binder.show_tooltip_ = false;
          this.updateTitle("Show pixel information");
        } else {
          binder.show_tooltip_ = true;
          this.updateTitle("Hide pixel information");
        }
      }
    });

    // Default style
    this.current_color_ = "#000";
    this.override_color = false;
    this.current_fill_ = false;
    this.current_size_ = 1;
    this.current_transformation_ = new THREE.Matrix4();
    this.current_opacity_ = 1.0;
    this.projection_ = new THREE.Matrix4();
    this.loaded_ = true;

    // Whether the dimensions are custom set by the user or automatically created
    this.custom_dims_ = false;

    // Handle the selected markers logic.
    this.clickIn_ = false;
    this.clickOut_ = false;
    this.selectedMarker_ = undefined;
    this.hoveredMarker_ = undefined;

    try {
      this.markersWidget_ = Create2DMarkersWidget();
      // Ugly hack: 2d renderer needs an update to be scheduled in order to render again.
      let originalSet = this.markersWidget_.set;
      this.markersWidget_.set = (transformations) => {
        this.update(null);
        originalSet(transformations);
      };

      this.markersWidgetName_ = name + "-2D";
      InteractiveMarkersManager().addMarkersWidget(this.markersWidgetName_, {
        "reset": () => {
          this.markersWidget_.reset();
        },
        "set": this.markersWidget_.set,
        "enableInteraction": this.markersWidget_.enableInteraction,
        "disableInteraction": this.markersWidget_.disableInteraction,
        "containsMarker": this.markersWidget_.containsMarker,
        "addMarker": (channel, path, pathTransformTranslation, pathTransformOrientation) => {
          this.markersWidget_.addMarker(path, pathTransformTranslation, pathTransformOrientation);
        },
      }, this.base_referential_frame_);

      this.cursorPositionCanvas_ = undefined;
      this.onCanvasMouseMove = (event) => {
        this.cursorPositionCanvas_ = new THREE.Vector2(event.offsetX, event.offsetY);
      }

      this.markersWidgetTransformationMode_ = undefined;
      this.onCanvasMouseDown = (event) => {
        if (!this.markersWidgetMouseTransformationMode_) {
          switch (event.button) {
            case 0: { // left
              this.markersWidgetMouseTransformationMode_ =
                  this.markersWidget_.desiredTransformationMode();
              this.clickIn_ = true;
              this.clickOut_ = false;
              break;
            }
          }
        }

      };
      this.onDocumentMouseUp = (event) => {
        switch(event.button) {
          case 0: { // left
            this.markersWidgetMouseTransformationMode_ = undefined;
            this.clickOut_ = true;
            this.clickIn_ = false;
            break;
          }
        }
      };

      this.keyboardInputState_ = {};
      this.onWindowKeyDown = (event) => {
        if (!event.repeat) {
          this.keyboardInputState_[event.code] = true;
        }
      };
      this.onWindowKeyUp = (event) => {
        if (!event.repeat) {
          this.keyboardInputState_[event.code] = false;
        }
      };
    } catch (e) {
      console.error(e);
    }

    this.canvas_.addEventListener('mousemove', this.onCanvasMouseMove, false);
    this.canvas_.addEventListener('mousedown', this.onCanvasMouseDown, false);
    document.addEventListener('mouseup', this.onDocumentMouseUp, false);
    window.addEventListener('keydown', this.onWindowKeyDown, false);
    window.addEventListener('keyup', this.onWindowKeyUp, false);
  }

  // TODO: call this somewhere?
  unregisterGlobalEventListeners() {
    this.canvas_.removeEventListener('mousemove', this.onCanvasMouseMove, false);
    this.canvas_.removeEventListener('mousedown', this.onCanvasMouseDown, false);
    document.removeEventListener('mouseup', this.onDocumentMouseUp, false);
    window.removeEventListener('keydown', this.onWindowKeyDown, false);
    window.removeEventListener('keyup', this.onWindowKeyUp, false);
  }


  // Returns the default label
  getLabelTitle() {  // override
    return "2D Drawing";
  }

  // Returns the Renderer type
  getRendererType() {
    return "2d";
  }

  // Get the data of the image
  getDataImage(type) {
    return this.render_canvas_.toDataURL(type);
  }

  // Default special config
  mergeSpecialConfig(config) {
    config.custom_dimensions = this.custom_dims_;
  }

  // Default special config
  parseSpecialConfig(config) {
    this.custom_dims_ = config.custom_dimensions === true;
    if (config.zoom) this.zoom = config.zoom;
  }

  // Get the canvas with the rendering
  getCanvas() {
    return this.render_canvas_;
  }

  applyInput() {
    if (typeof this.base_referential_frame_ === 'undefined') return;
    let static_T_base =
        PoseTree().get(this.static_frame_, this.base_referential_frame_, this.current_time_);
    if (!static_T_base || !this.baseReferentialFrame_T_current_) return;

    const canvas_T_static = new THREE.Matrix4().getInverse(
        this.baseReferentialFrame_T_current_.clone().multiply(this.current_transformation_)
                                                    .multiply(static_T_base));
    let cursorRayDirectionBaseReferentialFrame = undefined;
    let cursorRayPointBaseReferentialFrame = undefined;
    if (this.cursorPositionCanvas_) {
      // Cursor point in the space referred to as "the current coordinate system", namely the space
      // of points output by transformPoint_.
      const cursorPointCurrent =
            new THREE.Vector2(
              this.cursorPositionCanvas_.y/this.zoom,
              this.cursorPositionCanvas_.x/this.zoom
            );

      // Cursor ray start in homogeneous base referential frame coordinates.
      let cursorRayStartBaseReferentialFrameHomogeneous =
          new THREE.Vector4(cursorPointCurrent.x, cursorPointCurrent.y, 0.0, 1.0).applyMatrix4(
              canvas_T_static);
      // Cursor ray end in homogeneous base referential frame coordinates.
      let cursorRayEndBaseReferentialFrameHomogeneous =
          new THREE.Vector4(cursorPointCurrent.x, cursorPointCurrent.y, -1, 1.0).applyMatrix4(
              canvas_T_static);

      // Cursor ray position in base referential frame coordinates.
      cursorRayPointBaseReferentialFrame =
        new THREE.Vector3(
          cursorRayStartBaseReferentialFrameHomogeneous.x/
              cursorRayStartBaseReferentialFrameHomogeneous.w,
          cursorRayStartBaseReferentialFrameHomogeneous.y/
              cursorRayStartBaseReferentialFrameHomogeneous.w,
          cursorRayStartBaseReferentialFrameHomogeneous.z/
              cursorRayStartBaseReferentialFrameHomogeneous.w
        );
      // Cursor ray end point in base referential frame coordinates.
      let cursorRayEndPointBaseReferentialFrame =
          new THREE.Vector3(
            cursorRayEndBaseReferentialFrameHomogeneous.x/
                cursorRayEndBaseReferentialFrameHomogeneous.w,
            cursorRayEndBaseReferentialFrameHomogeneous.y/
                cursorRayEndBaseReferentialFrameHomogeneous.w,
            cursorRayEndBaseReferentialFrameHomogeneous.z/
                cursorRayEndBaseReferentialFrameHomogeneous.w
          );
      // Cursor ray direction in base referential frame coordinates
      cursorRayDirectionBaseReferentialFrame = cursorRayEndPointBaseReferentialFrame.clone();
      cursorRayDirectionBaseReferentialFrame.sub(cursorRayPointBaseReferentialFrame);
    }

    this.hoveredMarker_ = undefined;
    let desiredTransformationMode = undefined;

    // Update the status of the markers labels
    for (let i in this.markers_channels_) {
      try {
        const markerName = this.markers_channels_[i].c;
        let marker = InteractiveMarkersManager().getMarker(markerName);
        if (marker !== null) {
          this.updateMarkerLabel(markerName, 'enable');
        } else {
          this.updateMarkerLabel(markerName, 'invalid');
        }
      } catch (e) {
        console.error(e);
      }
    }

    if (this.cursorPositionCanvas_) {
      for (let i in this.markers_channels_) {
        const channel = this.markers_channels_[i];
        const name = channel.c;
        if (!channel.a) {
          continue;
        }
        let transformBaseReferentialFrame;
        if (channel.i) {
          if (!this.markersWidget_.containsMarker(name)) continue;
          transformBaseReferentialFrame = this.markersWidget_.transform(name);
        } else {
          // Since it's a non itreactive marker I need to query the PoseTree (It's not registered
          // in the markersWidget)
          if (typeof this.base_referential_frame_ === 'undefined') continue;
          const matrix = PoseTree().get(this.base_referential_frame_, name, this.current_time_);
          if (matrix == null) continue;
          let trans = new THREE.Vector3();
          let quat = new THREE.Quaternion();
          let scale = new THREE.Vector3();
          matrix.decompose(trans, quat, scale);
          transformBaseReferentialFrame = {
            position: trans,
            orientation: quat
          };
        }
        let scaleFactor = 1.0;
        if (typeof channel.config.size !== 'undefined' && channel.config.size !== null) {
          scaleFactor = channel.config.size;
        }
        let geometry = this.markerGeometryCanvas_(transformBaseReferentialFrame,
                                                  static_T_base, scaleFactor);
        let positionCanvas = geometry.centerCanvas;

        let cursorDistanceSquaredCanvas =
            Math.pow(this.cursorPositionCanvas_.x - positionCanvas.x, 2) +
            Math.pow(this.cursorPositionCanvas_.y - positionCanvas.y, 2);
        const kRelativeHitRegionWidth = 0.4; // Should be < 0.5
        const innerRadiusCanvas = this.markerRadiusCanvas_() * scaleFactor *
            (1 - kRelativeHitRegionWidth);
        const outerRadiusCanvas = this.markerRadiusCanvas_() * scaleFactor *
            (1 + kRelativeHitRegionWidth);
        const innerRadiusCanvasSquared = Math.pow(innerRadiusCanvas, 2);
        const outerRadiusCanvasSquared = Math.pow(outerRadiusCanvas, 2);
        // Distance from axis (line segment) defined by 'a' and 'b' to the point 'p'.
        // More precisely, this calculates min {distance(x, p) | x is on line segment }
        const axisDistance = (a, b, p) => {
          const aq = p.clone().sub(a);
          // Line segment vector
          const ab = b.clone().sub(a);
          // Projected distance of point along the line segment, measured from 'a' along the line
          // segment vector, 'ab'.
          const qx = aq.dot(ab)/ab.length();
          if (qx < 0) {
            // Projected distance is smaller than that of end point 'a', so the answer is the
            // distance from 'p' to 'a'.
            return p.distanceTo(a);
          } else if (qx > ab.length()) {
            // Projected distance is larger than that of end point 'b', so the answer is the
            // distance from 'p' to 'b'.
            return p.distanceTo(b);
          } else {
            // Projected distance is between that of end points 'a' and 'b', so the answer is the
            // perpendicular distance from the line segment to 'p'.
            const qPerp = aq.clone().addScaledVector(ab, -qx/ab.length());
            return qPerp.length();
          }
        };
        const axisDistances = [
          axisDistance(
            geometry.centerCanvas, geometry.xArrowEndCanvas,
            this.cursorPositionCanvas_
          ),
          axisDistance(
            geometry.centerCanvas, geometry.axisEndCanvas[1],
            this.cursorPositionCanvas_
          ),
        ];
        const hitAxis =
            (axisDistances[0] < this.markerRadiusCanvas_()*scaleFactor*kRelativeHitRegionWidth) ||
            (axisDistances[1] < this.markerRadiusCanvas_()*scaleFactor*kRelativeHitRegionWidth);
        const hitRing =
              cursorDistanceSquaredCanvas > innerRadiusCanvasSquared &&
                  cursorDistanceSquaredCanvas < outerRadiusCanvasSquared;
        if (hitAxis) {
          this.hoveredMarker_ = name;
          desiredTransformationMode = "translation";
        } else if (hitRing) {
          this.hoveredMarker_ = name;
          desiredTransformationMode = "orientation";
        }
      }

      // Check if we have a possible select or unselect
      if (this.clickIn_) {
        // This is an inmediate selection
        if (this.selectedMarker_ === undefined && this.hoveredMarker_ !== undefined) {
          const channel = this.markers_channels_.find(ch => ch.c === this.hoveredMarker_);
          if (channel.i) {
            this.selectedMarker_ = this.hoveredMarker_;
          }
        }
        // Mark the click as procesed
        this.clickIn_ = false;
        this.clickOut_ = false;
      }

      if (this.clickOut_) {
        if (this.selectedMarker_ === undefined && this.hoveredMarker_ !== undefined) {
          const channel = this.markers_channels_.find(ch => ch.c === this.hoveredMarker_);
          if (channel.i) {
            // Make a new  normal selection
            this.selectedMarker_ = this.hoveredMarker_;
          }
        }
        if (this.selectedMarker_ != this.hoveredMarker_ && this.hoveredMarker_ !== undefined
              && !this.markersWidget_.orientationGuideLineSegment()) {
          const channel = this.markers_channels_.find(ch => ch.c === this.hoveredMarker_);
          if (channel.i) {
            // Change the current selection
            this.selectedMarker_ = this.hoveredMarker_;
          }
        }
        if (this.selectedMarker_ !== undefined && this.hoveredMarker_ === undefined
              && !this.markersWidget_.orientationGuideLineSegment()) {
          // Unselect
          this.selectedMarker_ = undefined;
        }
        // Mark the click as procesed
        this.clickIn_ = false;
        this.clickOut_ = false;
      }

    }

    this.markersWidget_.update(
      this.hoveredMarker_,
      this.selectedMarker_,
      desiredTransformationMode,
      cursorRayPointBaseReferentialFrame,
      cursorRayDirectionBaseReferentialFrame,
      this.markersWidgetMouseTransformationMode_
    );
  }

  feedBackActiveMarkers() {
    for (let name of this.markersWidget_.markers()) {
      try {
        if (this.markersWidget_.activeInteraction(name)) {
          // Get the current (as the user is editing) pose
          let v = this.markersWidget_.transform(name).position;
          let q = this.markersWidget_.transform(name).orientation;
          // We have path which is basically baseReferentialFrame_T_rhs, we need to update the
          // PoseTree with lhs_T_rhs, so the actual new pose is
          // (baseReferentialFrame_T_lhs)^-1 * (baseReferentialFrame_T_rhs) =
          // (lhs_T_baseReferentialFrame) * (baseReferentialFrame_T_rhs) = lhs_T_rhs
          const baseReferentialFrame_T_rhs = new THREE.Matrix4().makeRotationFromQuaternion(q)
                .setPosition(v);
          let marker = InteractiveMarkersManager().getMarker(name);
          let lhs = marker.lhs;
          let rhs = marker.rhs;
          if (!PoseTree().areConnected(lhs, this.base_referential_frame_)) continue;
          const lhs_T_baseReferentialFrame = PoseTree().get(lhs, this.base_referential_frame_,
                                                            this.current_time_);
          const lhs_T_rhs =
                new THREE.Matrix4().multiplyMatrices(
                  lhs_T_baseReferentialFrame,
                  baseReferentialFrame_T_rhs
                );
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
      } catch (e) {
        console.error(e);
      }
    }
  }

  updateInactiveMarkers() {
    let updatedMarkers = InteractiveMarkersManager().getMarkers(this.base_referential_frame_);
    for (let path in updatedMarkers) {
      try {
        if (this.markersWidget_.interactionEnabled(path)) {
          // Disable interaction.
          if (this.markersWidget_.activeInteraction(path)) {
            delete updatedMarkers[path];
          } else {
            // User is not interacting at the moment. Disable interaction.
            // This is also not great UX: the user may be on their way to interact with the widget
            // and we just move it.
            // Works for now, though.
            this.markersWidget_.disableInteraction(path);
          }
        }
      } catch (e) {
        console.error(e);
      }
    }

    try {
      this.markersWidget_.set(updatedMarkers);
      for (let path in updatedMarkers) {
        this.markersWidget_.enableInteraction(path);
      }
    } catch (e) {
      console.error(e);
    }
  }

  postApplyInput() {
    this.feedBackActiveMarkers();
  }

  preRenderFrame() {
    this.updateInactiveMarkers();
  }

  // Request an animation from the WebGL Renderer
  annimate() {}

  // Create a div to edit the config
  getLabelEdit(config) {
    const binder = this;
    let div = document.createElement('div');
    div.className = "label-edit"
    // Add slider for opacity
    let opacity_group = document.createElement('label');
    opacity_group.appendChild(document.createTextNode("Opacity: "));
    let opacity_holder = document.createElement('div');
    opacity_holder.className = "edit-slider";
    let opacity = document.createElement('input');
    opacity.type = "range";
    opacity.className = "slider-2d-rendering";
    opacity.min = 1;
    opacity.max = 255;
    opacity.value = ((config.opacity || 1.0) * 255) | 0;
    opacity_holder.appendChild(opacity)
    opacity_group.appendChild(opacity_holder);
    let opacity_text = document.createElement('div');
    opacity_text.innerHTML = opacity.value;
    opacity_group.appendChild(opacity_text);
    opacity.oninput = function() {
      config.opacity = parseFloat(this.value) / 255.0;
      opacity_text.innerHTML = this.value;
      binder.update_scheduled_ = true;
    }

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
    size_group.appendChild(document.createTextNode("Default size: "));
    let size_holder = document.createElement('div');
    size_holder.className = "edit-slider";
    let size = document.createElement('input');
    size.type = "range";
    size.className = "slider-2d-rendering";
    size.min = 1;
    size.max = 200;
    size.value = (20 * (config.size || 1.0)) | 0;
    size_holder.appendChild(size)
    size_group.appendChild(size_holder);
    let size_text = document.createElement('div');
    size_text.innerHTML = parseFloat(size.value) / 20.0;
    size_group.appendChild(size_text);
    size.oninput = function() {
      config.size = parseFloat(this.value) / 20.0;
      size_text.innerHTML = config.size;
      binder.update_scheduled_ = true;
    }

    div.appendChild(opacity_group);
    div.appendChild(color_group);
    div.appendChild(size_group);
    return div;
  }

  // Render the list of operations using a callback function
  render(op, time) {
    const backup_size = this.current_size_;
    const backup_color = this.current_color_;
    const backup_fill = this.current_fill_;
    const backup_scale = this.current_scale;
    const backup_projection = this.projection_.clone();
    const backup_transformation = this.current_transformation_.clone();
    const backup_alpha = this.ctx_.globalAlpha;
    if (op.s) {
      if (op.s.a) this.ctx_.globalAlpha *= op.s.a;
      if (op.s.c && !this.override_color) this.current_color_ = op.s.c;
      if (op.s.f) this.current_fill_ = op.s.f;
      if (op.s.s) this.current_size_ = op.s.s * this.current_size_factor_;
    }
    if (op.p) {
      // Check whether the cavans frame is selected or the world from.
      if (op.p.t === "c") {
        this.current_scale = 1.0;
        this.projection_ = new THREE.Matrix4();
        this.projection_.elements[10] = 0;
        this.current_transformation_ = SightChannel.GetTransform(op, this.static_frame_, time);
      } else {
        if (op.p.t === "f") {
          this.resetCameraProjection();
        }
        const scale = op.p.s || 1.0;
        this.current_scale *= scale;
        const mat = SightChannel.GetTransform(op, this.static_frame_, time);
        if (mat != null) {
          this.projection_.multiply(mat);
          this.current_transformation_.multiply(mat);
        } else {
          this.current_transformation_ = null;
        }
      }
    }
    if (this.current_transformation_) {
      if (op.t === "sop") {
        for (let i in op.d) {
          this.render(op.d[i], time);
        }
      } else if (op.t == "img") {
        this.drawImage_(op);
      } else if (op.t == "point_cloud") {
        this.drawPointCloud_(op);
      } else if (op.t == "line") {  // line
        this.drawLine_(op);
      } else if (op.t == "sphr") {  // sphere (aka circle)
        this.drawCircle_(op);
      } else if (op.t == "text") {  // text
        this.drawText_(op);
      } else if (op.t == "cube") {  // cuboid (aka rectangle)
        this.drawRect_(op);
      } else if (op.t == "pnts") {  // point
        this.drawPoint_(op);
      } else if (op.t == "asset") {  // asset
        this.drawAsset_(op);
      }
    }
    this.current_fill_ = backup_fill;
    this.current_color_ = backup_color;
    this.projection_ = backup_projection;
    this.current_transformation_ = backup_transformation;
    this.current_scale = backup_scale;
    this.current_size_ = backup_size;
    this.ctx_.globalAlpha = backup_alpha;
  }

  // Reset the camera projections to the base channel transformation.
  resetCameraProjection() {
    let baseTransformation =
        this.channels_[0].c.getTransform(this.static_frame_, this.current_time_);
    if (baseTransformation == null) {
      throw "Cannot get the base transform of " + this.name_;
    }
    this.current_transformation_.getInverse(baseTransformation);
    this.projection_ = this.channels_[0].c.get2DProjection().multiply(this.current_transformation_);
    // TODO:
    // The inverse of getInvertible2DTransform is in all current and forseeable cases is trivial to
    // write down directly.
    // No need to calculate the general matrix inverse here.
    // Instead make channel API expose getInvertible2DTransformInverse that just returns the inverse
    // directly and then use that instead.
    // The so-called "current" space gets it's name from the transformPoint_ function, which is
    // documented to output points in a space named "the current coordinate system".
    this.baseReferentialFrame_T_current_ =
        new THREE.Matrix4().getInverse(this.channels_[0].c.getInvertible2DTransform());
    this.current_scale = 1.0 / this.channels_[0].c.getScale();
  }

  // Reset the base referential frame
  resetBaseFrame() {  // override
    this.base_referential_frame_ = this.channels_[0].c.getFrame();
  }

  // Runs just before starting the rendering
  preRendering() {  // override:
    this.resetCameraProjection();
    this.projection_matrix = this.channels_[0].c.get2DProjection();
    this.ctx_ = this.render_canvas_.getContext("2d");
    this.ctx_.save();
    this.clear_();
  }

  // Runs just after the rendering
  postRendering() {  // override:
    this.ctx_.restore();
    this.draw_();
    delete this.current_channels;
  }

  // Call before rendering the channel
  preChannelRendering(channel_id) {  // override:
    this.current_channels = channel_id;
    const config = this.channels_[channel_id].config;
    this.current_size_factor_ = config.size || 1;
    if (config.color) {
      this.current_color_ = config.color;
      this.override_color = true;
    } else {
      this.current_color_ = "#000";
      this.override_color = false;
    }
    this.current_fill_ = config.fill || false;
    this.current_size_ = config.size || 1;
    this.ctx_.globalAlpha = config.opacity || 1.0;
  }

  // Returns the div containing the config
  getConfigDiv() {
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
    let dims_lock = document.createElement("input");
    dims_width.type = "number";
    dims_height.type = "number"
    dims_lock.type = "checkbox";
    dims_lock.checked = this.custom_dims_ === true;
    let dims_lock_label = document.createElement("label");
    dims_lock_label.style.display = "block";
    dims_lock_label.appendChild(dims_lock);
    dims_lock_label.appendChild(document.createTextNode("Automatic"));
    dims_lock_label.style.fontWeight = "normal";
    dims_lock_label.style.color = "#7f7f7f";
    dims_width.value = this.render_canvas_.width;
    dims_height.value = this.render_canvas_.height;
    dims_holder.appendChild(dims_height);
    dims_holder.appendChild(document.createTextNode(" x "));
    dims_holder.appendChild(dims_width);
    dims_holder.appendChild(dims_lock_label);
    dims_lock.addEventListener("click", function() {
      dims_width.disabled = dims_lock.checked;
      dims_height.disabled = dims_lock.checked;
    });
    dims_lock.click();
    append("Window dimensions", dims_holder);

    // Create the Frame of the window component
    let frame_holder = createDiv();
    let frame_static_sel = document.createElement("select");
    let frame_names = PoseTree().getNodeNames();
    let idx = 0;
    for (let name in frame_names) {
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
      frame_static_sel.disabled = frame_lock.checked;
    });
    frame_lock.click();
    frame_static_sel.style.display = "block";
    frame_lock_label.style.display = "block";
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

    // Then add the interactive markers
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
      config.renderer = "2d";
      config.dims = {width: dims_width.value, height: dims_height.value};
      config.custom_dimensions = dims_lock.checked !== true;
      config.custom_frame = frame_lock.checked !== true;
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
      that.update(null);
    });

    append("", create);

    div.append(list_holder);
    div.append(marker_list_holder);
    return div;
  }

  /// private methods

  // Handle mouse event
  mousewheel_(evt) {
    let delta = evt.detail ? evt.detail*(-120) : evt.wheelDelta;
    if (delta < 0) {
      if (this.zoom > 0.05) {
        this.zoom /= 1.1;
      }
    } else {
      if (this.zoom < 20) {
        this.zoom *= 1.1;
      }
    }
    this.draw_();
    if (evt.preventDefault) {
      evt.preventDefault();
    } else {
      return false;
    }
  };

  // Create if needed and return the tooltip div
  getTooltip_() {
    if (!this.tooltip_) {
      this.tooltip_ = document.createElement('div');
      this.tooltip_.className = 'smoothie-chart-tooltip';
      this.tooltip_.style.position = 'absolute';
      this.tooltip_.style.display = 'none';
      this.tooltip_.style.backgroundColor = '#FFFFFF7F';
      document.body.appendChild(this.tooltip_);
    }
    this.tooltip_.style.zIndex = this.canvas_.parentNode.style.zIndex + 1;
    this.tooltip_.style.zIndex = 100000000000;
    return this.tooltip_;
  };

  // Handle mouse over
  mousemove_(evt) {
    if (!this.show_tooltip_) return;
    let tooltip = this.getTooltip_();
    let x = ((evt.offsetX-1) / this.zoom)|0;
    let y = ((evt.offsetY-1) / this.zoom)|0;
    tooltip.innerHTML = "Pos: (" + x + "," + y + ")<br />";
    let color = this.render_canvas_.getContext('2d').getImageData(x, y, 1, 1).data;
    tooltip.innerHTML += "Color: (" + color[0] + "," + color[1] + "," + color[2] + ")";
    tooltip.style.top = Math.round(evt.pageY - tooltip.offsetHeight / 2) + 'px';
    tooltip.style.left = Math.round(evt.pageX + 10) + 'px';
    this.tooltip_.style.display = 'block';
  };

  // Handle mouse over
  mouseout_(evt) {
    if (!this.show_tooltip_) return;
    if (this.tooltip_) {
      this.tooltip_.style.display = 'none';
    }
  };

  // Transform a point 2D into the current coordinate system
  transformPoint_(x, y, z) {
    let pt = new THREE.Vector4(x, y, z, 1.0).applyMatrix4(this.projection_);

    // Check if the projection is behind the projected plan (only possible for non orthographic
    // projection), if that's the case, we do not render it.
    if (pt.w < 0.01) {
      return {}
    }
    return {x: pt.x / pt.w, y: pt.y / pt.w, z: pt.z / pt.w};
  }

  // Transform a point from the "base referential frame" space, which corresponds to the node with
  // name this.base_referential_frame_, into canvas space.
  baseReferentialFrameToCanvas_(pBaseReferentialFrame) {
    const pCurrent =
          this.transformPoint_(
            pBaseReferentialFrame.x,
            pBaseReferentialFrame.y,
            pBaseReferentialFrame.z
          );
    const pCanvas = new THREE.Vector2(pCurrent.y*this.zoom, pCurrent.x*this.zoom);
    return pCanvas;
  }

  markerRadiusCanvas_() {
    // This assumes that the markers are circles on the canvas.
    // That's not true in general -- it's possible to have a pinhole projection.
    // However, this function is only used for hit testing during input, which is only used/useable
    // when there's orthographic projection.
    const radiusBaseReferentialFrame = 0.5;

    const centerBaseReferentialFrame = new THREE.Vector3(0, 0, 0);
    const pBaseReferentialFrame = new THREE.Vector3(radiusBaseReferentialFrame, 0, 0);

    const centerCanvas = this.baseReferentialFrameToCanvas_(centerBaseReferentialFrame);
    const pCanvas = this.baseReferentialFrameToCanvas_(pBaseReferentialFrame);

    return Math.sqrt(Math.pow(centerCanvas.x - pCanvas.x, 2) +
               Math.pow(centerCanvas.y - pCanvas.y, 2));
  }

  // Draw the render_canvas into the displayed canvas
  draw_() {
    this.canvas_.width = this.zoom * this.render_canvas_.width;
    this.canvas_.height = this.zoom * this.render_canvas_.height;
    let ctx = this.canvas_.getContext("2d");
    ctx.save();
    ctx.scale(this.zoom, this.zoom);
    ctx.drawImage(this.render_canvas_, 0, 0);
    ctx.restore();

    // Draw the markers on top
    let static_T_base =
        PoseTree().get(this.static_frame_, this.base_referential_frame_, this.current_time_);
    if (static_T_base) {
      this.markers_canvas_.width = this.canvas_.width;
      this.markers_canvas_.height = this.canvas_.height;
      let markers_ctx = this.markers_canvas_.getContext("2d");
      markers_ctx.clearRect(0, 0, this.markers_canvas_.width, this.markers_canvas_.height);
      this.drawMarkersImage_(markers_ctx, static_T_base);
      ctx.drawImage(this.markers_canvas_, 0, 0);
    }
  };

  // Draw an image on the canvas
  drawImage_(op, identity = false) {
    if (this.current_channels == 0 && this.custom_dims_ === false) this.resize_(op.img);
    if (!identity) {
      const s = this.current_scale;
      const pose = Renderer2D.GetPose2(this.current_transformation_, this.projection_matrix);
      this.ctx_.save();
      this.ctx_.translate(pose.x, pose.y);
      this.ctx_.rotate(pose.angle);
      this.ctx_.scale(s, s);
    }
    if (op.img.center) {
      this.ctx_.translate(-op.img.width / 2, -op.img.height / 2);
    }
    this.ctx_.drawImage(op.img, 0, 0);
    if (op.img.center) {
      this.ctx_.translate(op.img.width / 2, op.img.height / 2);
    }
    if (!identity) {
      this.ctx_.restore();
    }
  }

  // Resizes the canvas
  resize_(op) {
    if (this.render_canvas_.width != op.width || this.render_canvas_.height != op.height) {
      this.render_canvas_.width = op.width;
      this.render_canvas_.height = op.height;
    }
  }

  // Clears the screen with the saved image or the provided color or white color
  clear_(color = null) {
    this.ctx_.clearRect(0, 0, this.render_canvas_.width, this.render_canvas_.height);
    if (color === undefined || color == null) {
      color = 'rgba(0, 0, 0, 255)';
    }
    this.ctx_.fillStyle = color;
    this.ctx_.fillRect(0, 0, this.render_canvas_.width, this.render_canvas_.height);
  }

  // Draw a circle
  drawCircle_(op) {
    const pt = this.transformPoint_(op.c[0], op.c[1], op.c[2] || 0);
    if (pt.x === undefined || pt.y === undefined) return;
    this.ctx_.beginPath();
    if (Renderer2D.IsPerspectiveProjection(this.projection_matrix)) {
      let ptx = this.transformPoint_(op.c[0] + op.r, op.c[1], op.c[2] || 0);
      this.ctx_.moveTo(ptx.y, ptx.x);
      for (let i = 0.0; i <= 2.0 * Math.PI; i += Math.PI * 0.05) {
        ptx = this.transformPoint_(op.c[0] + op.r * Math.cos(i), op.c[1] + op.r * Math.sin(i),
                                   op.c[2] || 0);
        this.ctx_.lineTo(ptx.y, ptx.x);
      }
    } else {
      let r = op.r;
      r *= this.current_scale;
      this.ctx_.arc(pt.y, pt.x, r, 0, 2*Math.PI);
    }
    this.ctx_.closePath();
    this.ctx_.lineWidth = this.current_size_ / this.zoom;
    if (this.current_fill_) {
      this.ctx_.fillStyle = this.current_color_;
      this.ctx_.fill();
    } else {
      this.ctx_.strokeStyle = this.current_color_;
      this.ctx_.stroke();
    }
  }

  // Draw text
  drawText_(op) {
    this.ctx_.font = this.current_size_ + "px Arial";
    this.ctx_.fillStyle = this.current_color_;
    const pt = this.transformPoint_(op.p[0], op.p[1], op.p[2] || 0);
    if (pt.x === undefined || pt.y === undefined) return;
    this.ctx_.fillText(op.text, pt.y, pt.x + this.current_size_);
  }

  // Draw a point cloud
  drawPointCloud_(op) {
    const kSmallestRadiusToDrawCircle = 2;

    function make_color(r, g, b) {
      return 'rgba(' + r + ', ' + g + ', ' + b + ', 255)';
    }

    function decode_coordinates(base64_string, dim) {
      let buffer = Base64Binary.decodeArrayBuffer(base64_string);  // ArrayBuffer
      let num_bytes = buffer.byteLength;
      let bytes_per_coordinate = dim * 4;  // (x, y) with 4 bytes per float
      let aligment = num_bytes % bytes_per_coordinate;
      if (aligment) {
        // remove last extra bytes, added by encoder for aligment
        buffer = buffer.slice(0, num_bytes - aligment);
      }
      return new Float32Array(buffer)
    }

    let dim = op.dim;
    let coordinates = decode_coordinates(op.points, dim);
    let colors = new Float32Array(Base64Binary.decodeArrayBuffer(op.colors));
    let n_points = coordinates.length / dim;
    let has_colors = colors.length / 3 == n_points;

    const r = this.current_size_ / this.zoom;

    if (!has_colors) {
      this.ctx_.fillStyle = this.current_color_;
    }

    for (let i = 0; i < n_points; i++) {
      const pt = this.transformPoint_(coordinates[dim * i + 0], coordinates[dim * i + 1], 0);
      if (pt.x === undefined || pt.y === undefined) continue;
      if (has_colors) {
        this.ctx_.fillStyle = make_color(colors[3 * i + 0], colors[3 * i + 1], colors[3 * i + 2]);
      }
      this.ctx_.beginPath();
      if (r < kSmallestRadiusToDrawCircle) {
        const half_r = 0.5 * r;
        this.ctx_.fillRect(pt.y - half_r, pt.x - half_r, r, r);
      } else {
        this.ctx_.arc(pt.y, pt.x, r, 0, 2 * Math.PI);
      }
      this.ctx_.closePath();
      this.ctx_.fill();
    }
  }

  // Draw a line
  drawLine_(op) {
    this.ctx_.beginPath();
    this.ctx_.lineWidth = this.current_size_ / this.zoom;
    const pt0 = this.transformPoint_(op.p[0][0], op.p[0][1], op.p[0][2] || 0);
    let draw = false;
    if (pt0.x !== undefined && pt0.y !== undefined) {
      this.ctx_.moveTo(pt0.y, pt0.x);
      draw = true;
    }
    for (let i = 1; i < op.p.length; i++) {
      const pt = this.transformPoint_(op.p[i][0], op.p[i][1], op.p[i][2] || 0);
      if (pt.x && pt.y) {
        if (draw) {
          this.ctx_.lineTo(pt.y, pt.x);
        } else {
          this.ctx_.moveTo(pt.y, pt.x);
          draw = true;
        }

      } else {
        draw = false;
      }
    }
    if (this.current_fill_) {
      this.ctx_.fillStyle = this.current_color_;
      this.ctx_.fill();
    } else {
      this.ctx_.strokeStyle = this.current_color_;
      this.ctx_.stroke();
    }
  }

  // Draw a point
  drawPoint_(op) {
    if (this.current_fill_) {
      // Disable fill to draw lines
      this.current_fill_ = false;
      this.drawLine_(op);
      this.current_fill_ = true;
      return;
    }
    const size = this.current_size_ / this.zoom;
    const half_size = 0.5 * size;
    for (let i = 0; i < op.p.length; i++) {
      const pt = this.transformPoint_(op.p[i][0], op.p[i][1], op.p[i][2] || 0);
      if (pt.x === undefined || pt.y === undefined) continue;
      this.ctx_.fillStyle = this.current_color_;
      this.ctx_.fillRect(pt.y - half_size, pt.x - half_size, size, size);
    }
  }

  // Draw a rectangle
  drawRect_(op) {
    let x1 = Math.min(op.a[0], op.b[0]);
    let x2 = Math.max(op.a[0], op.b[0]);
    let y1 = Math.min(op.a[1], op.b[1]);
    let y2 = Math.max(op.a[1], op.b[1]);
    let z1 = Math.min(op.a[2] || 0, op.b[2] || 0);
    let z2 = Math.max(op.a[2] || 0, op.b[2] || 0);

    let list = [];
    if (z1 != z2) {
      // Render the six faces of the cuboid
      list.push([[x1, y1, z1], [x1, y2, z1], [x2, y2, z1], [x2, y1, z1]]);
      list.push([[x1, y1, z2], [x1, y2, z2], [x2, y2, z2], [x2, y1, z2]]);
      list.push([[x1, y1, z1], [x1, y1, z2], [x2, y1, z2], [x2, y1, z1]]);
      list.push([[x1, y2, z1], [x1, y2, z2], [x2, y2, z2], [x2, y2, z1]]);
      list.push([[x1, y1, z1], [x1, y1, z2], [x1, y2, z2], [x1, y2, z1]]);
      list.push([[x2, y1, z1], [x2, y1, z2], [x2, y2, z2], [x2, y2, z1]]);
    } else {
      // Render a 2d rectangle
      list.push([[x1, y1, z1], [x1, y2, z1], [x2, y2, z1], [x2, y1, z1]]);
    }
    for (let i in list) {
      let l = list[i];
      const pt1 = this.transformPoint_(l[0][0], l[0][1], l[0][2]);
      const pt2 = this.transformPoint_(l[1][0], l[1][1], l[1][2]);
      const pt3 = this.transformPoint_(l[2][0], l[2][1], l[2][2]);
      const pt4 = this.transformPoint_(l[3][0], l[3][1], l[3][2]);
      this.ctx_.lineWidth = this.current_size_ / this.zoom;
      this.ctx_.beginPath();
      this.ctx_.moveTo(pt1.y, pt1.x);
      this.ctx_.lineTo(pt2.y, pt2.x);
      this.ctx_.lineTo(pt3.y, pt3.x);
      this.ctx_.lineTo(pt4.y, pt4.x);
      this.ctx_.lineTo(pt1.y, pt1.x);
      this.ctx_.closePath();
      if (this.current_fill_) {
        this.ctx_.fillStyle = this.current_color_;
        this.ctx_.fill();
      } else {
        this.ctx_.strokeStyle = this.current_color_;
        this.ctx_.stroke();
      }
    }
  }

  // Draw Asset
  drawAsset_(op) {
    const is_perspective = Renderer2D.IsPerspectiveProjection(this.projection_matrix);
    const asset = is_perspective
        ? {img: AssetManager().getMeshImage(op.n, this.current_transformation_,
                                            this.projection_matrix, this.current_color_)}
        : AssetManager().getAsset2D(op.n);
    if (asset && asset.img) {
      this.current_scale *= asset.img.scale;
      this.drawImage_(asset, is_perspective);
      this.current_scale /= asset.img.scale;
    } else {
      const pt = this.transformPoint_(0, 0, 0);
      let r = 0.45 * this.current_scale;
      this.ctx_.beginPath();
      this.ctx_.arc(pt.y, pt.x, r, 0, 2*Math.PI);
      this.ctx_.closePath();
      this.ctx_.fillStyle = "#0f0";
      this.ctx_.fill();
    }
  }

  markerGeometryCanvas_(transform, static_T_base, scaleFactor) {
    // Convert a point in the marker's space to canvas space
    const static_T_marker = static_T_base.clone().multiply(
        new THREE.Matrix4().makeRotationFromQuaternion(transform.orientation)
                           .setPosition(transform.position));
    let markerToCanvas = (pMarker) => {
      const pBaseReferentialFrame = pMarker.clone().applyMatrix4(static_T_marker);
      const pCanvas = this.baseReferentialFrameToCanvas_(pBaseReferentialFrame);
      return pCanvas;
    };

    const centerMarker = new THREE.Vector4(0, 0, 0, 1);
    const centerCanvas = markerToCanvas(centerMarker);

    let axisEndMarker = [
        new THREE.Vector4(0.5 * scaleFactor, 0, 0, 1),
        new THREE.Vector4(0, 0.5 * scaleFactor, 0, 1)
    ];
    let xArrowEndMarker =
        new THREE.Vector4(0.7 * scaleFactor, 0.0, 0, 1);

    let xArrowHeadLegEndMarker = [
        new THREE.Vector4(0.6 * scaleFactor, 0.1 * scaleFactor, 0, 1),
        new THREE.Vector4(0.6 * scaleFactor, -0.1 * scaleFactor, 0, 1)
    ];
    let axisEndCanvas = [
      markerToCanvas(axisEndMarker[0]),
      markerToCanvas(axisEndMarker[1])
    ];
    let xArrowHeadLegEndCanvas = [
      markerToCanvas(xArrowHeadLegEndMarker[0]),
      markerToCanvas(xArrowHeadLegEndMarker[1])
    ];
    let xArrowEndCanvas = markerToCanvas(xArrowEndMarker);

    return {
      "xArrowEndCanvas": xArrowEndCanvas,
      "axisEndCanvas": axisEndCanvas,
      "centerCanvas": centerCanvas,
      "xArrowHeadLegEndCanvas": xArrowHeadLegEndCanvas,
    };
  }

  drawMarkersImage_(ctx, static_T_base) {
      let static_T_base_translation = new THREE.Vector3();
      let static_T_base_orientation = new THREE.Quaternion();
      let static_T_base_scale = new THREE.Vector3();
      static_T_base.decompose(
          static_T_base_translation, static_T_base_orientation, static_T_base_scale);
    let drawMarker = (geometry, circleColor, axisColor) => {
      ctx.lineWidth = 2;
      ctx.strokeStyle = circleColor;

      // Circle around the marker.
      // It's a circle in marker space, but in general it's an ellipse in canvas space.
      let startAngle = 0;
      let endAngle = 2*Math.PI;
      ctx.beginPath();
      ctx.save();
      // Note: We normalize the axes, so we're only doing rotation and translation.
      // It would have been convenient to just put the axes in without normalizing, and then simply
      // draw a circle.
      // Unfortunately canvas line thickness is not thickness on screen, but rather in the source
      // space, so line thickness on screen would vary along the ellipse. (!?)
      let circleXAxisCanvas = geometry.axisEndCanvas[0].clone();
      circleXAxisCanvas.sub(geometry.centerCanvas);
      circleXAxisCanvas.normalize();
      let circleYAxisCanvas = geometry.axisEndCanvas[1].clone();
      circleYAxisCanvas.sub(geometry.centerCanvas);
      circleYAxisCanvas.normalize();
      ctx.transform(
          circleXAxisCanvas.x,
          circleXAxisCanvas.y,
          circleYAxisCanvas.x,
          circleYAxisCanvas.y,
          geometry.centerCanvas.x,
          geometry.centerCanvas.y
      );
      let rotation = 0;
      // Note: We need to draw an ellipse, rather than a scaled circle, because if we draw a scaled
      // circle then the line thickness will also be scaled. (?!)
      ctx.ellipse(
          0,
          0,
          geometry.centerCanvas.distanceTo(geometry.axisEndCanvas[0]),
          geometry.centerCanvas.distanceTo(geometry.axisEndCanvas[1]),
          rotation,
          startAngle,
          endAngle
      );
      ctx.stroke();
      ctx.restore();

      // Coordinate axes
      let lineWidth = 2;
      // +x axis
      ctx.beginPath();
      ctx.moveTo(geometry.centerCanvas.x, geometry.centerCanvas.y);
      ctx.lineTo(geometry.xArrowEndCanvas.x, geometry.xArrowEndCanvas.y);
      ctx.strokeStyle = axisColor;
      ctx.lineWidth = lineWidth;
      ctx.stroke();
      // +y axis
      ctx.beginPath();
      ctx.moveTo(geometry.centerCanvas.x, geometry.centerCanvas.y);
      ctx.lineTo(geometry.axisEndCanvas[1].x, geometry.axisEndCanvas[1].y);
      ctx.strokeStyle = axisColor;
      ctx.lineWidth = lineWidth;
      ctx.stroke();
      // +x axis arrow head legs
      ctx.beginPath();
      ctx.lineTo(geometry.xArrowEndCanvas.x, geometry.xArrowEndCanvas.y);
      ctx.lineTo(
        geometry.xArrowHeadLegEndCanvas[0].x,
        geometry.xArrowHeadLegEndCanvas[0].y
      );
      ctx.strokeStyle = axisColor;
      ctx.lineWidth = lineWidth;
      ctx.stroke();
      ctx.beginPath();
      ctx.lineTo(geometry.xArrowEndCanvas.x, geometry.xArrowEndCanvas.y);
      ctx.lineTo(
        geometry.xArrowHeadLegEndCanvas[1].x,
        geometry.xArrowHeadLegEndCanvas[1].y
      );
      ctx.strokeStyle = axisColor;
      ctx.lineWidth = lineWidth;
      ctx.stroke();
    };

    let drawLabel = (text, geometry, color) => {
      const markerSizeCanvas = Math.max(
          geometry.centerCanvas.distanceTo(geometry.axisEndCanvas[0]),
          geometry.centerCanvas.distanceTo(geometry.axisEndCanvas[1])
      );
      const size = markerSizeCanvas + 0.4;
      const posX = geometry.centerCanvas.x;
      const posY = geometry.centerCanvas.y + markerSizeCanvas*1.4;
      ctx.fillStyle = color;
      ctx.textAlign="center";
      ctx.textBaseline = "top";
      ctx.font = size + "px Arial";
      ctx.fillText(text, posX, posY);
    };

    // TODO: draw in decreasing-z order
    for (let i in this.markers_channels_) {
      const channel = this.markers_channels_[i];
      const name = channel.c;
      if (!channel.a) continue;
      let scaleFactor = 1.0;
      if (typeof channel.config.size !== 'undefined' && channel.config.size !== null) {
        scaleFactor = channel.config.size;
      }
      let transform = null;
      let active = false;
      if (channel.i) {
        if (!this.markersWidget_.containsMarker(name)) continue;
        transform = this.markersWidget_.transform(name);
        active = this.markersWidget_.activeInteraction(name);
      } else {
        // Since it's a non itreactive marker I need to query the PoseTree (It's not registered
        // in the markersWidget)
        const matrix = PoseTree().get(this.base_referential_frame_, name, this.current_time_);
        if (matrix) {
          let trans = new THREE.Vector3();
          let quat = new THREE.Quaternion();
          let scale = new THREE.Vector3();
          matrix.decompose(trans, quat, scale);
          transform = {
            position: trans,
            orientation: quat
          };
        }
      }
      if (!transform) continue;
      // TODO: calculate highlighted color
      let color = "#a00000";
      let selectedColor = '#aaffaa';
      let highlightedColor = "#ffaaaa";
      if (typeof channel.config.color !== 'undefined' && channel.config.color !== null) {
        color = channel.config.color;
      }
      if (typeof channel.config.opacity !== 'undefined' && channel.config.opacity !== null) {
        let alpha = (channel.config.opacity * 255.0).toString(16);
        if (alpha.length == 1) {
          alpha = '0' + alpha;
        }
        color += alpha;
        selectedColor += alpha;
        highlightedColor += alpha;
      }
      let circleColor = color;
      let axisColor = color;
      if (active) {
        switch(this.markersWidget_.desiredTransformationMode()) {
          case "translation": {
            axisColor = highlightedColor;
            circleColor = selectedColor;
            break;
          }
          case "orientation": {
            axisColor = selectedColor;
            circleColor = highlightedColor;
            break;
          }
        }
      } else if (this.selectedMarker_ == name) {
        axisColor = selectedColor;
        circleColor = selectedColor;
      }
      const geometry = this.markerGeometryCanvas_(transform, static_T_base, scaleFactor);
      drawMarker(geometry, circleColor, axisColor);
      if (name == this.hoveredMarker_) {
        drawLabel(name, geometry, '#ff0000');
      }
    }

    let orientation_guide_line_segment = this.markersWidget_.orientationGuideLineSegment();
    if (orientation_guide_line_segment) {
      let start = this.baseReferentialFrameToCanvas_(
          orientation_guide_line_segment.start.clone().applyMatrix4(static_T_base));
      let end = this.baseReferentialFrameToCanvas_(
          orientation_guide_line_segment.end.clone().applyMatrix4(static_T_base));

      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(end.x, end.y);
      ctx.strokeStyle = "#ffff00";
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }

  // Returns whether or not a projection matrix is a perspective matrix (coming form a pinhole
  // camera model).
  static IsPerspectiveProjection(projection) {
    return projection.elements[15] == 0;
  }

  // Return a Pose2 from a Matrix4
  static GetPose2(matrix, projection) {
    if (Renderer2D.IsPerspectiveProjection(projection)) {
      return {x: 0.0, y: 0.0, angle: 0.0};
    }
    return {x: matrix.elements[13], y: matrix.elements[12],
            angle: Math.atan2(matrix.elements[4], matrix.elements[0])};
  }
}
