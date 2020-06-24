/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/* OccupancyGridMapLayer - A class representating single waypoints map layer */
class WaypointMapLayer {
  constructor(map, layer_name, layer_type, data) {
    this.map_ = map;
    this.name_ = layer_name;
    this.type_ = layer_type;
    this.fabric_canvas_ = MapContainer().getFabricCanvas();
    this.rendered_ = false;
    this.enabled_ = false;
    this.waypoints_ = [];
    this.has_objects_ = false;
    // Default radius for waypoints.
    this.default_radius_ = 1.0;
    this.selectable_ = false;
  }

  /* Returns name of the layer. */
  name() {
    return this.name_;
  }

  /* Returns type of the layer */
  type() {
    return this.type_;
  }

  /* Enables layer */
  enable() {
    if (this.enabled_ === true) {
      return;
    }
    this.enabled_ = true;
    this.rendered_ = false;
  }

  /* Enable editing for layer */
  enableEditingForLayer() {
    this.enable();
    this.makeObjectsSelectable(true);
  }

  /* Disable editing for layer */
  disableEditingForLayer() {
    this.makeObjectsSelectable(false);
  }

  /* Makes layer as selectable or non-selectable */
  setSelectable(selectable) {
    if (this.selectable_ === selectable) {
      return;
    }
    this.selectable_ = selectable;
  }

  /* Makes all existing ways point objes selectable / non-selectable */
  makeObjectsSelectable(selectable) {
    this.setSelectable(selectable);
    for (let waypoint of this.waypoints_) {
      // Make all objects of this layer selectable.
      let object = this.fabric_canvas_.getItemByID(this.getIdForWaypoint_(waypoint));
      if (object !== null) {
        object.selectable = selectable;
      }
    }
  }

  /* Returns if the layer is selectable */
  selectable() {
    return this.selectable_;
  }

  /* Returns if the current layer is enabled. */
  enabled() {
    return this.enabled_;
  }

  /* Disables layer */
  disable() {
    if (this.enabled_ === false) {
      return;
    }
    for (let waypoint of this.waypoints_) {
      // Remove all objects of this layer.
      let object = this.fabric_canvas_.getItemByID(this.getIdForWaypoint_(waypoint));
      if (object !== null) {
        this.fabric_canvas_.remove(object);
      }
    }
    this.waypoints_ = [];
    this.enabled_ = false;
  }

  /* Checks if the layer really has objects */
  hasObjects() {
    // When waypoints_.length is 0 and has_objects_ is true, it means
    // this client hasn't drawn any objects for this layer but other
    //  clients has and layers needs an update when enabled.
    if (this.waypoints_.length > 0 || this.has_objects_ === true) {
      return true;
    }
    return false;
  }

  /* Marks that the layer has objects */
  setHasObjects(has_objects) {
    this.has_objects_ = has_objects;
  }

  // Visualizes layer with given json data
  renderLayer(data) {
    if (this.rendered_ === true) {
      // Layer needs to be fully rendered only once
      // and then there will be specific updates.
      return;
    }
    const waypoints = data["waypoints"];
    if (waypoints === null) {
      return;
    }
    for (let waypoint of waypoints) {
      this.addObjectFromJson(waypoint);
    }
    this.rendered_ = true;
    this.enabled_ = true;
  }

  /* Loads waypoint from json and draw it on canvas */
  addObjectFromJson(data) {
    if (!data.hasOwnProperty("name")) {
      return;
    }
    let may_be_object = this.fabric_canvas_.getItemByID(this.getIdForWaypoint_(data.name));
    if (may_be_object !== null) {
      if (may_be_object.localType === "waypoint") {
        return;
      }
    }
    let radius = this.default_radius_;
    if (data.hasOwnProperty("radius")) {
      radius  = data.radius;
    }
    const radius_in_pixels = this.getRadiusInPixels_(radius);
    let that_ = this;
    fabric.Image.fromURL('./map-assets/arrow_upward.png', function(waypoint) {
      // Backend rotation is counter clockwise
      let wp_angle = MapContainer().getAngleForFrontend(data["pose"][0]);
      const display_coordinates =
          MapContainer().getDisplayCoordinates(data["pose"][1], data["pose"][2]);
      let wp_color = data["color"];
      if (wp_color[0] !== '#') {
        wp_color = MapContainer().getColorHex(wp_color);
      }
      const wp_name = data["name"];
      that_.drawWaypoint_(waypoint, {
        x : display_coordinates.x,
        y : display_coordinates.y,
        angle : wp_angle,
        radius: radius_in_pixels,
        color : wp_color,
        name : wp_name,
      }, false);
    });
  }

  /* Draws a waypoint object on canvas at given point */
  drawObjectAtPoint(op) {
    let options = this.fabric_canvas_.getPointer(op);
    this.showDialogForWaypointInput_(options.x, options.y);
  }

  /* Show dialog for waypoint input */
  showDialogForWaypointInput_(x, y) {
    let dialog_div = document.getElementById(MapContainer().MAP_CONTAINER_LAYER_INPUT_DIV_ID);
    if (dialog_div !== null) {
      dialog_div.parentNode.removeChild(dialog_div);
    }
    dialog_div = document.createElement('div');
    dialog_div.id = MapContainer().MAP_CONTAINER_LAYER_INPUT_DIV_ID;
    dialog_div.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_DIV_CLASS_ID);
    // Approximate calculations to find the start position of the div so that its center is x, y.
    // TODO(apatole): find better way to do this.
    let start = x - 180;
    if (start < 0) {
      start = 0;
    }
    dialog_div.style.left = start + "px";
    dialog_div.style.top = y + "px";
    // Fixed some width so that it fits on one line always.
    let name_label = document.createElement('label');
    name_label.appendChild(document.createTextNode("Name:"));
    name_label.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID);
    name_label.classList.add("map-container-label-text");
    let name_input = document.createElement('input');
    name_input.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID);
    name_input.type = "text";
    name_input.size = "10";
    let radius_label = document.createElement('label');
    radius_label.appendChild(document.createTextNode("Radius:"));
    radius_label.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID);
    radius_label.classList.add("map-container-label-text");
    let radius_input = document.createElement('input');
    radius_input.type = "text";
    radius_input.size = "1";
    radius_input.value = this.default_radius_;
    radius_input.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID);
    radius_input.classList.add("map-container-label-text");
    let ok_btn = document.createElement('input');
    ok_btn.type = "button";
    ok_btn.value = "OK";
    ok_btn.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID);
    ok_btn.classList.add(MapContainer().MAP_LAYER_DIALOG_PRIMARY_BTN_CLASS);
    let that_ = this;
    let cancel_btn = document.createElement('input');
    cancel_btn.type = "button";
    cancel_btn.value = "Cancel";
    cancel_btn.classList.add(MapContainer().MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID);
    cancel_btn.classList.add(MapContainer().MAP_LAYER_DIALOG_SECONDARY_BTN_CLASS);
    ok_btn.onclick = function() {
      let waypoint_name = name_input.value;
      if (!waypoint_name) {
        waypoint_name = "wp" + Math.random().toString().replace('0.', '');
      }
      if (that_.waypoints_.length > 0 && that_.waypoints_.indexOf(waypoint_name) !== -1) {
        let error_message = "Waypoint with the name '" + waypoint_name +
            "' already exists in layer " + that_.name_;
        msgPopup(dialog_div, 'error', error_message);
        return;
      }
      // TODO(apatole): Add more input validation
      dialog_div.parentNode.removeChild(dialog_div);
      that_.drawObjectAtPoint_({
        x: x,
        y: y,
        name: waypoint_name,
        radius: radius_input.value,
      });
    }
    cancel_btn.onclick = function() {
      dialog_div.parentNode.removeChild(dialog_div);
    }
    dialog_div.appendChild(name_label);
    dialog_div.appendChild(name_input);
    dialog_div.appendChild(radius_label);
    dialog_div.appendChild(radius_input);
    dialog_div.appendChild(ok_btn);
    dialog_div.appendChild(cancel_btn);
    let parent = document.getElementById(MapContainer().MAP_LAYER_CONTAINER_ID);
    if (parent !== null) {
      parent.appendChild(dialog_div);
    }
  }

  /* Draws a waypoint object on canvas at given point */
  drawObjectAtPoint_(options) {
    let that_ = this;
    let canvas_elem = document.createElement('map-canvas');
    // TODO(apatole): Radius here should be coming from user when creating a new waypoint.
    const radius_in_pixels = this.getRadiusInPixels_(options.radius);
    fabric.Image.fromURL('./map-assets/arrow_upward.png', function(waypoint) {
      waypoint.layer = that_.name_;
      that_.drawWaypoint_(waypoint, {
        x : options.x,
        y : options.y,
        angle : MapContainer().getAngleForFrontend(0),
        radius: radius_in_pixels,
        color : MapContainer().getObjectColor(),
        // TODO:(apatole) generating some random string for waypoint names for now,
        // it will be replaced with client entered waypoint name.
        name : options.name,
      }, true);
    });
  }

 /* Modifies a existing waypoint object on canvas */
  modifyObject(data) {
    let may_be_object = this.fabric_canvas_.getItemByID(this.getIdForWaypoint_(data.name));
    if (may_be_object !== null) {
      if (may_be_object.localType === "waypoint") {
        this.fabric_canvas_.remove(may_be_object);
        this.waypoints_ = this.waypoints_.filter(item => item !== may_be_object.id);
      }
    }
    this.addObjectFromJson(data);
  }

  /* Removes a existing waypoint object with the given name */
  removeObject(name) {
    let may_be_object = this.fabric_canvas_.getItemByID(this.getIdForWaypoint_(name));
    if (may_be_object !== null) {
      this.fabric_canvas_.remove(may_be_object);
      this.waypoints_ = this.waypoints_.filter(item => item !== name);
    }
  }

  /* Draw a waypoint object */
  drawWaypoint_(waypoint_image, options, fire_event) {
    waypoint_image.originX = 'center';
    waypoint_image.originY = 'center';
    waypoint_image.scaleToWidth(options.radius * 1.5);
    waypoint_image.scaleToHeight(options.radius * 1.5);
    let circle = new fabric.Circle({
      radius: options.radius,
      stroke: options.color,
      originX: 'center',
      originY: 'center',
      fill: options.color,
    });
    let group = new fabric.Group([circle, waypoint_image], {
      left: options.x, top: options.y, angle: options.angle, selectable: this.selectable_
    });
    group.originX = 'center';
    group.originY = 'center';
    group.transparentCorners = true;
    group.cornerColor = options.color;
    group.localType = "waypoint";
    group.localColor = options.color;
    group.waypoint_radius = this.getRadiusInMeters_(options.radius);
    // TODO:(apatole) generating some random string for waypoint names for now,
    // it will be replaced with client entered waypoint name
    group.id = this.getIdForWaypoint_(options.name);
    group.name = options.name;
    group.centeredScaling = true;
    // Show only required controls when waypoint is selected for editing -
    // bottom left/right, top left/right scaling controls and rotation control.
    group.setControlsVisibility({
      mt: false,
      mb: false,
      ml: false,
      mr: false,
      bl: true,
      br: true,
      tl: true,
      tr: true,
      mtr: true,
    });
    group.should_trigger_event = fire_event;
    this.fabric_canvas_.add(group);
    group.on('mouseover', function(op) {
      MapContainer().showHoverText({
        name: options.name,
        x: group.left,
        y: group.top,
      });
    });
    group.on('mouseout', function(op) {
      MapContainer().hideHoverText();
    });
    this.waypoints_.push(options.name);
    this.has_objects_ = true;
  }

  /* Converts radius  from meters to pixels */
  getRadiusInPixels_(radius) {
    return radius / this.map_.cellSize();
  }

  /* Converts radius from pixels to meters */
  getRadiusInMeters_(radius) {
    return radius * this.map_.cellSize();
  }

  /* Returns a string that can be used to identify a particular waypoint */
  getIdForWaypoint_(waypoint_name) {
    return this.name_ + "_" + waypoint_name;
  }
}
