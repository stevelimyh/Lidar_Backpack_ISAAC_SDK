/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/* PolygonMapLayer - A class representating single polygon map layer */
class PolygonMapLayer {
  constructor(map, layer_name, layer_type, data) {
    this.map_ = map;
    this.name_ = layer_name;
    this.type_ = layer_type;
    this.rendered_ = false;
    this.enabled_ = false;
    this.polygons_ = [];
    this.current_polygon_points_ = [];
    this.current_polygon_lines_ = [];
    this.has_objects_ = false;
    this.recording_polygon_ = false;
    let that_ = this;
    this.fabric_canvas_ = MapContainer().getFabricCanvas();
    if (data !== undefined && data.hasOwnProperty("color")) {
      this.setColor_(data["color"]);
    }
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

  /* Makes all existing ways point objects selectable / non-selectable */
  makeObjectsSelectable(selectable) {
    this.setSelectable(selectable);
    for (let polygon of this.polygons_) {
      // Make all objects of this layer selectable.
      let object = this.fabric_canvas_.getItemByID(this.getIdForPolygon_(polygon));
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
    this.removeAllPolygons_();
    this.removeAllPolygonLines_();
    this.current_polygon_points_ = [];
    this.enabled_ = false;
  }

  /* Checks if the layer really has objects */
  hasObjects() {
    // When polygons_.length is 0 and has_objects_ is true, it means
    // this client hasn't drawn any objects for this layer but other
    //  clients has and layers needs an update when enabled.
    if (this.polygons_.length > 0 || this.has_objects_ === true) {
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
    if (data !== undefined && data.hasOwnProperty("color")) {
      this.setColor_(data["color"]);
    }
    const polygons = data["polygons"];
    if (polygons === null) {
      return;
    }
    for (let polygon of polygons) {
      this.addObjectFromJson(polygon);
    }
    this.rendered_ = true;
    this.enabled_ = true;
  }

  /* Loads polygon from json and draw it on canvas */
  addObjectFromJson(data) {
    if (!data.hasOwnProperty("name")) {
      return;
    }
    let may_be_object = this.fabric_canvas_.getItemByID(this.getIdForPolygon_(data.name));
    if (may_be_object !== null) {
      if (may_be_object.localType === "polygon") {
        return;
      }
    }
    let points = data["points"];
    let points_with_xy = [];
    for (let point in points) {
      const display_coordinates =
          MapContainer().getDisplayCoordinates(points[point][0], points[point][1]);
      points_with_xy.push({x:display_coordinates.x, y:display_coordinates.y});
    }
    this.drawPolygon_({
      points: points_with_xy,
      name: data.name,
    }, false);
  }

 /* Modifies a existing polygon object on canvas */
  modifyObject(data) {
    let may_be_object = this.fabric_canvas_.getItemByID(this.getIdForPolygon_(data.name));
    if (may_be_object !== null) {
      if (may_be_object.localType === "polygon") {
        this.fabric_canvas_.remove(may_be_object);
        this.polygons_ = this.polygons_.filter(item => item !== may_be_object.id);
      }
    }
    this.addObjectFromJson(data);
  }

  /* Removes a existing polygon object with the given name */
  removeObject(name) {
    let may_be_object = this.fabric_canvas_.getItemByID(this.getIdForPolygon_(name));
    if (may_be_object !== null) {
      this.fabric_canvas_.remove(may_be_object);
      this.polygons_ = this.polygons_.filter(item => item !== name);
    }
  }

  /* Handle mouse double click for polygon */
  handleDoubleClick(op) {
    let options = this.fabric_canvas_.getPointer(op);
    this.current_polygon_points_.push([options.x, options.y]);
    if (this.recording_polygon_ === true) {
      // Remove drawn lines and draw complete polygon
      this.recording_polygon_ = false;
      this.removeAllPolygonLines_();
      let poly_points = [];
      for (let pt of this.current_polygon_points_) {
        poly_points.push({x:pt[0], y:pt[1]});
      }
      this.current_polygon_points_ = [];
      const name_or_id = "poly"+Math.random().toString().replace('0.', '');
      this.drawPolygon_({
        points: poly_points,
        name: name_or_id,
      }, true);
    }
  }

  /* Draws a polygon line from last point to current mouse click point */
  drawObjectAtPoint(op) {
    let options = this.fabric_canvas_.getPointer(op);
    if (this.recording_polygon_ === false) {
      this.recording_polygon_ = true;
      this.current_polygon_points_ = [];
    } else {
      let last_point = this.current_polygon_points_[this.current_polygon_points_.length - 1];
      let line = new fabric.Line([last_point[0], last_point[1], options.x, options.y], {
        stroke: this.color_,
        hasControls: false,
        hasBorders: false,
        lockMovementX: true,
        lockMovementY: true,
        hoverCursor: 'default'
      });
      this.fabric_canvas_.add(line);
      this.current_polygon_lines_.push(line);
    }
    this.current_polygon_points_.push([options.x, options.y]);
  }

  /* Draw a polygon object with given options */
  drawPolygon_(options, fire_event) {
    let polygon = new fabric.Polygon(options.points, {
      fill: this.color_,
      opacity: 0.7,
      name : options.name,
      selectable: true,
      transparentCorners: true,
      cornerColor: this.color_,
      borderColor: this.color_,
      selectable: this.selectable_,
    });
    polygon.id  = this.getIdForPolygon_(options.name);
    polygon.localType = "polygon";
    polygon.localColor = this.color_;
    polygon.should_trigger_event = fire_event;
    this.fabric_canvas_.add(polygon);
    this.polygons_.push(options.name);
    this.has_objects_ = true;
  }

  /* Remove all polygons from the layer */
  removeAllPolygons_() {
    for (let polygon of this.polygons_) {
      let object = this.fabric_canvas_.getItemByID(this.getIdForPolygon_(polygon));
      if (object !== null) {
        this.fabric_canvas_.remove(object);
      }
    }
    this.polygons_ = [];
  }

  /* Remove all polygon lines drawn while creating polygon */
  removeAllPolygonLines_() {
    for (let line of this.current_polygon_lines_) {
      this.fabric_canvas_.remove(line);
    }
    this.current_polygon_lines_ = [];
  }

  /* Set color of the polygon layer, convert to hex if required */
  setColor_(color) {
    if (color[0] !== '#') {
      this.color_ = MapContainer().getColorHex(color);
    } else {
      this.color_ = color;
    }
  }

  /* Returns a string that can be used to identify a particular polygon */
  getIdForPolygon_(polygon_name) {
    return this.name_ + "_" + polygon_name;
  }

}
