/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/* Map - Class representing a single map from map container */
class IsaacMap {
  constructor(name, data) {
    this.name_ = name;
    this.layers_ = {};
    this.default_occupancy_layer_ = null;
    this.default_waypoints_layer_ = null;
    this.default_polygon_layer_ = null;
    if (data.hasOwnProperty("layers")) {
      this.addLayers_(data.layers);
    }
    this.is_active_ = false;
    this.active_layer_name_ = null;
    this.cell_size_ = 0.1;
    if (data.hasOwnProperty("cell_size")) {
      this.cell_size_ = data.cell_size;
    }
  }

  /* Returns name of the map */
  name() {
    return this.name_;
  }

  /* Returns cell size for map visualization */
  cellSize() {
    return this.cell_size_;
  }

  /* Activates this map */
  setActive() {
   this.is_active_ = true;
    for (let layer in this.layers_) {
      MapContainer().updateMapContainerForLayer(this.name_, this.layers_[layer].name(),
          this.layers_[layer].type(), MapContainer().MapActions.LAYER_ADDED);
    }
    this.getDataForDafaultActiveLayers_();
  }

  /* Returns active layer */
  getActiveLayerName() {
    return this.active_layer_name_;
  }

  /* Returns active layer type */
  getActiveLayerType() {
    if (this.active_layer_name_ !== null && this.layers_.hasOwnProperty(this.active_layer_name_)) {
      return this.layers_[this.active_layer_name_].type();
    }
  }

  /* Disables the map */
  disable() {
    this.is_active_ = false;
    this.active_layer_name_ = null;
    for (let layer in this.layers_) {
      this.layers_[layer].disable();
    }
  }

  /* Handles map messages */
  handleMessage(message) {
    if (message.type === MapContainer().MapEvents.MAP_EVENT_GET_LAYER_JSON) {
      this.onGetLayerJson_(message);
    } else if (message.type === MapContainer().MapEvents.MAP_EVENT_OBJECT_ADDED) {
      this.onObjectAdded_(message);
    } else if(message.type === MapContainer().MapEvents.MAP_EVENT_OBJECT_MODIFIED) {
      this.onObjectModified_(message);
    } else if (message.type === MapContainer().MapEvents.MAP_EVENT_OBJECT_REMOVED) {
      this.onObjectRemoved_(message);
    } else if (message.type === MapContainer().MapEvents.MAP_EVENT_LAYER_ADDED) {
      this.onLayerAdded_(message);
    } else if (message.type === MapContainer().MapEvents.MAP_EVENT_LAYER_REMOVED) {
      this.onLayerRemoved_(message);
    }
  }

  /* Adds a new layer to the map */
  addNewLayer(layer_name, layer_type, data) {
    this.addLayer_({name: layer_name, type : layer_type, data: data});
    this.layers_[layer_name].setHasObjects(false);
  }

  /* Removes given layer from the map. */
  removeLayer(layer_name) {
    if (this.layers_.hasOwnProperty(layer_name)) {
      this.layers_[layer_name].disable();
      delete this.layers_[layer_name];
    }
  }

  /* Updates layer status of a given layer - enable/disable layer */
  updateLayerStatus(layer_name, layer_type, status) {
    if (this.layers_.hasOwnProperty(layer_name)) {
      if (status === true && !this.layers_[layer_name].enabled()) {
        this.layers_[layer_name].enable();
        if (this.layers_[layer_name].hasObjects() === true) {
          this.getJson_(layer_name, this.layers_[layer_name].type());
        }
      } else {
        this.layers_[layer_name].disable();
      }
    }
  }

  /* Enables editing for a given layer */
  enableEditingForLayer(layer_name, layer_type) {
    if (!this.layers_.hasOwnProperty(layer_name)) {
      return;
    }
    if (this.active_layer_name_ !== null) {
      this.disableEditingForLayer(this.active_layer_name_,
          this.layers_[this.active_layer_name_].type());
    }
    this.active_layer_name_ = layer_name;
    if (this.layers_[layer_name].enabled() && this.layers_[layer_name].selectable()) {
      return;
    }
    this.layers_[layer_name].enableEditingForLayer();
    if (this.layers_[layer_name].hasObjects() === true) {
      this.getJson_(layer_name, this.layers_[layer_name].type());
    }
  }

  /* Disables editing for a given layer */
  disableEditingForLayer(layer_name, layer_type) {
    if (!this.layers_.hasOwnProperty(layer_name)) {
      return;
    }
    this.layers_[layer_name].disableEditingForLayer();
    MapContainer().disableEditingForLayer(layer_name, layer_type);
  }

  /* Stops editing layers from this map */
  stopEditing() {
    if (this.active_layer_name_ !== null) {
      this.layers_[this.active_layer_name_].disableEditingForLayer();
    }
    this.active_layer_name_ = null;
  }

  /* Handles mouse events on canvas. */
  onMouseLeftButtonDown(op) {
    if (this.active_layer_name_ === null || !this.layers_.hasOwnProperty(this.active_layer_name_)) {
      return;
    }
    if (MapContainer().editingAction() === MapContainer().EditingAction.DRAWING_WAYPOINTS ||
        MapContainer().editingAction() === MapContainer().EditingAction.DRAWING_POLYGONS) {
      this.layers_[this.active_layer_name_].drawObjectAtPoint(op);
    }
  }

  /* Handles mouse double click event */
  onMouseRightButtonDown(op) {
    if (this.active_layer_name_ === null || !this.layers_.hasOwnProperty(this.active_layer_name_)) {
      return;
    }
    if (MapContainer().editingAction() === MapContainer().EditingAction.DRAWING_POLYGONS) {
      this.layers_[this.active_layer_name_].handleDoubleClick(op);
    }
  }

  /* Checks if the given name can be used as a layer name for new layer */
  isValidLayerName(new_layer_name) {
    if (this.layers_.hasOwnProperty(new_layer_name)) {
      return {invalid: true, message: 'Layer with name ' + new_layer_name + ' already exists!'};
    }
    if (new_layer_name === MapContainer().MAP_LAYER_NAME_INITIAL_TEXT || new_layer_name === "") {
      return {invalid: true, message: 'Enter layer name!'};
    }
    const forbidden_chars_pattern = /[\/]/;
    // Currently, similar to backend component name, only  '/' is added as forbidden character.
    // This needs to be in sync with isaac::alice::kForbiddenNameCharacters.
    if (forbidden_chars_pattern.test(new_layer_name)) {
      return {invalid: true, message: 'Layer name may not contain forbidden character: /'};
    }
    // TODO(apatole): Check if we need to add any extra checks here like name with special
    // characters, non english characters, etc.
    return {invalid: false};
  }

  /* Removes object with the given name from the active layer */
  removeObject(name) {
    if (this.active_layer_name_ !== null && this.layers_.hasOwnProperty(this.active_layer_name_)) {
      this.layers_[this.active_layer_name_].removeObject(name);
    }
  }

  /* Add layers to the map as a part of 'discovery' */
  addLayers_(layers) {
    for (let layer of layers) {
      if (!this.isSupportedLayer_(layer.type)) {
        continue;
      }
      this.addLayer_(layer);
      this.layers_[layer.name].setHasObjects(true);
    }
  }

  /* Return if the layer type is suported */
  isSupportedLayer_(layer_type) {
    if (layer_type === MapContainer().MapLayerTypes.OCCUPANCY_MAP_LAYER ||
        layer_type === MapContainer().MapLayerTypes.WAYPOINT_MAP_LAYER ||
        layer_type === MapContainer().MapLayerTypes.POLYGON_MAP_LAYER) {
      return true;
    }
    return false;
  }

  /* Add a layer in map */
  addLayer_(layer) {
    if (!layer.hasOwnProperty("name") || !layer.hasOwnProperty("type")) {
      return;
    }
    if (this.layers_.hasOwnProperty(layer.name)) {
      return;
    }
    if (layer.type === MapContainer().MapLayerTypes.OCCUPANCY_MAP_LAYER) {
      this.layers_[layer.name] = new OccupancyGridMapLayer(this, layer.name, layer.type);
      if (this.default_occupancy_layer_ === null) {
        this.default_occupancy_layer_ = layer.name;
      }
    } else if (layer.type === MapContainer().MapLayerTypes.WAYPOINT_MAP_LAYER) {
      this.layers_[layer.name] = new WaypointMapLayer(this, layer.name, layer.type);
      if (this.default_waypoints_layer_ === null) {
        this.default_waypoints_layer_ = layer.name;
      }
    } else if (layer.type === MapContainer().MapLayerTypes.POLYGON_MAP_LAYER) {
      this.layers_[layer.name] = new PolygonMapLayer(this, layer.name, layer.type, layer.data);
      if (this.default_polygon_layer_ === null) {
        this.default_polygon_layer_ = layer.name;
      }
    }
  }

  /* Get data for layers that are activated by default. */
  getDataForDafaultActiveLayers_() {
    if (this.default_occupancy_layer_ !== null) {
      if (this.layers_.hasOwnProperty(this.default_occupancy_layer_)) {
        let layer = this.layers_[this.default_occupancy_layer_];
        if (layer.hasObjects()) {
          layer.enable();
          this.getJson_(layer.name(), layer.type());
          MapContainer().updateMapLayerStatus(this.name_, layer.name(),layer.type(), true);
        }
      }
    }
    let that_ = this;
    // setTimeout is a workaround for bug ISAAC-581 where second message is
    // ignored when first message is still being handled in tick().
    setTimeout(function(){
      that_.getDataForDafaultActiveWaypointsLayer_();
    }, 100);
  }

  /*
   * Get data for default active waypoint layer.
   * TODO(apatole) : remove this once bug ISAAC-581 is fixed.
   */
  getDataForDafaultActiveWaypointsLayer_() {
    if (this.default_waypoints_layer_ !== null) {
      if (this.layers_.hasOwnProperty(this.default_waypoints_layer_)) {
        let layer = this.layers_[this.default_waypoints_layer_];
        if (layer.hasObjects()) {
          layer.enable();
          this.getJson_(layer.name(), layer.type());
          MapContainer().updateMapLayerStatus(this.name_, layer.name(), layer.type(), true);
        }
      }
    }
    let that_ = this;
    setTimeout(function(){
      that_.getDataForDafaultActivePolygonLayer_();
    }, 100);
  }

  /*
   * Get data for default active polygon layer.
   * TODO(apatole) : remove this once bug ISAAC-581 is fixed.
   */
  getDataForDafaultActivePolygonLayer_() {
    if (this.default_polygon_layer_ !== null) {
      if (this.layers_.hasOwnProperty(this.default_polygon_layer_)) {
        let layer = this.layers_[this.default_polygon_layer_];
        if (layer.hasObjects()) {
          layer.enable();
          this.getJson_(layer.name(), layer.type());
          MapContainer().updateMapLayerStatus(this.name_, layer.name(), layer.type(), true);
        }
      }
    }
  }

  /* Get json data for given map layer. */
  getJson_(layer_name, layer_type) {
    if (MapContainer().webSocket()) {
      MapContainer().webSocket().send(JSON.stringify({
        type: 'map',
        data: {
          request : MapContainer().MapEvents.MAP_EVENT_GET_LAYER_JSON,
          map_name : this.name_,
          layer_name : layer_name,
          layer_type : layer_type,
        }
      }));
    }
  }

  /* Handle response of 'get_layer_json' message. */
  onGetLayerJson_(message) {
    if (this.layers_.hasOwnProperty(message.layer_name)) {
      this.layers_[message.layer_name].renderLayer(message.data);
    }
  }

  /* Handle response of 'object_added' message. */
  onObjectAdded_(message) {
    if (!this.layers_.hasOwnProperty(message.layer_name)) {
      this.addLayer_({name: message.layer_name, type: message.layer_type});
      MapContainer().updateMapContainerForLayer(this.name_, message.layer_name, message.layer_type,
          MapContainer().MapActions.LAYER_ADDED);
    }
    this.layers_[message.layer_name].setHasObjects(true);
    if (this.layers_[message.layer_name].enabled() == true) {
      this.layers_[message.layer_name].addObjectFromJson(message.data);
    }
  }

  /* Handle response of 'object_modified' message. */
  onObjectModified_(message) {
    if (this.layers_.hasOwnProperty(message.layer_name)) {
      if (this.layers_[message.layer_name].enabled() === true) {
        this.layers_[message.layer_name].modifyObject(message.data);
      }
    }
  }

  /* Handle response of 'object_removed' message. */
  onObjectRemoved_(message) {
    if (this.layers_.hasOwnProperty(message.layer_name)) {
      if (this.layers_[message.layer_name].enabled() === true) {
        this.layers_[message.layer_name].removeObject(message.data);
      }
    }
  }

  /* Handle response of 'layer_added' message */
  onLayerAdded_(message) {
    this.addNewLayer(message.layer_name, message.layer_type, message.data);
    MapContainer().updateMapContainerForLayer(this.name_, message.layer_name, message.layer_type,
        MapContainer().MapActions.LAYER_ADDED);
  }

  /* Handles response of 'layer_removed' message */
  onLayerRemoved_(message) {
    if (this.layers_.hasOwnProperty(message.layer_name)) {
      this.removeLayer(message.layer_name);
      MapContainer().updateMapContainerForLayer(this.name_, message.layer_name, message.layer_type,
          MapContainer().MapActions.LAYER_DELETED);
    }
  }

}
