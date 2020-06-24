/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/* Map Container - main class that manages visualization and editing of map container. */
class MapContainerImpl {
  constructor() {
    this.maps_ = {};
    this.view_ = null;
    this.canvas_ = null;
    this.fabric_canvas_ = null;
    this.web_socket_ = null;
    this.color_chooser_ = null;
    this.discovered_ = false;
    this.active_map_ = null;
    const MapLayerTypes = {
      OCCUPANCY_MAP_LAYER : "isaac::map::OccupancyGridMapLayer",
      WAYPOINT_MAP_LAYER : "isaac::map::WaypointMapLayer",
      POLYGON_MAP_LAYER : "isaac::map::PolygonMapLayer",
    };
    this.MapLayerTypes = MapLayerTypes;
    const EditingAction = {
      NONE: 0,
      DRAWING_WAYPOINTS: 1,
      DRAWING_POLYGONS: 2,
      REMOVING_OBJECTS: 3,
    };
    this.EditingAction = EditingAction;
    this.current_editing_action_ = EditingAction.NONE;
    const MapEvents = {
      MAP_EVENT_DISCOVERY : "discovery",
      MAP_EVENT_LAYER_ADDED: "layer_added",
      MAP_EVENT_LAYER_REMOVED: "layer_removed",
      MAP_EVENT_GET_LAYER_JSON : "get_layer_json",
      MAP_EVENT_OBJECT_ADDED : "object_added",
      MAP_EVENT_OBJECT_MODIFIED : "object_modified",
      MAP_EVENT_OBJECT_REMOVED: "object_removed",
    };
    this.MapEvents = MapEvents;
    const MapActions = {
      LAYER_ADDED : 1,
      LAYER_DELETED : 2      ,
    }
    this.MapActions = MapActions;
    this.WIN_MAP_CONTAINER_ID = "__win-map-container";
    this.MAP_INFO_CONTAINER_ID = "map-info-container";
    this.MAP_LAYER_CONTAINER_ID = "map-layer-container";
    this.MAP_LAYER_INFO_CONTAINER_ID = "map-layer-info-container";
    this.MAP_LIST_DIV_ID = "map-list-div";
    this.MAP_LIST_ID = "map-list";
    this.MAP_CANVAS_ID = "map-canvas";
    this.COLOR_PICKER_DIV_ID = "map-node-color-picker-div";
    this.COLOR_PICKER_ID = "map-node-color-picker";
    this.MAP_EDITING_TOOLS_ID = "map_editing_tools";
    this.MAP_LAYER_DIALOG_INPUT_TEXT_CLASS = "map-container-layer-dialog-input-text";
    this.MAP_LAYER_DIALOG_PRIMARY_BTN_CLASS = "map-container-layer-dialog-button-primary";
    this.MAP_LAYER_DIALOG_SECONDARY_BTN_CLASS = "map-container-layer-dialog-button-secondary";
    this.MAP_LAYER_DELETE_CONFIRMATION_DIV_ID = "delete_layer_confirmation_div";
    this.MAP_ADD_NEW_LAYER_DIALOG_DIV_ID = "add_new_layer_layer_div";
    this.MAP_CONTAINER_LAYER_CANVAS_WRAPPED_ID = "map-container-layer-canvas-wrapper";
    this.MAP_CONTAINER_OBJECT_HOVER_TEXT_ID = "map-container-object-hover-text";
    this.MAP_CONTAINER_LAYER_INPUT_FIELD_CLASS_ID = "map-container-layer-input-field";
    this.MAP_CONTAINER_LAYER_INPUT_DIV_ID = "map-container-layer-input-dialog";
    this.MAP_CONTAINER_LAYER_INPUT_DIV_CLASS_ID = "map-container-layer-input-div";
    this.MAP_LAYER_NAME_INITIAL_TEXT = "Enter layer name";
  }

  /* Sets web socket to be used for communication with server */
  setWebSocket(socket) {
    this.web_socket_ = socket;
    // If the MapContainer is already there remove it.
    if (this.view_) {
      WindowManager().deleteWindow(this.view_.parentElement.parentElement);
    }
    this.maps_ = {};
    this.view_ = null;
    this.canvas_ = null;
    this.fabric_canvas_ = null;
    this.color_chooser_ = null;
    this.discovered_ = false;
    this.active_map_ = null;
  }

  /* Resets websocket when websocket is closed */
  resetWebSocket() {
    this.web_socket_ = null;
    this.discovered_ = false;
  }

  /* Gets websocket for communication with server */
  webSocket() {
    return this.web_socket_;
  }

  /* Returns current editing action */
  editingAction() {
    return this.current_editing_action_;
  }

  /* Returns the color to be used for drawing objects */
  getObjectColor() {
    return this.color_chooser_.value;
  }

  /* Handle messages from WebsightServer */
  handleMapContainerMassage(message) {
    if (message.type === this.MapEvents.MAP_EVENT_DISCOVERY) {
      this.loadMapContainer_(message);
    } else {
      // Pass all messages to the corresponding map.
      if (this.maps_.hasOwnProperty(message.map_name)) {
        this.maps_[message.map_name].handleMessage(message);
      }
    }
  }

  /*
   * Update map for layers actions.
   * Called by maps when new layers are discovered and added to the maps
   * or when the layers are deleted from the map by other clients.
   * This normally happens when the map is activated or when the new layers
   * are created/deleted by other clients.
   */
  updateMapContainerForLayer(map_name, layer_name, layer_type, action) {
    if (action === this.MapActions.LAYER_ADDED) {
      this.addMapLayer_(map_name, layer_name, layer_type);
    } else if (action === this.MapActions.LAYER_DELETED) {
      this.removeMapLayer_(map_name, layer_name, layer_type);
    }
  }

  addMapLayer_(map_name, layer_name, layer_type) {
    if (this.active_map_ !== null) {
      const layer_div_id = this.getIdForLayerDiv_(this.active_map_.name(), layer_name, layer_type);
      let layer_div_elem = document.getElementById(layer_div_id);
      if (layer_div_elem !== null) {
        return;
      }
      this.addLayerDiv_(map_name, layer_name, layer_type);
    }
  }

  /* Removes given layer from map container UI */
  removeMapLayer_(map_name, layer_name, layer_type) {
    const layer_div_id = this.getIdForLayerDiv_(this.active_map_.name(), layer_name, layer_type);
    let layer_div_elem = document.getElementById(layer_div_id);
    if (layer_div_elem !== null) {
      this.stopEditing_();
      layer_div_elem.parentNode.removeChild(layer_div_elem);
    }
  }

  /*
   * Enable/Disable given layer.
   * Called by currently active map to change given map layer status
   * or when user activates a different layer.
   */
  updateMapLayerStatus(map_name, layer_name, layer_type, enabled) {
    const element_id = this.getIdForLayerCheck_(map_name, layer_name, layer_type);
    let elem = document.getElementById(element_id);
    if (elem === null || elem === undefined) {
      return;
    }
    elem.checked = enabled;
  }

  /*
   * Returns fabric canvas object to be used for map editing operations.
   * Creates a new fabric canvas if it doesn't exists.
   */
  getFabricCanvas() {
    let canvas_elem = document.getElementById(this.MAP_CANVAS_ID);
    if (this.canvas_ === null) {
      let canvas_wrapper = document.createElement('div');
      canvas_wrapper.id = this.MAP_CONTAINER_LAYER_CANVAS_WRAPPED_ID;
      canvas_elem = document.createElement('canvas');
      canvas_elem.id = this.MAP_CANVAS_ID;
      canvas_wrapper.appendChild(canvas_elem);
      let container_elem = document.getElementById(this.MAP_LAYER_CONTAINER_ID);
      container_elem.appendChild(canvas_wrapper);
      this.canvas_ = canvas_elem;
      this.fabric_canvas_ = new fabric.Canvas(this.MAP_CANVAS_ID);
      this.fabric_canvas_.hoverCursor = 'default';
      this.fabric_canvas_.fireRightClick = "true";
      this.fabric_canvas_.stopContextMenu = "true";
      // Handle mouse events on occupancy layer canvas
      // 'object:added' is triggered when new fabric object is added to the canvas.
      // This will send object_added message to the WebsightServer to update backend.
      let that_ = this;
      this.fabric_canvas_.on('object:added', function(op) {
        that_.onObjectAdded_(op);
      });
      // 'object:modified' is triggered when existing fabric object is modified.
      // This will send 'object_modified' message to the WebsightServer to update backend.
      this.fabric_canvas_.on('object:modified', function(op) {
        that_.onObjectModified_(op);
      });
      // 'object:selected' is triggered when any fabric object is selected by mouse.
      this.fabric_canvas_.on('object:selected', function(op) {
        that_.onObjectSelected_(op);
      });
      // 'object:removed' is triggered when any fabric objects is removed.
      this.fabric_canvas_.on('object:removed', function(op) {
        that_.onObjectRemoved_(op);
      });
    }
    return this.fabric_canvas_;
  }

  /* Returns counter clockwise angle in radians as required for backend */
  getAngleForBackend(angle) {
    // 1. Get the correct angle first, objects are drawn with the 180 degrees starting angle to
    // match the '0 degrees is downward' behaviour and show the editing controls in correct
    // direction.
    let backend_angle = angle - 180;
    // 2. Make it counter clockwise as required by backend.
    backend_angle = -backend_angle;
    // 3. Convert it radians as required by backend.
    backend_angle = THREE.Math.degToRad(backend_angle);
    return backend_angle;
  }

  /* Returns clockwise angle in degrees for drawing objects */
  getAngleForFrontend(angle) {
    // 1. Convert angle to degrees first.
    let frontend_angle = THREE.Math.radToDeg(angle);
    // 2. Make it clockwise as required by fabricjs/canvas.
    frontend_angle = -frontend_angle;
    // 3. Objects are drawn with the 180 degrees starting angle to match
    // the '0 degrees is downward' behaviour and show the editing controls in correct direction.
    frontend_angle += 180;
    return frontend_angle;
  }

  /* Get coordinates in pixels for visualization */
  getDisplayCoordinates(x, y) {
    let coordinates = {};
    const cell_size = this.active_map_.cellSize();
    // NOTE: we exchange x and y here.
    coordinates.x = y / cell_size;
    coordinates.y = x / cell_size;
    return coordinates;
  }

  /* Get coordinates(in meters) for map backend */
  getMapCoordinates(x, y) {
    let coordinates = {};
    const cell_size = this.active_map_.cellSize();
    // NOTE: we exchange x and y here.
    coordinates.x = y * cell_size;
    coordinates.y = x * cell_size;
    return coordinates;
  }

  /* Get RGB color from hex string */
  getColorRGB(hex_string) {
    let fabric_color = new fabric.Color(hex_string);
    return fabric_color.getSource();
  }

  /* Get hex string from RGB color */
  getColorHex(rgb_color) {
    let fabric_color = new fabric.Color();
    fabric_color.setSource(rgb_color);
    const hex_color = '#' + fabric_color.toHex();
    return hex_color;
  }

  /* Show object info like name on mouse over objects like waypoint/polygon */
  showHoverText(options) {
    let hover_text_div = document.getElementById(this.MAP_CONTAINER_OBJECT_HOVER_TEXT_ID);
    if (hover_text_div !== null) {
      hover_text_div.parentNode.removeChild(hover_text_div);
    }
    hover_text_div = document.createElement('div');
    hover_text_div.id = this.MAP_CONTAINER_OBJECT_HOVER_TEXT_ID;
    hover_text_div.style.zIndex = "99999";
    hover_text_div.style.left = options.x + "px";
    hover_text_div.style.top = options.y + "px";
    hover_text_div.style.position = "absolute";
    let hover_text_label = document.createElement('label');
    hover_text_label.style.position = "relative";
    hover_text_label.style.backgroundColor = "transparent";
    hover_text_label.appendChild(document.createTextNode(options.name));
    hover_text_div.appendChild(hover_text_label);
    let parent = document.getElementById(MapContainer().MAP_LAYER_CONTAINER_ID);
    if (parent !== null) {
      parent.appendChild(hover_text_div);
    }
  }

  /* Hides object hover text if required */
  hideHoverText() {
    let hover_text_div = document.getElementById(this.MAP_CONTAINER_OBJECT_HOVER_TEXT_ID);
    if (hover_text_div !== null) {
      hover_text_div.parentNode.removeChild(hover_text_div);
    }
  }

  // Update layer UI when editing for given layer is disabled */
  disableEditingForLayer(layer_name, layer_type) {
    this.highlightLayerText_(layer_name, layer_type, false);
  }

  /* Loads a map container from json data sent by Websight server. */
  loadMapContainer_(message) {
    if (this.discovered_ === true) {
      return;
    }
    if (!message.hasOwnProperty("data")) {
      return;
    }
    const data = message.data;
    if (!data.hasOwnProperty("maps")) {
      return;
    }
    this.view_ = this.getMapContainerView_();
    const maps = data.maps;
    for (let m in maps) {
      const map = maps[m];
      if (map.hasOwnProperty("name") && map.hasOwnProperty("data")) {
        this.addMapNameToList_(map.name);
        this.maps_[map.name] = new IsaacMap(map.name, map.data);
      }
    }
    this.discovered_ = true;
    // By default, first map is the active map.
    this.setDefaultActiveMap_();
  }

  /* Adds map name to the selection list */
  addMapNameToList_(map_name) {
    let map_list_div = document.getElementById(this.MAP_LIST_DIV_ID);
    let map_list = document.getElementById(this.MAP_LIST_ID);
    if (map_list_div === null || map_list_div === undefined) {
      map_list_div = document.createElement("div");
      map_list_div.id = this.MAP_LIST_DIV_ID;
      let label = document.createElement('label');
      const mapl_list_text = "Select map from map container:";
      label.classList.add("map-container-label-text");
      label.style.marginLeft = "5px";
      label.appendChild(document.createTextNode(mapl_list_text));
      map_list_div.appendChild(label);
      map_list = document.createElement("select");
      map_list.classList.add("map-container-label-text");
      map_list.style.marginLeft = "5px";
      map_list.id = this.MAP_LIST_ID;
      let that_ = this;
      map_list.onchange = function () {
        that_.stopEditing_();
        that_.setActiveMap_(this.value);
      };
      map_list_div.appendChild(map_list);
      let add_layers_div = document.createElement("div");
      add_layers_div.style.display = "inline";
      let add_waypoints_elem = new Image();
      add_waypoints_elem.onclick = function() {
        that_.onAddNewLayer_(that_.MapLayerTypes.WAYPOINT_MAP_LAYER);
        that_.fabric_canvas_.hoverCursor = 'default';
      }
      add_waypoints_elem.onmouseover = function() {
        this.style.opacity = 1.0;
      };
      add_waypoints_elem.onmouseup = function() {
        this.style.opacity = 0.5;
      };
      add_waypoints_elem.onmouseout = function() {
        this.style.opacity = 0.5;
      };
      add_waypoints_elem.classList.add("map-container-add-new-layer-icon");
      add_waypoints_elem.title = "Add new waypoint layer";
      add_waypoints_elem.src = "./map-assets/waypoint_layer_plus.png";
      add_waypoints_elem.style.opacity = ".5";
      let add_polygons_elem = new Image();
      add_polygons_elem.onclick = function() {
        that_.onAddNewLayer_(that_.MapLayerTypes.POLYGON_MAP_LAYER);
        that_.fabric_canvas_.hoverCursor = 'default';
      }
      add_polygons_elem.onmouseover = function() {
        this.style.opacity = 1.0;
      };
      add_polygons_elem.onmouseup = function() {
        this.style.opacity = 0.5;
      };
      add_polygons_elem.onmouseout = function() {
        this.style.opacity = 0.5;
      };
      add_polygons_elem.src = "./map-assets/polygon_layer_plus.png";
      add_polygons_elem.classList.add("map-container-add-new-layer-icon");
      add_polygons_elem.title = "Add new polygon layer";
      add_polygons_elem.style.opacity = ".5";
      add_layers_div.appendChild(add_waypoints_elem);
      add_layers_div.appendChild(add_polygons_elem);
      map_list_div.appendChild(add_layers_div);
      let container_elem = document.getElementById(this.MAP_INFO_CONTAINER_ID);
      container_elem.appendChild(map_list_div);
    }
    if(map_list.options.namedItem(map_name) === null) {
      let option = document.createElement("option");
      option.value = map_name;
      option.text = map_name;
      option.id = map_name;
      option.classList.add("map-container-label-text");
      map_list.appendChild(option);
    }
  }

 /*
  * Adds a fieldset element for give layer type.
  * Layers of same type are grouped together by using same 'fieldset'
  * for all the layers in one type.
  */
  getFieldsetForLayerType_(layer_type) {
    let fieldset_elem = document.getElementById(layer_type);
    let that_ = this;
    if (fieldset_elem === null || fieldset_elem === undefined) {
      fieldset_elem = document.createElement('fieldset');
      fieldset_elem.id = layer_type;
      let legend_elem = document.createElement('legend');
      legend_elem.style.border = "none";
      legend_elem.style.margin = "0px";
      fieldset_elem.appendChild(legend_elem);
      let container_elem = document.getElementById(this.MAP_LAYER_INFO_CONTAINER_ID);
      container_elem.appendChild(fieldset_elem);
    }
    return fieldset_elem;
  }

  /*
   * Generate id for each layer radio element so that a layer can be
   * easily identified to enable/disable.
   */
  getIdForLayerCheck_(map_name, layer_name, layer_type) {
    let id = map_name;
    id += "::";
    id += layer_type;
    id += "::";
    id += layer_name;
    return id;
  }

  /* Generates id for layer text holder */
  getIdForLayerTextHolder_(map_name, layer_name, layer_type) {
    let id = map_name;
    id += "::";
    id += layer_type;
    id += "::";
    id += layer_name;
    id += "::text_holder";
    return id;
  }

  /* Generates id for layer 'div' */
  getIdForLayerDiv_(map_name, layer_name, layer_type) {
    let id = map_name;
    id += "::";
    id += layer_type;
    id += "::";
    id += layer_name;
    id += "::layer_div";
    return id;
  }

  /* Returns div containing checkbox for a layer and edit options */
  addLayerDiv_(map_name, layer_name, layer_type) {
    let fieldset = this.getFieldsetForLayerType_(layer_type);
    let legend = document.createElement('div');
    legend.id = this.getIdForLayerDiv_(map_name, layer_name, layer_type);
    legend.style.display = "inline-flex";
    legend.style.width = "100%";
    legend.style.alignItems = "center";
    let label = document.createElement('label');
    let close_button = document.createElement('i');
    let layer_img = new Image();
    layer_img.style.marginLeft = "5px";
    layer_img.style.opacity = ".5";
    layer_img.title = layer_type;
    if (layer_type === this.MapLayerTypes.OCCUPANCY_MAP_LAYER) {
      layer_img.src = "./map-assets/baseline-map-24px.png";
    } else if (layer_type === this.MapLayerTypes.WAYPOINT_MAP_LAYER) {
      layer_img.src = "./map-assets/waypoint_layer.png";
    } else if (layer_type === this.MapLayerTypes.POLYGON_MAP_LAYER) {
      layer_img.src = "./map-assets/polygon_layer.png";
    }
    layer_img.style.width = "25px";
    layer_img.style.height = "25px";
    legend.appendChild(layer_img);
    let divspan = document.createElement('div');
    divspan.className += " control__indicator";
    let check = document.createElement('input');
    check.type = "checkbox";
    check.name = layer_type;
    check.checked = false;
    check.id = this.getIdForLayerCheck_(map_name, layer_name, layer_type);
    check.layer_name = layer_name;
    let that_ = this;
    check.addEventListener('change', function() {
      that_.stopEditing_();
      that_.closeAnyOpenDialogs();
      that_.active_map_.updateLayerStatus(this.layer_name, this.name, this.checked);
    });
    label.className = "control control--checkbox";
    label.style.marginLeft = "15px";
    label.appendChild(check);
    label.appendChild(divspan);
    legend.appendChild(label);
    let text_holder = document.createElement("div");
    text_holder.id = this.getIdForLayerTextHolder_(map_name, layer_name, layer_type);
    const layer_text = map_name + "/" + layer_name + " (" + layer_type +")";
    text_holder.appendChild(document.createTextNode(layer_text));
    text_holder.title = layer_text;
    text_holder.check = check;
    text_holder.style.marginLeft = "15px";
    text_holder.style.cursor = "pointer";
    legend.appendChild(text_holder);
    if (layer_type !== this.MapLayerTypes.OCCUPANCY_MAP_LAYER) {
      let edit_img = new Image();
      edit_img.src = "./map-assets/baseline-edit-24px.png";
      edit_img.title = "Edit layer";
      edit_img.style.opacity = ".5";
      edit_img.classList.add("map-container-layer-options");
      edit_img.classList.add("nvidia-color-on-hover");
      edit_img.onmouseover = function() {
        this.style.opacity = 1.0;
      };
      edit_img.onmouseup = function() {
        this.style.opacity = 0.5;
      };
      edit_img.onmouseout= function() {
        this.style.opacity = 0.5;
      };
      edit_img.onclick = function() {
        that_.fabric_canvas_.hoverCursor = 'default';
        that_.closeAnyOpenDialogs();
        that_.enableEditingForLayer_(layer_name, layer_type);
      }
      legend.appendChild(edit_img);
      let delete_layer = new Image();
      delete_layer.src = "./map-assets/baseline-delete-24px.png";
      // TODO(apatole): Modify title once we support deleting layers.
      delete_layer.title = "Delete layer (This operation is not yet supported)";
      delete_layer.classList.add("map-container-layer-options");
      delete_layer.classList.add("map-container-delete-layer-option");
      delete_layer.style.opacity = ".5";
      delete_layer.onmouseover = function() {
        this.style.opacity = 1.0;
      };
      delete_layer.onmouseup = function() {
        this.style.opacity = 0.5;
      };
      delete_layer.onmouseout= function() {
        this.style.opacity = 0.5;
      };
      delete_layer.onclick = function() {
        that_.fabric_canvas_.hoverCursor = 'default';
        // TODO(apatole): uncomment below line once we support deleting layers.
        // that_.onDeleteLayer_(layer_name, layer_type);
      }
      legend.appendChild(delete_layer);
    }
    fieldset.appendChild(legend);
  }

  /* Set first map as default active map. */
  setDefaultActiveMap_() {
    let map_list = document.getElementById(this.MAP_LIST_ID);
    if (map_list === null || map_list === undefined) {
      return;
    }
    if (map_list.options.length > 0) {
      map_list.selectedIndex = 0;
    }
    this.setActiveMap_(map_list.options[0].value);
  }

  /* Set active map from the maps list. */
  setActiveMap_(map_name) {
    if (this.active_map_ !==  null) {
      this.active_map_.disable();
      let editing_tools = document.getElementById(this.MAP_EDITING_TOOLS_ID);
      if (editing_tools !== null) {
        editing_tools.innerHTML = '';
      }
      let color_chooser_div = document.getElementById(this.COLOR_PICKER_ID);
      if (color_chooser_div !== null) {
        color_chooser_div.style.visibility = "hidden";
      }
    }
    if (this.maps_.hasOwnProperty(map_name)) {
      let map_layers_info_elem = document.getElementById(this.MAP_LAYER_INFO_CONTAINER_ID);
      map_layers_info_elem.innerHTML = '';
      // keep occupancy layer always at the top;
      this.getFieldsetForLayerType_(this.MapLayerTypes.OCCUPANCY_MAP_LAYER);
      this.active_map_ = this.maps_[map_name];
      this.maps_[map_name].setActive();
    }
  }

  /* Send a message to the WebsightServer. */
  sendMessage_(layer_name, layer_type, message, data) {
    if (this.web_socket_ === null) {
      return;
    }
    this.web_socket_.send(JSON.stringify({
      type: 'map',
      data: {
        request : message,
        map_name : this.active_map_.name(),
        layer_name : layer_name,
        layer_type : layer_type,
        data : data,
      }
    }));
  }

  /* Closes 'Add New Layer' dialog if already open */
  closeAddNewLayerDialog_() {
    let new_layer_dialog_div = document.getElementById(this.MAP_ADD_NEW_LAYER_DIALOG_DIV_ID);
    if (new_layer_dialog_div !== null) {
      new_layer_dialog_div.parentNode.removeChild(new_layer_dialog_div);
    }
  }

  /* Closes 'Delete Layer Confirmation' dialog if already open */
  closeDeleteLayerConfirmationDialog_() {
    let delete_layer_dialog_div =
        document.getElementById(this.MAP_LAYER_DELETE_CONFIRMATION_DIV_ID);
    if (delete_layer_dialog_div !== null) {
      delete_layer_dialog_div.parentNode.removeChild(delete_layer_dialog_div);
    }
  }

  /* Closes any open 'layer object input' dialog. */
  closeLayerObjectInputDialog_() {
    let delete_layer_input_div = document.getElementById(this.MAP_CONTAINER_LAYER_INPUT_DIV_ID);
    if (delete_layer_input_div !== null) {
      delete_layer_input_div.parentNode.removeChild(delete_layer_input_div);
    }
  }

  /* Close any open dialogs */
  closeAnyOpenDialogs() {
    this.closeAddNewLayerDialog_();
    this.closeDeleteLayerConfirmationDialog_();
    this.closeLayerObjectInputDialog_();
  }

  /* Handles UI for adding a new layer */
  onAddNewLayer_(layer_type) {
    this.closeAnyOpenDialogs();
    let layer_input_div = document.createElement('div');
    layer_input_div.id = this.MAP_ADD_NEW_LAYER_DIALOG_DIV_ID;
    layer_input_div.style.width = "600px";
    layer_input_div.style.marginTop = "5px";
    layer_input_div.style.alignItems = "center";
    let input = document.createElement('input');
    input.type = "text";
    input.value = this.MAP_LAYER_NAME_INITIAL_TEXT;
    input.style.marginLeft = "30px";
    input.size = "15";
    input.classList.add(this.MAP_LAYER_DIALOG_INPUT_TEXT_CLASS);
    let fieldset_elem = this.getFieldsetForLayerType_(layer_type);
    let btn = document.createElement('input');
    btn.style.marginLeft = "20px";
    btn.type = "button";
    btn.value = "OK";
    btn.classList.add(this.MAP_LAYER_DIALOG_PRIMARY_BTN_CLASS);
    let cancel_btn = document.createElement('input');
    cancel_btn.type = "button";
    cancel_btn.value = "Cancel";
    cancel_btn.classList.add(this.MAP_LAYER_DIALOG_SECONDARY_BTN_CLASS);
    cancel_btn.style.marginLeft = "10px";
    let that_ = this;
    input.addEventListener("keyup", function(event) {
      // Handle 'Enter' and 'Esc'
      event.preventDefault();
      if (event.keyCode === 13) {
        btn.click();
      } else if (event.keyCode === 27) {
        cancel_btn.click();
      }
    });
    cancel_btn.addEventListener("click", function() {
      fieldset_elem.removeChild(layer_input_div);
    });
    let container_elem = document.getElementById(this.MAP_LAYER_INFO_CONTAINER_ID);
    layer_input_div.appendChild(input);
    let color_picker = document.createElement('input');
    if (layer_type === this.MapLayerTypes.POLYGON_MAP_LAYER) {
      color_picker.type = "color";
      color_picker.style.marginLeft = "5px";
      color_picker.style.backgroundColor = "transparent";
      color_picker.style.width = "25px";
      color_picker.style.height = "25px";
      color_picker.style.padding = "2px";
      color_picker.value = "#ff0000";
      color_picker.title = "Choose color";
      let color_picker_label = document.createElement('label');
      color_picker_label.appendChild(document.createTextNode("Color :"));
      color_picker_label.style.marginLeft = "20px";
      layer_input_div.appendChild(color_picker_label);
      layer_input_div.appendChild(color_picker);
    }
    btn.onclick = function() {
      const maybe_layer_name = that_.active_map_.isValidLayerName(input.value);
      if (maybe_layer_name.invalid) {
        msgPopup(fieldset_elem, 'error', maybe_layer_name.message);
        return;
      }
      fieldset_elem.removeChild(layer_input_div);
      that_.addMapLayer_(that_.active_map_.name(), input.value, layer_type);
      let data = {};
      if (color_picker !== undefined) {
        const fabric_color = new fabric.Color(color_picker.value);
        const color = fabric_color.getSource();
        data["color"] = [color[0], color[1], color[2]];
      }
      that_.active_map_.addNewLayer(input.value, layer_type, data);
      that_.enableEditingForLayer_(input.value, layer_type);
      that_.sendMessage_(input.value, layer_type, that_.MapEvents.MAP_EVENT_LAYER_ADDED, data);
    }
    layer_input_div.appendChild(btn);
    layer_input_div.appendChild(cancel_btn);
    fieldset_elem.appendChild(layer_input_div);
    input.select();
  }

  /* Confirmation dialog to delete a layer */
  onDeleteLayer_(layer_name, layer_type) {
    const layer_div_id = this.getIdForLayerDiv_(this.active_map_.name(), layer_name, layer_type);
    let layer_div_elem = document.getElementById(layer_div_id);
    if (layer_div_elem === null) {
      return;
    }
    this.closeAnyOpenDialogs();
    let delete_confirmation_div = document.createElement('div');
    delete_confirmation_div.id = this.MAP_LAYER_DELETE_CONFIRMATION_DIV_ID;
    let label = document.createElement('label');
    label.style.marginLeft = "100px";
    const label_text = "Delete " + layer_name + "?";
    label.appendChild(document.createTextNode(label_text));
    label.style.color = "#ff0000";
    let ok_btn = document.createElement('input');
    ok_btn.style.marginLeft = "20px";
    ok_btn.type = "button";
    ok_btn.value = "OK";
    ok_btn.classList.add(this.MAP_LAYER_DIALOG_PRIMARY_BTN_CLASS);
    let cancel_btn = document.createElement('input');
    cancel_btn.type = "button";
    cancel_btn.value = "Cancel";
    cancel_btn.style.marginLeft = "10px";
    cancel_btn.classList.add(this.MAP_LAYER_DIALOG_SECONDARY_BTN_CLASS);
    delete_confirmation_div.appendChild(label);
    delete_confirmation_div.appendChild(ok_btn);
    delete_confirmation_div.appendChild(cancel_btn);
    let fieldset = this.getFieldsetForLayerType_(layer_type);
    if (layer_div_elem.nextSibling !== null) {
      fieldset.insertBefore(delete_confirmation_div, layer_div_elem.nextSibling);
    } else {
      fieldset.appendChild(delete_confirmation_div);
    }
    let that_ = this;
    ok_btn.onclick = function() {
      fieldset.removeChild(delete_confirmation_div);
      that_.removeMapLayer_(that_.active_map_.name(), layer_name, layer_type);
      that_.active_map_.removeLayer(layer_name)
      that_.sendMessage_(layer_name, layer_type, that_.MapEvents.MAP_EVENT_LAYER_REMOVED);
    }
    cancel_btn.onclick = function() {
      fieldset.removeChild(delete_confirmation_div);
    }
  }


  /* Update status of all layers when layer radio boxes are programatically selected. */
  onUpdateAllLayerStatus_(layer_type) {
    let radios = document.getElementsByName(layer_type);
    for (let radio of radios) {
      this.active_map_.updateLayerStatus(radio.layer_name, radio.name, radio.checked);
    }
  }

  /* Enables editing for given layer */
  enableEditingForLayer_(layer_name, layer_type) {
    this.current_editing_action_ = this.EditingAction.NONE;
    let checkboxes = document.getElementsByName(layer_type);
    for (let checkbox of checkboxes) {
      if (checkbox.layer_name === layer_name && checkbox.name === layer_type) {
        checkbox.checked = true;
        this.active_map_.enableEditingForLayer(layer_name, layer_type);
        this.getEditingTools_(layer_type, false);
      } else {
        checkbox.checked = false;
        this.active_map_.updateLayerStatus(checkbox.layer_name, checkbox.name, checkbox.checked);
      }
      this.highlightLayerText_(checkbox.layer_name, checkbox.name, checkbox.checked);
    }
  }

  /* Highlights given layer text or removes highlight */
  highlightLayerText_(layer_name, layer_type, highlight) {
    const text_id = this.getIdForLayerTextHolder_(this.active_map_.name(), layer_name, layer_type);
    let text_elem = document.getElementById(text_id);
    if (text_elem !== null) {
      if (highlight) {
        text_elem.style.fontWeight = "bold";
      } else {
        text_elem.style.fontWeight = "normal";
      }
    }
  }

  /* Stops editing layers, updates layers UI accordingly */
  stopEditing_ () {
    this.current_editing_action_ = this.EditingAction.NONE;
    if (this.active_map_ !== null) {
      this.active_map_.stopEditing();
    }
    let waypoint_checkboxes = document.getElementsByName(this.MapLayerTypes.WAYPOINT_MAP_LAYER);
    for (let checkbox of waypoint_checkboxes) {
      this.highlightLayerText_(checkbox.layer_name, checkbox.name, false);
    }
    let polygon_checkboxes = document.getElementsByName(this.MapLayerTypes.POLYGON_MAP_LAYER);
    for (let checkbox of polygon_checkboxes) {
      this.highlightLayerText_(checkbox.layer_name, checkbox.name, false);
    }
    let editing_tools = document.getElementById(this.MAP_EDITING_TOOLS_ID);
    if (editing_tools !== null) {
      editing_tools.innerHTML = '';
    }
    let color_chooser_div = document.getElementById(this.COLOR_PICKER_ID);
    if (color_chooser_div !== null) {
      color_chooser_div.style.visibility = "hidden";
    }
  }

  /*
   * Creates and send json message to Websight server when new objects
   * are added to the fabric canvas.
   */
  onObjectAdded_(op) {
    if ((op.target.localType === 'waypoint' || op.target.localType === 'polygon' ) &&
        op.target.should_trigger_event === true) {
      const color = this.getColorRGB(op.target.localColor);
      // Backend Rotations are counter clockwise.
      let wp_angle = this.getAngleForBackend(op.target.get('angle'));
      const data = {
        poseType: '2d',
        name : op.target.name,
        color : [color[0], color[1], color[2]],
      };
      const layer_type = this.active_map_.getActiveLayerType();
      const layer_name = this.active_map_.getActiveLayerName();
      if (op.target.localType === 'waypoint') {
        const map_coordinates = this.getMapCoordinates(op.target.left, op.target.top);
        data["pose"] = [wp_angle, map_coordinates.x, map_coordinates.y];
        data["radius"] = op.target.get('waypoint_radius');
      } else if (op.target.localType === 'polygon') {
        let poly_points = op.target.get('points');
        let poly_points_xy = [];
        for (let point of poly_points) {
          const map_coordinates = this.getMapCoordinates(point.x, point.y);
          poly_points_xy.push([map_coordinates.x, map_coordinates.y]);
        }
        data["points"] = poly_points_xy;
      }
      this.sendMessage_(layer_name, layer_type, this.MapEvents.MAP_EVENT_OBJECT_ADDED, data);
    }
  }

  /*
   * Creates and sends json message to Websight server when existing
   * fabric canvas objects are modified.
   */
  onObjectModified_(op) {
    if (op.target.localType === 'waypoint' || op.target.localType === 'polygon') {
      const color = this.getColorRGB(op.target.localColor);
      // Backend Rotations are counter clockwise.
      let wp_angle = this.getAngleForBackend(op.target.get('angle'));
      const data = {
        poseType: '2d',
        name : op.target.name,
        color : [color[0], color[1], color[2]],
      };
      const layer_type = this.active_map_.getActiveLayerType();
      const layer_name = this.active_map_.getActiveLayerName();
      if (op.target.localType === 'waypoint') {
        const map_coordinates = this.getMapCoordinates(op.target.left, op.target.top);
        data["pose"] = [wp_angle, map_coordinates.x, map_coordinates.y];
        data["radius"] = op.target.get('waypoint_radius');
        data["radius"] = op.target.get('waypoint_radius') * op.target.scaleX;
      } else if (op.target.localType === 'polygon') {
        let poly_points = op.target.get('points');
        // Get transformed points for fabric polygon
        let matrix = op.target.calcTransformMatrix();
        let minX = fabric.util.array.min(poly_points, 'x');
        let minY = fabric.util.array.min(poly_points, 'y');
        let transformed_points = poly_points.map(function(p) {
          return new fabric.Point(p.x - minX - op.target.width/2,
              p.y - minY -  op.target.height/2);
          }).map(function(p) {
            return fabric.util.transformPoint(p, matrix);
          });
        let poly_points_xy = [];
        for (let point of transformed_points) {
          const map_coordinates = this.getMapCoordinates(point.x, point.y);
          poly_points_xy.push([map_coordinates.x, map_coordinates.y]);
        }
        data["points"] = poly_points_xy;
      }
      this.sendMessage_(layer_name, layer_type, this.MapEvents.MAP_EVENT_OBJECT_MODIFIED, data);
    }
  }

  /*
   * Creates and sends json message to Websight server when objects
   * are deleted from fabric canvas.
   */
  onObjectRemoved_(op) {
    if (this.current_editing_action_ === this.EditingAction.REMOVING_OBJECTS) {
      if (op.target.localType === 'waypoint' || op.target.localType === 'polygon') {
        const data = {
          name : op.target.name
        }
        const layer_type = this.active_map_.getActiveLayerType();
        const layer_name = this.active_map_.getActiveLayerName();
        this.sendMessage_(layer_name, layer_type, this.MapEvents.MAP_EVENT_OBJECT_REMOVED, data);
      }
    }
  }

  /*
   * Creates and sends json message to Websight server when objects
   * are selected on the fabric canvas.
   */
  onObjectSelected_(op) {
    if (this.current_editing_action_ === this.EditingAction.REMOVING_OBJECTS) {
      if (op.target.localType === 'waypoint' || op.target.localType === 'polygon') {
        this.active_map_.removeObject(op.target.name);
        this.hideHoverText();
      }
    }
  }

  /* creates a color chooser div */
  getColorChooser_() {
    let color_chooser = document.createElement('input');
    let color_chooser_div = document.createElement('div');
    color_chooser_div.id = this.COLOR_PICKER_DIV_ID;
    color_chooser.type = "color";
    color_chooser.id = this.COLOR_PICKER_ID;
    color_chooser.value = "#ff0000";
    color_chooser_div.appendChild(color_chooser);
    this.color_chooser_ = color_chooser;
    return color_chooser_div;
  }

  /*
   * Loads editing tools required for editing map layers
   * 1. Color chooser icon to select a object color, current choosen color is used for all
   *        the further drawing of objects. Color can be changed by clicking on color chooser icon
   *        and selecting color.
   * 2. 'Draw Waypoints/Polygons' icon which upon clicking activates 'Draw Waypoint/Polygon' action.
   *     Further clicks on canvas will draw waypoints at current mouse location.
   * 3. 'Stop Drawing' icon which upon clicking will stop drawing objects on mouse clicks.
   * 4. 'Remove Objects' icon which upon clicking will activate 'Remove Waypoints' action and
   *        after this, selecting any object will remove object from the layer and will trigger
   *        required events.
   */
  getEditingTools_(layer_type, is_drawing) {
    let editing_tools = document.getElementById(this.MAP_EDITING_TOOLS_ID);
    if (editing_tools !== null) {
      editing_tools.innerHTML = '';
    }
    let that_ = this;
    if (!is_drawing) {
      if (layer_type === this.MapLayerTypes.WAYPOINT_MAP_LAYER) {
        let draw_waypoint = document.createElement('img');
        draw_waypoint.src = "./map-assets/draw_waypoint.png";
        draw_waypoint.classList.add("map-container-editing-tools-icon");
        draw_waypoint.style.width = "35px";
        draw_waypoint.style.height = "35px";
        draw_waypoint.title = "Start drawing waypoints"
        draw_waypoint.addEventListener("click", function() {
          that_.current_editing_action_ = that_.EditingAction.DRAWING_WAYPOINTS;
          that_.fabric_canvas_.hoverCursor = 'crosshair';
          that_.getEditingTools_(layer_type, true);
        });
        editing_tools.appendChild(draw_waypoint);
      } else if (layer_type === this.MapLayerTypes.POLYGON_MAP_LAYER) {
        let draw_polygon = document.createElement('img');
        draw_polygon.classList.add("map-container-editing-tools-icon");
        draw_polygon.style.width = "25px";
        draw_polygon.style.height = "25px";
        draw_polygon.title = "Start drawing polygons"
        draw_polygon.src = "./map-assets/draw_polygon.png";
        draw_polygon.addEventListener("click", function() {
          that_.current_editing_action_ = that_.EditingAction.DRAWING_POLYGONS;
          that_.fabric_canvas_.hoverCursor = 'crosshair';
          that_.getEditingTools_(layer_type, true);
        });
        editing_tools.appendChild(draw_polygon);
      }
    } else {
      let stop_drawing = document.createElement('img');
      stop_drawing.src = "./map-assets/stop_recording.png";
      stop_drawing.classList.add("map-container-editing-tools-icon");
      stop_drawing.style.width = "28px";
      stop_drawing.style.height = "28px";
      stop_drawing.title = "Stop drawing";
      stop_drawing.addEventListener("click", function() {
        that_.current_editing_action_ = that_.EditingAction.NONE;
        that_.fabric_canvas_.hoverCursor = 'default';
        that_.getEditingTools_(layer_type, false);
      });
      editing_tools.appendChild(stop_drawing);
    }
    let remove_object = document.createElement('img');
    remove_object.src = "./map-assets/remove_object.png";
    remove_object.classList.add("map-container-editing-tools-icon");
    remove_object.style.width = "30px";
    remove_object.style.height = "30px";
    remove_object.title = "Start removing objects";
    remove_object.addEventListener("click", function() {
      that_.current_editing_action_ = that_.EditingAction.REMOVING_OBJECTS;
      that_.fabric_canvas_.hoverCursor = 'not-allowed';
      that_.getEditingTools_(layer_type, false);
    });
    editing_tools.appendChild(remove_object);
    let color_chooser_div = document.getElementById(this.COLOR_PICKER_ID);
    if (layer_type === this.MapLayerTypes.WAYPOINT_MAP_LAYER) {
      color_chooser_div.style.visibility = "visible";
    } else {
      color_chooser_div.style.visibility = "hidden";
    }
    editing_tools.style.left = this.getLeftForEditingTools_();
    return editing_tools;
  }

  /* Gets the left position for editing tools to position them dynamically always at right top */
  getLeftForEditingTools_() {
    let canvas_wrapper_elem = document.getElementById(this.MAP_CONTAINER_LAYER_CANVAS_WRAPPED_ID);
    let canvas_elem = document.getElementById(this.MAP_CANVAS_ID);
    let client_width = canvas_wrapper_elem.clientWidth;
    // Choose right of either canvas or wrapper, whichever is smaller.
    if (canvas_elem.clientWidth < client_width) {
      client_width = canvas_elem.clientWidth;
    }
    let editing_tools_elem = document.getElementById(this.MAP_EDITING_TOOLS_ID);
    let left = canvas_wrapper_elem.offsetLeft + client_width;
    left -= editing_tools_elem.offsetWidth;
    left -= 25; // Some margin on right side
    left += "px";
    return left;
  }

  /* Returns the div containing the map container view, if it does not exist yet create it. */
  getMapContainerView_() {
    let map_container_view = document.getElementById(this.WIN_MAP_CONTAINER_ID);
    if (map_container_view === null || map_container_view === undefined) {
      map_container_view = document.createElement("div");
      map_container_view.id = this.WIN_MAP_CONTAINER_ID;

      // TODO(apatole): Remove WIP flag once the window is completely functional.
      WindowManager().createWindow(map_container_view,
          "Map Container", {resize: false, onresize: function (obj) {
       // TODO(apatole): handle resize if required
      }});

      let editing_tools = document.createElement('div');
      editing_tools.id = this.MAP_EDITING_TOOLS_ID;
      map_container_view.appendChild(editing_tools);
      map_container_view.appendChild(this.getColorChooser_());
      let map_layer_container = document.createElement("div");
      map_layer_container.id = this.MAP_LAYER_CONTAINER_ID;
      let map_info_container = document.createElement("div");
      map_info_container.id = this.MAP_INFO_CONTAINER_ID;
      let map_layer_info_container = document.createElement("div");
      map_layer_info_container.id = this.MAP_LAYER_INFO_CONTAINER_ID;
      map_container_view.appendChild(map_layer_container);
      map_container_view.appendChild(map_info_container);
      map_container_view.appendChild(map_layer_info_container);
    }

    return map_container_view;
  }
}

let map_container_ = null;

/* Implement a "singleton" of MapContainer */
function MapContainer() {
  if (map_container_ === null) {
    map_container_ = new MapContainerImpl();
  }
  return map_container_;
}

// Fabric extensions
// Get fabric canvas object by id. While adding/modifying objects to the layer,
// this is required to check if the object with given id is already added
fabric.Canvas.prototype.getItemByID = function(myID) {
  let object = null,
  objects = this.getObjects();
  for (let i = 0, len = this.size(); i < len; i++) {
    if (objects[i].id && objects[i].id === myID) {
      object = objects[i];
      break;
    }
  }
  return object;
};
