/*
  Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

  NVIDIA CORPORATION and its licensors retain all intellectual property
  and proprietary rights in and to this software, related documentation
  and any modifications thereto. Any use, reproduction, disclosure or
  distribution of this software and related documentation without an express
  license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/*
  Creates a widget for controlling a set of 3D markers.
  Each marker lets the user represent (and edit) a position and an orientation (pose).
 */
function Create3DMarkersWidget(scene, camera, renderer) {
  /** @private Dictionary of ThreeJS objects that represent the markers */
  let markerObjects_ = {};
  /** @private Dictionary of markers status relative to this widget */
  let markers_ = {};
  /** @private Array that contains all the meshes for raycasting */
  let allComponents_ = [];
  /** @private Reference to the selected widget */
  let selectedMarker_ = null;
  /** @private Transformation mode */
  let transMode_ = 'translate';
  /** @private Material for the selected marker */
  let materialSelected_ = new THREE.MeshLambertMaterial( { color: 0x00ffff} );
  /** @private Temporal array of materials */
  let auxMaterials_ = [];
  /** @private Reference of the scene */
  const scene_ = scene;
  /** @private Reference of the camera */
  const camera_ = camera;
  /** @private Raycaster for picking a marker */
  const raycaster_ = new THREE.Raycaster();
  /** @private Manipulator controls for picking a marker */
  const manipulCtrl_ = new THREE.TransformControls(camera, renderer.domElement);
  // Validate scene
  if (typeof scene_ === 'undefined' || scene_ === null || !(scene_ instanceof THREE.Scene)) {
    throw "Invalid scene to create this markers widget";
  }
  // Validate camera
  if (typeof camera_ === 'undefined' || camera_ === null || !(camera_ instanceof THREE.Camera)) {
    throw "Invalid camera to create this markers widget";
  }
  // Validate renderer
  if (typeof renderer === 'undefined' || renderer === null ||
      !(renderer instanceof THREE.WebGLRenderer)) {
    throw "Invalid renderer to create this markers widget";
  }
  /** @private validate a marker name for this widget */
  let validateMarker_ = (markerName) => {
    if (typeof markerName === 'undefined' || markerName === null) {
      return false;
    }
    if (!markers_.hasOwnProperty(markerName)) {
      return false;
    }
    return true;
  };

  /** @private Make markerName visible/invisible in the scene */
  let setVisibility_ = (markerName, visibleValue) => {
    if (!validateMarker_(markerName)) {
      throw 'Cannot change visibility of ' + markerName + ' since it is not in this widget';
    }
    const marker = markerObjects_[markers_[markerName].key];
    // We set visibility for the object and all of it's children.
    // If we only set the top level object, it won't be rendered but it's children will still be
    // rendered when doing ray casting, so you could still interact with it.
    marker.children.forEach(
      function (group) {
        group.children.forEach(
          function (mesh) {
            mesh.visible = visibleValue;
          }
        );
        group.visible = visibleValue;
      }
    );
    marker.visible = visibleValue;
    // Register current value
    markers_[markerName].visibility = visibleValue;
  };

  /** @private Add all the markers's segement to our array for potential ray casting*/
  let addMarkerToArray_ = (marker) => {
    marker.children.forEach(
      function (group) {
        group.children.forEach(
          function (mesh) {
            allComponents_.push(mesh);
          }
        );
      }
    );
  };

  /** @private Add the marker to our internal array */
  let addMarkerInternalArray_ = (markerName, markerObject, interactivity) => {
    // Add ThreeJS object to our dictionary
    markerObjects_[markerObject.uuid] = markerObject;
    markers_[markerName] = {
      key : markerObject.uuid,
      activeIntreaction : false,
      visibility : true,
      interactivity : interactivity
    };
  };

  /** @private Reconstruct marker's array for ray casting */
  let reconstructMarkerArray_ = () => {
    allComponents_ = [];
    for (let key in markerObjects_) {
      addMarkerToArray_(markerObjects_[key]);
    }
  };

  /** @private Set this markers Pose */
  let setMarkerPose_ = (marker, positionWorld, rotationWorld, scale = 1.0) => {
    marker.position.copy(positionWorld);
    marker.quaternion.copy(rotationWorld);
    marker.scale.copy(new THREE.Vector3(scale, scale, scale));
  };

  /** @private Make newObject the new selected marker for transformation */
  let selectLogic_ = (newObject) => {
    if (newObject === null) {
      throw 'Impossible to select null object';
    }
    // Return previous selected object to original material
    if (selectedMarker_ !== null) {
      returnMaterial_(selectedMarker_);
    }
    // update selection
    selectedMarker_ = newObject;
    // Store the current selected object's material
    changeMaterial_();
    // add manipulation controls to object
    if (manipulCtrl_ !== null) {
      manipulCtrl_.enable = true;
      manipulCtrl_.setMode(transMode_);
      scene.add(manipulCtrl_);
      manipulCtrl_.attach(selectedMarker_);
    }
  };

  /** @private Remove selection i.e no marker is selected for transformation */
  let unSelectLogic_ = () => {
    // Return previous selected object to original material
    if (selectedMarker_ !== null) {
      returnMaterial_(selectedMarker_);
    }
    // unselect
    selectedMarker_ = null;
    // remove manipulator
    if (manipulCtrl_ !== null) {
      manipulCtrl_.enable = false;
      scene_.remove(manipulCtrl_);
    }
  };

  /** @private Given the current mouse position see if we hit a marker, if so select it
  if not, remove selection */
  let pickingLogic_ = (mouse) => {
    // update the picking ray with the camera and mouse position
    raycaster_.setFromCamera(mouse, camera_);
    // calculate objects intersecting the picking ray
    const intersections = raycaster_.intersectObjects(allComponents_);
    // see if we hit something
    if (intersections.length > 0) {
      const intersection = intersections[0];
      const group = intersection.object.parent;
      const object = group.parent;
      hoveredMarker = markerObjects_[object.uuid];
      // validate this to be an interactive marker
      let interactiveMarker;
      for (let name in markers_) {
        if (markers_[name].key == hoveredMarker.uuid) {
          interactiveMarker = markers_[name].interactivity;
          break;
        }
      }
      // is a new selection?
      if (hoveredMarker != selectedMarker_ && interactiveMarker) {
        selectLogic_(hoveredMarker);
      }
    } else if (!manipulCtrl_.dragging) {  // We only unselect if we are not manipulating
      unSelectLogic_();
    }
  };

  /** @private Store a copy of the current selected object's material and change it to selected
  material */
  let changeMaterial_ = () => {
    auxMaterials_ = [];
    for (let j = 0; j < selectedMarker_.children.length; ++j) {
      const group = selectedMarker_.children[j];
      let groupMaterials = [];
      for (let i = 0; i < group.children.length; ++i) {
        groupMaterials[i] = group.children[i].material;
        group.children[i].material = materialSelected_;
      }
      auxMaterials_[j] = groupMaterials;
    }
  };

  /** @private Apply the stored material to the given marker object */
  let returnMaterial_ = (markerObject) => {
    if (typeof markerObject === 'undefined' || markerObject === null) {
      throw 'Cannot return material: invalid marker Object';
      return;
    }
    for (let j = 0; j < markerObject.children.length; ++j) {
      const group = markerObject.children[j];
      for (let i = 0; i < group.children.length; ++i) {
        group.children[i].material = auxMaterials_[j][i];
      }
    }
  };

  /** Delete a marker from the widget */
  let deleteMarker_ = (markerObject) => {
    if (typeof markerObject === 'undefined' || markerObject === null) {
      throw 'Invalid Marker Object to delete';
      return;
    }
    if (markerObjects_[markerObject.uuid] === null) {
      throw 'This markerObject is not registered in this widget';
      return;
    }
    // Clean memory. Remember, this works because no two markers share resources
    markerObject.children.forEach(
      function (group) {
        group.children.forEach(
          function (mesh) {
            mesh.geometry.dispose();
            mesh.material.dispose();
          }
        );
      }
    );
    // I do not need to recursivelly remove it
    scene.remove(markerObject);
    delete markerObjects_[markerObject.uuid];
  };

  /** Reset this widget: remove all markers from this widget */
  let reset = () => {
    for (let key in markerObjects_) {
      deleteMarker_(markerObjects_[key]);
    }
    auxMaterials_ = [];
    markerObjects_ = {};
    selectedMarker_ = null;
    markers_ = {};
    if (manipulCtrl_ !== null) {
      manipulCtrl_.enable = false;
      scene_.remove(manipulCtrl_);
    }
  };

  /** Change manipulator control to rotate mode */
  let setRotationMode = () => {
    transMode_ = 'rotate';
    manipulCtrl_.setMode(transMode_);
  };

  /** Change manipulator control to translate mode */
  let setTranslationMode = () => {
    transMode_ = 'translate';
    manipulCtrl_.setMode(transMode_);
  };

  /** Check if a marker with the given name is in this widget */
  let containsMarker = (markerName) => {
    return validateMarker_(markerName);
  };

  /** Add a Marker named name with the given pose */
  let addMarker = (name, positionWorld, orientationWorld, interactivity) => {
    // validate no duplicate markers
    if (name in markers_) {
      throw 'Marker named: ' + name + ' already added';
    }
    // allocate object and update internal data structures
    const markerObject = createMarker();
    addMarkerInternalArray_(name, markerObject, interactivity);
    addMarkerToArray_(markerObject);
    // give initial pose to marker
    setMarkerPose_(markerObject, positionWorld, orientationWorld);
    // add marker to scene
    scene_.add(markerObject);
  };

  /** Sync channels and markers for hide/show  */
  let updateVisibilityStatus = (channels) => {
    for (let i in channels) {
      for (let name in markers_) {
        if (channels[i].c === name) {
          if (markers_[name].visibility != channels[i].a) {
            setVisibility_(name, channels[i].a);
          }
          break;
        }
      }
    }
  };

  /** Set all the markers to the given poses in the transform array */
  let set = (transforms) => {
    for (let name in transforms) {
      if (markers_[name].activeIntreaction) {
        throw "Forcefully setting transforms while widget '" + name +
            "' is interactive, please call disableInteraction method first";
      }
      const marker = markerObjects_[markers_[name].key];
      setMarkerPose_(marker, transforms[name].position, transforms[name].orientation,
                     transforms[name].scale);
    }
  };

  /** Return an array containing all the markers names on this widget */
  let getMarkersNames = () => {
    names = [];
    for (let key in markers_) {
      names.push(key);
    }
    return names;
  }

  /** Test if the marker named markerName is active
      "active" means the markers is currently being selected or manipulated */
  let markerIsActive = (markerName) => {
    if (!validateMarker_(markerName)) {
      throw 'Cannot test if ' + markerName + ' is active since it is not in this widget';
    }
    // see if is the selected marker
    if (selectedMarker_ === markerObjects_[markers_[markerName].key]) {
      return true;
    }

    return false;
  };

  /** Disable markerName for manipulating */
  let disableInteraction = (markerName) => {
    if (!validateMarker_(markerName)) {
      throw 'Cannot disable interaction of ' + markerName + ' since it is not in this widget';
    }
    // if this is the selected marker we cannot disable it
    if (selectedMarker_ === markerObjects_[markers_[markerName].key]) {
      throw "Cannot disable interaction gracefully while there is an active interaction, "
          + "please call stealInteraction";
    }
    markers_[markerName].activeIntreaction = false;
  };

  /** Enable markerName for manipulating */
  let enableInteraction = (markerName) => {
    if (!validateMarker_(markerName)) {
      throw 'Cannot enable interaction of ' + markerName + ' since it is not in this widget';
    }
    markers_[markerName].activeIntreaction = true;
  };

  /** Query interaction value */
  let interactionEnabled = (markerName) => {
    if (!validateMarker_(markerName)) return false;
    return markers_[markerName].activeIntreaction;
  }

  /** Get the Pose of the marker named markerName */
  let transform = (markerName) => {
    if (!validateMarker_(markerName)) {
      throw 'Cannot transform ' + markerName + ' since it is not in this widget';
    }
    const marker = markerObjects_[markers_[markerName].key];
    return {
      position: marker.position.clone(),
      orientation: marker.quaternion.clone()
    }
  }

  /** Query if a marker is currentlly selected in this widget */
  let isSelected = () => {
    return selectedMarker_ !== null;
  }

  /** If the widget contains a marker that the channels does not, make it invisible */
  let syncWithChannels = (channels) => {
    for (let name in markers_) {
      let found = false;
      for (let i in channels) {
        if (channels[i].c === name) {
          found = true;
          break;
        }
      }
      if (!found) {
        setVisibility_(name, false);
      }
    }
  }

  /* Try to update selection */
  let selectionEvent = (cursorPositionCanvas) => {
    if (typeof cursorPositionCanvas !== 'undefined' && cursorPositionCanvas !== null) {
      let cursorPosition_normalizedDeviceCoordinates = cursorPositionCanvas.clone();
      cursorPosition_normalizedDeviceCoordinates.multiplyScalar(2);
      // Test here is we are not transforming ?
      pickingLogic_(cursorPosition_normalizedDeviceCoordinates);
    }
  }

  return {
    "reset": reset,
    "setRotationMode": setRotationMode,
    "setTranslationMode": setTranslationMode,
    "containsMarker": containsMarker,
    "updateVisibilityStatus": updateVisibilityStatus,
    "addMarker": addMarker,
    "set": set,
    "selectionEvent": selectionEvent,
    "syncWithChannels": syncWithChannels,
    "transform": transform,
    "isSelected": isSelected,
    "getMarkersNames": getMarkersNames,
    "markerIsActive": markerIsActive,
    "disableInteraction": disableInteraction,
    "enableInteraction": enableInteraction,
    "interactionEnabled": interactionEnabled
  };
}
