/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/* OccupancyGridMapLayer - A class representating single occupancy map layer */
class OccupancyGridMapLayer {
  constructor(map, layer_name, layer_type) {
    this.map_ = map;
    this.name_ = layer_name;
    this.type_ = layer_type;
    this.fabric_canvas_ = null;
    this.rendered_ = false;
    this.enabled_ = false;
    this.has_objects_ = false;
    this.occupancy_object_ = null;
  }

  /* Returns name of the layer. */
  name() {
    return this.name_;
  }

  /** Returns type of the layer */
  type() {
    return this.type_;
  }

  /* Returns if the current layer is enabled. */
  enabled() {
    return this.enabled_;
  }

  /* Enables layer */
  enable() {
    if (this.enabled_ === true) {
      return;
    }
    this.enabled_ = true;
  }

  /* Disables layer */
  disable() {
    if (this.enabled_ === false) {
      return;
    }
    if (this.occupancy_object_ !== null) {
      this.fabric_canvas_.remove(this.occupancy_object_);
      this.occupancy_object_ = null;
    }
    this.enabled_ = false;
    this.rendered_ = false;
  }

  /* Marks that the layer has objects */
  setHasObjects(has_objects) {
    this.has_objects_ = has_objects;
  }

  /* Checks if the layer really has objects */
  hasObjects() {
    return this.has_objects_;
  }

  /* Visualize layer with given json data */
  renderLayer(message) {
    if (this.rendered_ === true) {
      // Layer needs to be fully rendered only once
      // and then there will be a specific update.
      return;
    }
    if (this.fabric_canvas_ === null) {
      this.fabric_canvas_ = MapContainer().getFabricCanvas();
    }
    this.fabric_canvas_.setWidth(message.width);
    this.fabric_canvas_.setHeight(message.height);
    let that_ = this;
    fabric.Image.fromURL(message.data, function(occupancyMapImg) {
      occupancyMapImg.hasBorders = false;
      occupancyMapImg.hasControls = false;
      occupancyMapImg.lockUniScaling = true;
      occupancyMapImg.selectable = false;
      occupancyMapImg.lockMovementX = true;
      occupancyMapImg.lockMovementY = true;
      occupancyMapImg.on('mousedown', function(op) {
        if (op.button === 1) {
          that_.map_.onMouseLeftButtonDown(op);
        } else if (op.button === 3) {
          that_.map_.onMouseRightButtonDown(op);
        }
      });
      that_.fabric_canvas_.add(occupancyMapImg);
      that_.rendered_ = true;
      that_.has_objects_ = true;
      that_.occupancy_object_ = occupancyMapImg;
      that_.fabric_canvas_.sendToBack(occupancyMapImg);
    });
  }

}
