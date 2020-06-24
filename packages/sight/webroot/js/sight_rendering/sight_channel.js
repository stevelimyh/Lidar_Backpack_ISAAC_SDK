/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Channels, hold a list of operations to be displayed
class SightChannel {
  constructor(name) {
    // List of renderer this channel is part of.
    this.renderers_ = [];
    // List of operations to be rendered
    this.operations_ = [];
    // how many object are currently be loaded
    this.loading_ = 0;
    // Name of the channel
    this.name_ = name;
    // Whether or not the channel is enabled.
    this.status_ = true;
  }

  // Returns whether or not a channel is empty
  empty() {
    return this.operations_.length == 0;
  }

  // Remove all operations.
  clear() {
    for (let i = 0; i < this.operations_.length; i++) {
      let op = this.operations_[i];
      if (op.data.delete_callback) {
        for (let c in op.data.delete_callback) {
          op.data.delete_callback[c](op.data);
        }
        op.data.delete_callback = [];
      }
    }
    this.operations_ = [];
  }

  // Returns the name of the channel
  getName() {
    return this.name_;
  }

  // Add an operation
  addOperation(ops) {
    this.process(ops.v, !ops.v.f && ops.t ? ops.t : null);
  }

  // If this channel is ready, notify the renderer to render
  notifyRenderer() {
    const that = this;
    for (let id in that.renderers_) {
      that.renderers_[id].update(that);
    }
  }

  // Add a renderer
  addRenderer(renderer) {
    // Make sure we don't add it twice;
    this.removeRenderer(renderer);
    this.renderers_.push(renderer);
  }

  // Remove a renderer
  removeRenderer(renderer) {
    this.renderers_ = this.renderers_.filter(item => item != renderer);
  }

  // Render the list of operations using a callback function
  render(renderer, target_time) {
    const op = this.getOperation(target_time);
    if (!op) return;
    const time = op.time === null ? target_time : op.time;
    renderer.render(op.data, time);
  }

  // Return the operation the closest of a given time
  getOperation(time) {
    if (time === undefined) {
      return this.operations_[this.operations_.length - 1];
    }
    if (this.operations_.length == 0) return null;
    let start = 0;
    let end = this.operations_.length;
    while (start + 1 < end) {
      const mid = (start + end) >> 1;
      if (this.operations_[mid].time <= time) {
        start = mid;
      } else {
        end = mid;
      }
    }
    return this.operations_[start];
  }

  // Notify of a change of status
  changeStatus(status) {
    if (status !== this.status_) {
      this.status_ = status;
      for (let id in this.renderers_) {
        this.renderers_[id].updateChannelStatus(this, status);
      }
    }
  }

  // Process a list of operations at a given time
  process(ops, time) {
    // Skip frame...
    if (this.loading_ > 0) return;
    let that = this;

    let postProcessImpl = function(op) {
      // How many seconds of data de we keep (2s)
      const kTimeWindow = 2.0;
      const start_time = PoseTree().now() - kTimeWindow;
      // Let renderer register callback when the object is being deleted.
      ops.delete_callback = [];
      that.operations_.push({data: ops, time: time});
      // Remove old operations by bulks
      if (that.operations_.length > 0 && start_time - that.operations_[0].time > 2 * kTimeWindow) {
        setTimeout(function() {
          let i = 0;
          for (; i + 1 < that.operations_.length && that.operations_[i + 1].time <= start_time; i++) {
            let op = that.operations_[i];
            if (op.data.delete_callback) {
              for (let c in op.data.delete_callback) {
                op.data.delete_callback[c](op.data);
              }
              op.data.delete_callback = [];
            }
          }
          that.operations_ = that.operations_.slice(i);
        }, 0);
      }
      that.notifyRenderer();
    }

    let processImpl = function(op) {
      if (op.t === "sop") {
        for (let i in op.d) {
          processImpl(op.d[i]);
        }
      } else if (op.t === "img") {
        that.loading_++;
        op.img = document.createElement("img");
        op.img.channel = that;
        op.img.src = op.data;
        op.img.onload = function() {
          this.channel.loading_--;
          if (this.channel.loading_ == 0) {
            postProcessImpl(ops);
          }
        };
      }
    }

    processImpl(ops);
    if (this.loading_ == 0) {
      postProcessImpl(ops);
    }
  }

  // The returned transformation will be invertible.
  // get2DProjection will be this transform followed by one that clears the z component.
  getInvertible2DTransform() {
    let mat = new THREE.Matrix4();
    const op = this.getOperation();
    if (!op) return mat;
    const trans = this.operations_[this.operations_.length-1].data.p;
    if (trans && trans.proj) {
      const f0 = trans.proj.f0;
      const f1 = trans.proj.f1;
      const c0 = trans.proj.c0;
      const c1 = trans.proj.c1;
      mat.set( 0, f0, c0, 0,
              f1,  0, c1, 0,
               0,  0, 0,  1,
               0,  0, 1,  0);
    }
    return mat;
  }

  // Get the current 2D projection.
  // This is guaranteed to be getInvertible2DTransform() followed by a straight z-projection.
  // (By straight z-projection, we mean the map (x, y, z, w) -> (x, y, 0, w).)
  get2DProjection() {
    // NOTE:
    // Our API guarantees this structure so that code using the transforms can make simplifying
    // assumptions.
    // If you change this basic structure, then you may be breaking assumptions of code that uses
    // this function and getInvertible2DTransform.
    const p = new THREE.Matrix4().set(
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 1
    );
    p.multiply(this.getInvertible2DTransform());
    return p;
  }

  // Returns the current scale
  getScale(target_time) {
    const op = this.getOperation(target_time);
    if (!op || !op.data.p) {
      return 1.0;
    }
    return op.data.p.s || 1.0;
  }

  // Returns the current referential frame
  getFrame(target_time) {
    const op = this.getOperation(target_time);
    if (!op || !op.data.p || op.data.p.t != "f") {
      return PoseTree().defaultFrame();
    }
    return op.data.p.f;
  }

  // Returns an incompatible frame if it exists
  findIncompatibleFrames(default_frame, time) {
    let frames = {};
    let extractFrame = function(op, current) {
      if (op.p && op.p.t === 'c') current = "";
      if (op.p && op.p.t === 'f') current = op.p.f;
      if (op.t == 'sop') {
        for (let i in op.d) {
          extractFrame(op.d[i], current);
        }
      }
      if (current !== "" && !PoseTree().areConnected(default_frame, current)) {
        frames[current] = true;
      }
    }
    if (this.operations_.length == 0) return frames;
    extractFrame(this.getOperation(time).data, PoseTree().defaultFrame());
    return frames;
  }

  // Returns whether the current operation should in the canvas frame
  isFrameCanvas() {
    const op = this.getOperation();
    if (!op || !op.data.p || op.data.p.t != "c") {
      return false;
    }
    return true;
  }

  // Returns the transformation from this operation frame to the static frame at a given time.
  // This transform can be assumed invertible.
  getTransform(static_frame, render_time) {
    const op = this.getOperation(render_time);
    if (!op) return new THREE.Matrix4();
    const time = op.time === null ? render_time : op.time;
    return SightChannel.GetTransform(op.data, static_frame, time);
  }

  // Returns the 3D transformation from an operation frame to the static frame at a given time.
  static GetTransform(op, static_frame, op_time) {
    let mat = new THREE.Matrix4();
    if (op && op.p) {
      const scale = op.p.s || 1.0;
      if (op.p.t === "2d") {
        const angle = op.p.p[0] || 0.0;
        const tx = op.p.p[1] || 0.0;
        const ty = op.p.p[2] || 0.0;
        const cos = Math.cos(angle) * scale;
        const sin = Math.sin(angle) * scale;
        mat.set(cos, -sin,    0,  tx,
                sin,  cos,    0,  ty,
                  0,    0, scale,  0,
                  0,    0,     0,  1);
      } else if (op.p.t === "3d") {
        // Unpack op.p.p to get quaternion and translation components.
        // Order defined in isaac/engine/gems/serialization/JsonFormatter.hpp
        const qw = op.p.p[0];
        const qx = op.p.p[1];
        const qy = op.p.p[2];
        const qz = op.p.p[3];
        const tx = op.p.p[4] || 0.0;
        const ty = op.p.p[5] || 0.0;
        const tz = op.p.p[6] || 0.0;

        // Mapping is applied when constructing THREE.Quaternion and THREE.Vector3
        const quat = new THREE.Quaternion(qx, qy, qz, qw);  //THREE.js format: Quaternion(x,y,z,w)
        mat.makeRotationFromQuaternion(quat)
           .setPosition(new THREE.Vector3(tx, ty, tz))
           .scale(new THREE.Vector3(scale, scale, scale));
      } else if (op.p.t === "f") {
        mat = PoseTree().get(static_frame, op.p.f, op_time);
        if (mat == null) return null;
        mat.scale(new THREE.Vector3(scale, scale, scale));
      }
    }
    return mat;
  }
}
