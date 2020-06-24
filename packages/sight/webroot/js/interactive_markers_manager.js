/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
class InteractiveMarkersManagerImpl {
  constructor () {
    this.socket_ = null;
    this.editable_edges_ = [];
    this.markers_ = {};
    this.widgets_ = {};
    this.widgetRootNodes_ = {};
    this.reset()

    // When the set of edges is cleared, we need to clear out a bunch of records of markers.
    PoseTree().registerEdgeSetClearCallback(() => {
      for (let widgetName in this.widgets_) {
        let widget = this.widgets_[widgetName];
        widget.reset();
      }
      this.editable_edges_ = [];
      this.markers_ = {};
    });

    // When the set of edges in pose tree expands, it may be that we are able to add markers
    // we weren't previouslt able to add.
    PoseTree().registerEdgeSetExpansionCallback(() => this.populateEdges_());
  }

  /** @desc Reset the list of markers */
  reset(socket) {
    this.backEndStartTime_ = 0;
    this.frontEndStartTime_ = 0;
  }

  /** @desc When we have the connection, store it and ask for initial list of edges */
  setWebSocket(socket) {
    this.socket_ = socket;
    this.sendMessageToServer('query-edges', -1);
  }

  /**
   * @desc Add a reference to a MarkesWidget.
   */
  addMarkersWidget(name, widget, rootNode) {
    if (name in this.widgets_) {
      console.warn("widget " + name + " already added");
    }
    this.widgets_[name] = widget;
    this.widgetRootNodes_[name] = rootNode;
  }

  /** @private @desc create json needed to query the backend for the list of editable edges */
  queryEdgesMsg_() {
    let data = {
      cmd: 'query',
      cmdparams: 'edges'
    }
    return data;
  }
  /**
    * @desc Alternative method to send an updated edge to front end.
    * @param {string} lhs name of the left node of the pose
    * @param {string} rhs name of the right node of the pose
    * @param {THREE.Quaterion} rotation: A quaternions representing the rotation of the pose
    * @param {THREE.Vector3} translation: A vector representing the translation os the pose
    * @param {number} miliseconds representing the timestamp of the new edge
    */
  notifyEdgeChange(lhs, rhs, rotation, position, milliSeconds) {
    if (!this.socket_ || this.socket_.readyState != 1) {
      throw "The socket is not open";
    }
    let msgData = {
      cmd: 'edit',
      cmdparams: 'edge',
      lhs: lhs,
      rhs: rhs,
      timestamp: milliSeconds,
      // Convention in back end is [w, x, y, z]
      rotation: [rotation.w, rotation.x, rotation.y, rotation.z],
      position: [position.x, position.y, position.z]
    }
    let json = {
      type: 'interactive_markers',
      data: msgData
    }
    this.socket_.send(JSON.stringify(json));
  }
  /**
   * @desc Converts our current time in milliseconds to a suitable timestamp for the backend
   * @param {number} In milliseconds timestamp
   */
  toBackendTime(milliSeconds) {
    const deltaSeconds = (milliSeconds - this.frontEndStartTime_) / 1000.0;
    return deltaSeconds + this.backEndStartTime_;
  }
  /**
   * @desc Send message to the backend
   * @param {int} index of the modified edge in the markers array.
   * @param {string} type: is this an edit notyfication msg or a query for edges msg
   */
  sendMessageToServer(type, index) {
    if (this.socket_ && this.socket_.readyState == 1) {
      let msgData;
      if (type == 'edit') {
        // If one of the editable edges changed we need to notify the backend
        msgData = this.poseChangedMsg_(index);
      } else if (type == 'query-edges') {
        msgData = this.queryEdgesMsg_();
      }
      let json = {
          type: 'interactive_markers',
          data: msgData
      }
      this.socket_.send(JSON.stringify(json));
    } else {
      console.log('Socket is not open!');
    }
  }

  /**
   * @desc Set the registered base frame for a given widget. In general this changes the set of
   * markers contained in the widget.
   * @param {widgetName} Widget name. Must be the name of a widget added via addMarkersWidget.
   * @param {baseFrameName} Name of the new base frame. Must be contained in PoseTree().
   */
  setWidgetBaseFrame(widgetName, baseFrameName) {
    // TODO: Remove from widget the markers which are not connected to the new base frame.
    // Not needed yet because we always ever change from the default base frame
    // (connected to nothing) to one which is actually connected to something.
    if (!(widgetName in this.widgetRootNodes_)) {
      throw "cannot change base frame of " + widgetName +
          ": no such widget registered with markers manager";
    }

    // Change base frame and update markers.
    this.widgetRootNodes_[widgetName] = baseFrameName;
    this.updateWidgetMarkers_();
  }

  /**
   * @private @desc
   * Add all markers from this.markers_ to all widgets which can show them.
   * If the marker is already in the widget, then update the transformation.
   */
  updateWidgetMarkers_() {
    for (let markerPath in this.markers_) {
      let marker = this.markers_[markerPath];
      const pathEnd = marker.rhs;
      // Ensure that this marker has an associated channel
      const channelName = RendererManager().markerChannelName(pathEnd);
      if (!RendererManager().getChannel(channelName)) {
        RendererManager().addChannel(channelName);
      }
      const channel = RendererManager().getChannel(channelName);
      for (let widgetName in this.widgets_) {
        let widget = this.widgets_[widgetName];
        const pathStart = this.widgetRootNodes_[widgetName];
        if (!PoseTree().areConnected(pathStart, pathEnd)) {
          continue;
        }
        let pathTransformMatrix = PoseTree().get(pathStart, pathEnd);
        let pathTransformTranslation = new THREE.Vector3();
        let pathTransformOrientation = new THREE.Quaternion();
        let pathTransformScale = new THREE.Vector3();
        pathTransformMatrix.decompose(pathTransformTranslation, pathTransformOrientation,
                                      pathTransformScale);
        if (widget.containsMarker(pathEnd)) {
          widget.disableInteraction(pathEnd);
          widget.set({
            [pathEnd]: {
              "position": pathTransformTranslation,
              "orientation": pathTransformOrientation
            }
          });
          widget.enableInteraction(pathEnd);
        } else {
          widget.addMarker(channel, pathEnd, pathTransformTranslation, pathTransformOrientation);
        }
      }
    }
  }

  /**
   * @private @desc
   * For each edge that comes from the backend, validate that also exist in the front end PosetTree
   */
  populateEdges_() {
    // See which of the edges exist in the front end PoseTree
    for (let i = 0; i < this.editable_edges_.length; i++) {
      const edgeName = this.editable_edges_[i][0] + '_T_' + this.editable_edges_[i][1];
      // If the edge exist, create a marker...
      if ((this.markers_[edgeName] === null || this.markers_[edgeName] === undefined) &&
          PoseTree().areConnected(this.editable_edges_[i][0], this.editable_edges_[i][1])) {
        // Create and store the marker
        this.markers_[edgeName] = new Object();
        this.markers_[edgeName].lhs = this.editable_edges_[i][0];
        this.markers_[edgeName].rhs = this.editable_edges_[i][1];
        this.markers_[edgeName].fullname = edgeName;
      }
    }

    this.updateWidgetMarkers_();
  }

  getMarker(endNode) {
    for (let p in this.markers_) {
      let marker = this.markers_[p];
      if (marker.rhs == endNode) {
        return marker;
      }
    }
    return null;
  }

  /**
   * @desc Get the current markers front end PoseTree, relative to some given node.
   */
  getMarkers(pathStartNode) {

    let updatedMarkers = {};
    for (let markerPath in this.markers_) {
      let marker = this.markers_[markerPath];
      let pathEndNode = marker.rhs;
      if (!PoseTree().areConnected(pathStartNode, pathEndNode)) continue;
      let poseMat = PoseTree().get(pathStartNode, pathEndNode);
      // Decompose matrix in rotation and translation
      let tmpTrans = new THREE.Vector3();
      let tmpQuat = new THREE.Quaternion();
      let tmpScale = new THREE.Vector3();
      poseMat.decompose(tmpTrans, tmpQuat, tmpScale);

      updatedMarkers[pathEndNode] = {
        "rhs": marker.rhs,
        "lhs": marker.lhs,
        "position": tmpTrans,
        "orientation": tmpQuat,
      };
    }

    return updatedMarkers;
  }
  /**
   * @desc receive mesg from back end.
   * Curentlly, the only message supported is receive the list of editable edges
   */
  onWebSocketMessage(msg) {
    if (msg.cmd == 'populate') {
      if (!Array.isArray(msg['edges'])) {
        console.warn("invalid 'populate' message received: 'edges' is not an array");
        return;
      }
      if (isNaN(msg['startTime'])) {
        console.warn("invalid 'populate' message received: 'startTime' is not a number");
        return;
      }
      // Receive the list of editable edges and start time
      this.editable_edges_ = msg['edges'];
      this.backEndStartTime_ = msg['startTime'];
      this.frontEndStartTime_ = Date.now();
      this.populateEdges_();
    }
  }

}

let interactive_markers_manager_ = null;

/* Implement a "singleton" of Interactive Markers Manager */
function InteractiveMarkersManager() {
  if (interactive_markers_manager_ === null) {
    interactive_markers_manager_ = new InteractiveMarkersManagerImpl();
  }
  return interactive_markers_manager_;
}
