/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
// A temporal pose tree to store relative coordinate system transformations over time
// This implementation does not support multiple paths between the same coordinate systems. It does
// however allow for multiple "roots". In fact the transformation relationships form an acylic,
// bi-directional, not necessarily fully-connected graph. (In other words: it's a forest).
class PoseTreeImpl {
  constructor() {
    // Dictionary of node_name -> Node
    this.nodes_ = {};
    // Latest time
    this.now_ = Number.MAX_VALUE;
    this.start_node_ = null;
    this.end_node_ = null;
    this.graph_view_ = null;
    this.edge_set_expansion_callbacks_ = [];
    this.edge_set_clear_callbacks_ = [];
  }

  // Returns whether or not the PoseTree widget exist
  hasWidget() {
    return this.graph_view_ !== null;
  }

  // Creates the PoseTree widget exist
  createWidget() {
    // Create a window displaying the PoseTree
    this.graph_view_ = document.createElement("div");
    this.graph_view_.id = "__win-posetree-graph-view";
    // Create a new window with a callback to call resize() on the cytoscape object.
    let that = this;
    WindowManager().createWindow(this.graph_view_, "PoseTree",
                                 {resize: true, hide: true, onresize: function (obj) {
      that.graph_view_.style.width = (obj.width) + "px";
      that.graph_view_.style.height = "100%";
      if (that.graph_view_.hasOwnProperty("cy")) that.graph_view_.cy.resize();
    }});

    // Create the div that output the Pose2/3
    this.legend_ = document.createElement("div");
    this.legend_.style.zIndex = "1000";
    this.legend_.style.padding = "10px";
    this.legend_.style.backgroundColor = "rgba(196, 196, 196, 0.75)";
    this.legend_.style.position = "absolute";
    this.legend_.style.top = "10px";
    this.legend_.style.left = "10px";
    this.graph_view_.appendChild(this.legend_);

    this.cy_graph_ = cytoscape({
      container: this.graph_view_,
      boxSelectionEnabled: false,
      autounselectify: true,
      style: cytoscape.stylesheet()
        .selector('node')
          .css({
            'width': 150,
            'height': 40,
            'shape': 'roundrectangle',
            'background-color': 'rgb(0, 182, 0)',
            'border-width': 1,
            'border-color': '#333',
            'border-style': 'solid',
            'content': 'data(name)',
            'color': '#000',
            'font-weight': 'normal',
            'text-valign': 'center',
            'transition-property': 'background-color',
            'transition-duration': '0.5s',
          })
        .selector('edge')
          .css({
            'curve-style': 'bezier',
            'target-arrow-shape': 'triangle',
            'source-arrow-shape': 'triangle',
            'width': 5,
            'line-color': '#666',
            'target-arrow-color': '#666',
          })
        .selector('.start_node')
          .css({
            'background-color': '#61bffc',
            'transition-property': 'background-color',
            'transition-duration': '0.5s',
          })
        .selector('.end_node')
          .css({
            'background-color': '#fcbf61',
            'transition-property': 'background-color',
            'transition-duration': '0.5s',
          })
    });
    this.cy_graph_.on('tap', 'node', function(e) {
      that._clickNode(e.target);
    });
    this.cy_graph_.on('tap', function(e) {
      if (e.target == that.cy_graph_) that._clickNode(null);
    });
    this.graph_view_.cy = this.cy_graph_;
  }

  // Update the legend that display the current selected transformation
  _updateLegend() {
    if (!this.hasWidget()) return;
    if (!this.start_node_ || !this.end_node_) {
      this.legend_.style.display = "none";
      return;
    }
    this.legend_.style.display = "";
    this.legend_.innerHTML =
        "<p><b>" + this.start_node_.id() + "</b>_T_<b>" + this.end_node_.id() + "</b></p>";
    const mat = this.get(this.start_node_.id(), this.end_node_.id());
    if (mat == null) {
      this.legend_.innerHTML += "<i>Not connected</i>";
      return;
    }
    let position = new THREE.Vector3();
    let quaternion = new THREE.Quaternion();
    mat.decompose(position, quaternion, new THREE.Vector3() /*scale*/);
    // Check if it's a pose2 or pose3
    const pose2 = position.z.toFixed(5) == 0.0 &&
                  quaternion.x.toFixed(5) == 0.0 &&
                  quaternion.y.toFixed(5) == 0.0;
    this.legend_.innerHTML += "<p>Translation: <ul>";
    this.legend_.innerHTML += "<li>X: " + position.x.toFixed(3) + "</li>";
    this.legend_.innerHTML += "<li>Y: " + position.y.toFixed(3) + "</li>";
    if (!pose2) this.legend_.innerHTML += "<li>Z: " + position.z.toFixed(3) + "</li>";
    this.legend_.innerHTML += "</ul></p>";
    if (pose2) {
      this.legend_.innerHTML += "<p>Angle: <ul>";
      let angle = quaternionTo2dAngle(quaternion);
      this.legend_.innerHTML +=  "<li>" + angle.toFixed(3) + "</li>";
    } else {
      this.legend_.innerHTML += "Quaternion: <ul>";
      this.legend_.innerHTML += "<li>X: " + quaternion.x.toFixed(3) + "</li>";
      this.legend_.innerHTML += "<li>Y: " + quaternion.y.toFixed(3) + "</li>";
      this.legend_.innerHTML += "<li>Z: " + quaternion.z.toFixed(3) + "</li>";
      this.legend_.innerHTML += "<li>W: " + quaternion.w.toFixed(3) + "</li>";
    }
    this.legend_.innerHTML += "</ul></p>";
  }

  // Update which node are selected when a click occurs.
  _clickNode(node) {
    // Click outside a node should unselect all
    if (node == null) {
      if (this.start_node_) {
        this.start_node_.removeClass("start_node");
        this.start_node_ = null;
      }
      if (this.end_node_) {
        this.end_node_.removeClass("end_node");
        this.end_node_ = null;
      }
    } else if (this.start_node_ == node) {
      // If we click on the starting node, we remove it and change the end as the start
      this.start_node_.removeClass("start_node");
      if (this.end_node_) {
        this.start_node_ = this.end_node_;
        this.start_node_.removeClass("end_node");
        this.start_node_.addClass("start_node");
        this.end_node_ = null;
      } else {
        this.start_node_ = null;
      }
    } else if (this.end_node_ == node) {
      this.end_node_.removeClass("end_node");
      this.end_node_ = null;
    } else if (this.start_node_ == null) {
      this.start_node_ = node;
      node.addClass("start_node");
    } else {
      if (this.end_node_) this.end_node_.removeClass("end_node");
      this.end_node_ = node;
      node.addClass("end_node");
    }
    this._updateLegend();
  }

  // Returns a defaultFrame() that no one is using
  defaultFrame() {
    return "####";
  }

  // Returns the current application time
  now() {
    return this.now_;
  }

  // Resets the PoseTree
  reset() {
    this.start_node_ = null;
    this.end_node_ = null;
    if (this.hasWidget()) {
      try {
        this.edge_set_clear_callbacks_.forEach((c) => c());
      } catch (e) {
        console.error(e);
      }
      WindowManager().deleteWindow(WindowManager().getWindow("PoseTree"));
      this.graph_view_ = null;
      this.legend_ = null;
      this.cy_graph_ = null;
    }
    this.nodes_ = {};
    this.now_ = Number.MAX_VALUE;
  }

  // Removes the edge between the lhs node and rhs node
  // Also remove all the precomputed path that contains this edge.
  _removeEdge(lhs, rhs) {
    for (let n in this.nodes_) {
      // If n is not connected to lhs, then no path from n can contain the edge.
      if (!this.areConnected(lhs, n)) continue;
      for (let p in this.nodes_[n].path) {
        let path = this.nodes_[n].path[p];
        // Check if the path contains the edge lhs_T_rhs
        for (let i = 1; i < path.length; i++) {
          if (path[i-1].name == lhs && path[i].name == rhs) {
            delete this.nodes_[n].path[p];
            break;
          }
        }
      }
    }
    // Remove the edge
    delete this.nodes_[lhs].edges[rhs];
    this.cy_graph_.remove('edge[source=\'' + lhs + '\'][target=\'' + rhs + '\']');
  }

  // Loads/updates a pose tree from a config
  loadFromConfig(data) {
    if (!this.hasWidget()) this.createWidget();
    // Data is:
    // {
    //   time: 0.123,
    //   nodes: ["a", "b", ...],
    //   edges: [[lhs_idx, rhs_idx, time, [Pose]], ...]
    // }
    const kLhs = 0;
    const kRhs = 1;
    const kTimestamp = 2;
    const kPose = 3;
    // Get the list of active edges
    let set_edges = new Set();
    for (let i in data.edges) {
      const edge = data.edges[i];
      const lhs = data.nodes[edge[kLhs]];
      const rhs = data.nodes[edge[kRhs]];
      set_edges.add(lhs + "_T_" + rhs);
      set_edges.add(rhs + "_T_" + lhs);
    }
    // Clean the PoseTree of edges/nodes who got removed
    let rebuild_graph = false;  // Hold wether we need to rebuild the connected components.
    for (let n in this.nodes_) {
      for (let e in this.nodes_[n].edges) {
        const name = n + '_T_' + e;
        if (!set_edges.has(name)) {
          this._removeEdge(n, e);
          rebuild_graph = true;
        }
      }
    }
    // Get the list of active nodes in a set for performance reason.
    let set_nodes = new Set();
    for (let i in data.nodes) {
      set_nodes.add(data.nodes[i]);
    }
    // Delete all the nodes that got removed.
    for (let n in this.nodes_) {
      if (!set_nodes.has(n)) {
        delete this.nodes_[n];
        rebuild_graph = true;
        // Make sure that we unselect the node before we remove it
        if (this.start_node_ && this.start_node_.id() == n) this._clickNode(null);
        if (this.end_node_ && this.end_node_.id() == n) this._clickNode(null);
        this.cy_graph_.remove('#' + n);
      }
    }
    // We need to rebuild the connected componnents
    if (rebuild_graph) {
      // Reset the nodes to have no parent
      for (let n in this.nodes_) {
        this.nodes_[n].parent = null;
      }
      // Loop through all the edges and reassign the parents
      for (let n in this.nodes_) {
        for (let e in this.nodes_[n].edges) {
          let parent_n = PoseTreeImpl._getNodeParent(this.nodes_[n]);
          const parent_e = PoseTreeImpl._getNodeParent(this.nodes_[e]);
          if (parent_n != parent_e) {
            parent_n.parent = parent_e;
          }
        }
      }
    }
    // Now the PoseTree is clean, we can add the new edges and update the transformation.
    for (let i in data.edges) {
      const edge = data.edges[i];
      const lhs = data.nodes[edge[kLhs]];
      const rhs = data.nodes[edge[kRhs]];
      const time = edge[kTimestamp];
      const pose = edge[kPose];
      const qw = pose[0];
      const qx = pose[1];
      const qy = pose[2];
      const qz = pose[3];
      const tx = pose[4];
      const ty = pose[5];
      const tz = pose[6];
      const quat = new THREE.Quaternion(qx, qy, qz, qw);  //THREE.js format: Quaternion(x,y,z,w)
      const mat = new THREE.Matrix4().makeRotationFromQuaternion(quat)
                                     .setPosition(new THREE.Vector3(tx, ty, tz));
      this._setImpl(lhs, rhs, mat, time);
    }
    this.now_ = data.time;
    this._updateLegend();
  }

  registerEdgeSetClearCallback(callback) {
    this.edge_set_clear_callbacks_.push(callback);
  }

  // Register a callback to be called whenever a new edge is created.
  registerEdgeSetExpansionCallback(callback) {
    this.edge_set_expansion_callbacks_.push(callback);
  }

  // Set a transformation (THREE.Matrix4) between two nodes and update the legend
  set(lhs, rhs, pose, timestamp) {
    if (!this.hasWidget()) return;
    this._setImpl(lhs, rhs, pose, timestamp);
    this._updateLegend();
  }

  // Set a transformations (THREE.Matrix4) between two nodes
  _setImpl(lhs, rhs, pose, timestamp) {
    if (lhs === this.defaultFrame() || rhs === this.defaultFrame()) {
      console.log("No connection to the default frame can be made");
      return;
    }
    let node_a = this._getNode(lhs);
    let node_b = this._getNode(rhs);
    if (node_a.edges[node_b.name] === undefined) {
      // Check there is no connection between the nodes
      if (this._areConnectedImpl(node_a, node_b)) {
        console.error('There is already a path between  ' + lhs + ' and ' + rhs + '');
        return;
      }
      node_a.edges[node_b.name] = {
          timeseries: [{ pose: pose, timestamp: timestamp}],
          latest() { return this.timeseries[this.timeseries.length-1]; }
      };
      node_b.edges[node_a.name] = {
          timeseries: [{ pose: new THREE.Matrix4().getInverse(pose), timestamp: timestamp}],
          latest() { return this.timeseries[this.timeseries.length-1]; }
      };
      // Connect the 2 components
      PoseTreeImpl._getNodeParent(node_a).parent = PoseTreeImpl._getNodeParent(node_b);
      this.edge_set_expansion_callbacks_.forEach((c) => c());
      // Update the graph
      this.cy_graph_.add({group: "edges", data: { source: rhs, target: lhs }});
      return;
    }
    // if timestamp comes out of order, just ignore it
    if (timestamp <= node_a.edges[node_b.name].latest().timestamp) return;
    node_a.edges[node_b.name].timeseries.push({
      pose: pose, timestamp: timestamp
    });
    node_b.edges[node_a.name].timeseries.push({
      pose: new THREE.Matrix4().getInverse(pose), timestamp: timestamp
    });
    const kMinHistory = 1024;
    // Keep between 1024 to 2048 poses per edges
    if (node_a.edges[node_b.name].timeseries.length > 2*kMinHistory) {
      node_a.edges[node_b.name].timeseries =
          node_a.edges[node_b.name].timeseries.slice(-kMinHistory);
      node_b.edges[node_a.name].timeseries =
          node_b.edges[node_a.name].timeseries.slice(-kMinHistory);
    }
  }

  // Returns the transformation at a given timestamp between two nodes.
  // If no transformation exists, then null will be returned.
  get(lhs, rhs, timestamp) {
    if (lhs === rhs) return new THREE.Matrix4();
    if (lhs === this.defaultFrame() || rhs === this.defaultFrame()) {
      return null;
    }
    if (!this.hasWidget()) return null;
    if (!this.hasNode(lhs) || !this.hasNode(rhs)) return null;
    let node_a = this._getNode(lhs);
    let node_b = this._getNode(rhs);
    if (!this._areConnectedImpl(node_a, node_b)) {
      return null;
    }
    const path = this._getPath(node_a, node_b);
    if (path === null) {
      console.warn("Did not find a path for: " + lhs + "_T_" + rhs);
      return null;
    }
    let ans = new THREE.Matrix4();
    if (timestamp === null || timestamp === undefined) {
      timestamp = this.now();
    }
    for (let i = 1; i < path.length; i++) {
      let pose = this._getInterpolatedPose(path[i-1].edges[path[i].name], timestamp);
      if (pose == null) {
        console.warn("Failed to interpolated in the path: " + lhs + "_T_" + rhs);
        return null;
      }
      ans.multiply(pose);
    }
    return ans;
  }

  // Returns whether there is a connection between two nodes (using name)
  areConnected(lhs, rhs) {
    // Check if the name are the same (needed in case defaultFrame is passed)
    if (lhs === rhs) return true;
    if (!this.hasWidget()) return false;
    if (!this.hasNode(lhs) || !this.hasNode(rhs)) return false;
    return this._areConnectedImpl(this.nodes_[lhs], this.nodes_[rhs]);
  }

  // Returns whether or not the node exist in the graph
  hasNode(node) {
    if (this.nodes_[node]) return true;
    return false;
  }

  // Returns the list of names of the node present in the graph
  getNodeNames() {
    let ans = [this.defaultFrame()];
    for (let node in this.nodes_) {
      ans.push(node);
    }
    return ans;
  }

  // Returns an array of names of the edges in this PoseTree
  getEdgesNames() {
    let edges = [];
    for (let n in this.nodes_) {
      for (let e in this.nodes_[n].edges) {
        // Lexicographical comparison to avoid duplicate edges
        if (n <= e) {
          const edge = {
            lhs: n,
            rhs: e,
            name: n + '_T_' + e
          }
          edges.push(edge);
        }
      }
    }
    return edges;
  }

  // get the length of the path between two nodes in this PoseTree
  getPathLength(lhs, rhs) {
    if (lhs === rhs) return 0;
    if (!this.hasNode(lhs) || !this.hasNode(rhs) || !this.areConnected(lhs, rhs)) return -1;
    const node_a = this._getNode(lhs);
    const node_b = this._getNode(rhs);
    const path = this._getPath(node_a, node_b);
    return path.length;
  }

  // Find the component parent
  static _getNodeParent(node) {
    if (node.parent === null) return node;
    node.parent = PoseTreeImpl._getNodeParent(node.parent);
    return node.parent;
  }

  // Returns the pose of an edge at a given time
  _getInterpolatedPose(edge, timestamp) {
    if (edge.timeseries[0].timestamp > timestamp) return edge.timeseries[0].pose;
    if (edge.latest().timestamp <= timestamp) return edge.latest().pose;
    // TODO impement a binary search
    let i = 1;
    while (edge.timeseries[i].timestamp <= timestamp) i++;
    let p = (timestamp - edge.timeseries[i-1].timestamp) /
            (edge.timeseries[i].timestamp - edge.timeseries[i-1].timestamp);
    return poseInterpolation(edge.timeseries[i-1].pose, edge.timeseries[i].pose, p);
  }

  // Returns whether there is a connection between two nodes (using the node directly)
  _areConnectedImpl(node_a, node_b) {
    return PoseTreeImpl._getNodeParent(node_a) === PoseTreeImpl._getNodeParent(node_b);
  }

  // Adds a new node if it does not exist or return the existing one.
  _getNode(node) {
    if (!this.hasNode(node)) {
      const new_node = {
        name: node,    // Name of the node
        parent: null,  // To know whih connected component it belongs to
        edges: {},     // List of edges
        path: {}       // Cache for the path between this node and other node
      };
      this.nodes_[node] = new_node;
      // Update the graph
      this.cy_graph_.add({
        group: "nodes",
        data: {id: node, name: node}
      });
      this._updateCyGraph();
    }
    return this.nodes_[node];
  }

  // Returns a path from a to b and cache it
  _getPath(node_a, node_b) {
    if (node_a.path[node_b.name] !== undefined && node_a.path[node_b.name] !== null) {
      return node_a.path[node_b.name];
    }
    // Keep the direction where a node came from (to recover the path)
    let direction = {};
    // Keep track of the node which were visited
    let visited = {};
    // List of open node to visit
    let open = [node_a.name];
    visited[node_a.name] = true;
    // While we have open node we process them (in dfs order)
    while (open.length > 0) {
      let current = this.nodes_[open.pop()];
      // Go through the list of neighbors of the current node
      for (let node in current.edges) {
        if (visited[node]) continue;
        visited[node] = true;
        direction[node] = current.name;
        open.push(node);
        // If we reached or target, we backtrack to find the path
        if (node === node_b.name) {
          let ans = [];
          while (node !== undefined && node !== null) {
            ans.push(this._getNode(node));
            node = direction[node];
          }
          // Cache the path
          node_b.path[node_a.name] = ans;
          node_a.path[node_b.name] = ans.slice().reverse();
          return node_a.path[node_b.name];
        }
      }
    }
    return null;
  }

  // Redraws the graph in a nice way.
  _updateCyGraph() {
    this.cy_graph_.layout({name: 'breadthfirst'}).run();
  }
};

// Returns the unique PoseTree
let pose_tree_ = null;
function PoseTree() {
  if (pose_tree_ === null) {
    pose_tree_ = new PoseTreeImpl();
  }
  return pose_tree_;
}
