/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Maintains a graph view of the app in a div
function UpdateAppGraph(message, div) {
  if (message.graph !== undefined && message.graph !== null) {
    AddNodesIfNecessary(message, div);
  } else {
    UpdateNodeStatus(message, div);
  }
}

// Creates nodes and edges based on a message containing the graph
function AddNodesIfNecessary(message, div) {
  let graph = message.graph;
  let stats = message.node_statistics;

  // Create cytoscape object if necessary
  if (!div.hasOwnProperty("cy")) {
    div.cy = cytoscape({
      container: div,

      boxSelectionEnabled: false,
      autounselectify: true,

      style: cytoscape.stylesheet()
        .selector('node')
          .css({
            'width': 'data(length)',
            'height': 40,
            'shape': 'roundrectangle',
            'border-width': 1,
            'border-color': '#333',
            'border-style': 'solid',
            'content': 'data(name)',
            'color': '#000',
            'font-weight': 'normal',
            'text-valign': 'center',
          })
        .selector('edge')
          .css({
            'curve-style': 'bezier',
            'target-arrow-shape': 'triangle',
            'width': 5,
            'line-color': '#666',
            'target-arrow-color': '#666',
          })
        .selector('.highlighted')
          .css({
            'background-color': '#61bffc',
            'line-color': '#61bffc',
            'target-arrow-color': '#61bffc',
            'transition-property': 'background-color, line-color, target-arrow-color',
            'transition-duration': '0.5s',
          }),
    });
    let legend = document.createElement("div");
    legend.style.padding = "10px";
    legend.style.backgroundColor = "rgba(196, 196, 196, 0.75)";
    legend.style.position = "absolute";
    legend.style.bottom = "10px";
    legend.style.left = "10px";
    legend.innerHTML =
      "Legend: <br />" +
      "<font color='#999'> - Not started</font><br />" +
      "<font color='#a80'> - Starting</font><br />" +
      "<font color='#fa1'> - Stalled or slow</font><br />" +
      "<span style='background-image: linear-gradient(to right, #070, #0f0); color:transparent;" +
      "-webkit-background-clip: text; background-clip: text;'> - Running</span><br />" +
      "<font color='#a11'> - Stopping</font><br />" +
      "<font color='#f33'> - Stopped</font>";
    div.appendChild(legend);
  }
  let cy = div.cy;

  let new_nodes = new Set();
  let anything_new = false;

  for (let i in graph.nodes) {
    let node = graph.nodes[i];
    let node_name = node.name;
    if (IsSubgraphNode(node_name)) continue;
    let node_name_filtered = MakeSafeIdName(node_name);
    let cynode = cy.nodes("#" + node_name_filtered);
    let style = {
      'background-color': NodeColor(stats[node_name])
    };
    let split_name = node_name.split(".");
    if (cynode.length == 0) {
      cy.add({
        group: "nodes",
        data: {
          id: node_name_filtered,
          name: split_name[split_name.length - 1],
          short_name: split_name[split_name.length - 1],
          fullname: node_name,
          length: Math.max(150, split_name[split_name.length - 1].length * 10)
        },
        style: style
      });
      new_nodes.add(node_name_filtered);
      anything_new = true;
    } else {
      cynode[0].css(style);
    }
  }

  cy.on('mouseover', 'node', function(evt) {
    let node = evt.target;
    node.data().name = node.data().fullname;
    node.addClass('fullname');  // Force redrawing
  });

  cy.on('mouseout', 'node', function(evt) {
    let node = evt.target;
    node.data().name = node.data().short_name;
    node.removeClass('fullname');  // Force redrawing
  });

  // Build redirections:
  let redirections = {};
  for (let i in graph.edges) {
    if (IsInterfaceEndpoint(graph.edges[i].source)) {
      if (!redirections.hasOwnProperty(graph.edges[i].source)) {
        redirections[graph.edges[i].source] = [];
      }
      redirections[graph.edges[i].source].push(graph.edges[i].target);
    }
  }
  // Function that find the list of all the endpoints connected out of a given endpoint.
  let getOutputEndpoints = function(endpoint) {
    if (!IsInterfaceEndpoint(endpoint)) {
      return [MakeSafeIdName(GetNodeFromEndpoint(endpoint))];
    }
    let ans = [];
    for (let i in redirections[endpoint]) {
      ans = ans.concat(getOutputEndpoints(redirections[endpoint][i]));
    }
    return ans;
  };

  for (let i in graph.edges) {
    if (IsInterfaceEndpoint(graph.edges[i].source)) continue;
    let source = MakeSafeIdName(GetNodeFromEndpoint(graph.edges[i].source));
    const endpoints = getOutputEndpoints(graph.edges[i].target);
    for (let i in endpoints) {
      let target = endpoints[i];
      let cyedge = cy.edges('edge[source = "' + source + '"][target = "' + target + '"]');
      if (cyedge.length == 0) {
        cy.add({
          group: "edges", data: { source: source, target: target }
        });
        anything_new = true;
      }
    }
  }

  if (anything_new) {
    let layout = cy.layout({
      name: 'cose-bilkent',
      numIter: 10000,
      nodeRepulsion: 5000.0,
      edgeElasticity: 0.1,
      nestingFactor: 0.01
    });
    layout.run();
  }
}

// Updates the node status without changing the graph topology
function UpdateNodeStatus(message, div) {
  if (!div.hasOwnProperty("cy")) return;
  let cy = div.cy;
  let node_stats = message.node_statistics;
  let codelet_stats = message.codelet_statistics;
  for (let node_name in node_stats) {
    if (IsSubgraphNode(node_name)) continue;
    // Check if we have any codelets and find the maximum tick frequency
    let max_frequency = 0.0;
    for (let codelet in codelet_stats[node_name]) {
      if (codelet_stats[node_name][codelet]) {
        max_frequency = Math.max(max_frequency, codelet_stats[node_name][codelet].frequency);
      }
    }
    let node_name_filtered = MakeSafeIdName(node_name);
    let cynode = cy.nodes("#" + node_name_filtered);
    let style = { 'background-color': NodeColor(node_stats[node_name].lifecycle, max_frequency) };
    cynode[0].css(style);
  }
}

// Transform a name to make it safe to use as id (not alpha numerical character will be converted
// to '-' + ASCII value, and '-' will be changed to '--' to avoid name conflict).
function MakeSafeIdName(name) {
  let ans = "";
  for (let i in name) {
    let c = name[i];
    if (c === "-") {
      ans += "--";
    } else if (c.match(/[a-z]|[A-Z]|[0-9]/i)) {
      ans += c;
    } else {
      ans += "-" + c.charCodeAt();
    }
  }
  return ans;
}

// Gets the node from from and endpoint id of the form "node/component/label"
function GetNodeFromEndpoint(endpoint) {
  return endpoint.substr(0, endpoint.indexOf("/"));
}

// Returns whether or not a node is a subgraph
function IsSubgraphNode(node) {
  return node.indexOf("subgraph") != -1;
}

// Returns whether or not an edge is part of the interface
function IsInterfaceEndpoint(endpoint) {
  return endpoint.indexOf("interface") != -1;
}

// Gets the color of a node based on its current statistics
function NodeColor(lifecycle, frequency) {
  if (lifecycle == 11) {
    let rate = frequency;
    if (rate > 0.1) {
      let q = (128 + (255 - 128) * (1 - Math.exp(-0.05*rate))) | 0;
      return "rgb(0, " + q + ", 0)";
    }
  }
  switch (lifecycle) {
    case 0: return '#999';   // Never started
    case 10: return '#a80';  // Before start
    case 11: return '#fa1';  // After start
    case 30: return '#a11';  // Before stop
    case 31: return '#f33';  // After stop
    default: return '#a3a';  // Running
  }
}

// Returns the div containing the graph view, if it does not exist yet create it.
function GetGraphView() {
  let graph_view = document.getElementById("__win-graph-view");
  if (graph_view === null || graph_view === undefined) {
    graph_view = document.createElement("div");
    graph_view.id = "__win-graph-view";
    // Create a new window with a callback to call resize() on the cytoscape object.
    WindowManager().createWindow(graph_view, "App Graph",
                                 {resize: true, hide: true, onresize: function (obj) {
      graph_view.style.width = (obj.width) + "px";
      graph_view.style.height = "100%";
      // The first time onresize is called the cytoscape object is not attached yet, checking this
      // prevent javascript to crash.
      if (graph_view.hasOwnProperty("cy")) graph_view.cy.resize();
    }});
  }
  return graph_view;
}

// Reset the graph view by deleting every edges
function ResetGraphView() {
  let div = GetGraphView();
  if (div.cy) {
    div.cy.remove(div.cy.elements('edge'));
    div.cy.remove(div.cy.elements('node'));
  }
}