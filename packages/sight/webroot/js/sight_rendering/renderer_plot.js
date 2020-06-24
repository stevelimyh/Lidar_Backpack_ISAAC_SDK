/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Customizable plot Renderer
class RendererPlot extends SightRenderer {
  constructor(name, new_renderer) {
    super(name, new_renderer);
    // Create the window
    const binder = this;

    this.plots_ = new SmoothieChart();
    this.plots_.counter = 1;
    this.plots_.streamTo(this.canvas_);
  }

  // Get the data of the image
  getDataImage(type) {
    return this.canvas_.toDataURL(type);
  }

  // Returns the option specific to the renderer
  getRendererOptions() {
    return {resize: true, close: true};
  }

  applyInput() {}
  postApplyInput() {}
  preRenderFrame() {}
  renderFrame() {}

  // Returns the window
  getWindow() {
    return this.canvas_.parentNode.parentNode;
  }

  // Returns the name of the renderer
  getName() {
    return this.name_;
  }

  getLabelTitle() {
    return "Plots";
  }

  // Create the legend of a given channel
  createLegend_(channel) {
    // Create the legend for the channel
    let legend = document.createElement('div');
    let binder = this;

    let color_chooser = document.createElement('input');
    color_chooser.style.minWidth = "44px";
    color_chooser.type = "color";
    color_chooser.value = channel.color;
    legend.appendChild(color_chooser);

    legend.className += " label-plot-list"
    let label = document.createElement('label');
    let check = document.createElement('input');
    let divspan = document.createElement('div');
    divspan.className += " control-label__indicator";
    divspan.style.background = channel.color;
    check.type = "checkbox";
    check.checked = channel.active;
    label.className = "control-label control-label--checkbox";
    label.appendChild(check);
    label.appendChild(divspan);
    label.appendChild(document.createTextNode(channel.name));
    legend.appendChild(label);

    legend.channel = channel;
    this.legend_div.appendChild(legend);
    check.channel = this.channels_[this.channels_.length - 1];

    check.addEventListener('change', function() {
      check.channel.active = this.checked;
      if (this.checked) {
        binder.plots_.addTimeSeries(plots_[this.channel.name],
                                    {lineWidth:2, strokeStyle: this.channel.color});
      } else {
        binder.plots_.removeTimeSeries(plots_[this.channel.name]);
      }
    });
    if (channel.active) {
      this.plots_.addTimeSeries(plots_[channel.name], {lineWidth:2, strokeStyle: channel.color});
    }
    channel.label = label;
    label.cb = check

    color_chooser.onchange = function() {
      channel.color = this.value;
      divspan.style.background = channel.color;
      if (check.checked) {
        binder.plots_.removeTimeSeries(plots_[channel.name]);
        binder.plots_.addTimeSeries(plots_[channel.name],
                                    {lineWidth:2, strokeStyle: channel.color});
      }
    }
  }

  // Add a channel
  addSeries(timeserie) {
    let ts = this.channels_.find(x => x.name === timeserie.name);
    if (ts !== null && ts !== undefined) return;

    if (timeserie.color === null || timeserie.color === undefined) {
      timeserie.color = RendererPlot.GetRandomColor(this.plots_.counter++);
    }

    if (timeserie.active !== false) timeserie.active = true;
    this.channels_.push(timeserie);
    this.createLegend_(timeserie);
  }

  // Remove a channel
  removeSeries(timeserie) {
    let ts = this.channels_.find(x => x === timeserie);
    if (ts === null || ts === undefined) return;

    if (ts.active) {
      this.plots_.removeTimeSeries(plots_[ts.name]);
    }

    for (let leg in this.legend_div.children) {
      let legend = this.legend_div.children[leg];
      if (legend.channel.name == ts.name) {
        this.legend_div.removeChild(legend);
        break;
      }
    }

    this.channels_ = this.channels_.filter(x => x !== timeserie);
  }

  // Create the config object.
  toConfig() {
    const win_style = this.getWindow().style;
    let config = {
      renderer: this.getRendererType(),
      counter: this.plots_.counter,
      dims: {
        width: parseInt(win_style.width),
        height: parseInt(win_style.height)
      },
      channels: []
    };
    for (let name in this.channels_) {
      config.channels.push({
        name: this.channels_[name].name,
        color: this.channels_[name].color,
        active: this.channels_[name].active
      });
    }
    return config;
  }

  // Parse from a config.
  parseFromConfig(config) {
    // Remove all the channels
    while (this.channels_.length > 0) {
      this.removeSeries(this.channels_[0]);
    }
    // Load the channels
    if (config.hasOwnProperty("channels")) {
      for (let c in config.channels) {
        const ts = this.channels_.find(x => x.name === config.channels[c].name);
        if (ts === null || ts === undefined) {
          this.addSeries(config.channels[c]);
        }
      }
    }
    if (config.hasOwnProperty("counter")) {
      this.plots_.counter = config.counter | 1;
    }
    if (config.hasOwnProperty("dims")) {
      this.setDimensions(config.dims);
    }
  }

  // Notify something has changed.
  update(channel) {
    this.getWindow().click();
  }

  // Returns the rendered type
  getRendererType() {
    return "plot";
  }

  // Returns the drawing canvas
  getCanvas() {
    return this.canvas_;
  }

  // Set the dimensions
  setDimensions(dimensions) {
    let win_style = this.getWindow().style;
    win_style.width = dimensions.width +"px";
    win_style.height = dimensions.height +"px";
    this.update(null);
  }

  getConfigDiv() {
    // TODO customize this view
    const that = this;
    // Create a new div to hold all the data
    let div = document.createElement("div");
    div.className = "win-manager-div";

    // Helper to create a div that keep the display property
    let createDiv = function() {
      let d = document.createElement("div");
      d.style.display = "inherit";
      return d;
    }

    // Helper to append an element with a label on the left size
    let append = function(title, elem) {
      let d = document.createElement("div");
      d.className += " win-manager-inside-div";
      let label = document.createElement("label");
      label.appendChild(document.createTextNode(title));
      d.append(label);
      d.append(elem);
      div.appendChild(d);
      let sep = document.createElement("hr");
      sep.style.width = "0px";
      div.appendChild(sep);
    }

    // Create the dimensions of the window component
    let dims_holder = createDiv();
    let dims_width = document.createElement("input");
    let dims_height = document.createElement("input");
    dims_width.type = "number";
    dims_height.type = "number";
    const win_style = this.getWindow().style;
    dims_width.value = parseInt(win_style.width);
    dims_height.value = parseInt(win_style.height);
    dims_holder.appendChild(dims_height);
    dims_holder.appendChild(document.createTextNode(" x "));
    dims_holder.appendChild(dims_width);
    append("Dimensions", dims_holder);

    // Create the Precision component
    let precision_input = document.createElement("input");
    precision_input.type = "number";
    precision_input.value = this.plots_.options.labels.precision;
    append("Decimal precision", precision_input);

    // Add the button to create the window.
    let create = document.createElement("button");
    create.appendChild(document.createTextNode("Update"));
    create.addEventListener('click', function() {
      that.plots_.options.labels.precision = parseInt(precision_input.value);
      let config = {};
      config.channels = [];
      config.renderer = "plot";
      config.dims = {width: dims_width.value, height: dims_height.value};

      for (let i = 0; i < list_holder.length; i++) {
        config.channels.push(list_holder[i]);
      }

      that.parseFromConfig(config);
      that.hideConfig();
    });

    append("", create);

    // Will hold the current list of added timeserie
    // TODO make it possible to reorganized the order and remove some
    let list_holder = [];
    // Select the timeserie component
    let channels_holder = createDiv();
    channels_holder.style.display = "inline-block";
    let timeserie_div = document.createElement("div");
    // Get the list of channels and sort them by name
    let names = [];
    for (let name in plots_) {
      names.push(name);
    }
    names.sort(function(a, b) {
      return a.localeCompare(b);
    });

    // Remove a timeserie from the list
    let removeFromList = function(name) {
      for (let c in list_holder) {
        if (list_holder[c].name == name) {
          list_holder.splice(c, 1);
          break;
        }
      }
    }

    // Helper to add a new timeserie;
    let addToList = function(name, active = true, color = null) {
      // Pick a default color
      if (color === null) {
        color = RendererPlot.GetRandomColor(that.plots_.counter++);
      }
      const conf = {
        name: name,
        color: color,
        active: active
      }
      list_holder.push(conf);
    }

    let tree_menu = new TreeMenu(
      function(name, status) {
        if (status) {
          addToList(name);
        } else {
          removeFromList(name);
        }
      },
      function(name) {
        for (let cha in that.channels_) {
          if (that.channels_[cha].name === name) {
            return {open: false, checked: true};
          }
        }
        for (let cha in that.channels_) {
          if (that.channels_[cha].name.startsWith(name)) {
            return {open: true, checked: false};
          }
        }
        return {open: false, checked: false};
      });

    timeserie_div.classList.add("control-group");
    timeserie_div.classList.add("vertical-list");
    for (let i in names) {
      tree_menu.addToTree(names[i]);
    }
    tree_menu.renderTree(timeserie_div);
    channels_holder.appendChild(timeserie_div);
    append("Variables", channels_holder);

    // Add current timeseries
    for (let cha in this.channels_) {
      addToList(this.channels_[cha].name, this.channels_[cha].a, this.channels_[cha].color);
    }

    return div;
  }

  static GetRandomColor(index) {
    const r = (123 * index % 256);
    const g = (215 * index % 256);
    const b = (313 * index % 256);
    return rgbToHex(r, g, b);
  }
}
