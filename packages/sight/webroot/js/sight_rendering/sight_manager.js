/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Renderer manager for sight
class SightManager {
  constructor() {
    // List of renderers
    this.renderers_ = {};
    // List of channels
    this.channels_ = {};
    // Name of the current app
    this.current_app_ = null;

    const update = () => {
      sightRequestAnimationFrame(update);
      for (let name in this.renderers_) {
        let renderer = this.renderers_[name];
        renderer.applyInput();
      }
      for (let name in this.renderers_) {
        let renderer = this.renderers_[name];
        renderer.postApplyInput();
      }
      for (let name in this.renderers_) {
        let renderer = this.renderers_[name];
        renderer.preRenderFrame();
      }
      for (let name in this.renderers_) {
        let renderer = this.renderers_[name];
        renderer.renderFrame();
      }
    };
    sightRequestAnimationFrame(update);
  }

  markerChannelName(rhs) {
    // Before we use it to distinguish markers channels.
    return rhs;
  }

  // Set the current app name
  setAppName(name) {
    if (this.current_app_ === name) {
      for (let name in plots_) {
        plots_[name].clear();
      }
      for (let name in this.channels_) {
        this.channels_[name].clear();
      }
      return;
    }
    // Resets the channels
    this.channels_ = {};
    // TODO: move the plots channel into the sight manager
    plots_ = {};

    // Save the current configuration
    WindowManager().save();
    this.save();
    // Get a copy of all the windows
    let renderer_name = [];
    for (let name in this.renderers_) renderer_name.push(name);
      // Delete all the windows
    for (let name in renderer_name) {
      WindowManager().deleteWindow(this.renderers_[renderer_name[name]].getWindow());
    }

    this.current_app_ = name;

    if (name !== null) {
      const config_txt = getConfig("__win-manager-config-" + name, "{\"windows\": {}}");
      try {
        const config = JSON.parse(config_txt);
        renderer_manager_.parseConfig(config);
      } catch(e) {
        console.warn("Fail to parse the config: " + config_txt);
        console.warn("Error: " + e);
      }
    }
  }

  // Return the current app name
  getAppName(name) {
    return this.current_app_;
  }

  // Load from config
  parseConfig(config) {
    if (config !== null && config.hasOwnProperty("windows")) {
      for (let name in config.windows) {
        this.createOrUpdateRenderer(config.windows[name], name);
      }
    }
  }

  // Output the current config
  toConfig() {
    let config = {};
    config.windows = {}
    for (let name in this.renderers_) {
      config.windows[name] = this.renderers_[name].toConfig();
    }
    return config;
  }

  // Notify of a change (Enable/Disable) for a given channel
  changeChannelStatus(name, status) {
    if (this.channels_.hasOwnProperty(name)) {
      this.channels_[name].changeStatus(status);
    }
  }

  // Save the current config
  save() {
    if (this.current_app_ === null) return;
    saveConfig("__win-manager-config-" + this.current_app_, JSON.stringify(this.toConfig()));
  }

  // Renderer factory/builder method. Create/Load a renderer. Support plot, 2D, or 3D renderer.
  // Add valid channels and time series. Setup, configure, and return the built renderer.
  createOrUpdateRenderer(info, name) {
    if (!this.renderers_.hasOwnProperty(name)) {
      if (info.renderer == "2d") {
        this.renderers_[name] = new Renderer2D(name, false);
      } else if (info.renderer == "3d") {
        this.renderers_[name] = new Renderer3D(name, false);
      } else if (info.renderer == "plot") {
        this.renderers_[name] = new RendererPlot(name, false);
      } else {
        console.log(info.renderer + " is not supported yet");
        return;
      }
    }

    // Make sure each channel exists before adding it to the manager.
    if (info.renderer !== "plot" && info.hasOwnProperty("channels")) {
      for (let i in info.channels) {
        let c = info.channels[i];
        if (!(c.name in this.channels_)) {
          this.addChannel(c.name)
        }
      }
    }

    // Make sure each time serie exists before adding it to the manager.
    if (info.renderer === "plot" && info.hasOwnProperty("channels")) {
      for (let i in info.channels) {
        let c = info.channels[i];
        if (!(c.name in plots_)) {
          plots_[c.name] = new TimeSeries();
        }
      }
    }

    // Setup the renderer from the configuration info.
    let renderer = this.renderers_[name];
    renderer.parseFromConfig(info);
    renderer.update(null);

    return renderer;
  }

  // Returns a list of renderer of a given type
  getRendererByType(type) {
    let renderers = [];
    for (let i in this.renderers_) {
      const renderer = this.renderers_[i];
      if (renderer instanceof type) {
        renderers.push(renderer);
      }
    }
    return renderers;
  }

  // Delete a renderer
  deleteRenderer(name) {
    if (!this.renderers_.hasOwnProperty(name)) {
      return;
    }
    let renderer = this.renderers_[name];
    for (let cha in this.channels_) {
      this.channels_[cha].removeRenderer(renderer);
    }
    delete this.renderers_[name];
  }

  // Process a request, returns whether or not the request has been handled
  process(req) {
    let chanel_name = this.getChannelName(req);
    if (chanel_name == null) {
      console.log("Request not valid: " + JSON.stringify(req));
      return;
    }
    if (!(chanel_name in this.channels_)) {
      this.addChannel(chanel_name)
    }
    this.channels_[chanel_name].addOperation(req);
  }

  // Returns whether or not a channel exist.
  hasChannel(name) {
    return this.channels_[name] !== undefined && this.channels_[name] !== null;
  }

  // Adds a new channel
  addChannel(name) {
    this.channels_[name] = new SightChannel(name);
  }

  getChannel(name) {
    return this.channels_[name];
  }

  getRenderer(name) {
    return this.renderers_[name];
  }

  // Returns the name of the channel (either from uuid or from the name directly).
  getChannelName(req) {
    let win_name = channel_mapping_[req.uuid];
    if (win_name !== undefined) {
      return win_name;
    }
    if (req.name !== undefined) {
      return req.name;
    }
    return null;
  }

  // Get the list of active channels sorted by name.
  getActiveChannelNames() {
    let names = [];
    for (let cha in this.channels_) {
      if (this.channels_[cha].operations_.length > 0) {
        names.push(this.channels_[cha].name_);
      }
    }
    names.sort(function(a,b) {
      return a.toLowerCase().localeCompare(b.toLowerCase())
    });
    return names;
  }
}

let renderer_manager_ = null;

function sightDrawHandle(lists) {
  const new_lists = (lists instanceof Array) ? lists : {lists};
  for (let id in new_lists) {
    RendererManager().process(new_lists[id]);
  }
};

// Disaplye the div to create a new drawing
function CreateNewWindow(type, show_config = true, default_name = '') {
  let name = prompt("Name of the window", default_name);
  if (name !== null && !renderer_manager_.renderers_.hasOwnProperty(name)) {
    let renderer = null;
    if (type === '2d') renderer = new Renderer2D(name);
    else if (type === '3d') renderer = new Renderer3D(name);
    else if (type === 'plot') renderer = new RendererPlot(name);
    if (renderer === null) return null;
    renderer_manager_.renderers_[name] = renderer;
    if (show_config) renderer.showConfig();
    return renderer;
  }
  return null;
}

// Returns the window manager
function RendererManager() {
  if (renderer_manager_ == null) {
    renderer_manager_ = new SightManager();
  }
  return renderer_manager_;
}
