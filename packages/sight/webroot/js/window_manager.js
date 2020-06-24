/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Window manager:
// Let you create floating window to a div, organize them, keep up to date the menu on the left side
class WindowManagerImpl {
  constructor(parent, menu = null) {
    // Parent div, where the floating window are added
    this.parent_ = parent;
    // Menu div. The list of window will be added to it.
    this.menu_ = menu;
    // Dictionary name -> window div
    this.windows_ = {};
    // Z-index counter, only increment, it is used to always bring the target window above all the
    // other windows.
    this.index_counter_ = 0;
    this.hidden_submenu_ = {};
  }

  // Brings the window on top of the others
  _bringWindowTop(win) {
    if (win.style.zIndex === "" || win.style.zIndex < this.index_counter_) {
      win.style.zIndex  = ++this.index_counter_;
    }
  }

  // Save the current configuration of the windows
  save() {
    for (let id in this.windows_) {
      let elem = this.windows_[id];
      let name = elem.id;
      saveConfig(name+"-visible", elem.check.checked);
      saveConfig(name+"-width", elem.style.width);
      saveConfig(name+"-height", elem.style.height);
      saveConfig(name+"-top", elem.style.top);
      saveConfig(name+"-left", elem.style.left);
      saveConfig(name+"-canvas-width", elem.obj.width);
      saveConfig(name+"-canvas-height", elem.obj.height);
    }
  }

  // Reorganize the windows in the filed parent to not have overlap
  rearrangeDivs() {
    const margin = 10;
    let twidth = window.innerWidth - this.parent_.offsetLeft;
    // TODO remove this hack
    // If the config is present
    let config = document.getElementById("config");
    if (config !== null) twidth -= config.offsetWidth + 10 + margin;

    // Group windows in number needed to fill a row
    const splitter = 20;
    let sizes = [];
    for (let id in this.windows_) {
      let div = this.windows_[id];
      if (div.style.display === "none") continue;
      const box = div.getBoundingClientRect();
      const width = box.width + 2 * margin;
      const height = box.height + 2 * margin;
      const num = (twidth / width) | 0;
      if (!sizes[num]) {
        sizes[num] = [];
      }
      sizes[num].push({w: width, h: height, d: div});
    }

    // Extract a list of window that fit next to the right most element until we reach its height
    const extract = function(aw, th) {
      let list = [];
      let h = 0;
      for (let i in sizes) {
        for (let e in sizes[i]) {
          let elem = sizes[i][e];
          if (!elem) continue;
          if (elem.w < aw && h + elem.h < th * 1.1) {
            list.push(elem);
            sizes[i][e] = null;
            h += elem.h;
            if (h > th) break;
          }
        }
        if (h > th) break;
      }
      return list;
    }
    // Place an element at a givent place
    const place = function(elem, y, x, aw) {
      elem.style.top = y + "px";
      elem.style.left = x + "px";
      if (elem.options && elem.options.resize === true) {
        elem.style.width = (aw - 2 * margin) + "px";
        elem.click();
      }
    }

    // Go through the group one by one and organize the window
    let y = margin;
    for (let i in sizes) {
      if (i <= 0) {
        // Special case for the window that does not fit
        for (let e in sizes[i]) {
          let elem = sizes[i][e];
          place(elem.d, y, margin, twidth);
          y += elem.h;
        }
      } else if (i == 1) {
        // Special case for the window that fit  alone (we still try to find another window to fit)
        for (let e in sizes[i]) {
          let elem = sizes[i][e];
          place(elem.d, y, margin);
          const aw = twidth - elem.w;
          const list = extract(aw, elem.h);
          let h = 0;
          for (let d in list) {
            place(list[d].d, y + h, elem.w + margin, aw);
            h += list[d].h;
          }
          y += Math.max(elem.h, h);
        }
      } else {
        // Organize the windows by columns, always add to the smallest one.
        let h = [];
        for (let v = 0; v < i; v++) h.push(0);
        const w = twidth / i;
        for (let e in sizes[i]) {
          let c = h.indexOf(Math.min(...h));
          let elem = sizes[i][e];
          if (!elem) continue;
          place(elem.d, y + h[c], margin + c * w, w);
          h[c] += elem.h;
        }
        let c = h.indexOf(Math.max(...h));
        y += h[c];
      }
    }
    this._resizeWindowManager();
    return true;
  }

  getWindow(win_name) {
    return this.windows_[win_name];
  }

  deleteWindow(win) {
    if (win.onclose != null) {
      win.onclose(win.name);
    }
    delete this.windows_[win.name];
    this.parent_.removeChild(win);
    this._displayMenu();
  }

  minimizeWindow(win) {
    win.style.display = "none";
    win.check.checked = false;
  }

  maximizeWindow(win) {
    win.style.display = "";
    win.check.checked = true;
    win.children[1].focus();
    win.click();
    this._bringWindowTop(win);
  }

  // Returns the first available position for a given window
  getPosition(width, height) {
    let twidth = window.innerWidth;
    let config = document.getElementById("config");
    const margin = 10;
    if (config !== null) twidth -= config.offsetWidth + margin;

    // Make sure that if width > twidth we place it anyway
    twidth = Math.max(this.parent_.offsetLeft + 1, twidth - width);
    let grid_size = 20;
    let divs = [];
    for (let id in this.windows_) {
      if (this.windows_[id].style.display === "none") continue;
      divs.push(this.windows_[id]);
    }
    // Loop through a grid of position until we reach an empty spot.
    for (let y = 0;; y += grid_size) {
      for (let x = this.parent_.offsetLeft; x <= twidth; x += grid_size) {
        let free = true;
        for (let div of divs) {
          const box = div.getBoundingClientRect();
          if (box.top >= margin + y + height) continue;
          if (box.bottom + margin <= y) continue;
          if (box.left >= x + width + margin) continue;
          if (box.right + margin <= x) continue;
          free = false;
          break;
        }
        if (free) return {x: x + margin - this.parent_.offsetLeft, y: y + margin};
      }
      // Increase the speed of the search
      if (y > 100 * grid_size) grid_size += margin;
    }
  }

  // Create a floating window and add it to the parent
  // Options:
  //   - resize: whether resize the object when the window is resized
  //   - onrsize: callback called when resized
  //   - close: Whether the window can be destroyed (default is false)
  //   - onclose: callback when the window is closed
  //   - width/height: dimensions of the window.
  //   - menu_label: Label of the menu.
  //   - hide: Hide the window at creation
  createWindow(obj, name, options = {}) {
    if (this.windows_.hasOwnProperty(name)) {
      this.deleteWindow(this.windows_[name]);
    }
    let binder = this;

    // Create floating div holding the canvas and legend
    let div = document.createElement('div');
    div.style.backgroundColor = "#fefefe";
    div.options = options;
    div.name = name;
    div.id = "plot-" + name;
    div.className += " sight-floating-window-div"
    // Position it at a new place every time, avoid stacking
    div.style.width = getConfig(div.id + "-width", "400px");
    div.style.height = getConfig(div.id + "-height", "280px");
    div.style.minHeight = "100px";

    if (options.hasOwnProperty("width")) {
      div.style.width = options.width + "px";
    }
    if (options.hasOwnProperty("height")) {
      div.style.height = options.height + "px";
    }

    const pos = this.getPosition(parseInt(div.style.width), parseInt(div.style.height));
    div.style.top = getConfig(div.id + "-top", pos.y + "px");
    div.style.left = getConfig(div.id + "-left", pos.x + "px");

    let header = document.createElement('div');

    let header_text = document.createElement('div');
    header.className += " sight-floating-window-header";
    header.id = "plot-" + name + "-header";
    this._bringWindowTop(div);

    header_text.title = name;
    header_text.innerHTML = "<b>&nbsp;" + name + "&nbsp;</b>";
    header_text.style.display = "inline-block";
    header_text.style.color = "#1a1a1a";
    header_text.style.fontSize = "16px";
    if (name.length > 100) header_text.style.width = "100rem";
    header.appendChild(header_text);
    // If the window is closable, let's add a button to close it
    if (options.close === true) {
      div.close = true;
      div.onclose = options.onclose;
    }
    let minimize_button = document.createElement('i');
    minimize_button.className += " material-icons";
    minimize_button.innerHTML = "remove";
    minimize_button.style.fontSize = "16px";
    minimize_button.style.color = "#999999";
    minimize_button.style.center = "true";
    minimize_button.style.float = "right";
    minimize_button.win = div;
    minimize_button.onclick = function() {
      binder.minimizeWindow(this.win);
    };

    header.style.backgroundColor = "#DDDDDD";
    header.style.padding = "5px";
    header.appendChild(minimize_button);
    div.appendChild(header);

    // Add border on the edges and the corner to handle resizing
    // left border.
    let scale_left = document.createElement('div');
    scale_left.className += " sight-floating-scale-left-border";
    div.appendChild(scale_left);
    this._resizeElement(scale_left, /* top = */ false, /* right = */ false, /* bottom = */ false,
                        /* left = */ true);
    // right border.
    let scale_right = document.createElement('div');
    scale_right.className += " sight-floating-scale-right-border";
    div.appendChild(scale_right);
    this._resizeElement(scale_right, /* top = */ false, /* right = */ true, /* bottom = */ false,
                        /* left = */ false);
    // top border.
    let scale_top = document.createElement('div');
    scale_top.className += " sight-floating-scale-up-border";
    div.appendChild(scale_top);
    this._resizeElement(scale_top, /* top = */ true, /* right = */ false, /* bottom = */ false,
                        /* left = */ false);
    // bottom border.
    let scale_bottom = document.createElement('div');
    scale_bottom.className += " sight-floating-scale-down-border";
    div.appendChild(scale_bottom);
    this._resizeElement(scale_bottom, /* top = */ false, /* right = */ false, /* bottom = */ true,
                        /* left = */ false);
    // top left corner.
    let scale_top_left = document.createElement('div');
    scale_top_left.className += " sight-floating-scale-top-left-border";
    div.appendChild(scale_top_left);
    this._resizeElement(scale_top_left, /* top = */ true, /* right = */ false, /* bottom = */ false,
                        /* left = */ true);
    // top right corner.
    let scale_top_right = document.createElement('div');
    scale_top_right.className += " sight-floating-scale-top-right-border";
    div.appendChild(scale_top_right);
    this._resizeElement(scale_top_right, /* top = */ true, /* right = */ true, /* bottom = */ false,
                        /* left = */ false);
    // bottom left corner.
    let scale_bottom_left = document.createElement('div');
    scale_bottom_left.className += " sight-floating-scale-bottom-left-border";
    div.appendChild(scale_bottom_left);
    this._resizeElement(scale_bottom_left, /* top = */ false, /* right = */ false,
                        /* bottom = */ true,  /* left = */ true);
    // bottom right corner.
    let scale_bottom_right = document.createElement('div');
    scale_bottom_right.className += " sight-floating-scale-bottom-right-border";
    // Add an icon in the bottom right corner to indicate we are in resize mode as before.
    scale_bottom_right.innerHTML = "<i class='material-icons'>filter_list</i>";
    div.appendChild(scale_bottom_right);
    this._resizeElement(scale_bottom_right, /* top = */ false, /* right = */ true,
                        /* bottom = */ true,  /* left = */ false);

    let obj_holder = document.createElement('div');
    obj_holder.tabIndex = "-1";
    obj_holder.className += " sight-floating-window-content";
    obj_holder.appendChild(obj);
    div.obj = obj;

    let legend = document.createElement('div');
    legend.className += " sight-floating-window-legend";
    legend.id          = "plot-" + name + "-legend";

    obj_holder.appendChild(legend);
    div.appendChild(obj_holder);

    this.parent_.appendChild(div);

    // Handle resizing
    div.addEventListener('mousedown', function() {
      binder._bringWindowTop(this);
    });
    if (options.resize === true) {
      div.addEventListener('click', function() {
        obj.width = this.clientWidth;
        obj.height = this.clientHeight - header.offsetHeight - legend.offsetHeight;
        if (options.hasOwnProperty("onresize")) {
          options.onresize(obj);
        }
      });
      div.click();
      obj_holder.style.overflow = "hidden";
    }
    this._dragElement(div, this.parent_);
    if (options.hasOwnProperty("onresize")) {
      options.onresize(obj);
    }
    if (!options.hasOwnProperty("menu_label")) {
      options.menu_label = "Others";
    }
    div.menu_label = options.menu_label;

    this.windows_[name] = div;
    this._displayMenu();

    obj.isVisible = function() {
      let node = this;
      // Let's first check if the elment has visibility none.
      while (node != document) {
        // Not attached to the document
        if (node === null) return false;
        // Parent has visibility set to none
        if (node.style.display === "none") return false;
        node = node.parentNode;
      }
      const box = this.getBoundingClientRect();

      return box.bottom >= 0 &&
             box.right >= 0 &&
             box.top <= (window.innerHeight || document.documentElement.clientHeight) &&
             box.left <= (window.innerWidth || document.documentElement.clientWidth)
    };

    const visible = getConfig(div.id + "-visible", "" + (options.hide !== true));
    if (visible == "true") {
      this.maximizeWindow(div);
    } else {
      this.minimizeWindow(div);
    }
    this._resizeWindowManager();
    return obj;
  }

  // Display the menu on the left.
  _displayMenu() {
    if (this.menu_ === null) return;
    let binder = this;
    while (this.menu_.firstChild) {
      this.menu_.removeChild(this.menu_.firstChild);
    }
    let sub_menus = {};
    // Create the menu item for each window and add it to its sub_menu
    for (let w in this.windows_) {
      let win = this.windows_[w];
      if (win.legend === null || win.legend === undefined) {
        let legend = document.createElement('div');
        legend.style.display = "flex";
        let label = document.createElement('label');
        let close_button = document.createElement('i');
        close_button.style.cursor = "pointer";
        close_button.className += " material-icons menu_window_close";
        close_button.innerHTML = "&#xe888;";
        close_button.style.fontSize = "16px";
        close_button.style.margin = "3px";
        close_button.title = "Delete window";
        if (win.close === true) {
          close_button.win = win;
          close_button.onclick = function() {
            binder.deleteWindow(this.win);
          };
        } else {
          close_button.style.visibility = "hidden";
        }
        legend.appendChild(close_button);
        let divspan = document.createElement('div');
        divspan.className += " control__indicator";
        let check = document.createElement('input');
        win.check = check;
        check.type = "checkbox";
        check.checked = true;
        check.win = win;
        check.addEventListener('change', function() {
          if (this.checked) {
            binder.maximizeWindow(this.win);
          } else {
            binder.minimizeWindow(this.win);
          }
        });
        label.className = "control control--checkbox";
        label.style.margin = "3px";
        label.appendChild(check);
        label.appendChild(divspan);
        legend.appendChild(label);
        let text_holder = document.createElement("div");
        text_holder.appendChild(document.createTextNode(win.name));
        text_holder.title = win.name;
        text_holder.check = check;
        text_holder.win = win;
        text_holder.style.marginTop = "4px";
        text_holder.style.cursor = "pointer";
        text_holder.addEventListener('click', function() {
          if (!this.check.checked) {
            this.check.click();
          }
          binder._bringWindowTop(this.win);
          let target_x = window.scrollX;
          let target_y = parseInt(this.win.style.top) - (window.innerHeight - parseInt(this.win.style.height)) / 2;
          if (target_y < 0) {
            target_y = 0;
          }
          this.win.children[1].focus();
          window.scrollTo(target_x, target_y);
          $(this.win).effect("shake", {times:2, distance:5}, 500);
          let title = this.win.firstChild;
          if (this.win.bgcolor === undefined) {
            this.win.bgcolor =
                window.getComputedStyle(title, null).getPropertyValue('background-color');
          }
          $(title).stop().css({"background-color":"#264000"})
                         .animate({"background-color": this.win.bgcolor}, 2000);
        });
        legend.appendChild(text_holder);
        win.legend = legend;
      }
      if (!sub_menus.hasOwnProperty(win.menu_label)) {
        sub_menus[win.menu_label] = [];
      }
      sub_menus[win.menu_label].push(win.legend);
    }
    // Create the submenu
    for (let s in sub_menus) {
      // Skipe window without label
      let sub_menu = sub_menus[s];
      let holder = document.createElement("div");
      holder.style.marginLeft = "5px";
      holder.style.display = "grid";
      for (let i = 0; i < sub_menu.length; i++) {
        holder.appendChild(sub_menu[i]);
      }
      // Additional icon for internal node
      let div_icon = document.createElement('div');
      div_icon.className += " div_icon";
      div_icon.className += " unselectable";
      // Name of the current node
      let div_text = document.createElement('div');
      div_text.className += " div_text";
      div_text.innerHTML = s;
      // The div the full line icon/name
      let div_full = document.createElement('div');
      div_full.appendChild(div_icon);
      div_full.appendChild(div_text);
      div_full.className += " div_full";
      this.menu_.appendChild(div_full);
      this.menu_.appendChild(holder);

      div_text.node = holder;
      div_icon.node = holder;
      div_text.icon = div_icon;
      div_icon.icon = div_icon;
      div_text.className += " folder_text";
      div_icon.innerHTML = "<i class='material-icons'>expand_more</i>";
      // Function that handle hiding and displaying the children div.
      let fct = function(event) {
        if (this.node.style.display == "none") {
          $(this.node).slideDown("fast");
          this.icon.innerHTML = "<i class='material-icons'>expand_more</i>";
          delete binder.hidden_submenu_[s];
        } else {
          $(this.node).slideUp("fast");
          this.icon.innerHTML = "<i class='material-icons'>chevron_right</i>";
          binder.hidden_submenu_[s] = true;
        }
      };
      div_text.addEventListener('click', fct);
      div_icon.addEventListener('click', fct);
      if (this.hidden_submenu_[s]) {
        holder.style.display = "none";
      }
    }
  }

  // Helper to create a window with a canvas
  // Returns the canvas directly.
  createWindowWithCanvas(name, options = {}) {
    let canvas = document.createElement('canvas');
    return this.createWindow(canvas, name, options);
  }

  // Helper to create a drag and drop window.
  _dragElement(element) {
    let binder = this;
    let pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    element.firstChild.onmousedown = dragMouseDown;

    // Called when the user click
    function dragMouseDown(e) {
      e = e || window.event;
      // get the mouse cursor position at startup:
      pos3 = e.clientX;
      pos4 = e.clientY;
      document.onmouseup = closeDragElement;
      // call a function whenever the cursor moves:
      document.onmousemove = elementDrag;
      binder._bringWindowTop(element);
    }

    // Called when the user is moving the mouse (while clicking).
    function elementDrag(e) {
      e = e || window.event;
      // calculate the new cursor position:
      pos1 = pos3 - e.clientX;
      pos2 = pos4 - e.clientY;
      pos3 = e.clientX;
      pos4 = e.clientY;
      // Check if the window would leave the parent.
      if (element.offsetTop - pos2 < 5) {
        pos2 = -5 + element.offsetTop;
      }
      if (element.offsetLeft - pos1 < 5) {
        pos1 = -5 + element.offsetLeft;
      }
      // set the element's new position:
      element.style.top = (element.offsetTop - pos2) + "px";
      element.style.left = (element.offsetLeft - pos1) + "px";
      e.preventDefault();
    }

    // Called when the user finish their click.
    function closeDragElement() {
      /* stop moving when mouse button is released:*/
      document.onmouseup = null;
      document.onmousemove = null;
      binder._resizeWindowManager();
    }
  }

  // Resize the dimensions of the windows manager
  _resizeWindowManager() {
    const kMargin = 200;
    let required_height = 0;
    let required_width = 0;
    for (let i in this.windows_) {
      const element = this.windows_[i];
      const top = parseInt(element.style.top);
      const left = parseInt(element.style.left);
      if (top > 0 && left > 0) {
        required_height = Math.max(required_height, top + element.offsetHeight);
        required_width = Math.max(required_width, left + element.offsetWidth);
      }
    }
    this.parent_.style.height = (required_height + kMargin) + "px";
    this.parent_.style.width = (required_width + kMargin) + "px";
  }

  // Helper to resize a window.
  // resize_xxx means we need to handle resize of this specific edge
  _resizeElement(element, resize_top, resize_right, resize_bottom, resize_left) {
    let binder = this;
    let dx = 0, dy = 0, px = 0, py = 0;
    element.onmousedown = dragMouseDown;

    // Called when the user click
    function dragMouseDown(e) {
      e = e || window.event;
      // get the mouse cursor position at startup:
      px = e.clientX;
      py = e.clientY;
      document.onmouseup = closeDragElement;
      // call a function whenever the cursor moves:
      document.onmousemove = elementDrag;
      binder._bringWindowTop(element.parentNode);
    }

    // Called when the user is moving the mouse (while clicking).
    function elementDrag(e) {
      e = e || window.event;
      // calculate the new cursor position:
      dx = e.clientX - px;
      dy = e.clientY - py;
      px = e.clientX;
      py = e.clientY;
      // set the element's new position:
      let style = element.parentNode.style;
      // Handle vertical resizing.
      if (resize_top) {
        style.top = (parseInt(style.top) + dy) + "px";
        style.height = (parseInt(style.height) - dy) + "px";
      } else if (resize_bottom) {
        style.height = (parseInt(style.height) + dy) + "px";
      }
      // Hand horizontal resizing.
      if (resize_left) {
        style.left = (parseInt(style.left) + dx) + "px";
        style.width = (parseInt(style.width) - dx) + "px";
      } else if (resize_right) {
        style.width = (parseInt(style.width) + dx) + "px";
      }
      e.preventDefault();
    }

    // Called when the user finish their click.
    function closeDragElement() {
      /* stop moving when mouse button is released:*/
      document.onmouseup = null;
      document.onmousemove = null;
      binder._resizeWindowManager();
    }
  }

  // Resize the dimensions of the windows manager
  _resizeWindowManager() {
    const kMargin = 200;
    let required_height = 0;
    let required_width = 0;
    for (let i in this.windows_) {
      const element = this.windows_[i];
      const top = parseInt(element.style.top);
      const left = parseInt(element.style.left);
      if (top > 0 && left > 0) {
        required_height = Math.max(required_height, top + element.offsetHeight);
        required_width = Math.max(required_width, left + element.offsetWidth);
      }
    }
    this.parent_.style.height = (required_height + kMargin) + "px";
    this.parent_.style.width = (required_width + kMargin) + "px";
  }
}

let window_manager_ = null;

// Implement a "singleton" of WindowManager
function WindowManager() {
  if (window_manager_ == null) {
    window_manager_ = new WindowManagerImpl(document.getElementById("data_windows"),
                                            document.getElementById("window_menu"));
  }
  return window_manager_;
}
