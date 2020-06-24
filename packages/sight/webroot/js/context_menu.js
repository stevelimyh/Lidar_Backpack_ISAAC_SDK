/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Create a context menu that can be attached to any object of the DOM.
class ContextMenu {
  constructor(menu) {
    // The main menu div.
    this.menu_ = document.createElement("div");
    this.menu_.className += " sight-context-menu";
    // Whether or not we should ignore the next click (it is used as when we show the menu we
    // trigger a click to hide all the other menu).
    this.ignore_next_click_ = false;
    // Whether or not the menu is currently being displayed
    this.showing_ = false;
    // List of the items in the menu
    this.items_ = [];
    // List of item with a submenu
    this.divs_with_submenu_ = [];
    // Add all the item one by one.
    for (let i in menu) {
      this.addItem(menu[i]);
    }
    // Disable context menu on the menu
    this.menu_.oncontextmenu = function(evt) {
      evt.preventDefault();
      return false;
    };
  }

  // Add a new item to the menu:
  // item: {
  //   text: String to be displayed
  //   callback: callback function for a clic or slider update
  //   enable: default true, wether or not we can interract with it
  //   slider: Information about the slider {min = 0, max = 1, value = 0.5},
  //   submenu: List of items to be displayed on a submenu (works recursively)
  // }
  addItem(item) {
    let div = document.createElement("div");
    div.appendChild(document.createTextNode(""));
    div.callback = item.callback;
    div.updateTitle = function (text) {
      div.firstChild.nodeValue = text;
    }
    if (item.text) div.updateTitle(item.text);
    if (item.slider) {
      item.enable = false;
      if (!item.slider.min) item.slider.min = 0.0;
      if (!item.slider.max) item.slider.max = 1.0;
      if (!item.slider.value) item.slider.value =  0.5 * (item.slider.max + item.slider.min);
      let slider = document.createElement('input');
      let slider_out = document.createElement('output');
      slider_out.style.display = "inline";
      slider.type = "range";
      slider.className = "slider-context-menu";
      slider.min = item.slider.min;
      slider.max = item.slider.max;
      slider.step = "any";
      slider.value = item.slider.value;
      slider_out.value = item.slider.value;
      div.appendChild(slider);
      div.appendChild(slider_out);
      slider.oninput = function(event) {
        // Round to two decimals
        this.value = ((100.0 * this.value) | 0) / 100;
        slider_out.value = this.value;
        item.callback(this.value);
      };
      // Prevent a change in the slidder to hide the menu.
      slider.onclick = function(event) { event.stopPropagation(); };
    }

    this.menu_.appendChild(div);
    this.items_.push(div);
    if (item.enable !== false) {
      this.enableItem(this.items_.length - 1);
    } else {
      this.disableItem(this.items_.length - 1);
    }
    if (item.submenu) {
      div.className += " sight-context-menu-with-submenu";
      let submenu = new ContextMenu(item.submenu);
      div.appendChild(submenu.menu_);
      div.onmouseoverFunction = function() {
        const x = div.parentNode.offsetLeft + div.parentNode.offsetWidth - 5;
        let y = div.offsetTop + div.parentNode.offsetTop;
        submenu.menu_.style.display = "block";
        const height = submenu.menu_.offsetHeight;
        if (y + height > window.innerHeight) y -= height - 24;  // Size of one line is 24
        submenu.menu_.style.left = x + "px";
        submenu.menu_.style.top = y + "px";
        submenu.addEventListeners();
        // Let set the onmouseleave lisener
        div.onmouseleave = function() {
          submenu.menu_.style.display = "";
          // clean the onmouseleave lisener
          div.onmouseleave = null;
          submenu.removeEventListeners();
        }
      }
      this.divs_with_submenu_.push(div);
    }
  }

  // Adds the onmouseover listener for all the item with a submenu and click listener for callback.
  addEventListeners() {
    if (this.showing_) return;
    // Adds all the submeny onmouseover
    for (let id in this.divs_with_submenu_) {
      this.divs_with_submenu_[id].addEventListener(
          "mouseover", this.divs_with_submenu_[id].onmouseoverFunction, false);
    }
    // Adds all the click listener for the div with a callback
    for (let id in this.menu_.children) {
      if (this.menu_.children[id].callback && this.menu_.children[id].enable) {
        this.menu_.children[id].addEventListener(
           "click", this.menu_.children[id].callback, false);
      }
    }
    const that = this;
    // Hide the menu after a click
    let removeMenu = function(evt) {
      // Ignore the first click that we throw when we open this menu
      if (that.ignore_next_click_) {
        that.ignore_next_click_ = false;
        return;
      }
      // Firefox fire an event click after the mouseup
      if (evt.button != 2 || that.menu_.parentNode === null ||
          evt.target.id != that.menu_.parentNode.id) {
        that.menu_.style.display = "";
        that.showing_ = false;
        that.removeEventListeners();
        // Clean this click listener
        window.removeEventListener("click", removeMenu, false);
      }
    };
    window.addEventListener("click", removeMenu, false);
  }

  // Remove the onmouseover listener for each item with a submenu.
  removeEventListeners() {
    for (let id in this.divs_with_submenu_) {
      this.divs_with_submenu_[id].removeEventListener(
          "mouseover", this.divs_with_submenu_[id].onmouseoverFunction, false);
    }
    for (let id in this.menu_.children) {
      if (this.menu_.children[id].callback && this.menu_.children[id].enable) {
        this.menu_.children[id].removeEventListener(
           "click", this.menu_.children[id].callback, false);
      }
    }
  }

  // Remove every element of the context menu
  clear() {
    for (let i = 0; i < this.items_.length; i++) {
      this.menu_.removeChild(this.items_[i]);
    }
    this.items_ = [];
  }

  // Remove an item from the menu
  removeItem(id) {
    this.menu_.removeChild(this.items_[id]);
    this.items_.splice(id, 1);
  }

  // Set an item as disable (can't click on it anymire)
  disableItem(id) {
    this.items_[id].false = false;
    this.items_[id].className = "sight-context-menu-item-disable";
  }

  // Enable an item (click on it will trigger its callback)
  enableItem(id) {
    this.items_[id].enable = true;
    this.items_[id].className = "sight-context-menu-item";
  }

  // Show the menu at a given position
  show(x, y) {
    this.addEventListeners();
    // Hide other menu.
    this.ignore_next_click_ = true;
    this.menu_.click();
    // Render the current menu at the position of the click
    this.menu_.style.display = "block";
    const height = this.menu_.offsetHeight;
    if (y + height > window.innerHeight) y -= height;
    this.menu_.style.left = x + "px";
    this.menu_.style.top = y + "px";
    this.showing_ = true;
  }

  // Attach the menu to an object.
  attachTo(parent) {
    if (parent.id == "") parent.id = '_' + Math.random().toString(36).substr(2, 9);
    // Append to the body as some object cannot contain the menu such as a canvas
    document.body.appendChild(this.menu_);

    const that = this;
    // Disable the default context menu
    parent.addEventListener("contextmenu", function(evt) {
      that.show(evt.clientX-1, evt.clientY-1);
      // Prevent the default context menu to show up
      evt.preventDefault();
    }, false);
  }
}