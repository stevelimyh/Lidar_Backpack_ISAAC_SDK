/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Helper class to create a Tree structure menu
class TreeMenu {
  constructor(callback, getStatus = null, save = null, contextmenu = null) {
    this.root_ = {};
    this.reset();
    this.callback = callback;
    this.update_tree_scheduled_ = false;
    this.getStatus = getStatus;
    this.save = save;
    this.contextmenu = contextmenu;
    if (this.save == null) this.save = function(n, v) {};
    if (this.getStatus == null) {
      this.getStatus = function(n, v) { return {open: false, checked: false}};
    }
  }

  reset() {
    this.redraw_required_ = true;
    this.root_ = {
      name: ".",
      fullname: "",
      id: null,
      nodes: {},
      parent: null,
      checkbox: null,
      open: true,
      checked: true
    };
  }

  // Returns a node given a name
  findNode(name) {
    let names = this.extractSubNames(name);
    let node = this.root_;
    for (let i = 0; i < names.length; i++) {
      node = node.nodes[names[i]];
      if (!node) {
        console.error("Could not find the node: " + name);
        return null;
      }
    }
    return node;
  }

  // Extract the list of names of the node in the tree for a given name:
  // Should be: "app_name/node_name/component_name/label.dot.separated"
  extractSubNames(name) {
    let sub_names = [];
    let slash_split = name.split("/");
    if (slash_split.length > 3) {
      sub_names.push(slash_split[0]);  // App name
      sub_names.push(slash_split[1]);  // Node name
      sub_names.push(slash_split[2]);  // Component name
      // Split the label.
      for (let i = 3; i < slash_split.length; i++) {
        let temp = slash_split[i].split(".");
        for (let s in temp) {
          sub_names.push(temp[s]);
        }
      }
    } else {
      console.error("The name (" + name + ") does not respect the format: " +
                    "'app_name/node_name/component_name/label.dot.separated'");
      sub_names = name.split(".");
    }
    return sub_names;
  }

  // Return the delimiting symbol given the position in the tree
  splitSymbol(prof) {
    if (prof == 0) return "";
    if (prof <= 3) return "/";  // Split app/node/component/label using "/"
    return ".";  // Split inside the label using "."
  }

  // Return the delimiting symbol given the position in the tree
  splitStyle(prof) {
    if (prof == 0) return "tree-app";
    if (prof == 1) return "tree-node";
    if (prof == 2) return "tree-component";
    return "tree-label";
  }

  // Add a new value to the tree.
  addToTree(fullname) {
    let sub_names = this.extractSubNames(fullname);
    let node = this.root_;
    let cur_name = "";
    for (let i = 0; i < sub_names.length; i++) {
      if (sub_names[i] == "") continue;
      cur_name += this.splitSymbol(i) + sub_names[i];
      if (node.nodes[sub_names[i]] === undefined) {
        let name = sub_names[i];
        name = "<span class='" + this.splitStyle(i) + "'>" + name + "</span>";
        node.nodes[sub_names[i]] = {
            name: name,
            fullname: cur_name,
            id: null,
            nodes: {},
            parent: node,
            checkbox: null,
            open: this.getStatus(cur_name).open,
            checked: this.getStatus(cur_name).checked
        };
      }
      node = node.nodes[sub_names[i]];
    }
    node.id = fullname;
    node.checked = this.getStatus(cur_name).checked;
    this.redraw_required_ = true;
  }

  // Update the status of each checkbox
  updateCheckbox() {
    this.updateCheckboxImpl(this.root_);
  }
  // Implementation:
  // Recursively update the status of each checkbox:
  // If it's a leaf, do nothing, if it's a group and all children have the same state, update
  // this node to have the same state. If the state is unknown, change the style to disable.
  // This function return -1 if all children are unchecked, 1 if they are all checked, and 0
  // otherwise
  updateCheckboxImpl(node) {
    // Set unset value
    let ret = -2;
    for (let i in node.nodes) {
      let child = this.updateCheckboxImpl(node.nodes[i]);
      if (ret == -2) {
        // First children, let set the return value to the same one
        ret = child;
      } else if (child != ret) {
        // Disagreement, let's update the value to 0
        ret = 0;
      }
    }

    if (node.checkbox == null) return ret;

    if (ret == 0) {
      node.checkbox.parentElement.className = "control control--disabled";
      node.checkbox.checked = true;
    } else {
      if (ret == -2) {
        // We are at a leaf
        return node.checkbox.checked ? 1 : -1;
      }
      node.checkbox.parentElement.className = "control control--checkbox";
      if (node.checkbox.checked != (ret == 1)) {
        node.checkbox.checked = !node.checkbox.checked;
        node.checkbox.onchange();
      }
    }
    return ret;
  }

  // make sure all the children have the proper set value.
  propagateChecked(node, value) {
    for (let i in node.nodes) {
      if (node.nodes[i].checkbox == null ||
          node.nodes[i].checkbox.checked == value) {
        this.propagateChecked(node.nodes[i], value);
      } else {
        node.nodes[i].checkbox.checked = !node.nodes[i].checkbox.checked;
        node.nodes[i].checkbox.onchange();
      }
    }
  }

  // Renter the menu in the given div
  renderTree(div) {
    if (!this.redraw_required_) {
      return;
    }
    // Clear the div
    while (div.hasChildNodes()) {
      div.removeChild(div.firstChild);
    }
    div.appendChild(this.renderTreeImpl(this.root_));
    this.updateCheckbox(this.root_);
    this.redraw_required_ = false;
    div.oncontextmenu = function(evt) {
      evt.preventDefault();
      return false;
    };
  }

  // Return a div containing the full tree starting from a node
  renderTreeImpl(node, prof = 1) {
    const that = this;
    // Get the list of the children
    let arr = [];
    for (let i in node.nodes) {
      arr.push(node.nodes[i])
    }
    // sort internal node first, leaf last, and then sort by name.
    arr.sort(function(a,b) {
      let isLeaf = function(node) {
        let count = 0;
        for (let i in node.nodes) {
          if (count >= 1) return false;
          if (!isLeaf(node.nodes[i])) return false;
          count++;
        }
        return true;
      }
      if (isLeaf(a) && !isLeaf(b)) return true;
      if (!isLeaf(a) && isLeaf(b)) return false;
      return a.name.toLowerCase().localeCompare(b.name.toLowerCase())
    });

    // div holding everything
    let ans = document.createElement("div");
    ans.style.width = "max-content";
    // Contain the full checkbox structure
    let label = document.createElement('label');
    // Additional icon for internal node
    let div_icon = document.createElement('div');
    // Name of the current node
    let div_text = document.createElement('div');
    // The div the full line checkbox/icon/name
    let div_name = document.createElement('div');
    // The actual checkbox
    let check = document.createElement('input');
    if (node.parent != null) {
      // Div holding the visual checkbox
      let divspan = document.createElement('div');
      divspan.className += " control__indicator";

      check.type = "checkbox";
      check.node = node;
      node.checkbox = check;
      check.checked = node.checked;
      check.tree = this;
      check.onchange = function() {
        // If we are the first node to change, then we are in charge of calling 'updateCheckbox'.
        let update = false;
        if (!this.tree.update_tree_scheduled_) {
          this.tree.update_tree_scheduled_ = true;
          update = true;
        }
        if (this.node.id != null) {
          try {
            this.tree.callback(this.node.id, this.checked);
          } catch (e) {}
        } else {
          this.tree.propagateChecked(this.node, this.checked);
        }
        if (update) {
          this.tree.updateCheckbox();
          this.tree.update_tree_scheduled_ = false;
        }
      };

      label.className = "control control--checkbox";
      label.appendChild(check);
      label.appendChild(divspan);

      div_icon.className += " div_icon";
      div_icon.className += " unselectable";

      div_text.className += " div_text";
      div_text.className += " unselectable";
      div_text.innerHTML = node.name;
      div_text.title = node.fullname;
      div_text.leaf = true;
      // Allow to copy the fullname using the middle mouse button
      div_text.addEventListener('mousedown', function(e) {
        switch(e.which) {
            case 2: // Middle
              const el = document.createElement('textarea');
              el.value = this.title;
              document.body.appendChild(el);
              el.select();
              document.execCommand('copy');
              document.body.removeChild(el);
              return true;
            case 1:  // Left
              break;
            case 3:  // Right
              if (that.contextmenu !== null) {
                let names = [];
                const fct = function(node) {
                  names.push(node.fullname);
                  for (let i in node.nodes) {
                    fct(node.nodes[i]);
                  }
                }
                fct(node);
                return that.contextmenu(e, names, node.fullname);
              }
        }
        return false;
      });

      div_name.className = "control_label";
      div_name.style.height = "25px";
      div_name.style.width = "auto";
      div_name.appendChild(label);
      div_name.appendChild(div_icon);
      div_name.appendChild(div_text);
      ans.appendChild(div_name);
    }
    // If we have children, create the tree for each children and append it.
    if (arr.length > 0) {
      div_text.leaf = false;
      let holder = document.createElement("div");
      holder.style.marginLeft = "25px";
      if (node.parent != null) {
        div_text.node = node;
        div_icon.node = node;
        div_text.className += " folder_text";
        if (node.open === false) {
          holder.style.display = "none";
          div_icon.innerHTML = "<i class='material-icons'>chevron_right</i>";
        } else {
          div_icon.innerHTML = "<i class='material-icons'>expand_more</i>";
        }
        // Function that handle hiding and displaying the children div.
        let fct = function(event) {
          if (holder.style.display == "none") {
            that.save(this.node.fullname, true);
            $(holder).slideDown("fast");
            div_icon.innerHTML = "<i class='material-icons'>expand_more</i>";
          } else {
            that.save(this.node.fullname, false);
            $(holder).slideUp("fast");
            div_icon.innerHTML = "<i class='material-icons'>chevron_right</i>";
          }
        };
        div_text.addEventListener('click', fct);
        div_icon.addEventListener('click', fct);
      }

      for (let i = 0; i < arr.length; i++) {
        holder.appendChild(this.renderTreeImpl(arr[i], prof + 1));
      }
      ans.appendChild(holder);
    }
    return ans;
  }
}
