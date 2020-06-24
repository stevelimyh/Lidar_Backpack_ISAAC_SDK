/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Creates a 3JS object to render a coordinate frame
function CreateIsaacAxes(size, width) {
  let axes = new THREE.AxesHelper(size);
  axes.material.linewidth = width;
  return axes;
}

// Create a 3JS object to render an occupancy map as a flat textured map
function CreateFlatFloorMap(image, pixelsize) {
  let scale = pixelsize;
  // a large quad on the ground
  let floor_geo = new THREE.Geometry();
  floor_geo.vertices.push(new THREE.Vector3(0,0,0));
  floor_geo.vertices.push(new THREE.Vector3(0,scale*image.width,0));
  floor_geo.vertices.push(new THREE.Vector3(scale*image.height, scale*image.width,0));
  floor_geo.vertices.push(new THREE.Vector3(scale*image.height,0,0));
  floor_geo.faces.push(new THREE.Face3(0, 1, 2, new THREE.Vector3(0,1,0), new THREE.Color(0xffffff), 0));
  floor_geo.faces.push(new THREE.Face3(0, 2, 3, new THREE.Vector3(0,1,0), new THREE.Color(0xffffff), 0));
  floor_geo.faceVertexUvs[0] = [];
  floor_geo.faceVertexUvs[0].push([
    new THREE.Vector2(0,1),
    new THREE.Vector2(1,1),
    new THREE.Vector2(1,0)
  ]);
  floor_geo.faceVertexUvs[0].push([
    new THREE.Vector2(0,1),
    new THREE.Vector2(1,0),
    new THREE.Vector2(0,0)
  ]);
  floor_geo.computeVertexNormals();
  let floor_mat = new THREE.MeshLambertMaterial({ color: 0x999999, side: THREE.DoubleSide });
  let texture = new THREE.Texture();
  texture.image = image;
  texture.format = THREE.RGBAFormat;
  texture.needsUpdate = true;
  floor_mat.map = texture;  //texloader.load(texture_name);
  floor_mat.map.magFilter = THREE.NearestFilter;
  floor_mat.map.minFilter = THREE.NearestFilter;
  let plane = new THREE.Mesh(floor_geo, floor_mat);
  plane.receiveShadow = true;
  plane.castShadow = false;
  return plane;
}

// Create a 3JS object to render an occupancy map as an extruded height map
function CreateExtrudedFloorMap(image, pixelsize, height) {
  let blockunits = pixelsize;
  let matrix = new THREE.Matrix4();

  // It seems that chrome sometimes have issue calling getImageData on big canvas, therefore we
  // split the image in smaller blocks.
  const max_dims = 256;

  // Returs a buffer image where we can easily access the pixel value.
  function getImageData(image, row, col) {
    let canvas = document.createElement('canvas');
    canvas.width = Math.min(max_dims, image.width - col);
    canvas.height = Math.min(max_dims, image.height - row);
    let context = canvas.getContext('2d');
    context.drawImage(image, -col, -row);
    return context.getImageData(0, 0, canvas.width, canvas.height);
  }

  // Returns 1 if the pixel correspon to a wall or 0 if it's empty.
  function getH(x, y, imgdata) {
    const q = imgdata.data[4*(x + imgdata.width*y)];
    return (q < 120) ? 1 : 0;
  }

  // Recursive function that explore the map as a quadtree and merge nodes that are either fully
  // blocked or fully empty.
  // It returns a data structure that contain the state (1 for wall, 0 for empty, -1 for a mix).
  // In addition it also contains the list of children if state == -1 and the boundary of the node.
  let extractQuadTree = function(min_x, max_x, min_y, max_y, imgdata) {
    const mid_x = ((min_x + max_x)/2) | 0;
    const mid_y = ((min_y + max_y)/2) | 0;
    // If we have a single row, we won't try to split in this dimension
    if (min_x + 1 == max_x) {
      // Check if the node is a single pixel.
      if (min_y + 1 == max_y) {
        return {
          min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y,
          state: getH(min_x, min_y, imgdata)
        };
      }
      // Let's recurse in the Y direction and see if we can merge both node or not.
      const h1 = extractQuadTree(min_x, max_x, min_y, mid_y, imgdata);
      const h2 = extractQuadTree(min_x, max_x, mid_y, max_y, imgdata);
      if (h1.state == h2.state &&  h2.state != -1) {
        return { min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y, state: h1.state };
      }
      return { min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y, state: -1,
               children: [h1, h2] };
    }
    // Let's check if the Y direction is a single column.
    if (min_y + 1 == max_y) {
      // Let's recurse in the X direction and see if we can merge both node or not.
      const h1 = extractQuadTree(min_x, mid_x, min_y, max_y, imgdata);
      const h2 = extractQuadTree(mid_x, max_x, min_y, max_y, imgdata);
      if (h1.state == h2.state &&  h2.state != -1) {
        return { min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y, state: h1.state };
      }
      return { min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y, state: -1,
               children: [h1, h2] };
    }
    // Let's split the node in 4 and extract the information of each node.
    let h11 = extractQuadTree(min_x, mid_x, min_y, mid_y, imgdata);
    let h21 = extractQuadTree(mid_x, max_x, min_y, mid_y, imgdata);
    let h12 = extractQuadTree(min_x, mid_x, mid_y, max_y, imgdata);
    let h22 = extractQuadTree(mid_x, max_x, mid_y, max_y, imgdata);
    // Check if they are all empty or a wall
    if (h11.state == h12.state && h11.state == h21.state && h11.state == h22.state &&
        h11.state != -1) {
      return { min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y, state: h11.state };
    }
    // Let's see if we can merge the child at (0, 0) with (1, 0) or (0, 1)
    if (h11.state == h12.state && h11.state >= 0) {
      h11.max_y = h12.max_y;
      h12 = null;
    } else if (h11.state == h21.state && h11.state >= 0) {
      h11.max_x = h21.max_x;
      h21 = null;
    }
    // Let's see if we can merge the child at (1, 1) with (1, 0) or (0, 1)
    if (h12 && h22.state == h12.state && h22.state >= 0) {
      h22.min_x = h12.min_x;
      h12 = null;
    } else if (h21 && h22.state == h21.state && h22.state >= 0) {
      h22.min_y = h21.min_y;
      h21 = null;
    }
    let children = [h11, h22];
    if (h12) children.push(h12);
    if (h21) children.push(h21);
    return { min_x: min_x, min_y: min_y, max_x: max_x, max_y: max_y, state: -1,
             children: children };
  };

  // Builds the geometry from a quad_tree and a given offset
  let buildGeometry = function(sx, sy, quad_tree, geometry) {
    // If it's all empty, we can just exit now.
    if (quad_tree.state == 0) return;
    // If it's all full, we need to put a box that cover the full node.
    if (quad_tree.state == 1) {
      const delta_x = (quad_tree.max_x - quad_tree.min_x);
      const delta_y = (quad_tree.max_y - quad_tree.min_y);
      matrix.makeTranslation(
          (sy + quad_tree.min_y + delta_y * 0.5) * blockunits,
          (sx + quad_tree.min_x + delta_x * 0.5) * blockunits,
           height / 2);
      geometry.merge(
          new THREE.BoxGeometry(delta_y * blockunits, delta_x * blockunits, height), matrix);
      return;
    }
    // Otherwise we need to recurse in all the children.
    for (let c in quad_tree.children) {
      buildGeometry(sx, sy, quad_tree.children[c], geometry);
    }
  };

  let tmpGeometry = new THREE.Geometry();
  // Let's break the image in subimage that can be loaded in memory and construct the quad tree
  // and reconstruct the geometry out of it.
  for (let block_y = 0; block_y * max_dims < image.height; block_y++) {
    for (let block_x = 0; block_x * max_dims < image.width; block_x++) {
      const start_x = block_x * max_dims;
      const start_y = block_y * max_dims;
      let imgdata = getImageData(image, start_y, start_x);
      buildGeometry(start_x, start_y, extractQuadTree(0, imgdata.width, 0, imgdata.height, imgdata),
                    tmpGeometry);
    }
  }

  let geometry = new THREE.BufferGeometry().fromGeometry(tmpGeometry);
  geometry.computeBoundingSphere();

  let mat = new THREE.MeshLambertMaterial({color: 0x333333, side: THREE.DoubleSide});

  let mesh = new THREE.Mesh(geometry, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

// Creates visualization for a map
function CreateMap(image, pixelsize, height = 200, floor_z = 0) {
  let floor = new THREE.Group();
  // flat floor map
  floor.add(CreateFlatFloorMap(image, pixelsize));
  // extruded floor map
  floor.add(CreateExtrudedFloorMap(image, pixelsize, height));
  floor.translateZ(floor_z);
  return floor;
}
