/*
  Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

  NVIDIA CORPORATION and its licensors retain all intellectual property
  and proprietary rights in and to this software, related documentation
  and any modifications thereto. Any use, reproduction, disclosure or
  distribution of this software and related documentation without an express
  license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

/*
  Defines a Mesh to represent a marker for 3D views.
  This object is actually a group of groups of meshes
 */

/* Helper function that creates the geometries needed for a the marker's mesh*/
function widgetGeometries() {
  const lerp = (a, b, t) => a*(1-t) + b*t;
  const unlerp = (a, b, x) => (x - a)/(b - a);

  function hornGeometry() {
    /*

      The majority of the widget geometry is simply the surface

      x²y² + y²z² + z²x² = 1

      In terms of cylindrical coordinates along the z axis, this function generates the chart
      corresponding to [zMin, zMax] x [-PI/4, +PI/4].

      Denote a point in cylindrical coordinates as (d, phi, z).
      We can rewrite the surface equation as

      d²(d²cos²(phi)sin²(phi) + z²) = 1
      (d²)² cos²(phi)sin²(phi) + z² d² - 1 = 0
      d² = ( sqrt(z⁴ + 4cos²(phi)sin²(phi)) - z² )/(2cos²(phi)sin²(phi))

    */

    let indices = [];
    let positions = [];
    let normals = [];

    const zMin = 1;
    const zMax = 2.5;

    const zSegmentCount = 21;
    const phiSegmentCount = 21;

    const zSliceCount = zSegmentCount + 1;
    const phiSliceCount = phiSegmentCount + 1;

    for (let zSliceIdx=0; zSliceIdx < zSliceCount; zSliceIdx++) {
      const z = lerp(zMin, zMax, zSliceIdx/zSegmentCount);
      for (let phiSliceIdx=0; phiSliceIdx < phiSliceCount; phiSliceIdx++) {
        const phi = lerp(-Math.PI/4, Math.PI/4, phiSliceIdx/phiSegmentCount);
        const d =
              Math.sqrt(
                (Math.sqrt(Math.pow(z, 4) + 4*Math.pow(Math.cos(phi)*Math.sin(phi), 2))
                     - Math.pow(z, 2))/(2*Math.pow(Math.cos(phi)*Math.sin(phi), 2))
              );

        const x = d*Math.cos(phi);
        const y = d*Math.sin(phi);

        const nl =
              2*Math.sqrt(
                Math.pow(x, 2)*Math.pow(Math.pow(y, 2) + Math.pow(z, 2), 2) +
                  Math.pow(y, 2)*Math.pow(Math.pow(z, 2) + Math.pow(x, 2), 2) +
                  Math.pow(z, 2)*Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 2)
              );
        const nx = 2*(x*(Math.pow(y, 2) + Math.pow(z, 2)))/nl;
        const ny = 2*(y*(Math.pow(z, 2) + Math.pow(x, 2)))/nl;
        const nz = 2*(z*(Math.pow(x, 2) + Math.pow(y, 2)))/nl;

        positions.push(x, y, z);
        normals.push(nx, ny, nz);
      }
    }

    for (let zSegmentIdx=0; zSegmentIdx < zSegmentCount; zSegmentIdx++) {
      for (let phiSegmentIdx=0; phiSegmentIdx < phiSegmentCount; phiSegmentIdx++) {
        for (let triangleIdx=0; triangleIdx < 2; triangleIdx++) {
          for (let vertexIdx=0; vertexIdx < 3; vertexIdx++) {
            const zSliceIdx = (([0,1,0])[vertexIdx] + triangleIdx) % 2;
            const phiSliceIdx = (([1,0,0])[vertexIdx] + triangleIdx) % 2;
            indices.push(phiSliceCount*(zSegmentIdx + zSliceIdx) + phiSegmentIdx + phiSliceIdx);
          }
        }
      }
    }

    let geometry = new THREE.BufferGeometry();
    geometry.setIndex(indices);
    geometry.addAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.addAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
    return geometry;
  }

  function capGeometry() {
    /*
      For a fixed phi around the z axis, the cap is a function of the form

      z(d) = a - (d/b)^p

      where p > 1.
      We want to satisfy the following constraints:

      z(0) = zMax
      z(dMin) = zMin
      z'(dMin) = k


      From the first constraint, a = zMax.
      Then, from the second it follows that
      b^(-p) = (zMax - zMin)/dMin^p
      Computing z'(d) = -pd^(p-1)b^(-p), and plugging the preceeding relation in, we get

      a = zMax
      b = dMin/(zMax - zMin)^((zMax - zMin)/(-k dMin))
      p = -k dMin/(zMax - zMin)

      The full formula becomes

      z(d) = zMax - (zMax - zMin) (d/dMin)^(-k dMin/(zMax - zMin))

      z'(d) =  k (d/dMin)^(-k dMin/(zMax - zMin) - 1)
    */

    let indices = [];
    let positions = [];
    let normals = [];

    const zMin = 2.5;
    const zMax = 3.0;

    const dSegmentCount = 21;
    const phiSegmentCount = 21;

    const dSliceCount = dSegmentCount + 1;
    const phiSliceCount = phiSegmentCount + 1;

    for (let phiSliceIdx=0; phiSliceIdx < phiSliceCount; phiSliceIdx++) {
      for (let dSliceIdx=0; dSliceIdx < dSliceCount; dSliceIdx++) {
        const phi = lerp(-Math.PI/4, Math.PI/4, phiSliceIdx/phiSegmentCount);
        const a = Math.pow(Math.cos(phi)*Math.sin(phi), 2);
        const b = Math.pow(zMin, 2);
        const dMin = Math.sqrt((Math.sqrt(Math.pow(b, 2) + 4*a) - b)/(2*a));
        const d = lerp(0, dMin, dSliceIdx/dSegmentCount);
        const k = -(2*Math.pow(dMin, 2)*a + Math.pow(zMin, 2))/(dMin*zMin);

        const z = zMax - (zMax - zMin)*Math.pow(d/dMin, -k*dMin/(zMax - zMin));
        const x = d*Math.cos(phi);
        const y = d*Math.sin(phi);

        const dzdd = k*Math.pow(d/dMin, -k*dMin/(zMax-zMin) - 1);

        const unx = Math.cos(phi);
        const uny = Math.sin(phi);
        const unz = -1/dzdd;
        let n;
        if (dSliceIdx == 0) {
          n = [0,0,1]
        } else {
          const unl = Math.sqrt(Math.pow(unx, 2) + Math.pow(uny, 2) + Math.pow(unz, 2));
          n = [unx/unl, uny/unl, unz/unl];
        }

        positions.push(x, y, z);
        normals.push(n[0], n[1], n[2]);
      }
    }

    for (let phiSegmentIdx=0; phiSegmentIdx < phiSegmentCount; phiSegmentIdx++) {
      for (let dSegmentIdx=0; dSegmentIdx < dSegmentCount; dSegmentIdx++) {
        for (let triangleIdx=0; triangleIdx < 2; triangleIdx++) {
          for (let vertexIdx=0; vertexIdx < 3; vertexIdx++) {
            const dSliceIdx = (([0,1,0])[vertexIdx] + triangleIdx) % 2;
            const phiSliceIdx = (([1,0,0])[vertexIdx] + triangleIdx) % 2;
            indices.push(phiSliceCount*(dSegmentIdx + dSliceIdx) + phiSegmentIdx + phiSliceIdx);
          }
        }
      }
    }

    let geometry = new THREE.BufferGeometry();
    geometry.setIndex(indices);
    geometry.addAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.addAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
    return geometry;
  }

  function sockGeometry() {
    /*
      The sock is the region of the surface with 0 <= z <= gamma, for some gamma >= 1.

      Let
      a = [0,  0, 1]
      b = [1, -1, 1]
      c = [1, +1, 1]

      This region of the surface is the intersection of the interior of the projective triangle
      given by a, b, c and the constraint 0 <= z <= gamma.

      We parameterize this region as a + (b - a) u + (c - a) v = [u+v, u-v, 1] where u and v is in
      the triangular area given by (0,0), (0,1), (1,0).
      We should not include all of this region, however. Because we want to end at z=gamma where
      the cap starts.
      To find the border z=gamma in the uv-plane, we consider the
      curve [x, y, z] = gamma[u+v, u-v, 1]
      of points on the surface for a constant gamma.
      (u+v)²(u-v)² + (u-v)² + (u+v)² = gamma⁻⁴
      (u²-v²)² + 2(u²+v²) = gamma⁻⁴

      The calculations are a bit easier in polar coordinates.
      r⁴(cos²(phi) - sin²(phi))² + 2r² = gamma⁻⁴
      r⁴cos²(2phi) + 2r² = gamma⁻⁴
      We can easily solve for r.

      To make it a bit easier, we also express the upper limit in polar coordinates.
      r cos(phi) + r sin(phi) = 1
      so
      r(phi) = 1/(cos(phi) + sin(phi))
    */

    let indices = [];
    let positions = [];
    let normals = [];

    const gamma = 1.0;
    const phiSegmentCount = 21;
    const rSegmentCount = 21;

    const phiSliceCount = phiSegmentCount + 1;
    const rSliceCount = rSegmentCount + 1;

    for (let phiSliceIdx = 0; phiSliceIdx < phiSliceCount; phiSliceIdx++) {
      const phi = lerp(0, Math.PI/2, unlerp(0, phiSegmentCount, phiSliceIdx));
      for (let rSliceIdx = 0; rSliceIdx < rSliceCount; rSliceIdx++) {
        const a = Math.pow(Math.cos(2*phi), 2);
        const b = 2;
        const c = -Math.pow(gamma, -4);
        const rMin = Math.sqrt((-b + Math.sqrt(Math.pow(b, 2) - 4*a*c))/(2*a));
        const rMax = 1/(Math.cos(phi) + Math.sin(phi));
        const r = lerp(rMin, rMax, unlerp(0, rSegmentCount, rSliceIdx));
        const u = r*Math.cos(phi);
        const v = r*Math.sin(phi);
        const z = Math.pow(Math.pow((u+v)*(v-u), 2) + 2*(Math.pow(u, 2) + Math.pow(v, 2)), -1/4);
        const x = (u + v)*z;
        const y = (v - u)*z;

        const nl =
              2*Math.sqrt(
                Math.pow(x, 2)*Math.pow(Math.pow(y, 2) + Math.pow(z, 2), 2) +
                  Math.pow(y, 2)*Math.pow(Math.pow(z, 2) + Math.pow(x, 2), 2) +
                  Math.pow(z, 2)*Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 2)
              );
        const nx = 2*(x*(Math.pow(y, 2) + Math.pow(z, 2)))/nl;
        const ny = 2*(y*(Math.pow(z, 2) + Math.pow(x, 2)))/nl;
        const nz = 2*(z*(Math.pow(x, 2) + Math.pow(y, 2)))/nl;

        positions.push(x, y, z);
        normals.push(nx, ny, nz);
      }
    }

    for (let phiSegmentIdx=0; phiSegmentIdx < phiSegmentCount; phiSegmentIdx++) {
      for (let rSegmentIdx=0; rSegmentIdx < rSegmentCount; rSegmentIdx++) {
        for (let triangleIdx=0; triangleIdx < 2; triangleIdx++) {
          for (let vertexIdx=0; vertexIdx < 3; vertexIdx++) {
            const rSliceIdx = (([0,1,0])[vertexIdx] + triangleIdx) % 2;
            const phiSliceIdx = (([1,0,0])[vertexIdx] + triangleIdx) % 2;
            indices.push(phiSliceCount*(rSegmentIdx + rSliceIdx) + phiSegmentIdx + phiSliceIdx);
          }
        }
      }
    }

    let geometry = new THREE.BufferGeometry();
    geometry.setIndex(indices);
    geometry.addAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.addAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
    return geometry;
  }

  return {
    "horn": hornGeometry(),
    "cap": capGeometry(),
    "sock": sockGeometry(),
  };
}


/**
  Returns a ThreeJS Mesh object.
  Actually, a 3D marker is a group of groups of Meshes.
  For performance it is actually based on BufferGeometry object
*/
function createMarker() {
  let object = new THREE.Group();
  object.rotation.order = 'YXZ';

  const geometries = widgetGeometries();

  let segments = {};
  for (let coordinateIdx=0; coordinateIdx < 3; coordinateIdx++) {
    segments[coordinateIdx] = [];
    for (let directionIdx=0; directionIdx < 2; directionIdx++) {
      segments[coordinateIdx][directionIdx] = [];
      let zAxis = [0,0,0];
      zAxis[coordinateIdx] = 1 - 2*directionIdx;
      let xAxis = [0,0,0];
      xAxis[(coordinateIdx + 1) % 3] = 1 - 2*directionIdx;
      let yAxis = [0,0,0];
      yAxis[(coordinateIdx + 2) % 3] = 1;
      for (let i=0; i<4; i++) {
        let segment = new THREE.Group();

        segments[coordinateIdx][directionIdx][i] = [];
        let highlighted = false;
        const color = [0,0,0];
        let colorScale;
        if ((i % 2) === 0) {
          colorScale = 0.8;
        } else {
          colorScale = 0.4;
        }
        for (let channelIdx = 0; channelIdx < 3; channelIdx++) {
          if (directionIdx == 0) {
            if (channelIdx == coordinateIdx) {
              color[channelIdx] = colorScale;
            } else {
              color[channelIdx] = 0;
            }
          } else {
            if (channelIdx == coordinateIdx) {
              color[channelIdx] = 0;
            } else {
              color[channelIdx] = colorScale;
            }
          }
        }

        let material =
            new THREE.MeshStandardMaterial(
              {
                color: new THREE.Color(color[0], color[1], color[2]),
                roughness: 0.5,
                metalness: 0.05,
                emissive: new THREE.Color(0,0,0)
              }
            );
        let horn = new THREE.Mesh(geometries.horn, material);
        horn.receiveShadow = true;
        horn.castShadow = true;
        const s = 1/12;
        let m;
        m =
          new THREE.Matrix4().makeBasis(
            new THREE.Vector3(xAxis[0], xAxis[1], xAxis[2]),
            new THREE.Vector3(yAxis[0], yAxis[1], yAxis[2]),
            new THREE.Vector3(zAxis[0], zAxis[1], zAxis[2])
          )
          .multiply(new THREE.Matrix4().makeRotationZ(i*Math.PI/2))
          .multiply(new THREE.Matrix4().makeScale(s, s, s))
        ;
        let cap = new THREE.Mesh(geometries.cap, material);
        let sock = new THREE.Mesh(geometries.sock, material);
        segment.add(cap);
        segment.add(horn);
        segment.add(sock);
        segment.applyMatrix(m);
        cap.receiveShadow = true;
        cap.castShadow = true;
        sock.receiveShadow = true;
        sock.castShadow = true;
        object.add(segment);

        segments[coordinateIdx][directionIdx][i] = segment;
      }
    }
  }

  return object;
}