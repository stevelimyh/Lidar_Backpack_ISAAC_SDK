..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

Websight (Backend)
-----------------------------------------

Rendering Inside a Codelet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Render inside a codelet with the sight::show function:

.. code-block:: c++

  sight::show("plot_1", timestamp, val1);
  sight::show("plot_2", val2);
  sight::show("some_image", time, image);
  sight::show("some_image", image);
  sight::show("some_drawing", time, [&](sight::Sop& sop) {
    ...
  });
  sight::show("some_drawing", [&](sight::Sop& sop) {
    ...
  });

The first parameter is always the name of the channel. The full channel name in Sight is
appname/node_name/codelet_name/channel. The timestamp and time parameters are optional. If
timestamp and value are omitted, the current time is used. Timestamp should be provided in
nanosecond while time is in second.

The sight::show function can render an image directly (Image1ub, Image3ub, and Image4ub). Use the
sight::Sop object to render more complex operations, as described in the following sections.

sight::SopStyle
................................

Each rendered element can have a specific style composed of the color, the size, and whether or not
the object should be filled or wireframe.
Use one of the sight::SopStyle constructors to create a new style:

.. code-block:: c++

  sight::SopStyle{color};
  sight::SopStyle{color, filled};
  sight::SopStyle{color, filled, size};

* color must be either a Pixel3ub or a string (or char*) representing a valid javascript color:

  - Color name directly: "red", "white", "blue", etc.
  - Hexadecimal code: "#ff0000", "#fff", etc.
  - Javascript function: "rgb(255,0,0)", "rgba(255,255,255,1.0)", etc. Note: alpha channel is in
    the range [0.0, 1.0], while colors are in the range [0, 255].
* filled is a boolean (false by default) determining whether the object is filled or wireframe.
* size is a scalar, the default being 1.0

sight::SopTransform
................................

Transformations can be passed in two ways: frame name or direct pose.

The preferred way is by frame name:

.. code-block:: c++

  sight::SopTransform{"robot"};

If you pass the name of a frame the SOP data will be posed inside the coordinate frame with the given name.
For example if you specify "robot" the things you show via sight will be placed in a coordinate frame with name "robot".
The concrete pose will be retrieved fully automatically from the application pose tree.
Most of the time your application is already using many coordinate frames to store the pose of various actors.
You can directly use these coordinate frames to pose your visualization data.
This method provides the most flexibility and you should use it whenever you can.

Alternatively you can also give the pose directly in form of a Pose2 or Pose3 object.
In this case the pose will be relative to general world coordinate frame.

.. code-block:: c++

  sight::SopTransform{pose};

Note that this coordinate frame does not have a name and can not be reached by the pose tree.
The pose parameter must be either a Pose2 or Pose3.

There are two more parameters which you can pass in addition: scale and pinhole.

.. code-block:: c++

  sight::SopTransform{pose, size};
  sight::SopTransform{pose, pinhole};
  sight::SopTransform{pose, size, pinhole};

The size parameter is a scalar used to scale the transform. For an image it corresponds to the pixel size.
The pinhole parameter enables sight to use an image as background image for an augmented camera image.
For example to render lines and rectangles on top of a recorded image.

sight::SopImage
................................

The sight::SopImage function encodes an image to display in sight. PNG and JPEG formats are
supported.

.. code-block:: c++

  sight::SopImage::Jpg(image);  // Fast but loss in quality)
  sight::SopImage::Png(image);  // No quality loss, but rather slow.

The image must be of type Image3ub or Image1ub. Image type Image4ub is supported only with the PNG
format.

sight::Sop (Show Operation)
................................

A sight operation is composed of a list of operations to be executed. It can be seen as a tree of
operations, each node containing a SopTransform applied to each children and a SopStyle which
containing the default style of all the children (if no style is specified). They also contain a
list of either primitives to be rendered with the default style or other Sop, hence the tree
structure.

The list of supported primitives (for all of them N = 2/3, K = double/float/int) are:

* geometry::LineSegement<K, N>(Vector<K, N> a, Vector<K, N> b); (Line from a to b)
* geometry::NSphere<K, N>{Vector<K, N> center, K radius}; (circle/sphere)
* geometry::NCuboid<K, N>{Vector<K, N> corner1, Vector<K, N> corner2}; (a rectangle or box)
* geometry::Polygon<K, N>{std::vector<Vector<K, N>>{polygon}};  (a polygon)
* Vector<K, N>() (a single point)
* std::vector<Vector<K, N>> (A list of point or a polyline if the style is set to filled)
* Image<K, N> image (automatically converted to SopImage at JPEG format)
* SopImage (a sop image already serialized)
* SopText{"Text", Vector<K, N> pos} (Text at a given position)
* SopAsset{asset_name} (Same object specified in the configuration)

To change the transform or style, override the 'transform' or 'style' object:

.. code-block:: c++

  show("channel", [&](sight::Sop& sop) {
    sop.transform = sight::SopTransform{world_T_robot};  // Set the transform where the robot is
    sop.style = sight::SopStyle{"red"};  // Set the color to red
    sop.add(geometry::CircleD({0.0, 0.0), 1.0);  // Draw a red circle at the position of the robot
    sop.add([&](sight::Sop& sop) {  // Recursive call
      sop.style = sight::SopStyle("#0000ff");
      for (const auto& pt : path) {
        sop.add(CircleD(pt, 0.2));  // Draw a small circle on the path of the robot
      }
    });
  })

Plot
................................

Render plots with one of the following show functions:

.. code-block:: c++

  show("channel", value);
  show("channel", timestamp, value);

If no timestamp is specified, the current time is used.

To group variables in the same plot, use the same prefix:
win1.var1 and win1.var2 are displayed in the same window while win2.var3 is in its own
window.

Websight Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Configuration
................................

The server can be configured from a configuration file similar to the following:

.. code-block:: c++

  {
    "websight": {
      "WebsightServer": {
        "webroot": "packages/sight/webroot",
        "assetroot": "external/isaac_assets",
        "port": 3000,
        "bandwidth": 10000000,
        "ui_config": {
          "windows": {
            "Renderer 2D": {
              "renderer": "2d",
              "dims": { "width": 256, "height": 256 },
              "channels": [
                { "name": "appname/node/codelet/channel1", "active": true },
                { "name": "appname/node/codelet/channel2", "active": true },
                { "name": "appname/node/codelet/channel3", "active": true }
              ]
            },
            "Renderer 3D": {
              "renderer": "3d",
              "dims": { "width": 256, "height": 256 },
              "channels": [
                { "name": "appname/node/codelet/channel1", "active": true },
              ]
            }
          },
          "assets": {
            "Asse name": {
              "obj": "apps/assets/carter.obj",
              "txt": "apps/assets/carter_albido.png",
              "norm": "apps/assets/carter_normal.png"
            }
          }
        }
      }
    }
  }

* webroot: Path to the folder containing the frontend code
* assetroot: Path to the assets folder
* port: The port the webserver is listening to
* bandwidth: The maximum bandwidth each channel can consume. If this value is too high and the
  network is saturated, the messages will accumulate on the server side until finally some of them
  are being dropped; this will also create some visual lag on the frontend between what is displayed
  and what the robot is actually doing. We recommend setting this value to throttle the network
  before it gets saturated.
* ui_config.windows: A list of renderer widget to be automatically displayed

  - renderer: "2d" or "3d"
  - dims: The size of the renderer
  - channels: The renderer channel list

    - name: Name of the channel
    - active: Whether or not the channel is active by default (default value is true)
* ui_config.assets: The list of assets:

  - obj: The obj file containing the mesh
  - txt: The texture file
  - norm: The file containing the normal information of the 3d mesh


Execution optimization
................................

The server is optimized to compute only what is going to be sent to the front end. When a sight::Sop
object is provided using a lambda function call, the function is executed if and only if at least
one client is currently listening to the channel. Therefore, do not hesitate to abuse the use of the
lambda function whenever you execute a complicated display operation such as normalizing, cropping,
or resizing, an image.
