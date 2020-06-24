..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _ros_bridge:

ROS Bridge
==========
Both Isaac and `Robot Operating System (ROS) <https://www.ros.org/>`_
make use of message passing to handle communication
between different parts of their respective systems.
Communicating between Isaac and ROS requires creating a message translation
layer between the two systems. This section presents an overview and the
workflow of this layer.

Install ROS
--------------------------------------

In order to use the ROS bridge you have to install ROS on your device. For example if you want to
use the ROS bridge on Jetson you have to install ROS on Jetson. If you want to use ROS on the
desktop you have to install ROS on the desktop.

The ROS version you install has to match the operating system of your device. For your desktop
system or Jetson Xavier (both running Ubuntu 18.04) you must install ROS Melodic Morenia.

To install ROS follow the instructions from the `ROS webpage`_.

.. _ROS webpage: http://wiki.ros.org/ROS/Installation

Creating and using a custom ROS package
----------------------------------------

The majority of ROS functionality is not required by Isaac, thus the included packages only offer
support for the messages commonly installed with a default ROS install, which are listed below.

.. code-block:: bash

    roscpp rospy actionlib_msgs control_msgs diagnostic_msgs geometry_msgs
    map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs stereo_msgs
    tf2_geometry_msgs tf2_msgs trajectory_msgs visualization_msgs

The third-party libraries required for the ROS Bridge are only supported on desktop and Jetson
Xavier.

If the message types above are sufficient for your application, please proceed to the next
subsection.  Otherwise, in order to make use of custom ROS messages, it is necessary to generate a
custom package and point the bazel build system to such a package. First generate a custom workspace
which contains the required packages. Refer to :code:`engine/build/scripts/ros_package_generation.sh`
for guidelines on how to create a custom package with the proper path structure. Once such a package
exists, modify :code:`third_party/ros.bzl` to point to the new workspace.

Begin by commenting out the platform specific default package, which should be similar
to the following:

.. code-block:: python

    isaac_http_archive(
        name = "isaac_ros_bridge_x86_64",
        build_file = clean_dep("//third_party:ros.BUILD"),
        sha256 = "b2a6c2373fe2f02f3896586fec0c11eea83dff432e65787f4fbc0ef82100070a",
        url = "https://developer.nvidia.com/isaac/download/third_party/ros_kinetic_x86_64.tar.gz",
    )

Replace the platform-specific package with the following to point to the new
workspace with the custom packages:

.. code-block:: python

    isaac_new_local_repository(
        name='isaac_ros_bridge_x86_64',
        path='path to the workspace',
        build_file = clean_dep("//third_party:ros.BUILD"),
    )

The name and build_file fields in the new section must match those of the
section being replaced.

Creating a ROS Bridge
---------------------

"ros_bridge" package provides a library of message converters and makes it easy to create new ones.
Thanks to the modular design, users can create various bridges with different connections and
configurations of converters, and easily create new converters if needed. While the rest of this
section will explain and build on this package, if user wants to directly work with
`roscpp <http://wiki.ros.org/roscpp>`_ instead, the codelet of sample application
":ref:`navigation_rosbridge`" illustrates how to do so.

An Isaac-ROS bridge consists of:

1. One and only one RosNode codelet,
2. A TimeSynchronizer codelet in the same node as RosNode codelet,
3. As many message and pose converter codelets as needed,
4. A behavior tree that starts converters once RosNode establishes connection with the roscore.

RosNode
"""""""

RosNode is the Isaac codelet that initializes a ROS node and waits until roscore is up. Every Isaac
application with ROS bridge needs to have one and only one node with a single component of this
type.

TimeSynchronizer
""""""""""""""""

Allows converting time stamps between Isaac notation and ROS notation.

Message Converter Bases
"""""""""""""""""""""""

Isaac provides users with base classes to quickly develop typical converters:

1. **ProtoToRosConverter**: This is a base class for codelets that convert Isaac proto messages to
   ROS messages and publish them to ROS. Please check OdometryToRos converter to see an example on
   how to create a new converter using ProtoToRosConverter: All we need to do is to define a
   protoToRos function.
2. **RosToProtoConverter**: This is a base class for codelets that receives ROS messages and
   convert them to Isaac proto messages. Please check RosToDifferentialBaseCommand converter to see
   an example on how to create a new converter using RosToProtoConverter: All we need to do is to
   define a rosToProto function.

Pose Synchronization
""""""""""""""""""""

The Isaac :ref:`Pose Tree <pose_tree>` and ROS `tf2 <http://wiki.ros.org/tf2>`_ can
be synchronized using the codelets below:

1. **PosesToRos**: For a list of pose mappings, this codelet reads poses from Isaac Pose Tree
   and writes them to ROS tf2.
2. **RosToPoses**: For a list of pose mappings, this codelet reads transformations from ROS
   tf2 and writes them to the Isaac Pose Tree.

Custom Codelets
"""""""""""""""

If a desired codelet doesn't match the codelets or base classes mentioned above, you can easily
create more advanced codelets. For example, GoalToRosAction receives two Isaac messages, runs an
ROS action client, and publishes an Isaac message.

An example: Using ROS Navigation Stack with Isaac
-------------------------------------------------

Please check :code:`packages/ros_bridge/apps/ros_navigation/ros_bridge.subgraph.json` for an example on how
to use RosNode and converter codelets. Note that the behavior tree ensures converters start after
RosNode is done initializing ROS connection.

This example subgraph is used to run `ROS navigation stack <http://wiki.ros.org/navigation>`_ with
Isaac simulators or with real robots through Isaac. Let's see
`TurtleBot 3 Waffle Pi <http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/>`_
navigated by `ROS navigation stack for TurtleBot 3 <http://wiki.ros.org/turtlebot3_navigation>`_
in :ref:`flatsim`.

1. In addition to ROS, install
   `ROS navigation stack for TurtleBot 3 <http://wiki.ros.org/turtlebot3_navigation>`_.

2. Following the instructions at
   `Virtual Navigation with TurtleBot3 <http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-navigation-with-turtlebot3>`_,
   run the following command for the Isaac small-warehouse scene:

  .. code-block:: bash

    bob@desktop:~/isaac$ TURTLEBOT3_MODEL=waffle_pi roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(realpath packages/ros_bridge/maps/small_warehouse.yaml)

  You don't need to start roscore yourself, roslaunch will do it for you.

3. Run Isaac application that has flatsim and ROS bridge:

  .. code-block:: bash

    bob@desktop:~/isaac$ bazel run packages/ros_bridge/apps:ros_to_navigation_flatsim -- --more apps/assets/maps/virtual_small_warehouse.json --config ros_navigation:packages/ros_bridge/maps/small_warehouse_map_transformation.config.json,ros_navigation:packages/ros_bridge/apps/ros_to_navigation_turtlebot3_waffle_pi.config.json

4. Open :samp:`http://localhost:3000/` to monitor through Isaac. Watch RViz window to monitor
   through ROS. Note that RViz may complain about "No transform from [wheel_left_link] to [map]".
   This information is not provided by the bridge since it is not used by ROS Navigation stack.
   However, it (and other data) can be published similar to other transforms if needed.

5. The robot should now be navigating to the goal, which can be easily modified by dragging the
   "pose_as_goal" marker of "Map" window on Sight around.

.. note:: Due to an issue in ROS navigation stack, ROS may print warnings such as the following.

    .. code-block:: bash

      Warning: Invalid argument "/map" passed to canTransform argument target_frame in tf2 frame_ids cannot start with a '/' like:
      at line 134 in /tmp/binarydeb/ros-melodic-tf2-0.6.5/src/buffer_core.cpp

 In this case, please apply the changes shown at
 `https://github.com/ROBOTIS-GIT/turtlebot3/pull/402/files <https://github.com/ROBOTIS-GIT/turtlebot3/pull/402/files>`_, i.e.,
 delete '/' character from frame names in global_costmap_params.yaml and local_costmap_params.yaml.
 These files may exist at /opt/ros/melodic/share/turtlebot3_navigation/param/ if you installed
 turtlebot3 package, or at the location where you cloned the turtlebot3 repository.
 For more details, please check out the discussion at
 `https://github.com/ros-planning/navigation/issues/794 <https://github.com/ros-planning/navigation/issues/794>`_.

.. note:: If turtlebot3_navigation does not install dwa-local-planner for you, ROS may fail to run with the following message:

  .. code-block:: bash

      [FATAL] [1567169699.791910573]: Failed to create the dwa_local_planner/DWAPlannerROS planner, are you sure it is properly registered and that the containing library is built? Exception: According to the loaded plugin descriptions the class dwa_local_planner/DWAPlannerROS with base class type nav_core::BaseLocalPlanner does not exist. Declared types are  base_local_planner/TrajectoryPlannerROS

 In this case, install the missing dependency for your ROS distro as follows:

  .. code-block:: bash

      sudo apt install ros-<your_distro>-dwa-local-planner

 For example, the command for Melodic Morenia would be

  .. code-block:: bash

      sudo apt install ros-melodic-dwa-local-planner

Building on this example bridge
----------------------------------
* To simulate with :ref:`isaac_sim_unity3d` instead of :ref:`flatsim`, launch the small-warehouse
  scene by running the following command:

  .. code-block:: bash

    bob@desktop:~isaac_sim_unity3d/builds$ ./sample.x86_64 --scene small_warehouse --scenarioFile ~/isaac/packages/navsim/scenarios/turtlebot3_waffle_pi.json --scenario 0

  Then, enter the following command on a separate terminal to run the application that communicates
  with both Unity and ROS:

  .. code-block:: bash

    bob@desktop:~/isaac$ bazel run packages/ros_bridge/apps:ros_to_navigation_unity3d -- --more apps/assets/maps/virtual_small_warehouse.json --config ros_navigation:packages/ros_bridge/maps/small_warehouse_map_transformation.config.json,ros_navigation:packages/ros_bridge/apps/ros_to_navigation_turtlebot3_waffle_pi.config.json

  Run ROS with the same command as above:

  .. code-block:: bash

    bob@desktop:~/isaac$ TURTLEBOT3_MODEL=waffle_pi roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(realpath packages/ros_bridge/maps/small_warehouse.yaml)

  As before, the goal can be modified by dragging the "pose_as_goal" marker in Sight.

* To run on a different map, simply change path to the map files in commands above (and in the
  simulator window if you are not using flatsim).

* To simulate for a different robot, change robot configurations in commands above (and in the simulator
  window if you are not using flatsim). ROS navigation stack should also be updated in this case.

Converting an Isaac map to ROS map
----------------------------------
Map file extensions and map frame conventions differ between Isaac and ROS:

* Isaac uses Portable Network Graphics (*.png*) maps, while ROS navigation stack uses
  Portable Graymap Format (*.pgm*).

* (0, 0) coordinate corresponds to upper-left corner and x direction points down in Isaac map,
  whereas the frame of `map_server of ROS <http://wiki.ros.org/map_server>`_ depends on the origin
  parameter of *.yaml* file.

Below are the steps to generate *.yaml* and *.pgm* files for use by ROS. The reverse conversion is
similar.

1. Convert Isaac map image:

   .. code-block:: bash

      bob@desktop:~/isaac$ convert apps/assets/maps/virtual_small_warehouse.png -flatten packages/ros_bridge/maps/small_warehouse.pgm

   You may also use the :code:`rotate` flag in this command if you like. We are going to deal with frame
   transformation in step 3.

2. Create a :code:`packages/ros_bridge/maps/small_warehouse.yaml` file that reads

   .. literalinclude:: ../maps/small_warehouse.yaml
      :language: yaml

   You may modify the origin as you like. However, the :code:`resolution` field should match the
   :code:`cell_size` in :code:`apps/assets/maps/virtual_small_warehouse.json`.

3. Find the correct transformation from the Isaac map frame to the ROS map frame to create
   :code:`packages/ros_bridge/maps/small_warehouse_map_transformation.config.json`:

   .. literalinclude:: ../maps/small_warehouse_map_transformation.config.json
      :language: json

   .. note:: One way of finding this transformation is described in
    `ROS Answers <https://answers.ros.org/question/69019/how-to-point-and-click-on-rviz-map-and-output-the-position/?answer=69295#post-id-69295>`_:

     1. Launch ROS navigation:

        .. code-block:: bash

           bob@desktop:~/isaac$ TURTLEBOT3_MODEL=waffle_pi roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$(realpath packages/ros_bridge/maps/small_warehouse.yaml)

     2. In a separate terminal, type the following:

        .. code-block:: bash

           bob@desktop:~/isaac$ rostopic echo /move_base_simple/goal

     3. Give a 2D Nav Goal that corresponds to the Isaac map frame described above (i.e. upper-left
        corner of the map, pointing down).  The pose printed on the terminal with :code:`rostopic`
        is :code:`ros_map_T_map.`
