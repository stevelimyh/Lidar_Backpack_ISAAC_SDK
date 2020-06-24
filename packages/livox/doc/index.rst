..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

Livox LIDAR
===========

Isaac SDK supports use of Livox LIDAR, including a compatible driver and a sample application.

Supported Hardware and Firmware
-------------------------------

The Livox LIDAR driver included in Isaac SDK supports the Livox Mid-40 and was qualified with
firmware version 03.04.0000.

Setting up and Running the Sample Application on the Desktop
------------------------------------------------------------

Isaac SDK includes a sample application demonstrating the usage of the Livox LIDAR driver and
related components.

To set up the sample, perform the following steps:

1. Turn on the Livox LIDAR and connect it to the host PC.

2. Download and install the Livox Viewer tool from the following website:
   `<https://www.livoxtech.com/>`_.

3. Follow the Livox quick start instructions to configure the LIDAR to use a static IP address of
   your choosing, such as 192.168.1.13.

4. Follow Livox quick start instructions to upgrade the LIDAR firmware to version 03.04.0000.

5. In the sample application configuration file
   :code:`packages/livox/apps/livox_lidar_visualization/livox_lidar_visualization.app.json`,
   update the IP address of the LIDAR. In this example:

   .. code-block:: javascript

      "config": {
        "livox_lidar_mid-40": {
          "driver": {
            "device_ip": "192.168.1.13",

6. Start the sample application with the following command:

   .. code-block:: bash

      bob@desktop:~/isaac$ bazel run //packages/livox/apps/livox_lidar_visualization

   The application begins running without errors and stabilizes.

Setting up and Running the Sample Application on a Robot
--------------------------------------------------------

To deploy and run the sample application on a robot, perform the following steps:

1. Deploy //packages/livox/apps/livox_lidar_visualization:livox_lidar_visualization-pkg to the robot as explained in :ref:`deployment_device`.

2. Run the sample application on the robot with the following command:

   .. code-block:: bash

      bob@jetson:~/$ ./packages/livox/apps/livox_lidar_visualization/livox_lidar_visualization


Viewing the Running Application in Sight
----------------------------------------

1. With the livox_LIDAR application running on either the desktop or a robot, start the Sight
   application in a web browser by loading :samp:`localhost:3000`. The metrics, such as point count
   over time, and other information displayed will be similar to the following:

   .. image:: images/livox_sight.png
      :alt: Isaac SDK Livox LIDAR Sample Application
      :align: center

2. Enable the Point Cloud Viewer to display incoming data:

   .. image:: images/livox_lidar.png
      :alt: Isaac SDK Livox LIDAR Point Cloud Visualization
      :align: center

To Go Further
-------------

The Livox LIDAR in Isaac SDK is yet another tool available for your robotic applications.
Your application may integrate the LIDAR for navigation, perception, depth, or other uses.

In the following example, reflectivity information was piped in the color channels over
a 10 second exposure period for a total of one million point in the accumulated cloud.

   .. image:: images/livox_reflectivity.png
      :alt: Isaac SDK Livox LIDAR Reflectivity Example
      :align: center

For more detailed information about Livox LIDAR, see the following website:
`<https://www.livoxtech.com/>`_
