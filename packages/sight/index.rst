..
   Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _sight-overview:

Sight
=================================

Robotics application can become quite complicated and it is necessary to inspect the inner workings of algorithms and components.
Two of the most important visualization tools of Isaac SDK are "sight" and "websight".
"sight" is an API which can be used to create variable plots, or to visualize data in 2D or 3D renderings.
"websight" is a web-based frontend which can be used to look at data which is provided via the sight API.
Applications communicate with websight by running an instance of WebsightServer.
The WebsightServer instance allows applications to display plots, 2D and 3D drawing, current application status (active nodes and connections), and update configuration.

To get started run one of the sample apps which have visualization setup, for example `apps/tutorials/opencv_edge_detection` or `apps/samples/stereo_dummy`.
Then simply open a web browser and navigate to :samp:`http://localhost:3000`.
You will see a couple of windows showing you visualization data about the application you are running.

.. toctree::
   :hidden:

   doc/frontend
   doc/backend
   doc/interactiveMarkers
