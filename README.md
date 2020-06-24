# Lidar_Backpack_ISAAC_SDK

System  Requirements:

 -Ubuntu 18.04 LTS

 -Nvidia GPU with compute capability of 3.5 or higher
 
 -



Software Version:

-Isaac SDK: 2020.1

-Isaac Sim: 2020.1 (Unity 3D)

-Unity 3D: 2019.3.15f1

-Bazel Version: 2.2.0

-CUDA: 10.0

-libcudnn: 7

-Nvidia GPU driver: 450

-
  
  
  

Filepath: ISAAC SDK Root directory

ex: 

steve@steve-Inspiron-7559:~/isaac/apps$ 

steve@steve-Inspiron-7559:~/isaac/packages$ 

-


Modifications:

1) 

modified file: isaac/packages/message_generators: BUILD

-rigid_body_3_group_generator added to module

-


2) 

new directory: isaac/apps/record_dummy

-

-record_dummy, record_test 
  
apply message generators to create dummy data, use Isaac Sight Recorder widget to record

-
  
-display_dummy. display_test
  
replay dummy data, visualize in Isaac Sight

-
  
-Imu_test
  
Test imu generation with rigid_body_generator and ImuSim

-
  
-Svo_test
  
test stereo visual odometry with message generators

-
  
  
3) modified directory: isaac/packages/flatsim/apps

-added gmapping component to flatsim app


4) modified directory: isaac/apps/navsim

a) run small warehouse scene with isaac sim

b) run navsim_record app to publish data while navigating (data can be recorded via Recorder widget on Isaac Sight)

c) -run navsim_sub_gmap to run gmapping by subscribing to published data, or
    
   -run navsim_replay_gamp to run gmapping by replaying recorded data (Isaac Sight Replay widget)
   

Note: 
     To run either (navsim_sub_gmap or navsim_replay_gmap or navsim_gmapping) 
     select_json_gmapping.sh need to be modified at line 29 by changing the app name accordingly
     
     line 29: engine/alice/tools/main --app apps/navsim/navsim_sub_gmap.app.json \
     
     
  
