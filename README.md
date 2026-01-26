![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Build Status](https://img.shields.io/badge/Build-Passing-success.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-purple.svg)
![Version](https://img.shields.io/badge/Version-2.0.0-blue.svg)

# ROS2 ORB SLAM3 V2.0 package 

A ROS2 package for ORB SLAM3 V1.0. Focus is on native integration with ROS2 ecosystem. This is version `3.0.0` that is built and tested to be compatible with ROS 2 Jazzy.

If you find this work useful please consider citing the original ORB-SLAM3 paper and my recent paper that uses this package in solving short-term relocalization (kidnapped robot problem) as shown below

```bibtex
@INPROCEEDINGS{kamal2024solving,
  author={Kamal, Azmyin Md. and Dadson, Nenyi Kweku Nkensen and Gegg, Donovan and Barbalata, Corina},
  booktitle={2024 IEEE International Conference on Advanced Intelligent Mechatronics (AIM)}, 
  title={Solving Short-Term Relocalization Problems In Monocular Keyframe Visual SLAM Using Spatial And Semantic Data}, 
  year={2024},
  volume={},
  number={},
  pages={615-622},
  keywords={Visualization;Simultaneous localization and mapping;Accuracy;Three-dimensional displays;Semantics;Robot vision systems;Pipelines},
  doi={10.1109/AIM55361.2024.10637187}}
```

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
           and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
 }
```

# ORB SLAM3 in Gazebo

To enable ORB SLAM 3 in Gazebo start the simulation environment with the following code

ros2 launch bb_bringup bb.yaml use_sim:=true

This can take a while as it loads the BlueROV2, inspection scenario and sensors. Then since we are still far away from the inspection platform, you can start the inspection python script located in the launch folder of slam-wrapper-v2. 

python3 eco_inspection.py

This should automatically drive to the top left eco-structure and after a couple of seconds start the inspection run. At this point in time we can start the ORB-SLAM3 pipeline

MONO-INERTIAL: ros2 launch ros2_orb_slam3 mono.launch.py
MONO: ros2 launch ros2_orb_slam3 mono.launch.py use_inertial:=false
STEREO-INERTIAL: ros2 launch ros2_orb_slam3 stereo.launch.py
STEREO: ros2 launch ros2_orb_slam3 stereo.launch.py use_inertial:=false

The output topics will always have the following structure

/ORB_SLAM3/node_name/type_of_output

Example for Stereo: /ORB_SLAM3/stereo_sim_node/odometry

To see all the output topics use 

ros2 topic list

# ORB SLAM3 in RL 

To enable ORB SLAM3 outside of Gazebo and using the realsense cameras. Simply add the parameter use_sim:=false next to the normal ORB SLAM3 launch command

# Dual EKF ORB SLAM3

To add the DVL measurements to the ORB SLAM3 output open the late_fusion_dvl branch of the state-estimation-ekf repository. Then run the following command

ros2 launch state_estimation_ekf ekf_dual.launch.py

The new output topic should be 

/est/odometry/filtered_global

# Future Work

Right now ORB SLAM3 alligns to Gazebo world at initialization and uses the Gazebo world frame convention. It is still left to figure out how the frames shall be set for scenarios outside of simulation and how to deal with loss of tracking. 
