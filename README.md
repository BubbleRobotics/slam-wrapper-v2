![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-purple.svg)

# ROS2 ORB SLAM3 

This ROS2 package acts as a communication layer between ROS and ORB SLAM3, thus enabling feeding 
data through ROS2 to ORB SLAM for accurate state estimation. Furtehrmore the package takes advantag 
of ROS transforms, making manual adding of spatial information about camera to IMU unnessasray. 

## Research Papers

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

## 0. Preamble and Libraries

This section is purely for information purpose. All dependencies and libraries are installed through 
the [docker image](https://github.com/BubbleRobotics/docker-dev-container) and can be used through 
the [development environment](https://github.com/BubbleRobotics/docker-dev-environment).

This package builds [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) `V1.0` as a shared internal library. Comes included with a number of Thirdparty libraries [DBoW2, g2o, Sophus]

`g2o` used packaged with `ORB-SLAM3` is a much older version and is incompatible with the latest release found here [g2o github page](https://github.com/RainerKuemmerle/g2o). If you are willing to make the forward port, please open a Ticket in this repository.

This package differs from other ROS1 wrappers, `thien94`s ROS 1 port and ROS 2 wrappers in GitHub by supprting/adopting the following
  * A separate python node to send data to the ORB-SLAM3 cpp node. This is purely a design choice.
  * C++17 and Cmake>=3.8
  * Eigen 3.3.0, OpenCV>= 4.2, 
  * Latest version of Pangolin
  * Comes with a small test image sequence from EuRoC MAV dataset (MH05) to quickly test installation

For newcomers into the ROS 2 ecosystem, this package serves as an example of `building a shared cpp library` and also `a package with both cpp and python nodes`.

* In **resource constrainted hardwares** such as Raspberry Pi 4, Jetson Nano Orin, you need to extend `SWAP` space to at least `16Gb`. 

## Configure

Configuiration of this pipeline is done through yaml files in the ```orb_slam3/config``` directory. 
We focus on  stereo implementations only. A detailed guide for all parameters set there can be found 
in the ```orb_slam3/config/Calibration``` directory. 

Important! Even though we can now read TF's between camera and IMU automatically, orb slam still expects
it to be set in the config file. Simply put the Identety matrix and you are good to go. 

Furthermore I'd advise the use of the pre-configured launch files in ```launch```. Adapt it to your 
needs by setting: 
- the config file ("settings_file")
- the ros topics used ("img0_topic" <- left, "img1_topic" <- right, "imu_topic")
- if the algorithm should also consider inertial information ("is_inertial")
- if the image time should be overwritten by current time ("manual_time_sync") ... for debugging
- if the IMU tf is to be read from the config file ("imu_from_yaml")
- if orb slams debugging window should be opened ("enable_debug_window")
- Orb vocabulary file ("voc_file") ... do not change

## Run 

if you changed any config/launch file rebuild:

```
colcon build 
```

Afterwards simply run:

```
ros2 launch ros2_orb_slam3 <launch_file>
```