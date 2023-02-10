# uav_project
A demo project for various uav tasks such as scanning and tracking

## Dependencies
The project is a simulation project based on [Gazebo](https://gazebosim.org/home) and [ROS](https://www.ros.org/) (Melodic or higher version).
It uses the [hector_quadrotor](http://wiki.ros.org/hector_quadrotor), a ROS package, for simulating the UAV, with a depth camera for capturing data.

- Gazebo-ROS packages (on Melodic for example)
  - ros-melodic-gazebo-ros-pkgs
  - ros-melodic-gazebo-ros-control
  - ros-melodic-effort-controllers
  - ros-melodic-joint-state-controller
  - ros-melodic-controller-manager
  - ros-melodic-geographic-msgs   
  - ros-melodic-sensor-msgs

- ROS packages
  - [hector_quadrotor](http://wiki.ros.org/hector_quadrotor)
  - [hector_gazebo](http://wiki.ros.org/hector_gazebo)
  - [hector_localization](http://wiki.ros.org/hector_localization)
  - [hector_sensor_description](http://wiki.ros.org/hector_sensors_description)

- Vision packages
  - [opencv](https://opencv.org/) and [opencv-python](https://pypi.org/project/opencv-python/)
  - [realsense sdk](https://github.com/IntelRealSense/librealsense) and [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

- Point Cloud Library (Optional, only for 3D point cloud process)
  - [point cloud library](https://pointclouds.org/) and [python-pcl](https://github.com/strawlab/python-pcl)

## Packages
- **uav_task_gazebo**, gazebo simulation of environment and quadrotor, including models and launch files.
- **uav_task_description**, urdf description of quadrotor and camera.
- **uav_task_control**, control settings for the quadrotor and camera.

## Tasks
### UAV Tracking
- Start environment with a mobile ugv and launch double quadrotors
  ```
  roslaunch uav_task_gazebo double_uav_tracking.launch
  ```
- GOAL: use two quadrotors to track the random moving UGV.
  - mission 1: create a package for UAV tracking.
  - mission 2: control the quadrotors to takeoff, flying, and landing using the related ROS topics.
  - mission 3: write a script to autonomously control the quadrotors to track the moving UGV while maintaining distance between the quadrotors to the UGV (avoiding collision), and display the captured image of each quadrotor.   

### UAV Scanning
- Start environment with a target airplane and launch a single quadrotor
  ```
  roslaunch uav_task_gazebo double_uav_scanning.launch
  ```
- GOAL: use multiple quadrotors to inspect the large airplane.
  - mission 1: create a package for UAV scanning.
  - mission 2: control the quadrotors to takeoff, flying, and landing using the related ROS topics.
  - mission 3: write a script to autonomously flying multiple quadrotors to do cooperative scanning while avoiding collision, and display the captured image of pointcloud data of each quadrotor.
