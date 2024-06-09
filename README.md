# CheckPoint 3 ROS2 Basics CPP

<a name="readme-top"></a>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#software-prerequisites">Software Prerequisites</a></li>
        <li><a href="#hardware-prerequisites">Hardware Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#real-robot">Real Robot</a></li>
  </ol>
</details>


## About The Project
This project demonstrates the application of ROS2 (Robot Operating System 2) concepts to control a real Turtlebot3 robot. Using the LIDAR and services the robot patrol the scenario without crashing into the obstacles.

![This is an image](images/preview.gif)

<!-- GETTING STARTED -->
## Getting Started

### Software Prerequisites
* Ubuntu 22.04
* ROS1 Noetic
* ROS2 Foxy

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- INSTALLATION -->
### Installation
1. Clone the repo:
   ```sh
   cd ~ && \
   git clone https://github.com/pvela2017/The-Construct-CheckPoint-3-ROS2-Basics-CPP
   ```
2. Compile the simulation:
   ```sh
   source /opt/ros/noetic/setup.bash && \
   cd ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/simulation_ws && \
   catkin_make && \
   cd ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/catkin_ws && \
   catkin_make && \
   source /opt/ros/foxy/setup.bash && \
   cd ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/ros2_ws && \
   colcon build
   ```
     
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE -->
## Usage
### Local Simulation & Real Robot
1. Launch the simulation:
   ```sh
   source /opt/ros/noetic/setup.bash && \
   source ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/simulation_ws/devel/setup.bash && \
   roslaunch realrobotlab main.launch
   ```
2. Launch the ros bridge:
   ```sh
   source /opt/ros/noetic/setup.bash && \
   source ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/catkin_ws/devel/setup.bash && \
   roslaunch load_params load_params.launch && \
   source /opt/ros/foxy/setup.bash && \
   ros2 run ros1_bridge parameter_bridge && \
   ```
3. Launch patrol node:
   ```sh
   source /opt/ros/foxy/setup.bash && \
   source ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/ros2_ws/install/setup.bash && \
   ros2 launch robot_patrol start_patrolling.launch.py
   ```
4. Launch and test the service the service:
   ```sh
   source /opt/ros/foxy/setup.bash && \
   source ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/ros2_ws/install/setup.bash && \
   ros2 launch robot_patrol start_direction_service.launch.py && \
   ros2 launch robot_patrol start_test_service.launch.py
   ```
5. Move the robot using the service:
   ```sh
   source /opt/ros/foxy/setup.bash && \
   source ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/ros2_ws/install/setup.bash && \
   ros2 launch robot_patrol main.launch.py
   
   ```
6. Move the robot using an action server:
   ```sh
   source /opt/ros/foxy/setup.bash && \
   source ~/The-Construct-CheckPoint-3-ROS2-Basics-CPP/ros2_ws/install/setup.bash && \
   ros2 launch robot_patrol start_gotopoint_action.launch.py
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- RESULTS -->
## Results
[![Demo](https://img.youtube.com/vi/s4jkbh0bWyQ/0.jpg)](https://www.youtube.com/watch?v=s4jkbh0bWyQ)


<!-- KEYS -->
## Key topics learnt
* Use services
* Use actions.
* Check if the custom action has been created.
```sh
ros2 interface show robot_patrol/action/GoToPoint
```
