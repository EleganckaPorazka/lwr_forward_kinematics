# KUKA LWR 4+ manipulator forward kinematics

<img src="https://img.shields.io/badge/ros--version-humble-green"/>  <img src="https://img.shields.io/badge/platform%20-Ubuntu%2022.04-orange"/>

## Description

The `lwr_forward_kinematics` package contains a forward kinematcs solver for the 7-DOF **KUKA LWR 4+** manipulator. It computes the end effector pose and the Jacobian of the manipulator.

## Installation

It is recommended to use Ubuntu 22.04 with [**ROS 2 Humble**](https://docs.ros.org/en/humble/index.html).

### Required packages

[**lwr_description**](https://github.com/EleganckaPorazka/kuka_lwr4plus_urdf.git)

### Building from source

```
mkdir -p ~/ros2_ws/src/lwr_forward_kinematics
cd ~/ros2_ws/src/lwr_forward_kinematics
git clone https://github.com/EleganckaPorazka/lwr_forward_kinematics.git .
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
```

## Running

To run the 'lwr_forward_kinematics' node, use the following command:
```
ros2 run lwr_forward_kinematics lwr_forward_kinematics 
```

To change the pose of the tool in the last link's frame, use (in other terminal):
```
ros2 param set /lwr_forward_kinematics tool '[x,y,z,qx,qy,qz,qw]'
```
where '[x, y, z]' is the position of the tool (in meters), and '[qx, qy, qz, qw]' is its orientation (in quaternions).

## Notes

This code is also uploaded to [**my other repository**](https://gitlab.com/lwolinski/lwr_forward_kinematics.git).
