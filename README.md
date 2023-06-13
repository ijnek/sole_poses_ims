# Sole Poses Interactive Marker Server

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_humble.yaml?query=branch:rolling)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_iron.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Provides an Interactive Marker Server for the sole poses of a robot.
See this [Video](https://www.youtube.com/watch?v=hvI5Hjhp1EM) for a demo of this package with the NAO robot.

## Topics

### Publishers

* ``motion/sole_poses`` (sole_poses_msgs/msg/SolePoses): The target sole poses of the robot

## Parameters

* ``l_sole_default_x`` (float): Default x position of the left sole interactive marker
* ``l_sole_default_y`` (float): Default y position of the left sole interactive marker
* ``l_sole_default_z`` (float): Default z position of the left sole interactive marker
* ``r_sole_default_x`` (float): Default x position of the right sole interactive marker
* ``r_sole_default_y`` (float): Default y position of the right sole interactive marker
* ``r_sole_default_z`` (float): Default z position of the right sole interactive marker

## Example:

For the NAO robot, run:

```sh
ros2 run sole_poses_ims sole_poses_ims --ros-args -p l_sole_default_y:=0.05 -p l_sole_default_z:=-0.33 -p r_sole_default_y:=-0.05 -p r_sole_default_z:=-0.33
```
