# Ros2_roboclaw
This package provides ROS2 wrapper for [Amber drivers](https://github.com/project-capo/amber-python-drivers). It provides communication methods with [Nav2](https://navigation.ros.org/) navigation stack using nav_msgs and geometry_msgs. It also provides communication interface for more manual steering, which could be used during laboratory classes.

For correct work of this package install:
```
sudo pip3 install transforms3d
```

## roboclaw_comm
This package contains message interface - `Speed.msg` - to send desired velocity on every of the CAPO robot wheel.

## ros2_roboclaw
The `src` directory contains all necessary Amber drivers.

In `ros2_roboclaw` directory the `roboclaw_node.py` implements Node responsible for setting wheels velocities.
The Node initialize connection to Roboclaws (ore one Roboclaw in 2-wheel version) and starts 2 subscribers and 1 publisher. First subscriber is responsible for handling roboclaw_comm\Speed messages. Second one handles messages coming from Nav2 stack. The publisher every 0.5 second publish odometry information calculated from encoders readings. Each entity logs messages that were received/sent.

## Launch
To start Roboclaw Node use following command:
```
ros2 run ros2_roboclaw steer
```

To send Speed messages with terminal use:
```
ros2 topic pub --once /manual roboclaw_comm/msg/Speed "{speed: [0,0,0,0]}"
```
where values in [] are the velocities to set on the front left, front right, rear left and rear right wheels respectively.
