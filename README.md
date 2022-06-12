# Speech-controlled Jetbot
A simple implementation of speech control for Jetbot

## Project Description
The project aims to control Jetbot wirelessly to do simple maneuver (i.e. going forward, going backward, rotating left, rotating right) with voice command (e.g. "go" = forward, "back" = backward, "left" = left, "right" = right), can be easily modified for any kind of robot platform. 

The project uses: 
* [ROS Framework](http://wiki.ros.org/) to alleviate concurrency management.
* [Jetbot](https://jetbot.org/master/), which is a differential wheeled robot and its rospackage [jetbot_ros](https://github.com/dusty-nv/jetbot_ros) for motor control.

