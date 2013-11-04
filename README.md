open-abb-driver
==============
Control ABB robots remotely with ROS, Python, or C++
--------------


What is it?
--------------
open-abb-driver consists of two main parts. The first is a program which is written in the ABB robot control language, RAPID, which allows remote clients to send requests for actions (such as joint moves, cartesian moves, speed changes, etc.). The second is a series of libraries to interact with the robot from remote computers, using several different control schemes. You can use the ROS driver, which allows control using ROS services and publishers. You can also include the Python or C++ libraries to communicate with the robot directly, and bypass ROS completely. 

Requirements
--------------
* ABB IRC5 controller
* 6 DOF robotic manipulator
* Robot must have the following factory software options
    * "PC Interface"
    * "Multitasking" (required for position feedback stream)

Quick Start
--------------
* Install the RAPID module 'SERVER' (using RobotStudio/Online Mode is the easiest way. Some robots also support using USB thumbdrives and the teach pendant). 
* Install the RAPID module 'LOGGER'
* In SERVER.mod, check to make sure the "ipController" specified is the same as your robot. The default robot IP is 192.168.125.1
* Start the programs on the robot (if everything is set up properly, Production Window->PP to Main, then press the play button). 