# open-abb-driver
### Control ABB robots remotely with ROS, Python, or C++



## What is it?
open-abb-driver consists of two main parts. The first is a program which is written in the ABB robot control language, RAPID, which allows remote clients to send requests for actions (such as joint moves, cartesian moves, speed changes, etc.). The second is a series of libraries to interact with the robot from remote computers, using several different control schemes. You can use the ROS driver, which allows control using ROS services and publishers. You can also include the Python or C++ libraries to communicate with the robot directly (both located in abb_node/packages/abb_comm), and bypass ROS completely. 

## Requirements
* ABB IRC5 controller
* 6 DOF robotic manipulator
* Robot must have the following factory software options
    * "PC Interface"
    * "Multitasking" (required for position feedback stream)

## Quick Start
### Robot Setup
* Install the RAPID module 'SERVER'
    * Using RobotStudio online mode is the easiest way to do this, check out the [wiki article](https://github.com/robotics/open-abb-driver/wiki/Configuring-an-ABB-Robot-for-OAD) for details.
* For position feedback, install the RAPID module 'LOGGER' into another task. 
* In SERVER.mod, check to make sure the "ipController" specified is the same as your robot. The default robot IP is 192.168.125.1
* Start the programs on the robot
    * Production Window->PP to Main, then press the play button. 

### Computer Setup
* Verify that your computer is on the same subnet as the robot.
    * Try pinging the robot (default IP is 192.168.125.1). 
* Before trying ROS, it's pretty easy to check functionality using the [simple python interface.](https://github.com/robotics/open-abb-driver/wiki/Python-Control)
    * Note that you must either copy abb_node/packages/abb_comm/abb.py to your local directory or somewhere included in your PYTHONPATH environment. 
* To set up the ROS node (Fuerte only at the moment), copy abb_node to somewhere in your $ROS_PACKAGE_PATH.
    * If you did that correctly, try:

        ```
        roscd abb_node
        rosmake abb_node
        roslaunch abb_node abb_tf.launch
        ```
