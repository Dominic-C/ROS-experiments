# Teleop on a differential drive robot
In this example, I document my process on running teleop on a simple differential drive vehicle. In the big picture, you control the robot via teleop by inputting keystrokes on the master device. The master device then publishes a topic containing information for motor motor speeds.

The slave device is then run to recieve information on these topics from the master device. The slave computer runs a rosserial node for the arduino to subscribe to. The arduino then drives the motors on the vehicle, reaching the end of the chain of command.

## Requirements
* 2 Computers with ROS installed(1 on the vehicle, one with a keyboard to run teleop)
* Robot chassis
* Arduino
* Power bank
* Batteries for the robot chassis

## Preparation
Before starting this tutorial, make sure you uploaded a program to your arduino to receive ROS messages. Simple tutorial can be found [here](https://github.com/Dominic-C/ROS-experiments/tree/master/ROS%20arduino). In our example, we will use `turtlebot_teleop keyboard_teleop.launch` to send messages to our arduino.

Make sure you configure your computers to communicate with each other. A simple tutorial can be found [here](https://github.com/Dominic-C/ROS-experiments/tree/master/ROS%20across%20two%20machines).

## Configuring slave device
To allow the slave computer to interface with the Arduino, it has to run rosserial. rosserial is a package contributed by the ROS community. The github link can be found [here](https://github.com/ros-drivers/rosserial).

In the slave computer, navigate to your catkin workspace source folder, i.e. `cd ~/catkin_ws/src` and do a git clone of the rosserial repository. Exit to catkin workspace root folder, i.e. `cd ~/catkin_ws` and run `catkin_make`. This should build the packages required to run rosserial.

With the slave device configured, we can set up the master device.

## Configuring master device
On the master device, we open the terminal and run `roscore`. In a separate terminal, we run `roslaunch turtlebot_teleop keyboard_teleop.launch`. On the slave device, we subscribe to these topics by running `rosrun rosserial_python setup.py /dev/ttyUSB0` (replace ttyUSB0 with whatever name your arduino is recognized as).

With everything set up, we should be able to move the motors on our robot by pressing the keys printed out in the turtlebot_teleop terminal. (make sure you click in the turtlebot_teleop terminal when trying to run teleop)
