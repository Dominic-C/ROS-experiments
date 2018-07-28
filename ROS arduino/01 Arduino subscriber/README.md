# Arduino subscriber

## Motivation
Inspired to design a robotics system for a project, I wanted to try making microcontrollers, most notably, the Arduino, subscribe to ROS messages.

So I decided to to a simple test. In this test, I subscribe to the topics published by the turtlebot teleop keyboard launch file.

## Prerequisites
* Arduino IDE with ros library installed.
  * In IDE install: `Sketch > Include Library > Manage Libraries > rosserial`
* Arduino
* rosserial packages or binaries
  * installation guide [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

## Subscribing to a message
In order to make an arduino subscriber, we have to find a topic to subscribe to. Here are a few commands we will use quite often
```
 rostopic list
 rostopic info /<topic name>
 rostopic echo /<topic name>
```
`rostopic list` shows us the current topics available on rosmaster. `rostopic info /<topic name>` tells us about the message type and publisher/subscriber information. `rostopic echo /<topic name>` shows us the variables being published in the chosen topic.

### General process
* First, we have to launch rosmaster by running `roscore`
* Then, we run the turtlebot teleop launch file by running `roslaunch turtlebot_teleop keyboard_teleop.launch`
    * Note: there is only a subtle difference between `roslaunch` and `rosrun`. `roslaunch` runs launchfiles, which run a collection of ros nodes. `rosrun` on the other hand, only runs a single node.
* next, we run `rostopic list` to show us all available rostopics. We should observe the following

```
~$ rostopic list
/cmd_vel_mux/input/teleop
/rosout
/rosout_agg
```
* to take a closer look at the topics we have to subscribe to, we run `rostopic info /cmd_vel_mux/input/teleop`. We should observe an output like this:
```
~$ rostopic info /cmd_vel_mux/input/teleop 
Type: geometry_msgs/Twist

Publishers: 
 * /turtlebot_teleop_keyboard (http://192.168.1.194:33235/)

Subscribers: None

```
* Now we see that we should subscribe to the topic `/cmd_vel_mux/input/teleop`, with the message type `geometry_msgs/Twist`.

## Variables in the message

Now that we know the message object, we need to find out the contents of the message, so that we can use its output for our program.

In this case, we'll use that output to make LED 13 on the arduino light up when we press the forward button on the turtlebot teleop publisher.

Making sure that we still have the teleop publisher `keyboard_teleop.launch` opened. We run `rostopic echo /cmd_vel_mux/input/teleop` to observe the variables being published by this topic.

We should be able to observe the following:
```
~$ rostopic echo /cmd_vel_mux/input/teleop
linear: 
  x: 0.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```
This tells us that the message being published has 2 Classes. The first class is `linear`, with methods `x`, `y` and `z`. The second class is `angular`, with methods `x`, `y` and `z`. Thus, we can refer to x in the linear class by `msg.linear.x` in our arduino code, where msg is the message of topic `geometry_msgs/Twist`.

## Launching the Publisher
Since our subscriber subscribes to messages published by the turtlebot teleop publisher, we run the turtlebot teleop node as our publisher. 
```
roslaunch turtlebot_teleop keyboard_teleop.launch
```

## Launching the Subscriber node
After uploading the code provided in this repository to your arduino, we need to run a rosserial node to allow the arduino a path to receive ROS messages.

With `roscore` still running in one terminalm, open a second terminal and run the following command:
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
**NOTE**: your arduino might have a different device id everytime you connect the arduino to your computer. to check, run `ls /dev/tty*` before and after connecting your arduino to the computer. Look for the new device id. That is your arduino's device id.

## Results and discussion
We now know how to subscribe to messages from ROS with an arduino! Amazing! The arduino code can be changed to utilize the other variables from the message to do cool stuff such as control a robot. We'll learn how to do this in the later tutorials. See you guys then!


