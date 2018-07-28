## Arduino subscriber test

### Motivation
Inspired to design a robotics system for a project, I wanted to try making microcontrollers, most notably, the Arduino, subscribe to ROS messages.

So I decided to to a simple test. In this test, I subscribe to the topics published by the turtlebot teleop keyboard launch file.

### Finding the message to subscribe to

* First, we have to launch rosmaster by running `roscore`
* Then, we run the turtlebot teleop launch file by running `roslaunch turtlebot_teleop keyboard_teleop.launch`
    * Note: there is only a subtle difference between `roslaunch` and `rosrun`. `roslaunch` runs launchfiles, which run a collection of ros nodes. `rosrun` on the other hand, only runs a single node.
* next, we run `rostopic list` to show us all available rostopics.We should observe the following

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
* Now we see that we should subscribe to the topic `geometry_msgs/Twist`

### Finding variables in the message

Now that we know the message object, we need to find out the contents of the message, so that we can use its output for our program.

In this case, we'll use that output to make LED 13 on the arduino light up when we press the forward button on the turtlebot teleop publisher.

To find the variables in the message, we run `rostopic echo /cmd_vel_mux/input/teleop`, making sure that we still have the teleop publisher `keyboard_teleop.launch` opened.

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

