# ROS-experiments
In this repository, I will document my tests and results with the Robot Operating System (ROS).

## Teleop sub test

### Background
In this experiment, I wanted to try and write a subscriber for the turtlebot keyboard teleop node (keyboard_teleop.launch). I'll assume you have a configured catkin workspace `devel`, `src` and `build` folders for this short tutorial.

### Writing the package
First, nagivate to the source folder of your catkin workspace. `~/catkin_ws/src`

To create the ros package, we run `catkin_create_pkg teleop_sub_test std_msgs roscpp message_generation geometry_msgs`. This creates a package 

navigate to the `src` folder of your newly created package `cd ~/catkin_ws/src/teleop_sub_test/src`, and copy over all the contents from my `teleop_sub_test` folder in this repository.

Navigate to the catkin workspace root directory `cd ~/catkin_ws` and run `catkin_make`. This will build the package.

At this point, you may want to run `rospack profile`. This allows ROS to find your package, and also enables tab completion for your package.

Congratulations! your ROS package is now created!

### Running the publisher and subscriber
* make sure rosmaster is running by running `roscore`.

* To run the publisher, we open a new instance of the terminal and simply run the turtlebot teleop node. This is done by running `roslaunch turtlebot_teleop keyboard_teleop.launch`

* To run the subscriber, we open another new instance of the terminal and run our newly created node, under the package name `teleop_sub_test`. So, we run `rosrun teleop_sub_test teleop_sub`.

You should now be able to see the linear and angular velocities. being published when you press the keys that moves the turtlebot.

#### Why we need `geometry_msgs`
keyboard_teleop.launch node publishes the topic `/cmd_vel_mux/input/teleop`. To have a closer look at the published topic, we run `rostopic info /cmd_vel_mux/input/teleop`.

We can see that the type of message being published is `geometry_msgs/Twist`. This is why we need to include the geometry_msgs dependency when creating our package. If you missed adding geometry_msgs in the earlier steps, you can always add them into the package.xml and CMakeLists.txt manually.

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

