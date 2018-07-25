# ROS-experiments
In this repository, I will document my tests and results with the Robot Operating System (ROS).

## Teleop sub test

### Background
In this experiment, I wanted to try and write a subscriber for the turtlebot keyboard teleop node (keyboard_teleop.launch). I'll assume you have a configured catkin workspace with the `devel`, `src` and `build` folders for this short tutorial.

### Writing the package
First, nagivate to the source folder of your catkin workspace. eg. ~/catkin_ws/src.

To create the ros package, we run `catkin_create_pkg teleop_sub_test std_msgs roscpp message_generation geometry_msgs`. copy over all the contents from this folder into the `src` folder of your newly created package (teleop_sub_test).

Navigate to the catkin workspace root directory `cd ~/catkin_ws` and run `catkin_make`. This will build the package.

At this point, you may want to run `rospack profile` to allow ros to find your package. (should still work without rospack profile).

Congratulations! your ROS package is now created!

### Running the publisher and subscriber
* make sure ros master is running. Run `roscore`.

* To run the publisher, we open a new instance of the terminal and simply run the turtlebot teleop node. This is done by running `roslaunch turtlebot_teleop keyboard_teleop.launch`

* To run the subscriber, we open another new instance of the terminal and run our newly created node, under the package name `teleop_sub_test`. So, we run `rosrun teleop_sub_test teleop_sub`.

You should now be able to see the linear and angular velocities. being published when you press the keys that moves the turtlebot.

#### Why we need `geometry_msgs`
keyboard_teleop.launch node publishes the topic `/cmd_vel_mux/input/teleop`. To have a closer look at the published topic, we run `rostopic info /cmd_vel_mux/input/teleop`.

We can see that the type of message being published is `geometry_msgs/Twist`. This is why we need to include the geometry_msgs dependency when creating our package. If you missed adding geometry_msgs in the earlier steps, you can always add them into the package.xml and CMakeLists.txt manually.


