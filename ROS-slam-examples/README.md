# Running ROS slam to view messages

# Running SLAM in ROS to view messages

## Background
I ran this test because i could'nt wrap my head around how to publish odom messages. So, I wanted to view what were the requirements in publishing an odom message. Also, I'm documenting this for future reference.

### Launching the turtlebot environment
Commands
* `roslaunch turtlebot_gazebo turtlebot_world.launch` - the world environment in gazebo
* `roslaunch turtlebot_teleop keyboard_teleop.launch` - the teleop terminal to move the robot around in gazebo

### Launching Rviz visualization
Commands
* `roslaunch turtlebot_rviz_launchers view_navigation.launch` - viewer
* `roslaunch turtlebot_gazebo gmapping_demo.launch` - mapping software