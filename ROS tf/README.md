# tf
tf is a package that lets you keep track of coordinate frames overtime.

coordinate frames may be base frame, world frame, and many more.

This can help in navigation, by giving the current pose of the robot with respect the the map. (pose of base frame in the map frame)

## main idea
Two main ideas:
* listening for transforms
* broadcasting transforms

### Resources
* [Tutorials](http://wiki.ros.org/tf/Tutorials) from the ROS official wiki.