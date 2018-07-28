# ROS experiments
In this repository, I will document adventures tinkering around with the Robot Operating System (ROS).

## Getting started
These documents will walk you through my tests and experiments I have done while tinkering with Arduinos and hardware with ROS

### Prerequisites
* You will need to install ROS from http://wiki.ros.org/ROS/Installation. I used ROS kinetic on an Ubuntu OS.
* Arduino IDE

### Creating a workspace
After installing the appropriate version of ROS, you will need to create a catkin workspace. Don't be intimidated with the terms used here. A catkin workspace is just a directory (or folder if that helps) where ROS looks for packages and builds them.

before building a workspace, make sure you have sourced the `setup.bash` script. if you have installed ROS kinetic, you can run the following command to source the `setup.bash` files for ROS.

```
$ source /opt/ros/kinetic/setup.bash
```
This will allow you to run ROS related commands in the terminal.

**Tip:** if you wish to avoid constantly running the command above to allow ROS run on new terminal instances, you can add it to the bottom of your `~/.bashrc` script.

To create a workspace, we just have to make a directory with a subfolder called `src`. Note that the name of the folder matters.

```
$ mkdir -p ~/catkin_ws/src
```
Now, we navigate to the root folder (main folder) of the catkin workspace and run `catkin_make`.

```
$ cd ~/catkin_ws/
$ catkin_make
```
Thats it! Now we just have to source the catkin workspaces' `setup.bash` script. While still in the root folder of your catkin workspace, run:
```
$ source devel/setup.bash
```
**Tip:** if you wish to avoid constantly running the command above to allow your workspace packages to be run on new terminal instances, you can add it to the bottom of your `~/.bashrc` script.

Now that we have set up our first catkin workspace, we can proceed with our experiments :)


### Resources
* A free and open source guide for ROS programming in C++ : http://wiki.ros.org/Books/ROS_Robot_Programming_English
* A friend of mine's repository for ROS: https://github.com/methylDragon/coding-notes/tree/master/Robot%20Operating%20System%20(ROS)/ROS
