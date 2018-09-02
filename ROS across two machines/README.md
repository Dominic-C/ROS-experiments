# ROS across multiple machines
In order to run ROS across multiple machines, we will have to run one machine as a master and the rest as slaves. To allow ROS to communicate between machines, they must have the same IP addresses in the ROS variable `ROS_MASTER_URI`.

The following assumes you have sourced your `setup.bash` in the devel folder of your catkin workspace, and in `/opt/ros/kinetic/` for each terminal instance. 

## Set up ROS master computer
quick tip: `export` creates variables in your instance of the shell

This device will be running roscore. On this device, do the following:
* `~$ export ROS_MASTER_URI="http://<ip address of master computer>:11311"`
* `~$ export ROS_IP="<ip address of master computer>"`

## Set up ROS slave computer(s)
The device(s) will be subscribing to roscore from the Master device. Run the following:
* `~$ export ROS_MASTER_URI="https://<ip address of master computer>:11311"`
* `~$ export ROS_IP="<ip of slave device>"`

### Note:
For each shell instance, we need to run `export ROS_IP=...`, so it is advisable to create an alias for it in your .bashrc

You should be able to test this by running a tutlesim node and checking the rqt_graph on either device

Forgetting to put the inverted commas results in communication failure. This is because ROS variables must be in the form of strings.


# USB camera example
This example shows how to stream videos over wifi via ROS, from one device to another.

## requirements
* USB camera
* 2 computers (1 of which could be a RaspberryPi 3b or later - for the wifi capabilities)


## Procedure
On the **slave device**, complete the setup documented above. Then run `lsusb`. This command lists all your connected USB devices. Plug in the USB camera and run `lsusb` again. You should be able to see the difference between the two lists to find the name of your camera. We do this to check if the slave computer recogizes the camera.

After which, we can install the ROS package for usb cameras by running `sudo apt install ros-kinetic-usb-cam`

Moving to the **master**, run `roscore`. This launches the rosmaster.

Back to the **slave device**, we run `roslaunch usb_cam usb_cam-test.launch`.

On either device, we should be able to run `rostopic list` to view our running topics. We should be able to see something similar to the following:

```
~$ rostopic list
/rosout
/rosout_agg
/usb_cam/camera_info
/usb_cam/image_raw
/usb_cam/image_raw/compressed
/usb_cam/image_raw/compressed/parameter_descriptions
/usb_cam/image_raw/compressed/parameter_updates
/usb_cam/image_raw/compressedDepth
/usb_cam/image_raw/compressedDepth/parameter_descriptions
/usb_cam/image_raw/compressedDepth/parameter_updates
/usb_cam/image_raw/theora
/usb_cam/image_raw/theora/parameter_descriptions
/usb_cam/image_raw/theora/parameter_updates

```
Finally, on the **master**, we can run `rosrun image_view image_view image:=/usb_cam/image_raw/`

We should now be able to stream videos though there is quite a severe lag over wifi to have any real time application. I will continue exploring alternatives to reduce the overhead. 