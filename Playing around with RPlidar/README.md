# RPlidar ROS package
While playing around with the ROS RPlidar package, i encountered a weird issue.

After successfully running the launch file, i could not run it again. I had the error:
```
[ERROR] [1535783087.451136294]: scan mode `Sensitivity' is not supported by lidar, supported modes:
[ERROR] [1535783087.451237412]: 	Standard: max_distance: 12.0 m, Point number: 2.0K
[ERROR] [1535783087.451274314]: 	Express: max_distance: 12.0 m, Point number: 4.0K
[ERROR] [1535783087.451312239]: 	Boost: max_distance: 12.0 m, Point number: 8.0K
[ERROR] [1535783087.451341687]: Can not start scan: 80008001!

```
While trying out solutions, I tried restarting roscore. This seemed to fix the issue. I'm still not sure why this happened though.