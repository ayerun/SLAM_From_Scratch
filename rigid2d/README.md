# Rigid 2D Transformation Library
A library for handling transformations in SE(2) and calculating differential drive odometry.

## Nodes
* fake_turtle
   - simulates real robot
   - useful for testing
* odometer
   - calculates and tracks robot odometry

## Launch Files
* fake_turtle_odom.launch
   - lauches fake_turtle and odom nodes
   - loads robot in Rviz
   - launches teleoperation node to move robot
```
roslaunch rigid2d fake_turtle_odom.launch
```