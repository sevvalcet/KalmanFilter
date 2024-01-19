# KalmanFilter

**The aim of This Project**


This project considers the problem of localization for trucks traveling on highways. An Extended Kalman Filter is constructed by fusing GPS and IMU sensors. 
The Linear Kalman Filter is also applied to the same system for comparison.
The algorithm is built as a ROS package as a framework in C++ following OOP paradigms


## DEPENDENCIES ##

 'PlotJuggler' was used to visualize the algorithm outputs and the measurements taken from GPS-IMU sensors on a graph.
 This application is installed in the system with the following command.

 `sudo apt install ros-$ROS_DISTRO-plotjuggler-ros`
 
 For the rest of the dependencies you can run this command:

`rosdep install --from-paths src --ignore-src -r -y` 

## Running Extended Kalman Filter  ##

`roslaunch kf ekf.launch` 


## Running PlotJuggler ##

`rosrun plotjuggler plotjuggler` 