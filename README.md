# Trajectory-Design-Using-Cubic-Polynomials-and-Implementation-With-Turtlebot--ROS-NOETIC
This project is about trajectory design for a TurtleBot3 using the cubic polynomial method.
The trajectory is executed in ROS NOETIC environment , using the Gazebo simulator. 
The project is written in python3.

The robot starts from the point [0,0,0] , reaches the point [16,8,N/A] and ends its trajectory at the point [16,-8,1.5825]. 
The duration that the robot takes each time to get from one point to another is 130 seconds. 
During the simulation in the terminal is printed in order position x, position y, time , total velocity for the robot. 
After the end of the execution the relevant graphs are displayed on the screen , where it is possible to observe the differences between design and reality.

The program is executed on linux , if all the necessary packages (ROS NOETIC,GAZEBO) are installed by using the following commands in the terminal.
1) roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
2) rosrun rotate rotate.py

The whole workspace can be found at the following link -->
https://drive.google.com/drive/folders/1X487MDtjN74Uv69YedfvdAkSDW03lRtS?usp=sharing

