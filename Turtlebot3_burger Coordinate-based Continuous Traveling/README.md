 Initialization: 
Clear variables, close all figures, and set up ROS 2 environment. The robot's initial pose is set to a specified position and orientation.
Setup ROS 2 Nodes and Publishers:
Create nodes for publishing velocity commands and subscribing to odometry and model state topics.
Set up the publisher for /cmd_vel to control the robot's velocity.
Create subscribers to obtain robot odometry (/odom) and model state (/gazebo/model_states).
Initial Data Collection:
Store the initial position of the robot using model state data.
Specify the desired coordinates for the robot to navigate.
 Control Loop:
For each coordinate:
 	     Initialize timer and other necessary variables.
Use a while loop to control the robot until it reaches the target coordinate (with a small tolerance).
Calculate the heading angle based on the slope of the line between the current and target coordinates.
Use proportional control (k_angle) to determine the angular velocity (omega_real) to reduce the error between current and desired heading.
Adjust the linear velocity (velocity_real) inversely proportional to the angular velocity to prevent slippage.
Send the calculated velocity commands.
Record data for plotting path, heading, and velocities.
Path Length Calculation:
During the while loop, calculate and accumulate the path length (PL) using the current and previous robot positions.
Plotting:
Plot the odometry path of the robot.
Plot the path using model states.
Plot heading vs time.
Plot commanded linear and angular velocities vs time.

 End of Execution:
Stop the robot by setting linear and angular velocities to zero.
Display the estimated drive time (DT) and path length (PL).
 Helper Function:
Define a function (odomMeasure) to get the current position from the odometry topic. This function is used in the main control loop to get fresh position data.

3.	The step to improve the robot 

1.	We adjusted the control of the linear velocity so that it varies proportionally with the angular velocity. When the angular velocity is large, let the linear velocity decrease to avoid unreasonable sliding of the robot; when the angular velocity is small, let the linear velocity increase to reach the destination more quickly
2.	We have transformed the angle of the vector between the target and present coordinates with respect to the world coordinates, converting angle values from -360 degrees to 360 degrees to a range of -180 degrees to 180 degrees, which greatly facilitates operations
3.	Our code is modular in form and can theoretically be extended from three to infinite target points, which provides a good foundation for subsequent work
