% Open loop implementations for driving path.
% This program is an implementation for the OL case based on time.
% ROS2 version
% Date: Oct 5, 2024

%Summary: This is a stop and shoot form of control based on running time 
% in which the robot stops after reaching the calculated time and then
% rotate time due to the angular velocity.

clear variables; close all; clc;

% Set the environment variables in MATLAB that point to ROS2 on your
% Ubuntu system; Turtlebot3=30!
setenv('ROS_DOMAIN_ID', '30');
ros2Command = 'ros2 service call /gazebo/set_entity_state "gazebo_msgs/srv/SetEntityState" "{state: {name: ''turtlebot3_burger'', pose: {position: {x: 0.0, y: 0.0, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"';

% Execute the command in the terminal using MATLAB's system function
[status, cmdout] = system(ros2Command);

% Setup drive commands to Turtlebot3
% Detect the /cmd_vel topic
detectCmdVel = ros2node("/detectCmdVel");
pause(1)

% Publishing velocities
% Create a Publisher object
PubVel = ros2publisher(detectCmdVel,'/cmd_vel','geometry_msgs/Twist');
pause(1)

% Create an object (container) for the velocities
velmsg = ros2message(PubVel);
pause(1)

% Model State topic currently not available!

% Set up Odometry subscription from Turtlebot3
% Detect the /odom topic
detectOdom = ros2node("/detectOdom");
pause(1)

% Create a subscriber for the /odom topic
odomSub = ros2subscriber(detectOdom, "/odom", "nav_msgs/Odometry");
pause(1)

% Obtain last odom data
Odom = receive(odomSub);
pause(1)

% Deconstruct odom data to obtain initial position
[OdomPos(1), OdomPos(2)] = deal(Odom.pose.pose.position.x, ...
    Odom.pose.pose.position.y);

% Create a subscriber for the /detectModelStates topic
detectModelStates = ros2node("/detectModelStates");
SubModelState = ros2subscriber(detectModelStates, '/gazebo/model_states', 'gazebo_msgs/ModelStates'); 

% Get the initial model state from Gazebo
ModelState = receive(SubModelState, 10);

% Specify the robot model name
robotName = 'turtlebot3_burger';
robotIndex = find(strcmp(ModelState.name, robotName));

ModelPos = [ModelState.pose(robotIndex).position.x, ModelState.pose(robotIndex).position.y];


% Initialize the plot variables here
% Pre-allocation with appropriate size will increase efficiency; later!
x = OdomPos(1);
y = OdomPos(2);
xm=[];
ym=[];
% Choose values for pure forward motion; do not exceed 0.4 m/s!
velStraight = 0.2; % forward velocity (m/s)
rotStraight = 0; % rotational rate (rad/s)

% Choose values for pure turn motion; adjust as needed.
velTurn = 0;
rotTurn = pi/20;

% Initialize data
lS = [3 3 3]; % lengths of triangle sides in meters
angT = [60 60 60]; % angles of triangle in degrees
angT = 180 - angT; % convert to turn angles


% Setup time measurement before entering loop
% Create rate object to run loop at 10 Hz
% Loop duration = 1000/10 = 100 ms
% Create a node for timer
ratenode = ros2node('rate_node');

% Set the desired loop rate (in Hz)
loopRate = ros2rate(ratenode, 10); % 10 Hz; 100 mS per pass

% Set up loop for three sides of triangle 3 X (forward + rotate).
% Exit loop when robot completes requirements - whatever you use!
% Convert to MATLAB function implementation later.
for nS = 1:3
    % Set up driving straight
    tS = lS(nS)/velStraight % estimated edge travel time
    tol = 10/100; %t || rTT < (tS - tS*tol) olerance
    % Fill container variable with velocities for forward motion
    velmsg.linear.x = velStraight;
    velmsg.angular.z = rotStraight;
    % Start driving straight
    send(PubVel, velmsg);
    % Initialize rTT: robot travel time
    rTT = 0;
    reset(loopRate) % initialize timer

    % Drive forward for estimated time
    while (rTT < tS*(1+tol))
        % Update rTT
        rTT = loopRate.TotalElapsedTime
        ModelState = receive(SubModelState, 10);
        ModelPos = [ModelState.pose(robotIndex).position.x, ModelState.pose(robotIndex).position.y];
        tol2=7/100; %Utilising coefficient which results in a more accurate algorithm & plot than without it 
        if rTT > (tS*(1+tol2))
            break
        end
        % Keep sending velocity commands for uninterrupted motion
        send(PubVel,velmsg);
        % Update plot variables here
        % Get fresh odomometer reading
        Odom = receive(odomSub);
        [OdomPos(1), OdomPos(2)] = deal(Odom.pose.pose.position.x, ...
            Odom.pose.pose.position.y);
        % Augment the plot vector with new data point
        x = [x OdomPos(1)];
        y = [y OdomPos(2)];
        xm = [xm, ModelPos(1)];
        ym = [ym, ModelPos(2)];
        % Burn rest of loop time
        waitfor(loopRate);
    end

    % Stop the robot after forward motion
    velmsg.linear.x = 0;
    velmsg.angular.z = 0;
    send(PubVel,velmsg);
    
    % Set up turn in place
    tR = (angT(nS)*pi/180)/rotTurn; % estimated angle rotation time
    % Fill container variable with velocities for forward motion
    velmsg.linear.x = velTurn;
    velmsg.angular.z = rotTurn;
    % Start turning
    send(PubVel, velmsg);
    % Initialize rTT: robot travel time
    rTT = 0;
    reset(loopRate) % initialize    

    % Turn in place for estimated time
    while (rTT < tR)
        rTT = loopRate.TotalElapsedTime;
        % Keep sending velocity commands for uninterrupted motion
        send(PubVel,velmsg);
        % Update the plot variables here
        % Get fresh odomometer reading
        Odom = receive(odomSub);
        [OdomPos(1), OdomPos(2)] = deal(Odom.pose.pose.position.x, ...
            Odom.pose.pose.position.y);
        ModelState = receive(SubModelState, 10);
        ModelPos = [ModelState.pose(robotIndex).position.x, ModelState.pose(robotIndex).position.y];
        % Augment the plot vector with new data point
        x = [x OdomPos(1)];
        y = [y OdomPos(2)];
        xm = [xm, ModelPos(1)];
        ym = [ym, ModelPos(2)];
        % Burn rest of loop time
        waitfor(loopRate);
    end

    % Stop the robot after turn motion
    velmsg.linear.x = 0;
    velmsg.angular.z = 0;
    % send(PubVel,velmsg);
end

%   Plot the accumulated values in the plot variable here
figure;
plot(x,y,'c*')
%   Equalize the axes
axis equal
%   Label the axes & add Title
xlabel('x')
ylabel('y')
title('Odom for Robot Paths')
hold on;

figure;
plot(xm,ym,'c*')
%   Equalize the axes
axis equal
%   Label the axes & add Title
xlabel('x')
ylabel('y')
title('Model States Robot Paths')
hold on;
% Stop the robot when mission accomplished
velmsg.linear.x = 0;
velmsg.angular.z = 0;
send(PubVel,velmsg);
