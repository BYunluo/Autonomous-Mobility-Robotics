% Clean up!
clear variables; close all; clc;

% Set the environment variables in MATLAB that point to ROS2 on your
% Ubuntu system; Turtlebot3=30!
setenv('ROS_DOMAIN_ID', '30');

% Reset robot pose to set the robot at original point and heading the
% positive x axis
ros2Command ='ros2 service call /gazebo/set_entity_state "gazebo_msgs/srv/SetEntityState" "{state: {name: ''turtlebot3_burger'', pose: {position: {x: 0.0, y: 0.0, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"';

% Execute the command in the terminal using MATLAB's system function
[status, cmdout] = system(ros2Command);

detectCmdVel = ros2node("/detectCmdVel");
% pause(1)

% Create a Publisher object to publish all kinds of velocity
PubVel = ros2publisher(detectCmdVel,'/cmd_vel','geometry_msgs/Twist');
% pause(1)

% Create an object (container) for the velocities
velmsg = ros2message(PubVel);
pause(1)

% Set up Odometry subscription from Turtlebot3
% Detect the /odom topic
detectOdom = ros2node("/detectOdom");
% pause(1)

% Create a subscriber for the /odom topic
odomSub = ros2subscriber(detectOdom, "/odom", "nav_msgs/Odometry");
% pause(1)

% Create a subscriber for the /detectModelStates topic
detectModelStates = ros2node("/detectModelStates");
SubModelState = ros2subscriber(detectModelStates, '/gazebo/model_states', 'gazebo_msgs/ModelStates'); 

% Get the initial model state from Gazebo
ModelState = receive(SubModelState, 10);

% Specify the robot model name
robotName = 'burger';
robotIndex = find(strcmp(ModelState.name, robotName));

%STore intial position
ModelPos = [ModelState.pose(robotIndex).position.x, ModelState.pose(robotIndex).position.y];

% Obtain last odom data
Odom = receive(odomSub);
pause(1)

% Setup access to LiDAR /scan topic
detectLiDAR = ros2node("/detectLiDAR");
% pause(1)

% Create a subscriber for the /scan topic
subLiDAR = ros2subscriber(detectLiDAR, "/scan", "sensor_msgs/LaserScan");
% pause(1)

% Read most recent frame of LiDAR data
scan = receive(subLiDAR);
pause(1)

%Specify the exact coordinates to be reached by the robot
% This array can be extended to any number of points
coordinates=[0, 0;
            -2,4;
             3,2];

%The number of execution edges is determined by the number of rows in the
%coordinates array
num = size(coordinates,1);

% In order to draw a curve with respect to time at the end of code
% Create a node for timer
ratenode = ros2node('rate_node');

% Set the desired loop rate (in Hz)
loopRate = ros2rate(ratenode, 10); % 10 Hz; 100 mS per pass

% Initial variable 
% Setup plot variables;
Langle = linspace(0,359, 360); % angles spaced 1 deg apart & STARTING ANGLE
x_plot =[];
y_plot =[];
xm=[];
ym=[];
% k_angle = 0.3;
k_angle = 0.35;
k_avoid = 0.5;
theta = [];
plot_t = [];
plot_theta = [];
PL = 0;
startTime = tic;
plot_linearvel = [];
plot_angularvel = [];
odomvel_linear = 0;
acquired_linear = []; 
odomvel_angular = 0;
acquired_angular = [];
average_V_a = 0;
average_Omega_c = 0;
average_V_c = 0;
average_Omega_a = 0;
array_averageVc = [];
array_averageOmega_c = [];
array_averageVa = [];
array_averageOmega_a = [];
allowable_distance_min = 1.2; 
allowable_angle_min = 30; %on th exam if need to detect larger range in front of robot, changing the parameter here
right = [];
right_w = [];
omega_sub = 0;
omega_main = 0;
omega_real = 0;
threshold_max = 360 - allowable_angle_min; 
threshold_min = allowable_angle_min;
left_weight = 0;
right_weight = 0;
omega_avoid = 0.3;
row_crash_l = [];
crash_l = [];
crash_avoid_range_l = [];
crash_avoid_l = [];
row_crash_r = [];
crash_r = [];
crash_avoid_range_r = [];
crash_avoid_r = [];
safety_distance = 0.4;
crash_left = [];
crash_right = [];
percent_factor = 0;
minimal_linear_vel_factor = 0.25;
% Set up loop for desired number of executions in coordinates array
for nS = 1:num
    % Initialize rTT: robot travel time
    rTT = 0; 
    reset(loopRate) % initialize timer
    prex = NaN;% Used for path length calculation
    prey = NaN;
   
    %Initilialisation
    len = 100000000; %Anything greater than zero to enter the loop
     %Receive current odom pose
     Odom = receive(odomSub);
     [OdomPos(1), OdomPos(2), OdomPos(3), OdomPos(4)] = deal(Odom.pose.pose.orientation.x, ...
        Odom.pose.pose.orientation.y, Odom.pose.pose.orientation.z, Odom.pose.pose.orientation.w);
    % Setting error factor
    percent_factor = 15/100;
    range_min = 0;
    range_max = percent_factor + range_min;
    
    %Based on the difference in distance between current coordinate and exact
    %coordinates to be reached. Until the distance is 0 plus an error
    %factor, then the robot is not actually at the exact coordinates
    while (len > range_max)
        % This high-frequency refresh while loop correct the robot state in real time to better orient it heading the target pints
        ModelState = receive(SubModelState);
        ModelPos = [ModelState.pose(robotIndex).position.x, ModelState.pose(robotIndex).position.y];

        rTT = loopRate.TotalElapsedTime; %not used to contorla nything

        Odom = receive(odomSub);
        [OdomPos(5), OdomPos(6)] = deal(Odom.pose.pose.position.x, ...
    Odom.pose.pose.position.y);
        
        % This part for subscribe the Lidar topic and initialize the
        % variables 
        scan = receive(subLiDAR);
        dist = scan.ranges;
        dist(dist == inf) = 0;
        row = find(dist ~= 0);
        distance = dist(row);
        detect = [row, distance];

        % This segment for avoid crash with obstacles
        row_crash_l = (1:150)'; % detect range for left side 
        crash_l = dist(1:150);
        crash_avoid_range_l = [row_crash_l, crash_l];
        crash_avoid_l = crash_avoid_range_l(crash_avoid_range_l(:, 2) ~= 0, :);
        row_crash_r = (210:360)'; % detect range for right side
        crash_r = dist(210:360);
        crash_avoid_range_r = [row_crash_r, crash_r];
        crash_avoid_r = crash_avoid_range_r(crash_avoid_range_r(:, 2) ~= 0, :);
        
        % This part screen the appropriate values from the array above
        crash_left = crash_avoid_l(crash_avoid_l(:,2) <= safety_distance,:);
        crash_right = crash_avoid_r(crash_avoid_r(:,2) <= safety_distance,:);
        left = detect(detect(:,1) <= threshold_min, :);
        right = detect(detect(:,1) >= threshold_max, :);
        right_w = right(1:end-1,:);

        % Reading the current position by using data from Odom topic 
        cur_coordinates = [OdomPos(5), OdomPos(6)];
        %Based on the coordinates array, infer the next target coordinates
        target_coordinates = coordinates(mod(nS, size(coordinates,1))+1, :);

    % In order to convert the angle between current coordinate and target coordinate (from -2pi to 2pi) into -pi to pi,
    % our arithmetic based on the slope of the line between two
    % coordinates and contain the special angle like pi and -pi
        slope = (target_coordinates(2)-cur_coordinates(2)) / (target_coordinates(1)-cur_coordinates(1));
        thd = atan(slope);
    % We divide all of situation into two parts: slope greater than 0,
    % and slope less than 0. Some situation which not include will discuss below 
    if slope >= 0 
        theta = [thd -pi+thd];
    else 
        theta = [thd pi+thd];
    end
    % 
    if slope == 0 % Discuss about when angle is +- pi/2
       if target_coordinates(1)-cur_coordinates(1) > 0
           thdr = theta(1);
       else
           thdr = theta(2);
       end
    elseif target_coordinates(1) == cur_coordinates(1) % Discuss about when angle is 0 or +-pi
            thdr = theta(1);
    % There were four situation here, we classify them into same sign group
    % and different sign group for slope and x's difference values
    elseif slope * (target_coordinates(1) - cur_coordinates(1)) > 0 
        thdr = theta(theta > 0);
    elseif slope * (target_coordinates(1) - cur_coordinates(1)) < 0
        thdr = theta(theta < 0);
    end
    % The quaternion after we convert the abgle into -pi to pi
     thdr_quat = [cos(thdr/2), 0, 0, sin(thdr/2)];
     eual_target=quat2eul(thdr_quat);

        [x y] = odomMeasure(odomSub);%Receive the current x and y coordinates from Odom
        x1 = x;
        y1 = y;
        [OdomPos(1), OdomPos(2), OdomPos(3), OdomPos(4)] = deal(Odom.pose.pose.orientation.x, ...
    Odom.pose.pose.orientation.y, Odom.pose.pose.orientation.z, Odom.pose.pose.orientation.w);
        quat = [OdomPos(4), OdomPos(1), OdomPos(2), OdomPos(3)]; % Setting its sequence as w, x, y, z
        % Get current angle from odometry quaternion
        eul_current = quat2eul(quat);
        [x1, y1] = odomMeasure(odomSub); % Current coordinate
        x_1 = target_coordinates(1,1);
        y_1 = target_coordinates(1,2);
        % The len variable is showing the distance of current coordinates and target coordinate.
        len = sqrt((x1-x_1)^2 + (y1-y_1)^2);
        
        % Determin the error between current Euler angle and target Euler
        % angle in order to setting angular velocity
        % In order to avoid error_angle greater than pi or less than -pi, we using wrapToPi
        error_angle = wrapToPi(eual_target(1) - eul_current(1)); 
        % Setting angular velocity by proportional control
        omega_main = k_angle * error_angle;

        % The main part of judging if it suitable to applying the avoid
        % crash cases or keeping obstacles away from path
        if ~isempty(crash_left) && ~isempty(crash_right)
            omega_sub = 0;
        elseif ~isempty(crash_left) && isempty(crash_right)
            omega_sub = -k_avoid *omega_avoid;
        elseif isempty(crash_left) && ~isempty(crash_right)
            omega_sub = k_avoid *omega_avoid;
        elseif isempty(crash_left) && isempty(crash_right)
            if ~isempty(left) && ~isempty(right)
                left_weight = size(left, 1);
                right_weight = size(right_w, 1);
                if left_weight < right_weight
                    omega_sub = omega_avoid;
                else
                    omega_sub = - omega_avoid;
                end
            elseif ~isempty(left) && isempty(right)
                omega_sub = - omega_avoid;
            elseif isempty(left) && ~isempty(right)
                omega_sub =  omega_avoid;
            else
                omega_sub = omega_main;
            end
         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Using the obstacle avoidance algorithm or desired path algorithm
        if any(left(:,2) <= allowable_distance_min) || any(right(:,2) <= allowable_distance_min) || any(crash_right(:,2) <= safety_distance) || any(crash_left(:,2) <= safety_distance)
        %if any(left(:,2) <= allowable_distance_min) || any(right(:,2) <= allowable_distance_min)
            omega_real = omega_sub;
        else
            omega_real = omega_main;
        end

        % We want the robot linear velocity getting lower when angular velocity is very high
        % in order to avoid slip, and expect linear velocity getting bigger
        % when angular velocity is small inorder to reach the goal as fast
        % as robot can
        if omega_real > 0
            velocity_real = - (1/(3*pi)) * omega_real + minimal_linear_vel_factor;
        % angular velocity have positive and negative, but linear velocity
        % only allow the positive values
        else
            velocity_real = 1/(3*pi) * omega_real + minimal_linear_vel_factor;
        end
        % Send the velocity command as what we calculate before
        velmsg.linear.x = double(velocity_real);
        velmsg.angular.z = double(omega_real);
        send(PubVel,velmsg);  
        waitfor(loopRate);
        %Recording the linear velocity and angular velocity to draw the
        %curve at end 
        currentTime = toc(startTime);
         % Recording the timer
         plot_t = [plot_t, currentTime];
         % Recording the z axis of the current Euler angle in order to draw
         % the heading  plot
         plot_theta = [plot_theta, eul_current(1)]; 
        plot_linearvel = [plot_linearvel, velocity_real];
        average_V_c = mean(plot_linearvel);
        array_averageVc = ones(1,length(plot_t)) * average_V_c;
        plot_angularvel = [plot_angularvel, omega_real];
        average_Omega_c = mean(plot_angularvel);
        array_averageOmega_c = ones(1,length(plot_t)) * average_Omega_c; %O for omega

        odomvel_linear = Odom.twist.twist.linear.x;
        acquired_linear = [acquired_linear, odomvel_linear];
        average_V_a = mean(acquired_linear);
        array_averageVa = ones(1,length(plot_t)) * average_V_a;
        odomvel_angular = Odom.twist.twist.angular.z;
        acquired_angular = [acquired_angular, odomvel_angular];
        average_Omega_a = mean(acquired_angular);
        array_averageOmega_a = ones(1,length(plot_t)) * average_Omega_a; %O for omega
        %Refresh the odom and modelstates array
        x_plot = [x_plot, x1];
        y_plot = [y_plot, y1];
        xm = [xm, ModelPos(1)];
        ym = [ym, ModelPos(2)];
        % Accumulation calculatethe path length
        if ~isnan(prex)
            PL_dis = sqrt((OdomPos(5) - prex) ^ 2 + (OdomPos(6) - prey) ^ 2);
            PL = PL + PL_dis;
        end
        % Reset the previous position of robot
        prex = OdomPos(5);
        prey = OdomPos(6);
       
    end

nS = nS + 1;%Get to next side after the moving and rotation 

end

% Stop the robot when mission accomplished
velmsg.linear.x = 0;
velmsg.angular.z = 0;
send(PubVel,velmsg);

%Plot the accumulated values in the plot variable here
figure;
axis equal
for x=1:num
    plot(coordinates(x,1),coordinates(x,2),"r*");
   hold on;
end
plot(x_plot,y_plot,'c*')
%   Equalize the axes

%   Label the axes & add Title
xlabel('x')
ylabel('y')
title('Odom data of Robot Paths')
hold on;

figure;
plot(xm,ym,'c*')
% for x=1:num
%     plot(coordinates(x,1),coordinates(x,2),"r*");
%    hold on;
% end
%   Equalize the axes
axis equal
%   Label the axes & add Title
xlabel('x')
ylabel('y')
title('ModelStates data of Robot Paths')
hold on;
%The heading plot using what we record before
figure;
plot(plot_t, plot_theta, 'c*')
xlabel('time/s')
ylabel('Heading/rad')
title('Heading vs t')
grid on;
hold on;

figure;
plot(plot_t, x_plot, 'b');
hold on;
plot(plot_t, y_plot, 'r');
hold on;
plot(plot_t, plot_theta, 'g');
hold on;
xlabel('Time/s');
ylabel('x, y & heading');
title('Path and heading vs Time');
legend('x(m)','y(m)', 'Heading(rad)','Location','best');
grid on;
hold on;

% The two velocity plot using the data we record
figure;
plot(plot_t, plot_linearvel, 'r');%calculated essentially from code
hold on;
plot(plot_t, plot_angularvel, 'b');
hold on;
plot(plot_t, array_averageVc, 'r--');
hold on;
plot(plot_t, array_averageOmega_c, 'b--');
xlabel('Time/s');
ylabel('V_c && Omega_c');
title('Commanded Linear and Angular Velocity vs Time');
legend('Commanded_LinearVelocity(m/s)','Commanded_AngularVelocity(rad/s)', 'Average_Vc(m/s)','Average_Oc(rad/s)','Location','best');
grid on;
hold on;

figure;
plot(plot_t, acquired_linear, 'r'); %came from odom
hold on;
plot(plot_t, acquired_angular, 'b');
hold on;
plot(plot_t, array_averageVa, 'r--');
hold on;
plot(plot_t, array_averageOmega_a, 'b--');
xlabel('Time/s');
ylabel('V_a && Omega_a');
title('Acquired Linear and Angular Velocity vs Time');
legend('Acquired_LinearVelocity(m/s)','Acquired_AngularVelocity(rad/s)','Average_Va(m/s)','Average_Oa(rad/s)','Location','best');
grid on;
hold on;
% Display the entire running time
DT = toc(startTime);
disp(['Estimate of Drive Time (DT): ', num2str(DT), ' seconds']);
disp(['Estimate of Path Length (PL): ', num2str(PL), ' m']);
%Due to repeating this code several times, a function was used to make it
%just a single line instead of writing multiple lines of the same code over
%and over again    



function [current_x,current_y] = odomMeasure(odomSub)
        % Update plot variables here
        % Get fresh odomometer reading
    Odom = receive(odomSub);
    [OdomPos(1), OdomPos(2)] = deal(Odom.pose.pose.position.x, ...
            Odom.pose.pose.position.y);
        % Augment the plot vector with new data point
     current_x = [OdomPos(1)];
     current_y = [OdomPos(2)];
end