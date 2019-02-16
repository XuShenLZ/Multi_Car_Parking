%% Multi Car Parking Using MPC For Tracking
% Author: Xu Shen (xu_shen@berkeley.edu)
% MPC Lab, UC Berkeley

%% Start Simulation
clear('all');
close all;

%% Car Model
auto.w = 2.0;                         % car width [m]
auto.db = 1.2;                        % rear axis position, from back [m]
auto.df = 1.0;                        % front axis position, from front [m]
auto.d = 2.8;                         % axel distance [m] %L in the midterm
auto.l = auto.d + auto.df + auto.db;  % car length [m]
auto.tyr = 0.8;                       % tyre diameter [m]
auto.dmax = 45*pi/180;                % maximum front wheel steering angle [rad]
auto.drat = 14.5;                     % ratio between steering wheel angle and front wheel angle
auto.amax = 0.2*9.8;                  % maximum acceleration
auto.vmax = 5;                        % maximum speed

%% Map construction
fig = loadmap();

%% Occupancy Map
% Construction the struct which contains the 
% location, dimension, and the occupancy of 
% each slot
% 
% x-direction: 4.5
% y-direction: 6
space = [4.5, 6];
% location of corners
corners.x = [11.75 16.25 20.75 25.25 29.25,...
			33.75 38.25 42.75 46.75 51.25 55.75];
corners.y = [6.25 24.75 31.25 49.75];
% occupancy status
occupancy = false(4, 11);
occupancy(1, [3,5,7]) = true;
occupancy(2, [1,2,3,7,9]) = true;
occupancy(3, [5,10]) = true;
occupancy(4, [3,4,10]) = true;
slots = map_occupy(space, corners, occupancy);

%% Lane Computing 
% Way point of the first lane
% The way points are defined in 'routePlan.mat'
data = load('routePlan.mat');
routePlan = data.routePlan;
% Modify some parameters to simplify the route
routePlan.StartPose(1,1:2) = [4 10.5];
routePlan.EndPose(1,1:2) = [57 10.5];
routePlan.StartPose(2,1:2) = routePlan.EndPose(1,1:2);
routePlan.EndPose(2,1:2) = [71 18.5];
routePlan.StartPose(3,1:2) = routePlan.EndPose(2,1:2);
routePlan.EndPose(3,1:2) = [71 32.5];
routePlan.StartPose(4,1:2) = routePlan.EndPose(3,1:2);
routePlan.EndPose(4,1:2) = [54 39.5];
routePlan.StartPose(5,:) = routePlan.EndPose(4,:);
routePlan.EndPose(5,:) = routePlan.EndPose(4,:);
routePlan.EndPose(5,1:2) = [10 39.5];
% Convert the angular value to radius value
routePlan.StartPose(:,3) = routePlan.StartPose(:,3)*pi/180;
routePlan.EndPose(:,3) = routePlan.EndPose(:,3)*pi/180;

% routePlan2: The second lane
routePlan2 = routePlan;
routePlan2.StartPose(1,1:2) = [4 14.5];
routePlan2.EndPose(1,1:2) = [53 14.5];
routePlan2.StartPose(2,1:2) = routePlan2.EndPose(1,1:2);
routePlan2.EndPose(2,1:2) = [67 22.5];
routePlan2.StartPose(3,1:2) = routePlan2.EndPose(2,1:2);
routePlan2.EndPose(3,1:2) = [67 28.5];
routePlan2.StartPose(4,1:2) = routePlan2.EndPose(3,1:2);
routePlan2.EndPose(4,1:2) = [54 35.5];
routePlan2.StartPose(5,1:2) = routePlan2.EndPose(4,1:2);
routePlan2.EndPose(5,1:2) = [10 35.5];

%% Path Planning - Using Kinematic Point Model
factor = 2; % The number of divisions in the path
% The optimized path and inputs
[zdata, udata] = path_planner(auto, routePlan, factor);
[zdata2, udata2] = path_planner(auto, routePlan2, factor);

% Plot Solution
% Plot on the map generated in the last step
plot(zdata(1,:),zdata(2,:),'r')
hold on
z0 = routePlan.StartPose(1,:)';
car_plot(auto, udata, z0);

plot(zdata2(1,:),zdata2(2,:),'r')
hold on
z0_2 = routePlan2.StartPose(1,:)';
car_plot(auto, udata2, z0_2);

%% Sample the path
% Sample path and generate a speed provile over the path
refPoses = RefSample(zdata, udata);
refPoses2 = RefSample(zdata2, udata2);

%% The parking target
% Specify the slot that the car is going to park
g_slot = [4,5]; % The forth row and second col
global_manu = park_maneuver(g_slot, slots);
g_slot2 = [3,4]; % The forth row and second col
global_manu2 = park_maneuver(g_slot2, slots);

%% Path Follower
% Start from a certain place
v_sim.start_pose = [4; 10.5; 0];
% Stop at the starting point of parking maneuver
v_sim.stop_pose = global_manu(1:3,1);
v_sim = path_follower_MPC(auto, refPoses, v_sim, 0);
% car_plot_bike(auto, v_sim.z, v_sim.u, v_sim.z(:,1));

plot(global_manu(1,:), global_manu(2, :), 'b')
v_sim_p.start_pose = v_sim.z(1:3,end);
v_sim_p.stop_pose = global_manu(1:3,end);
v_sim_p = path_follower_MPC(auto, global_manu(1:3,:), v_sim_p, 0.2);
% car_plot_bike(auto, v_sim_p.z, v_sim_p.u, v_sim_p.z(:,1));

%% Vehicle 2
% Start from a certain place
v_sim2.start_pose = [4; 14.5; 0];
% Stop at the starting point of parking maneuver
v_sim2.stop_pose = global_manu2(1:3,1);
v_sim2 = path_follower_MPC(auto, refPoses2, v_sim2, 0);
% car_plot_bike(auto, v_sim2.z, v_sim2.u, v_sim2.z(:,1));

plot(global_manu2(1,:), global_manu2(2, :), 'b')
v_sim_p2.start_pose = v_sim2.z(1:3,end);
v_sim_p2.stop_pose = global_manu2(1:3,end);
v_sim_p2 = path_follower_MPC(auto, global_manu2(1:3,:), v_sim_p2, 0.2);
% car_plot_bike(auto, v_sim_p2.z, v_sim_p2.u, v_sim_p2.z(:,1));

