%% Multi Car Parking Using MPC For Tracking
% Author: Xu Shen (xu_shen@berkeley.edu)
% MPC Lab, UC Berkeley

%% Start Simulation
clear('all');
close all;

%% Car Model
auto.w = 2.0;                         % car width [m]
auto.db = 1.3;                        % rear axis position, from back [m]
auto.df = 1.0;                        % front axis position, from front [m]
auto.d = 2.7;                         % axel distance [m] %L in the midterm
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
% location of slot corners
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
lane{1}.routePlan = data.routePlan;
% Modify some parameters to simplify the route
lane{1}.routePlan.StartPose(1,1:2) = [4 10.5];
lane{1}.routePlan.EndPose(1,1:2) = [57 10.5];
lane{1}.routePlan.StartPose(2,1:2) = lane{1}.routePlan.EndPose(1,1:2);
lane{1}.routePlan.EndPose(2,1:2) = [71 18.5];
lane{1}.routePlan.StartPose(3,1:2) = lane{1}.routePlan.EndPose(2,1:2);
lane{1}.routePlan.EndPose(3,1:2) = [71 32.5];
lane{1}.routePlan.StartPose(4,1:2) = lane{1}.routePlan.EndPose(3,1:2);
lane{1}.routePlan.EndPose(4,1:2) = [54 39.5];
lane{1}.routePlan.StartPose(5,:) = lane{1}.routePlan.EndPose(4,:);
lane{1}.routePlan.EndPose(5,:) = lane{1}.routePlan.EndPose(4,:);
lane{1}.routePlan.EndPose(5,1:2) = [10 39.5];
% Convert the angular value to radius value
lane{1}.routePlan.StartPose(:,3) = lane{1}.routePlan.StartPose(:,3)*pi/180;
lane{1}.routePlan.EndPose(:,3) = lane{1}.routePlan.EndPose(:,3)*pi/180;

% lane{2}: The second lane
lane{2}.routePlan = lane{1}.routePlan;
lane{2}.routePlan.StartPose(1,1:2) = [4 14.5];
lane{2}.routePlan.EndPose(1,1:2) = [53 14.5];
lane{2}.routePlan.StartPose(2,1:2) = lane{2}.routePlan.EndPose(1,1:2);
lane{2}.routePlan.EndPose(2,1:2) = [67 22.5];
lane{2}.routePlan.StartPose(3,1:2) = lane{2}.routePlan.EndPose(2,1:2);
lane{2}.routePlan.EndPose(3,1:2) = [67 28.5];
lane{2}.routePlan.StartPose(4,1:2) = lane{2}.routePlan.EndPose(3,1:2);
lane{2}.routePlan.EndPose(4,1:2) = [54 35.5];
lane{2}.routePlan.StartPose(5,1:2) = lane{2}.routePlan.EndPose(4,1:2);
lane{2}.routePlan.EndPose(5,1:2) = [10 35.5];

%% Path Planning - Using Kinematic Point Model
factor = 2; % The number of divisions in the path
% The optimized path and inputs
[lane{1}.zdata, lane{1}.udata] = path_planner(auto, lane{1}.routePlan, factor);
[lane{2}.zdata, lane{2}.udata] = path_planner(auto, lane{2}.routePlan, factor);

% Plot Solution
% Plot on the map generated in the last step
% plot(lane{1}.zdata(1,:),lane{1}.zdata(2,:),'r')
% hold on
% z0 = lane{1}.routePlan.StartPose(1,:)';
% car_plot(auto, lane{1}.udata, z0);

% plot(lane{2}.zdata(1,:),lane{2}.zdata(2,:),'r')
% hold on
% z0 = lane{2}.routePlan.StartPose(1,:)';
% car_plot(auto, lane{2}.udata, z0);

%% Sample the path
% Sample path and generate a speed provile over the path
lane{1}.refPoses = RefSample(lane{1}.zdata, lane{1}.udata);
lane{2}.refPoses = RefSample(lane{2}.zdata, lane{2}.udata);

plot(lane{1}.zdata(1,:),lane{1}.zdata(2,:),'-ro')
[u,v] = pol2cart(lane{1}.zdata(3,:),1);
quiver(lane{1}.zdata(1,:),lane{1}.zdata(2,:),u,v,'r','linewidth',2,'AutoScaleFactor', 0.1);
plot(lane{1}.refPoses(1,:),lane{1}.refPoses(2,:),'b')

plot(lane{2}.zdata(1,:),lane{2}.zdata(2,:),'-ro')
[u,v] = pol2cart(lane{2}.zdata(3,:),1);
quiver(lane{2}.zdata(1,:),lane{2}.zdata(2,:),u,v,'r','linewidth',2,'AutoScaleFactor', 0.1);
plot(lane{2}.refPoses(1,:),lane{2}.refPoses(2,:),'b')

%% The parking target
% Specify the slot that the car is going to park
car{1}.g_slot = [4,7]; % The 4 row and 7 col
car{1}.global_manu = park_maneuver(car{1}.g_slot, slots);
car{2}.g_slot = [3,4]; % The 3 row and 4 col
car{2}.global_manu = park_maneuver(car{2}.g_slot, slots);

%% Path Follower
% Vehicle 1
% Start from a certain place
car{1}.v_sim.start_pose = [4; 10.5; 0];
% Stop at the starting point of parking maneuver
car{1}.v_sim.stop_pose = car{1}.global_manu(1:3,1);
car{1}.v_sim = path_follower_MPC(auto, lane{1}.refPoses, car{1}.v_sim, 0);
% car_plot_bike(auto, car{1}.v_sim.z, car{1}.v_sim.u, car{1}.v_sim.z(:,1));

plot(car{1}.global_manu(1,:), car{1}.global_manu(2, :), 'b')
car{1}.v_sim_p.start_pose = car{1}.v_sim.z(1:3,end);
car{1}.v_sim_p.stop_pose = car{1}.global_manu(1:3,end);
car{1}.v_sim_p = path_follower_MPC(auto, car{1}.global_manu(1:3,:), car{1}.v_sim_p, 0.2);
% car_plot_bike(auto, car{1}.v_sim_p.z, car{1}.v_sim_p.u, car{1}.v_sim_p.z(:,1));

% Vehicle 2
% Start from a certain place
car{2}.v_sim.start_pose = [4; 14.5; 0];
% Stop at the starting point of parking maneuver
car{2}.v_sim.stop_pose = car{2}.global_manu(1:3,1);
car{2}.v_sim = path_follower_MPC(auto, lane{2}.refPoses, car{2}.v_sim, 0);
% car_plot_bike(auto, car{2}.v_sim.z, car{2}.v_sim.u, car{2}.v_sim.z(:,1));

plot(car{2}.global_manu(1,:), car{2}.global_manu(2, :), 'b')
car{2}.v_sim_p.start_pose = car{2}.v_sim.z(1:3,end);
car{2}.v_sim_p.stop_pose = car{2}.global_manu(1:3,end);
car{2}.v_sim_p = path_follower_MPC(auto, car{2}.global_manu(1:3,:), car{2}.v_sim_p, 0.2);
% car_plot_bike(auto, car{2}.v_sim_p.z, car{2}.v_sim_p.u, car{2}.v_sim_p.z(:,1));

