%% path_follower_MPC: follow the ref path using MPC
% ========== INPUTS ============
% auto: The struct contains the parameter of the vehicle
% rePoses: The 3*K matrix contains the [x;y;psi] of reference points
% v_sim: The struct contains the start and stop position
% t_interval: If the time intervals of reference points are the same, 
% pass 0 here, or pass in the specific interval
% ========== OUTPUTS ==========
% v_sim: v_sim.z and v_sim.u are added as the path following result
function [v_sim] = path_follower_MPC(auto, refPoses, v_sim, t_interval)
	start_pose = v_sim.start_pose;
	stop_pose = v_sim.stop_pose;

	%% The start and end pose
	% The index on refPose which is planned to start from
	start_pose_dis = (refPoses(1,:) - start_pose(1)).^2 +...
					(refPoses(2,:) - start_pose(2)).^2;
	start_idx = find(start_pose_dis == min(start_pose_dis), 1);

	% The index on refPose which is planned to end at
	end_pose_dis = (refPoses(1,:) - stop_pose(1)).^2 +...
					(refPoses(2,:) - stop_pose(2)).^2;
	end_idx = find(end_pose_dis == min(end_pose_dis), 1);

	% The section of refPose to be tracked
	path_len = end_idx - start_idx + 1;
	full_path = refPoses(:, start_idx:end_idx);
	
	%% The speed profile
	% Initialize with all ones
	speed_profile = ones(1,path_len) * auto.vmax;
	% Solve the approx time interval between points
	if t_interval == 0
		t_interval = norm(full_path(1:2,2) - full_path(1:2,3)) / auto.vmax;
	end
	% Acceleration stage
	k = 1;
	stage_acc(k) = 0;
	while stage_acc(k) <= speed_profile(k)
		% Full acceleration
		stage_acc(k+1) = stage_acc(k) + t_interval*auto.amax;
		k = k + 1;
	end
	% If the accelerating stage ends before half of the 
	% path, then use all the acceleration stage. 
	% Otherwise, truncate at the half
	acc_end_idx = min(k-1, floor(0.5*path_len));
	speed_profile(1:acc_end_idx) = stage_acc(1:acc_end_idx);

	% Deceleration stage
	% Firstly it is a fliped version of acc
	stage_dec = flip(stage_acc);
	% Then check whether the stage is too long
	dec_start_idx = max(path_len-k+2, ceil(0.5*path_len));
	speed_profile(dec_start_idx:end) = stage_dec(2:end);

	% Store
	full_path = [full_path; speed_profile];
	v_sim.full_path = full_path;

	%% MPC follower
	N = 5; % Horizon
	dt = 0.2;

	% The state vector
	% z = [x; y; psi; v]
	v_sim.z = v_sim.start_pose;
	v_sim.z(4) = 0;

	% The input vector
	% u = [delta; a]
	v_sim.u = [];

	% The error vector between path and current position
	current_dis = (full_path(1,:) - v_sim.z(1)).^2 +...
				(full_path(2,:) - v_sim.z(2)).^2;
	% The closest index
	current_idx = find(current_dis == min(current_dis), 1);
	% Look ahead for 5 points
	% This ahead amount is for path following
	% Not the same as MPC horizon N, but can be set according to 
	% the relationship with N, dt, and speed
	num_ahead = 5;
	goal_idx = current_idx + num_ahead;

	disp('Path Following with MPC...');
	k = 1;
	while goal_idx <= path_len
		% Optimizer
		z = sdpvar(4, N+1);
		u = sdpvar(2, N);

		% Objective Function
		% Obj = (z(N)-z_goal)^2 + sum(u_i^2)
		state_err = z(:, N+1) - full_path(:, goal_idx);
		obj = state_err' * state_err;
		for i = 1:N
			obj = obj + 0.1 * u(:,i)' * u(:,i);
		end

		% Constraints
		constr = z(:,1) == v_sim.z(:,end);
		for i = 1:N
			constr = constr+...
				[z(1,i+1) == z(1,i) + dt * z(4,i) * cos(z(3,i)),...    
	        	z(2,i+1) == z(2,i) + dt * z(4,i) * sin(z(3,i)),...
	        	z(3,i+1) == z(3,i) + dt * z(4,i) * tan(u(1,i)) / auto.l,...
	        	z(4,i+1) == z(4,i) + dt * u(2,i)];
	        constr = constr+...
	        	[-auto.dmax <= u(1,i) <= auto.dmax,...
	        	-auto.amax <= u(2,i) <= auto.amax];
	    end
        
        options = sdpsettings('verbose', 0, 'solver','ipopt');

        optimize(constr, obj, options);

        k = k + 1;
        v_sim.z(:, k) = double(z(:,2));
        v_sim.u(:, k-1) = double(u(:,1));

        p = plotcar(v_sim.z(1, k),v_sim.z(2, k),v_sim.z(3, k),v_sim.u(1, k-1),auto,gcf,[0.3 0.3 0.3]);

        current_dis = (full_path(1,:) - v_sim.z(1, end)).^2 +...
				(full_path(2,:) - v_sim.z(2, end)).^2;
		% The closest index
		current_idx = find(current_dis == min(current_dis), 1);
		% Look ahead for 5 points
		% This ahead amount is for path following
		% Not the same as MPC horizon N, but can be set according to 
		% the relationship with N, dt, and speed
		num_ahead = 5;
		goal_idx = current_idx + num_ahead;

	end

	% Converge to the stop point
	tol = 1e-4;
	disp('Approaching the end of the path...');
	while current_dis > tol
		% Optimizer
		z = sdpvar(4, N+1);
		u = sdpvar(2, N);

		% Objective Function
		% Obj = (z(N)-z_goal)^2 + sum(u_i^2)
		obj = 0;
		for i = 1:N
			state_err = z(:, i) - full_path(:, end);
			obj = obj + 5*state_err' * state_err;
			obj = obj + 0.1 * u(:,i)' * u(:,i);
		end
		state_err = z(:, N+1) - full_path(:, end);
		obj = obj + 5*state_err' * state_err;

		% Constraints
		constr = z(:,1) == v_sim.z(:,end);
		for i = 1:N
			constr = constr +...
				[z(1,i+1) == z(1,i) + dt * z(4,i) * cos(z(3,i)),...    
	        	z(2,i+1) == z(2,i) + dt * z(4,i) * sin(z(3,i)),...
	        	z(3,i+1) == z(3,i) + dt * z(4,i) * tan(u(1,i)) / auto.l,...
	        	z(4,i+1) == z(4,i) + dt * u(2,i)];
	        constr = constr+...
	        	[0 <= z(4,i) <= auto.vmax,...
	        	-auto.dmax <= u(1,i) <= auto.dmax,...
	        	-auto.amax <= u(2,i) <= auto.amax];
	    end
        
        options = sdpsettings('verbose', 0, 'solver','ipopt');

        optimize(constr, obj, options);

        % Check whether the solution is converging
		if all(abs(double(u(:,1))) <= tol*[1;1])
			disp('Solving Terminated since the solution is not improving anymore.');
			break;
		end
        
        k = k + 1;
        v_sim.z(:, k) = double(z(:,2));
        v_sim.u(:, k-1) = double(u(:,1));

        p = plotcar(v_sim.z(1, k),v_sim.z(2, k),v_sim.z(3, k),v_sim.u(1, k-1),auto,gcf,[0.3 0.3 0.3]);

        current_dis = (full_path(1,:) - v_sim.z(1, end)).^2 +...
				(full_path(2,:) - v_sim.z(2, end)).^2;

	end

end
