% Centralized Controller without collision 
% avoidance constraint
clear
Cars = car_init(3);
plot_map()
% Loop parameter
M = 30;
% Horizon Length
N = 20;

% Reference Velocity
v_ref = 5;

% Weights
Q = 1;
R = 1;
S = 1;

% Centralized Problem
for t=1:M
	obj = 0;
	constr = [];

	for i=1:3
		x{i} = sdpvar(2, N+1);
		u{i} = sdpvar(1, N);
		
		% Objective function
		for k=1:N+1
			obj = obj + Q * (x{i}(2, k) - v_ref)^2;

		end

		for k=1:N-1
			obj = obj + R * (u{i}(k+1) - u{i}(k))^2;
		end

		for k=1:N
			obj = obj + S * u{i}(k)^2;
		end

		% Constraints
		constr = [constr, x{i}(:,1) == Cars{i}.x(:,end)];

		for k=1:N
			% System Dynamics
			constr = [constr,...
					x{i}(:,k+1) == dyn_model(x{i}(:,k), u{i}(k))];
			% Speed Constraints
			constr = [constr,...
					Cars{i}.vmin <= x{i}(2,k) <= Cars{i}.vmax];
			% Input Constraints
			constr = [constr,...
					Cars{i}.umin <= u{i}(k) <= Cars{i}.umax];
		end

		constr = [constr,...
				Cars{i}.vmin <= x{i}(2,N+1) <= Cars{i}.vmax];

		% Mean Velocity constraint
		% Mean >= Mean is equvalent to Sum >= Sum
		constr = [constr,...
				sum(x{i}(2,:)) >= Cars{i}.s_out - Cars{i}.x(1,1)];

	end

	% Solve
	options = sdpsettings('verbose', 0, 'solver','quadprog');

	diagnostics = optimize(constr, obj, options)

	for i=1:3
	    xOpt = double(x{i});
	    uOpt = double(u{i});
	    Cars{i}.x = [Cars{i}.x, xOpt(:,2)];
	    Cars{i}.u = [Cars{i}.u, uOpt(1)];
	end

end

% Plot
for t=1:M
	for i=1:3
		plot_map()
		plot_car(Cars, t)
		pause(0.01)
	end
end