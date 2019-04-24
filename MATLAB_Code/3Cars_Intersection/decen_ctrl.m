% Centralized Controller without collision 
% avoidance constraint
clear
Cars = car_init(3);
plot_map()
% Horizon Length
N = 30;

% Reference Velocity
v_ref = 5;

% Safety Distance
d_safe = 5;

% Weights
Q = 1;
R = 1;
S = 1;

% Time
t = 0;

% Decentralized Problem
while is_inside(Cars)
	t = t+1;

	for i=1:3
		x = sdpvar(2, N+1);
		u = sdpvar(1, N);

		obj = 0;
		constr = [];
		
		% Objective function
		for k=1:N+1
			obj = obj + Q * (x(2, k) - v_ref)^2;

		end

		for k=1:N-1
			obj = obj + R * (u(k+1) - u(k))^2;
		end

		for k=1:N
			obj = obj + S * u(k)^2;
		end

		% Constraints
		constr = [constr, x(:,1) == Cars{i}.x(:,end)];

		for k=1:N
			% System Dynamics
			constr = [constr,...
					x(:,k+1) == dyn_model(x(:,k), u(k))];
			% Speed Constraints
			constr = [constr,...
					Cars{i}.vmin <= x(2,k) <= Cars{i}.vmax];
			% Input Constraints
			constr = [constr,...
					Cars{i}.umin <= u(k) <= Cars{i}.umax];
		end

		constr = [constr,...
				Cars{i}.vmin <= x(2,N+1) <= Cars{i}.vmax];

		% Mean Velocity constraint
		% Mean >= Mean is equvalent to Sum >= Sum
		constr = [constr,...
				sum(x(2,:)) >= Cars{i}.s_out - Cars{i}.x(1,1)];

		% Collision Avoidance
		for k=1:N
			% The first time of MPC doesn't add this constraint
			if t==1
				break
			end

			for j=1:3
				if j<i
				% if ~eq(i, j)
					d_ji = abs(s_last{j}(k) - Cars{j}.sc(i));
					if d_ji<d_safe
						constr = [constr,...
							(x(1,k)-Cars{i}.sc(j))^2 >= (d_safe-d_ji)^2];
					end
				end
			end
		end

		% Solve
		disp(['Time=', num2str(t), ', Car=', num2str(i)]);
		options = sdpsettings('verbose', 0, 'solver','ipopt');
		diagnostics = optimize(constr, obj, options)

		xOpt{i} = double(x);
	    uOpt = double(u);
	    Cars{i}.x = [Cars{i}.x, xOpt{i}(:,2)];
	    Cars{i}.u = [Cars{i}.u, uOpt(1)];

	end

	for i=1:3
	    s_last{i} = xOpt{i}(1, 2:end);
	end
end

% Plot
for t0=1:t
	for i=1:3
		plot_map()
		plot_car(Cars, t0)
		pause(0.01)
	end
end

%% is_out: whether the cars are inside the conflict region
function flag = is_inside(Cars)
	flag = false;
	for i=1:3
		flag = flag || (Cars{i}.x(1,end)<=Cars{i}.s_out);
	end
end