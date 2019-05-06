% ADMM solution
% avoidance constraint
clear
Cars = car_init(3);
% Horizon Length
N = 35;
% Time interval
dt = 0.1;

% Reference Velocity
v_ref = 5;

% Safety Distance
d_safe = 5;

% Weights
Q = 1;
R = 0.5;
S = 1;

% Penalty
c = 10;

% Time
t = 0;

% Safety constraint
h = @(xi, xj, xi_Bar, xj_Bar, i, j) ...
	abs(xi_Bar - Cars{i}.sc(j)) + abs(xj_Bar - Cars{j}.sc(i)) - d_safe...
	+ (xi_Bar - Cars{i}.sc(j))/abs(xi_Bar - Cars{i}.sc(j)) * (xi - xi_Bar)...
	+ (xj_Bar - Cars{j}.sc(i))/abs(xj_Bar - Cars{j}.sc(i)) * (xj - xj_Bar);

% Initialization
for i=1:3
	lambda{i} = ones(1, N+1);

	wOpt{i}(1) = Cars{i}.x(1,1);
	v = Cars{i}.x(2,1);
	for k=1:N
		wOpt{i}(k+1) = wOpt{i}(k) + dt * v;
	end

	for j=1:3
		% However, the data where i=j is useless
		lambda_i_to_j{i, j} = ones(1, N+1);
		lambda_i_fr_j{i, j} = ones(1, N+1);
		w_i_fr_j{i, j}      = ones(1, N+1);
	end
end

options = sdpsettings('verbose', 0, 'solver','quadprog');

% Decentralized Problem
while is_inside(Cars)
	t = t+1;

	% Prediction Stage
	for i=1:3
		disp(['Prediction, T=', num2str(t), ', Car=', num2str(i)]);

		x = sdpvar(2, N+1);
		u = sdpvar(1, N);

		obj = 0;
		constr = [];
		
		% Objective function
		for k=1:N+1
			% Original J function
			obj = obj + Q * (x(2, k) - v_ref)^2;
			% lambda' (x-w)
			obj = obj + lambda{i}(k) * (x(1, k) - wOpt{i}(k));
			% c/2 * ||x-w||^2
			obj = obj + c/2 * (x(1, k) - wOpt{i}(k))^2;
			for j=1:3
				if ~isequal(i, j)
					obj = obj + lambda_i_fr_j{i, j}(k) * (x(1, k) - w_i_fr_j{i, j}(k));
					obj = obj + c/2 * (x(1, k) - w_i_fr_j{i, j}(k))^2;
				end
			end
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

		% Solve
		% options = sdpsettings('verbose', 0, 'solver','quadprog');
		diagnostics = optimize(constr, obj, options)

		xOpt{i} = double(x);
		uOpt = double(u);
		Cars{i}.x = [Cars{i}.x, xOpt{i}(:,2)];
		Cars{i}.u = [Cars{i}.u, uOpt(1)];
	end

	% Coordination Stage
	for i=1:3
		disp(['Coordination, T=', num2str(t), ', Car=', num2str(i)]);

		w = sdpvar(1, N+1);
		
		for j=1:3
			w_i_to_j{i, j} = sdpvar(1, N+1);
		end

		obj = 0;
		constr = [];
		
		% Objective function
		for k=1:N+1
			% lambda' (x-w)
			obj = obj + lambda{i}(k) * (xOpt{i}(1, k) - w(k));
			% c/2 * ||x-w||^2
			obj = obj + c/2 * (xOpt{i}(1, k) - w(k))^2;
			for j=1:3
				if ~isequal(i, j)
					obj = obj + lambda_i_to_j{i, j}(k) * (xOpt{j}(1, k) - w_i_to_j{i, j}(k));
					obj = obj + c/2 * (xOpt{j}(1, k) - w_i_to_j{i, j}(k))^2;
				end
			end
		end

		% Constraints
		for k=3:N+1
			for j=1:3
				if ~isequal(i, j)
					constr = [constr,...
						h(w(k), w_i_to_j{i, j}(k), xOpt{i}(1, k), xOpt{j}(1, k), i, j) >= 0];
				end
			end
		end

		% Solve
		% options = sdpsettings('verbose', 0, 'solver','quadprog');
		diagnostics = optimize(constr, obj, options)

		wOpt{i} = double(w);
		for j=1:3
			wOpt_i_to_j{i, j} = double(w_i_to_j{i, j});
		end
	end

	% Mediation stage
	for i=1:3
		disp(['Mediation, T=', num2str(t), ', Car=', num2str(i)]);
		lambda{i} = lambda{i} + c * (xOpt{i}(1, :) - wOpt{i});
		for j=1:3
			if ~isequal(i, j)
				lambda_i_to_j{i, j} = lambda_i_to_j{i, j} + c * (xOpt{j}(1, :) - wOpt_i_to_j{i, j});
			end
		end
	end

	% Final Communication
	for i=1:3
		for j=1:3
			lambda_i_fr_j{i, j} = lambda_i_to_j{j, i};
			w_i_fr_j{i, j} = wOpt_i_to_j{j, i};
		end
	end

end

%% Plot
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