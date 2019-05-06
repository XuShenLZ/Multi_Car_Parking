% ADMM solution and speed up use optimizer method
%% Clear
clear
%% Init
Cars = car_init(3);
% Horizon Length
N = 35;
% Time interval
dt = 0.1;

% Reference Velocity
v_ref = 5;

% Safety Distance
d_safe = 2;

% Weights
Q = 1;
R = 0.5;
S = 1;

% Penalty
c_penalty = 100;

% Time
t = 0;

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

%% Optimizer Setup
% Prediction Stage Optimizer Setup
for i=1:3

	x = sdpvar(2, N+1);
	u = sdpvar(1, N);

	x0 = sdpvar(2, 1);
	lam = sdpvar(1, N+1);
	lam_i4j = sdpvar(3, N+1);
	w_star = sdpvar(1, N+1);
	w_i4j = sdpvar(3, N+1);
	c = sdpvar(1,1);

	obj = 0;
	constr = [];
	
	% Objective function
	for k=1:N+1
		% Original J function
		obj = obj + Q * (x(2, k) - v_ref)^2;
		% lambda' (x-w)
		obj = obj + lam(k) * (x(1, k) - w_star(k));
		% c/2 * ||x-w||^2
		obj = obj + c/2 * (x(1, k) - w_star(k))^2;
		for j=1:3
			if ~isequal(i, j)
			% if j<i
				obj = obj + lam_i4j(j, k) * (x(1, k) - w_i4j(j, k));
				obj = obj + c/2 * (x(1, k) - w_i4j(j, k))^2;
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
	constr = [constr, x(:,1) == x0];

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
			dt * sum(x(2,:)) >= Cars{i}.s_out - x0(1)];

	% Solve
	% options = sdpsettings('verbose', 0, 'solver','quadprog');
	Predictor{i} = optimizer(constr, obj, options,...
	 {c, x0, lam, lam_i4j, w_star, w_i4j}, {x, u});
end

% Coordination Stage Optimizer Setup
for i=1:3

	w = sdpvar(1, N+1);
	w_i_to_j = sdpvar(3, N+1);

	lam = sdpvar(1, N+1);
	lam_i2j = sdpvar(3, N+1);
	x = sdpvar(3, N+1);
	x_c = sdpvar(3, N+1);
	d = sdpvar(1,1);
	c = sdpvar(1,1);

	obj = 0;
	constr = [];
	
	% Objective function
	for k=1:N+1
		% lambda' (x-w)
		obj = obj + lam(k) * (x(i, k) - w(k));
		% c/2 * ||x-w||^2
		obj = obj + c/2 * (x(i, k) - w(k))^2;
		for j=1:3
			if ~isequal(i, j)
			% if j<i
				obj = obj + lam_i2j(j, k) * (x(j, k) - w_i_to_j(j, k));
				obj = obj + c/2 * (x(j, k) - w_i_to_j(j, k))^2;
			end
		end
	end

	% Constraints
	for k=3:N+1
		for j=1:3
			if ~isequal(i, j)
			% if j<i
				% constr = [constr,...
				% 	h(w(k), w_i_to_j(j, k), x(i, k), x(j, k), i, j) >= 0];
				constr = [constr,...
					x_c(i, k) + x_c(j, k) - d...
					+ (x(i, k) - Cars{i}.sc(j))/x_c(i, k) * (w(k) - x(i, k))...
					+ (x(j, k) - Cars{j}.sc(i))/x_c(j, k) * (w_i_to_j(j, k) - x(j, k)) >= 0];
			end
		end
	end

	% Solve
	% options = sdpsettings('verbose', 0, 'solver','quadprog');
	Coordinator{i} = optimizer(constr, obj, options,...
		{d, c, lam, lam_i2j, x, x_c}, {w, w_i_to_j});

end

%% Decentralized Problem
while is_inside(Cars)
	t = t+1;

	% Prediction Stage
	for i=1:3
		disp(['Prediction, T=', num2str(t), ', Car=', num2str(i)]);
		x0 = Cars{i}.x(:,end);
		lam = lambda{i};
		for j=1:3
			lam_i4j(j, :) = lambda_i_fr_j{i, j};
			w_i4j(j, :) = w_i_fr_j{i, j};
		end
		w_star = wOpt{i};
		[sol, errorcode] = Predictor{i}(c_penalty, x0, lam, lam_i4j, w_star, w_i4j);
		xOpt{i} = sol{1};
		uOpt = sol{2};
		Cars{i}.x = [Cars{i}.x, xOpt{i}(:,2)];
		Cars{i}.u = [Cars{i}.u, uOpt(1)];
	end

	% Coordination Stage
	for i=1:3
		disp(['Coordination, T=', num2str(t), ', Car=', num2str(i)]);
		lam = lambda{i};
		for j=1:3
			lam_i2j(j, :) = lambda_i_to_j{i, j};
			x(j, :) = xOpt{j}(1, :);
			if j==i
				x_c(j, :) = abs(x(i, :) - Cars{i}.sc(j));
			else
				x_c(j, :) = abs(x(j, :) - Cars{j}.sc(i));
			end
		end

		[sol, errorcode] = Coordinator{i}(d_safe, c_penalty, lam, lam_i2j, x, x_c);		

		wOpt{i} = sol{1};
		for j=1:3
			if j==i
				wOpt_i_to_j{i, j} = zeros(1, N+1);
			else
				wOpt_i_to_j{i, j} = sol{2}(j, :);
			end
		end
	end

	% Mediation stage
	for i=1:3
		disp(['Mediation, T=', num2str(t), ', Car=', num2str(i)]);
		lambda{i} = lambda{i} + c_penalty * (xOpt{i}(1, :) - wOpt{i});
		for j=1:3
			if ~isequal(i, j)
				lambda_i_to_j{i, j} = lambda_i_to_j{i, j} + c_penalty * (xOpt{j}(1, :) - wOpt_i_to_j{i, j});
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