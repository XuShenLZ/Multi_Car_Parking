% Centralized Controller without collision 
% avoidance constraint
clear
Cars = car_init(3);
% Horizon Length
N = 30;
% Time interval
dt = 0.1;

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
		disp(['Time=', num2str(t), ', Car=', num2str(i)]);

		if t == 1
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

			% Solve
			options = sdpsettings('verbose', 0, 'solver','ipopt');
			diagnostics = optimize(constr, obj, options)

			xOpt{i} = double(x);
			uOpt = double(u);
			Cars{i}.x = [Cars{i}.x, xOpt{i}(:,2)];
			Cars{i}.u = [Cars{i}.u, uOpt(1)];
		else
			U = sdpvar(N, N);
			u = sdpvar(N, 1);

			x0 = Cars{i}.x(:,end);

			[P0, q0T, Mv_i, v_v0] = sdp_cost(N, x0, dt, v_ref, Q, R, S);

			obj = trace(U*P0) + q0T*u;

			constr = [U u; u' 1] >= 0;

			% Input constraint
			constr = [constr,...
				Cars{i}.umin*ones(N,1) <= u <= Cars{i}.umax*ones(N,1)];

			% State constraint
			constr = [constr,...
				Cars{i}.vmin*ones(N,1) <= dt*Mv_i*u + Mv_i*v_v0 <= Cars{i}.vmax*ones(N,1)];

			% Mean velocity constraint
			constr = [constr,...
					sum(dt*Mv_i*u + Mv_i*v_v0) >= Cars{i}.s_out - Cars{i}.x(1,1)];

			% Collision Avoidance
			for j=1:3
				if j<i
					for k=1:N
						d_ji = abs(s_last{j}(k) - Cars{j}.sc(i));
						if d_ji<d_safe
							[Pl, qlT, rl] = sdp_constr(N, x0, dt, Cars{i}.sc(j), d_ji, d_safe, k);
							constr = [constr,...
								trace(U*Pl) + qlT*u + rl <= 0];
						end
					end
				end
			end

			options = sdpsettings('verbose', 0, 'solver','sdpt3');
			diagnostics = optimize(constr, obj, options)
			UOpt = double(U);
			uOpt = double(u);

			xOpt{i} = x0;
			for k=1:N
				xOpt{i}(:, k+1) = dyn_model(xOpt{i}(:, k), uOpt(k));
			end
			Cars{i}.x = [Cars{i}.x, xOpt{i}(:,2)];
			Cars{i}.u = [Cars{i}.u, uOpt(1)];

		end

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