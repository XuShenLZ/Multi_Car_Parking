%% random_sol: function description
function u_output = random_sol(mean_u, cov_U, P0, q0T, dt, Mv_i, v_v0, Cars, d_safe, N, i)
	N_sample = 500;
	u_output = mean_u;
	cost_min = Cost(u_output, P0, q0T, dt, Mv_i, v_v0, Cars, d_safe, N, i);

	% R = chol(cov_U);

	for r=1:N_sample
		% Draw sample
		% u = mean_u + R * randn(size(mean_u));
		u = mvnrnd(mean_u', cov_U)';
		c = Cost(u, P0, q0T, dt, Mv_i, v_v0, Cars, d_safe, N, i);
		if c < cost_min
			cost_min = c;
			u_output = u;
		end
	end

%% Cost: function description
function result = Cost(u, P0, q0T, dt, Mv_i, v_v0, Cars, d_safe, N, i)
	result = inf;
	if hard_constr(u, dt, Mv_i, v_v0, Cars, d_safe, N, i)
		% The normal cost
		result = trace(u*u'*P0) + q0T*u;
		x(:,1) = Cars{i}.x(:, end);
		for k=1:N
			x(:,k+1) = dyn_model(x(:,k), u(k));
		end

		e_d = 0;
		for j=1:3
			if j<i
				for k=2:N+1
					d = abs(Cars{j}.s_last(k) - Cars{j}.sc(i)) + abs(x(1,k)-Cars{i}.sc(j));
					if d<d_safe
						e_d = e_d + d_safe - d;
					end
				end
			end
		end

		pd = 1e4;

		disp(e_d);
		result = result + pd * e_d;


	end

%% hard_constr: function description
function result = hard_constr(u, dt, Mv_i, v_v0, Cars, d_safe, N, i)
	result = dt * sum(dt*Mv_i*u + Mv_i*v_v0) >= Cars{i}.s_out - Cars{i}.x(1,end);
	result = result & all(dt*Mv_i*u + Mv_i*v_v0 >= Cars{i}.vmin);
	result = result & all(u <= Cars{i}.umax);
	result = result & all(u >= Cars{i}.umin);