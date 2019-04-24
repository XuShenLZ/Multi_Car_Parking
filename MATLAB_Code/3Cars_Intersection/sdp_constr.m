%% sdp_constr: function description
% N: Horizon Length, 1x1
% x0: Initial Value, [s0; v0]
% dt: Time interval, 1x1
% s_cl: The collision point, 1x1
% d_ci: The predicted distance of agent l to collision point, 1x1
% d_safe: The safty distance, 1x1
% k: current time step
function [Pl, qlT, rl] = sdp_constr(N, x0, dt, s_cl, d_ci, d_safe, k)
	s0 = x0(1);
	v0 = x0(2);

	%% Safty constraint
	dia_Ms_1 =  ones(1, N);
	dia_Ms_2 = -ones(1, N-1);
	Ms = diag(dia_Ms_1, 0) + diag(dia_Ms_2, -1);
	Ms_i = inv(Ms);
	Mv_i = Ms_i;

	v_v0   = [v0; zeros(N-1, 1)];
	
	dia_Msv = ones(1, N-1);
	Msv = diag(dia_Msv, -1);

	v_s0 = [s0; zeros(N-1, 1)];

	Pl  = zeros(N, N);
	qlT = zeros(1, N);
	rl  = 0;

	Mk = zeros(N);
	Mk(k, k) = 1;
	ek = zeros(N, 1);
	ek(k) = 1;

	Pl =...
		- (dt^2 * Ms_i * Msv * Mv_i)' *...
		Mk'*Mk *...
		(dt^2 * Ms_i * Msv * Mv_i);

	qlT =...
		- 2*(dt * Ms_i * Msv * Mv_i * v_v0 + dt * Ms_i * v_v0 + Ms_i * v_s0)' *...
		Mk'*Mk *...
		(dt^2 * Ms_i * Msv * Mv_i) +...
		2*s_cl*ek' *...
		(dt^2 * Ms_i * Msv * Mv_i);

	rl =...
		- (dt * Ms_i * Msv * Mv_i * v_v0 + dt * Ms_i * v_v0 + Ms_i * v_s0)' *...
		Mk'*Mk *...
		(dt * Ms_i * Msv * Mv_i * v_v0 + dt * Ms_i * v_v0 + Ms_i * v_s0) +...
		2*s_cl*ek' *...
		(dt * Ms_i * Msv * Mv_i * v_v0 + dt * Ms_i * v_v0 + Ms_i * v_s0) +...
		(d_safe - d_ci)^2 - s_cl^2;

