%% sdp_cost: function description
% N: Horizon Length, 1x1
% x0: Initial Value, [s0; v0]
% dt: Time interval, 1x1
% v_ref: ref speed, 1x1
% Q, R, S: Weights, 1x1
function [P0, q0T, Mv_i, v_v0] = sdp_cost(N, x0, dt, v_ref, Q, R, S)
	s0 = x0(1);
	v0 = x0(2);

	%% Cost function
	% Quadratic term
	dia_Mv_1 =  ones(1, N);
	dia_Mv_2 = -ones(1, N-1);
	Mv = diag(dia_Mv_1, 0) + diag(dia_Mv_2, -1);
	Mv_i = inv(Mv);

	dia_Mdu_1 = -ones(1, N-1);
	Mdu_1 = [diag(dia_Mdu_1) zeros(N-1, 1)];
	dia_Mdu_2 =  ones(1, N-1);
	Mdu_2 = [zeros(N-1, 1) diag(dia_Mdu_2)];
	Mdu = Mdu_1 + Mdu_2;

	P0 = Q * dt^2 * Mv_i' * Mv_i +...
		 R * Mdu' * Mdu +...
		 S * eye(N);

	% Linear term
	v_v0   = [v0; zeros(N-1, 1)];
	v_vref = v_ref * ones(N, 1);

	q0T = 2 * Q * (Mv_i' * v_v0)' * dt * Mv_i +...
		  -2 * Q * v_vref' * dt * Mv_i;

	% Constant term
	% Can be omitted

