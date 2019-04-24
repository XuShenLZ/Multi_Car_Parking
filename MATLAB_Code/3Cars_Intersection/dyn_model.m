%% dyn_model: function description
% u is the input: accel
% x is the 2-d state: [s; v]
function x1 = dyn_model(x, u)
	dt = 0.1;

	A = [1 dt;...
		 0 1];

	B = [0; dt];

	x1 = A*x + B*u;