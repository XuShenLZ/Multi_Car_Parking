% Initialization of car objects
%% car_init: function description
function Cars = car_init(num)
	% Initialization
	Cars = {};

	% Dimension
	L = 2.87;
	W = 1.85;

	for i=1:num
		Cars{i}.L = L;
		Cars{i}.W = W;
	end
	
	% Initial Conditions
	v0   = 5;
	vmin = 0;
	vmax = 8;
	umin = -9;
	umax =  5;

	% Cars-specific parameters
	Cars{1}.s = 0;
	Cars{2}.s = -1;
	Cars{3}.s = -3;

	for i=1:num
		Cars{i}.v    = v0;
		Cars{i}.umin = umin;
		Cars{i}.umax = umax;
		Cars{i}.vmin = vmin;
		Cars{i}.vmax = vmax;
		Cars{i}.x    = [Cars{i}.s; Cars{i}.v];
		Cars{i}.u    = [];
	end

	Cars{1}.s_in  = 3;
	Cars{1}.s_out = 3 + 4.5*pi/2;
	Cars{2}.s_in  = 3;
	Cars{2}.s_out = 3 + 6;
	Cars{3}.s_in  = 3;
	Cars{3}.s_out = 3 + 6;

	% The collision points
	Cars{1}.sc = [0, 0, 0];
	Cars{1}.sc(2) = 3 + acos(1/3)*4.5;
	Cars{1}.sc(3) = 3 + asin(1/3)*4.5;

	Cars{2}.sc = [0, 0, 0];
	Cars{2}.sc(1) = 9 - sqrt(4.5^2 - 1.5^2);
	Cars{2}.sc(3) = 9 - 1.5;

	Cars{3}.sc = [0, 0, 0];
	Cars{3}.sc(1) = 9 - sqrt(4.5^2 - 1.5^2);
	Cars{3}.sc(2) = 3 + 1.5;
