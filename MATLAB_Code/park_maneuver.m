%% park_maneuver: coordinate transformation of 
% Parking maneuver
function global_manu = park_maneuver(g_slot, slots)
	center = slots{g_slot(1), g_slot(2)}.center';
	load('park_maneuver.mat')
	% Get the xdata as the global_manu
	global_manu = xdata;
	switch g_slot(1) % Which row it is
		case 1
			R_matrix = eye(2);
			u_park_manu = udata;
			global_manu(3, :) = global_manu(3, :);
		case 2
			R_matrix = [1 0; 0 -1];
			u_park_manu = -udata;
			global_manu(3, :) = -global_manu(3, :);
		case 3
			R_matrix = [-1 0; 0 1];
			u_park_manu = -udata;
			global_manu(3, :) = -global_manu(3, :) + pi;
		otherwise
			R_matrix = [cosd(180) -sind(180); sind(180) cosd(180)];
			u_park_manu = udata;
			global_manu(3, :) = global_manu(3, :) + pi;
	end
	global_manu(1:2, :) = R_matrix * xdata(1:2, :);
	global_manu(1:2, :) = global_manu(1:2, :) + center;
end