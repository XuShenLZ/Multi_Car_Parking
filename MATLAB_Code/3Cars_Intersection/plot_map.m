%% plot_map: function description
function plot_map()
	figure(1)

	% Plot Road Boundaries
	H_line = [-9,  3;...
			  -3,  3;...
			   3,  3;...
			   9,  3;...
			  -9, -3;...
			  -3, -3;...
			   3, -3;...
			   9, -3];
	for k=1:4
		plot(H_line(2*k-1:2*k, 1), H_line(2*k-1:2*k, 2), 'k', 'LineWidth', 2)
		hold on
	end

	V_line = [-3, 3;...
			  -3, 9;...
			   3, 3;...
			   3, 9;...
			  -3,-9;...
			  -3,-3;...
			   3,-9;...
			   3,-3];
	for k=1:4
		plot(V_line(2*k-1:2*k, 1), V_line(2*k-1:2*k, 2), 'k', 'LineWidth', 2)
	end

	C_line = [-9,  0;...
			  -3,  0;...
			   3,  0;...
			   9,  0;...
			   0, -9;...
			   0, -3;...
			   0,  3;...
			   0,  9];
	for k=1:4
		plot(C_line(2*k-1:2*k, 1), C_line(2*k-1:2*k, 2), 'k--', 'LineWidth', 1)
	end

