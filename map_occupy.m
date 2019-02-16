%% map_occupy: function description
function slots = map_occupy(space, corners, occupancy)
	corners_x = corners.x;
	corners_y = corners.y;
	slots = park_slots(space, corners_x, corners_y, occupancy);
	for i = 1:size(occupancy,1)
		for j = 1:size(occupancy,2)
			if slots{i, j}.occupied
				scatter(slots{i, j}.center(1), slots{i, j}.center(2), 'r','filled')
			else
				scatter(slots{i, j}.center(1), slots{i, j}.center(2), 'r')
			end
		end
	end
end

%% park_slots: function description
function slots = park_slots(space, corners_x, corners_y, occupancy)
	% SPACE is the size of the parking slot
	% CORNER_X is the x value of upper-left corner
	% CORNER_Y is the y value
	% By default, the map is 1-2-1 configuration from up to bottom
	[row, col] = size(occupancy);
	for i = 1:row
		for j = 1:col
			slots{i, j}.center(1) = corners_x(j) + space(1)/2;
			slots{i, j}.center(2) = corners_y(i) - space(2)/2;
			% The dimension of the slot
			slots{i, j}.space = space;
			% Whether there is car or not
			slots{i, j}.occupied = occupancy(i,j);
			if mod(i, 2) ~= 0
				% Odd number of row
				% The slot is open from the up
				slots{i, j}.open = 'up';
			else
				% Even number of row
				% The slot is open from the bottom
				slots{i, j}.open = 'bottom';
			end
		end
	end

end