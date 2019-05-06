%% car_pos: function description
function [x, y] = car_pos(s, car_idx)
	% Car 1 go straight - curve - straight
	arc_len = 4.5*pi/2;
	switch car_idx
		case 1
			if s <= 3
				x = -6 + s;
				y = -1.5;
				% phi = 0;
			elseif s>3 && s<3+arc_len
				fraction = (s-3) / arc_len;
				phi = fraction * pi/2;
				x = -3 + 4.5*sin(phi);
				y =  3 - 4.5*cos(phi);
			else
				x = 1.5;
				y = 3 + s - (3 + arc_len);
				% phi = pi/2;
			end
		case 2
			% Car 2 go straight horizontally
			% From right to left
			x = 6 - s;
			y = 1.5;
			% phi = pi;
		case 3
			% Car 3 go straight vertically
			% From top to bottom
			x = -1.5;
			y = 6 - s;
			% phi = -pi/2;
			
		otherwise
			x = 0;
			y = 0;
			disp('Car Index is wrong');
	end
