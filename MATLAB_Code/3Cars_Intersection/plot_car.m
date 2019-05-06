%% plot_car: function description
% plot car position given 
function plot_car(Cars, time)
	% Car 1 go straight - curve - straight
	arc_len = 4.5*pi/2;
	if Cars{1}.x(1, time) <= 3
		x = -6 + Cars{1}.x(1, time);
		y = -1.5;
		x0 = [x;y];
		phi = 0;
		carBox(Cars{1}, 1, x0, phi)
	elseif Cars{1}.x(1, time)>3 && Cars{1}.x(1, time)<3+arc_len
		fraction = (Cars{1}.x(1, time)-3) / arc_len;
		phi = fraction * pi/2;
		x = -3 + 4.5*sin(phi);
		y =  3 - 4.5*cos(phi);
		x0 = [x;y];
		carBox(Cars{1}, 1, x0, phi)
	else
		x = 1.5;
		y = 3 + Cars{1}.x(1, time) - (3 + arc_len);
		x0 = [x;y];
		phi = pi/2;
		carBox(Cars{1}, 1, x0, phi)
	end

	% Car 2 go straight horizontally
	% From right to left
	x = 6 - Cars{2}.x(1, time);
	y = 1.5;
	x0 = [x;y];
	phi = pi;
	carBox(Cars{2}, 2, x0, phi)

	% Car 3 go straight vertically
	% From top to bottom
	x = -1.5;
	y = 6 - Cars{3}.x(1, time);
	x0 = [x;y];
	phi = -pi/2;
	carBox(Cars{3}, 3, x0, phi)

	hold off
	xlim([-10 10])
	ylim([-10 10])

%% carBox: function description
function carBox(car, car_num, x0, phi)
	w = car.W / 2;
	l = car.L / 2;
	car1 = x0 + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0 + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0 - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0 - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    colors = 'rgb';
    plot([car1(1),car2(1),car4(1),car3(1),car1(1)],[car1(2),car2(2),car4(2),car3(2),car1(2)], colors(car_num))
    x = [car1(1),car2(1),car4(1),car3(1)];
    y = [car1(2),car2(2),car4(2),car3(2)];
    fill(x,y,colors(car_num))
