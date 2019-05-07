dt = 0.1;

t = size(Cars{1}.x, 2);

d1 = abs(Cars{1}.x(1, :) - Cars{1}.sc(2));
d2 = abs(Cars{2}.x(1, :) - Cars{2}.sc(1));
d = d1 + d2;
time = dt * (1:t);
plot(time, d, 'linewidth', 2)
hold on;

d1 = abs(Cars{1}.x(1, :) - Cars{1}.sc(3));
d2 = abs(Cars{3}.x(1, :) - Cars{3}.sc(1));
d = d1 + d2;
time = dt * (1:t);
plot(time, d, 'linewidth', 2)

d1 = abs(Cars{2}.x(1, :) - Cars{2}.sc(3));
d2 = abs(Cars{3}.x(1, :) - Cars{3}.sc(2));
d = d1 + d2;
time = dt * (1:t);
plot(time, d, 'linewidth', 2)

margin_y = [4, 4];
margin_x = [dt, dt*t];
plot(margin_x, margin_y, 'k--', 'linewidth', 1)

legend('Vehicle 1 & 2', 'Vehicle 1 & 3', 'Vehicle 2 & 3', 'Safety Margin',...
		'Fontsize', 15)

ylim([0, inf])
xlabel('Time (s)')
ylabel('Distance (m)')
grid on;
title('Inter-vehicle Distance of vehicle pairs - SDP')
set(gca,'FontSize',20, 'Fontname', 'times new roman');

%% Velocity Figure
figure(2)
plot(time, Cars{1}.x(2, :), 'r', 'linewidth', 2)
hold on
plot(time, Cars{2}.x(2, :), 'g', 'linewidth', 2)
plot(time, Cars{3}.x(2, :), 'b', 'linewidth', 2)
legend('Vehicle 1', 'Vehicle 2', 'Vehicle 3')
ylim([0, inf])
xlabel('Time (s)')
ylabel('Speed (m/s)')
grid on;
title('Longitudinal speed of vehicle - SDP')
set(gca,'FontSize',20, 'Fontname', 'times new roman');


