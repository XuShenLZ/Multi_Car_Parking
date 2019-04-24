function car_plot_bike(auto, x, u, z0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the motion of a car according to the kinematic point model:
% {x}_{k+1} &=& x_k-R_k*sin(\theta_k)+R_k*sin(\theta_k+\beta_k) 
% {y}_{k+1} &=& y_k+R_k*cos(\theta_k)-R_k*cos(\theta_k+\beta_k) 
% {\theta}_{k+1} &=& \theta_k+\beta_k 
%---------------------------------------------------
% with inputs u=[R_k,\beta_k] and state z=[z,y,\theta]
% u is  2xN matrix where u(:,k) are the two inputs at time k
% z0 is a 3x1 vector of initial conditions.
%--------------------------------------------
% In the plot, red wheels are the front vehicle wheels

% Sampling parameter
% rep = 40;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation of a simple kinematic vehicle

% xsim = [];
% usim = [];

N = size(u,2);

% xsim(:,1) = z0;
% dt = 0.1;

% for k = 1:N
%     xsim(1,k+1) = xsim(1,k) + dt * xsim(4,k) * cos(xsim(3,k));    
%     xsim(2,k+1) = xsim(2,k) + dt * xsim(4,k) * sin(xsim(3,k));
%     xsim(3,k+1) = xsim(3,k) + dt * xsim(4,k) * tan(u(1,k)) / auto.l;
%     xsim(4,k+1) = xsim(4,k) + dt * u(2,k);
% end

xsim = x;
usim = u;
usim(:,N+1) = usim(:,N);

X = xsim';
U = usim';

fig = gcf;
hold on

% Trajectory style
plottraj.linestyle = '-';
plottraj.linewidth = 1;
plottraj.color = [0 0 1]; % RGB value
plottraj.marker = 'none';

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Vehicle trajectory (oversampled, to show the path)
for i = 1:N+1
    % Check whether the inputs are actual turning inputs
    delta = U(i,1);

    % Plotcar is the provided toolbox function
    p = plotcar(X(i,1),X(i,2),X(i,3),delta,auto,fig,[0.3 0.3 0.3]);
    % pause(0.002)
    pause(0.005)
    if i<=N & i>1
        % delete(p)
    end
    if i<N
        plot([X(i,1),X(i+1,1)],[X(i,2),X(i+1,2)],plottraj);
    end
end
% Mark all points in optimization result
for i = 1:N+1
    plot(X(i,1),X(i,2),'go')
    [u,v] = pol2cart(X(i,3),1);
    quiver(X(i,1),X(i,2),u,v,'g','linewidth',2,'maxheadsize',2);
end

%%%%%%
% Plot model states


%% Attribution
% ME C231A and EECS C220B, UC Berkeley, Fall 2016