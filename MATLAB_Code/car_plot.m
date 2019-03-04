function car_plot(auto, u, z0)
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
rep = 40;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation of a simple kinematic vehicle
t = 1;
xsim = [];
usim = [];
xsim(:,1) = z0;
N = size(u,2);
for k = 1:N        
    for j = 1:rep
        t = t + 1;
        if u(1,k)~=0
            % When the car is turning
            % Use the kinematic point model
            xsim(1,t) = xsim(1,t-1) - u(1,k)*sin(xsim(3,t-1)) + u(1,k)*sin(xsim(3,t-1)+1/rep*u(2,k));
            xsim(2,t) = xsim(2,t-1) + u(1,k)*cos(xsim(3,t-1)) - u(1,k)*cos(xsim(3,t-1)+1/rep*u(2,k));
            xsim(3,t) = xsim(3,t-1) + 1/rep*u(2,k);
            usim(:,t-1) = [u(1,k); 1/rep*u(2,k)];
        else
            % When the road is a straight line
            % Simply add every segment iteratively
            xsim(1,t) = xsim(1,t-1) + 1/rep*u(2,k)*cos(xsim(3,t-1));
            xsim(2,t) = xsim(2,t-1) + 1/rep*u(2,k)*sin(xsim(3,t-1));
            xsim(3,t) = xsim(3,t-1);
            usim(:,t-1) = [u(1,k); 1/rep*u(2,k)];
        end
    end
end

% This duplication is set to prevent the error in plotting
% When plotting, there is 't' positions in xsim so there 
% should also be 't' steering angles, which is stored in usim
usim(:,t) = usim(:,t-1);
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
for i = 1:N*rep+1
    % Check whether the inputs are actual turning inputs
    if U(i,1) ~= 0
        % If U(i, :) is [radius, angle]
        % Set the steering angle approximately equal to turning angle
        delta = U(i,2);
    else
        % If the car is going straight
        % U(i, :) is not actual [radius, angle] for kinematic point model
        % Steering angle is apparently 0
        delta = 0;
    end
    % Plotcar is the provided toolbox function
    p = plotcar(X(i,1),X(i,2),X(i,3),delta,auto,fig,[0.3 0.3 0.3]);
    pause(0.002)
    if i<=N*rep & i>1
        delete(p)
    end
    if i<N*rep
        plot([X(i,1),X(i+1,1)],[X(i,2),X(i+1,2)],plottraj);
    end
end
% Mark all points in optimization result
for i = 1:rep:N*rep+1
    plot(X(i,1),X(i,2),'go')
    [u,v] = pol2cart(X(i,3),1);
    quiver(X(i,1),X(i,2),u,v,'g','linewidth',2,'maxheadsize',2);
end

%%%%%%
% Plot model states


%% Attribution
% ME C231A and EECS C220B, UC Berkeley, Fall 2016