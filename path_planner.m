function [zdata, udata] = path_planner(auto, routePlan, factor)

% Every interval [start, end) has 'factor' points
% The total number of points = N+1
N = factor*length(routePlan.StartPose(:,1));

%% Input constraints
% Model: Kinematic Point Model
umin(2) = 0;
umin(1) = auto.l / tan(auto.dmax);

%Initial, terminal conditions
z0 = routePlan.StartPose(1,:)';
zT = routePlan.EndPose(N/factor,:)';

%% Setup the Navigation Problem
options = sdpsettings('solver','ipopt');
%options = sdpsettings('solver','fmincon','verbose',1);

% Total number of points = N+1
z = sdpvar(3,N+1);
% Total number of control inputs = N
% u(1): turn radius
% u(2): turn angle
u = sdpvar(2,N);
slk = sdpvar(3,N/factor);

constr = [z(:,1) == z0];
cost = 0;

% Loop through all routePlan points
for k = 1:N/factor
    % Check if the starting and end points have same heading
    if routePlan.StartPose(k,3)~=routePlan.EndPose(k,3)
        % If the headings are different
        % The control need to consider the turning
        for l = 2*(k-1)+1:2*(k-1)+factor
            constr = constr+...
            [z(1,l+1) == z(1,l)-u(1,l)*sin(z(3,l))+u(1,l)*sin(z(3,l)+u(2,l)),...    
            z(2,l+1) == z(2,l)+u(1,l)*cos(z(3,l))-u(1,l)*cos(z(3,l)+u(2,l)),...
            z(3,l+1) == z(3,l)+u(2,l),...
            umin(1) <= u(1,l), umin(2) <= u(2,l),...
            u(2,l) <= 2*pi];
        end
        constr = constr+[z(:,l+1) == routePlan.EndPose(k,:)'];
    else
        % If the headings are the same
        for l = 2*(k-1)+1:2*(k-1)+factor
            p = l-2*(k-1);
            % The difference between starting and end pose
            dstep = routePlan.EndPose(k,1:2)'-routePlan.StartPose(k,1:2)';
            dstep3 = [dstep;0];
            % Let u(1)=1, and let u(2) be the length of each section
            % Here the u is not [turn radius, turn angle]
            constr = constr+[u(1,l)==0, u(2,l)==(norm(dstep,2)/factor)];
            % z is evenly distributed from start pose to end pose
            constr = constr+[z(:,l+1)==(routePlan.StartPose(k,1:3)'+(p/factor)*dstep3)];
        end
    end
    % cost = cost + 0.1*slk(:,k)'*slk(:,k);

    for l=2*(k-1)+1:2*(k-1)+factor
        % If there is a turn, Angle * Raduis = Arc Length
        % Penalizing the arc length
        % If the path section is straight, there is no way 
        % to optimize
        cost = cost + (u(2,l)*u(1,l))^2;
    end
end

%% Compute Navigation Solution
diagonistic = optimize(constr,cost,options)
zdata = double(z);
udata = double(u);

end