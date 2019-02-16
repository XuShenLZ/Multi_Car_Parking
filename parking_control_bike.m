clear all
clear yalmip
%Car Parameters
% auto.df = 1.7;
% auto.db = 1.2;
% auto.lf = 1.1;
% auto.df = 1.0;
% auto.d = auto.db + auto.df;
% auto.l = auto.df + auto.lf;
% auto.tyr = 0.8;
% auto.w=2;
% auto.dmax=25*pi/180;
auto.w = 2.0;                         % car width [m]
auto.db = 1.2;                        % rear axis position, from back [m]
auto.df = 1.0;                        % front axis position, from front [m]
auto.d = 2.8;                         % axel distance [m] %L in the midterm
auto.l = auto.d + auto.df + auto.db;  % car length [m]
auto.tyr = 0.8;                       % tyre diameter [m]
auto.dmax = 45*pi/180;                % maximum front wheel steering angle [rad]
auto.drat = 14.5;                     % ratio between steering wheel angle and front wheel angle
auto.vmax = 5;
auto.amax = 0.2*9.8;

%Ego car G and g
G = [-1 0; 1 0;0 -1; 0 1];
g = [0; auto.l; auto.w/2; auto.w/2];

% umin(2)=-pi;
% umin(1)=auto.l / tan(auto.dmax);

%% Constraints:
umin(1) = -auto.dmax;
umax(1) = auto.dmax;

% umin(2) = -5;
% umax(2) = 5;
umin(2) = -auto.amax;
umax(2) = auto.amax;

zmin = [-20; 0;-2*pi;0];
zmax = [20;20;2*pi;0];

z0 = [-6;7.25;0;0];
zT = [0;auto.db;-pi/2;0];

thetaT = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles
%Obstacle list
i=1;
obs{i}.center=[-4.5;0];
obs{i}.LW=[4.5;6];
obs{i}.theta=0/180; %(in radiants)
i=i+1;
obs{i}.center=[4.5;0];
obs{i}.LW=[4.5;6];
obs{i}.theta=0/180; %(in radiants)
% i=i+1;
% obs{i}.center=[0;14];
% obs{i}.LW=[1;8];
% obs{i}.theta=0/180; %(in radiants)


%% some obtacle postprocessing  and variable defintion


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Navigation control
N = 30;


for j=1:length(obs)
    t = obs{j}.theta;
    % generate T matrix for each obstacle
    obs{j}.T = [cos(t) -sin(t); sin(t) cos(t)]*diag(obs{j}.LW/2);
    % polyehdral representaion
    obs{j}.poly = obs{j}.T*unitbox(2)+obs{j}.center;
    [AA{j},bb{j}] = double(obs{j}.poly);
    lambda{j} = sdpvar(size(AA{j},1),N,'full');
    mu{j} = sdpvar(size(G,1),N,'full');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup the CFTOC
x = sdpvar(4,N+1); % [X;Y;psi;v]
u = sdpvar(2,N); % [delta; a]
%set terminal constraint
constr = [x(:,N+1)==zT, x(:,1) == z0];
constr = constr + [u(1,N) == thetaT];
%set terminal cost
cost = 0;
slw = sdpvar(length(obs),N);
dt = 0.2;

for k = 1:N-1
	cost = cost + 2*(u(:,k+1) - u(:,k))' * (u(:,k+1) - u(:,k));
end

for k = 1:N
	constr = constr+...
		[x(1,k+1) == x(1,k) + dt * x(4,k) * cos(x(3,k)),...    
        x(2,k+1) == x(2,k) + dt * x(4,k) * sin(x(3,k)),...
        x(3,k+1) == x(3,k) + dt * x(4,k) * tan(u(1,k)) / auto.l,...
        x(4,k+1) == x(4,k) + dt * u(2,k),...
        0 <= x(4,k) <= auto.vmax,...
        umin(1) <= u(1,k) <= umax(1),...
        umin(2) <= u(2,k) <= umax(2)];
        %model.x.min <= x(:,k+1),x(:,k+1)<=model.x.max];
	cost = cost + u(:,k)' * u(:,k);
	
	Rx = [cos(x(3,k)), -sin(x(3,k));sin(x(3,k)) cos(x(3,k))];
	for j = 1:length(obs)
		% constr = constr + [(AA{j}*...
		% 		[x(1,k);...
		% 		x(2,k)]...
		% 		-bb{j})'*lambda{j}(:,k)>=0.2];
		% constr = constr + [lambda{j}(:,k)'*AA{j}*(AA{j})'*lambda{j}(:,k)<=1]; 
		% constr = constr + [lambda{j}(:,k)>=0];
		% rep = 4;
		% for jr = 1:rep
		% 	% xsim(1, :) = x(1,k)-u(1,k)*sin(x(3,k))+u(1,k)*sin(x(3,k)+jr/rep*u(2,k));
		% 	% xsim(2, :) = x(2,k)+u(1,k)*cos(x(3,k))-u(1,k)*cos(x(3,k)+jr/rep*u(2,k));
		% 	% xsim(3, :) = x(3,k)+ jr/rep*u(2,k);
		% 	constr = constr + [(AA{j}*...
		% 		[x(1,k)-u(1,k)*sin(x(3,k))+u(1,k)*sin(x(3,k)+jr/rep*u(2,k));...
		% 		x(2,k)+u(1,k)*cos(x(3,k))-u(1,k)*cos(x(3,k)+jr/rep*u(2,k))]...
		% 		-bb{j})'*lambda{j}(:,k)>=0.2];
		% constr = constr + [-g'*mu{j}(:,k)+(AA{j}*[x(1,k);x(2,k)]-bb{j})'*lambda{j}(:,k)>= slw(j,t)+0.1];
		constr = constr + [-g'*mu{j}(:,k)+(AA{j}*...
			[x(1,k);x(2,k)]-bb{j})'*lambda{j}(:,k)>= 0.1];

		constr = constr + [G'*mu{j}(:,k)+Rx'*(AA{j})'*lambda{j}(:,k)==0];
		constr = constr + [lambda{j}(:,k)'*AA{j}*(AA{j})'*lambda{j}(:,k)<=1];
		constr = constr + [lambda{j}(:,k)>=0];
		constr = constr + [mu{j}(:,k)>=0];
		% constr = constr + [slw(j,k)>=0];
		% cost=cost+slw(j,k);
		% end
	end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Run MPC
%load data xinit uinit
% load('init.mat')
% assign(x,xinit);
% assign(u,uinit);
%assign(lambda{1},zeros(2,5));

ifusex0 = 0;
options = sdpsettings('solver','ipopt','usex0', ifusex0);
optimize(constr,cost,options)
xdata = double(x);
udata = double(u);
% if ~ifusex0
% 	xinit = xdata;
% 	uinit = udata;
% 	save('init.mat', 'xinit', 'uinit');
% end
	
%Plot Open Loop
figure
%plot obstacles
for j=1:length(obs)
	plot(polytope(AA{j},bb{j}));
	hold on;
end
plot(xdata(1,:),xdata(2,:),'or')
axis equal
car_plot_bike(auto, xdata, udata, z0)
