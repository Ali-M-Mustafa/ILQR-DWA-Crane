function [] = dwa_crane_optimized(map_id)
% Crane-adapted Dynamic Window Approach with the DE-tuned integral LQR crane.
% dwa_crane_optimized(1) -> map 1 (Fig. 7)
% dwa_crane_optimized(2) -> map 2 (Fig. 8)   [default]
%
% Adapted from DynamicWindowApproachSample.m,
% Copyright (c) 2014 Atsushi Sakai, Modified BSD License.
% https://github.com/AtsushiSakai/MATLABRobotics
if nargin < 1
    map_id = 2;
end

close all;
clc;

disp('Dynamic Window Approach sample program start!!')

global dt z01 x01; 
dt=0.1;% Time step [s]
z01 = zeros(2,1);       % Initial integrator state
x01 = zeros(8,1);       % Initial system state

% Initial state: [x; y; v_x; v_y]
x = [0; 0; 0; 0];

goal=[20,20];% Goal position [x(m),y(m)]

% Obstacle list [x(m) y(m)]
% obstacle=[0 2;
%           4 2;
%           4 4;
%           5 4;
%           5 5;
%           5 6;
%           5 9
%           8 9
%           7 9];
% map1 = [4 8; 7 10; 10 10; 12 6; 12 12; 14 2; 14 10];
%map2 = [4 4; 6 6; 4 8; 7 10; 10 10; 12 6; 12 12; 14 2; 14 10]; 
%map3 = [4 4; 4 8; 6 6; 8 4; 7 10; 10 6; 10 10; 12 12; 12 6; 14 2; 14 10; 16 12; 16 16; 18 16];
% map4 = [4 4; 4 5; 4 8; 6 6; 6.5 5.5; 7 5; 7.5 4.5; 8 4; 7 10; 8 8; 10 6; 10 10; 12 6; 12 12; 14 2; 14 10; 16 12;16.5 12; 17 12; 17.5 12; 18 12; 18.5 12; 19 12; 19.5 12; 20 12; 20 11.5; 20 11; 20 10.5; 20 10; 20 9.5; 18 6; 18 16; 16 16];
map = [
    4 4; 4 5; 4 8; 6 6; 6.5 5.5; 7 5; 7.5 4.5; 8 4;
    7 10; 8 8; 10 6; 10 10; 12 6; 12 12; 13 2.5; 14 10;
    18 6; 18 16; 16 16; 
    % New obstacles forming the green path
    10 15; 10.5 15; 11 15; 11.5 15; 12 15; 12.5 15; 13 15;
    0 12; 0.5 12; 1 12; 1.5 12; 2 12; 2.5 12; 3 12;
    13 15.5; 13 16; 13 16.5; 13 17; 13 0;
    13 .5; 13 1; 13 1.5; 13 2
];

% Map 1, read from Fig. 7 of the paper (same points as map1 above).
% Fig. 7 shows 7 obstacles; the "9" in the paper text is a typo.
map_fig7 = [4 8; 7 10; 10 10; 12 6; 12 12; 14 2; 14 10];
% Map 2, Fig. 8 of the paper (= map above).
map_fig8 = map;

switch map_id
    case 1
        obstacle = map_fig7;
    case 2
        obstacle = map_fig8;
    otherwise
        error('map_id must be 1 (Fig. 7) or 2 (Fig. 8)');
end


obstacleR=0.5;% Radius of obstacles for collision detection

% [max_speed, max_acceleration, speed_resolution, acceleration_resolution]
Kinematic = [1.0, 1.0, 0.02, 0.02];

% Evaluation function parameters [heading,dist,velocity,predictDT]
evalParam=[0.8,0.6,0.5,2.0];

area=[-1 20 -1 20];% Simulation area size [xmin xmax ymin ymax]

% Simulation results
result.x=[];
result.x1=[];
result.x2=[];
% selected_path = []; % To store selected path points
tic;
%movcount=0;

% prev_pos = [0;0];

for i=1:5000
    % Calculate input values using DWA
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    [x, crane_pos1]=f(x,u);

    % % % 3) Fuse back: overwrite x with the true position and estimated velocity
    % if i == 1
    %     % no previous pos yet, assume zero velocity
    %     vel_est = [0;0];
    % else
    %     vel_est = (crane_pos - prev_pos) / dt;
    % end
    % % 
    % 
    % x = [ crane_pos(1);
    %       crane_pos(2);
    %       u(1);
    %       u(2) ];
    % % 
    % prev_pos = crane_pos;
    % 
    % Save simulation results
    result.x=[result.x; x'];
    result.x1=[result.x1; crane_pos1'];
    % result.x2 = [result.x2; crane_pos2'];
    % Check goal condition
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end
    
%     % ==== Animation ====
    hold off;
    ArrowLength=0.5; % Length of the arrow
    % Robot
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    plot(result.x(:,1),result.x(:,2),'b');hold on;
    plot(result.x1(:,1),result.x1(:,2),'r');hold on;
    h1 = plot(goal(1),goal(2),'*r', 'DisplayName','Goal');hold on;
    h2 = plot(obstacle(:,1),obstacle(:,2),'*k', 'DisplayName','Obstacle');hold on;
    % Display search trajectory
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    title("Path Planning using DWA");
    xlabel("X");
    ylabel("Y");
    legend([h1, h2]);
end
%result_x = result.x(:, 1:2);  % Assuming columns 1 and 2 are x and y

% csvwrite('allData.csv', result_x, result.x1, result.x2, obstacle, goal);
% csvwrite('dwa.csv', result.x(:,1:2));
% csvwrite('crane1.csv', result.x1);
% csvwrite('obs.csv', obstacle);
%csvwrite('crane2.csv', result.x2);

% plot(result.x(:,1),result.x(:,2),'-b');hold on;
% plot(result.x1(:,1),result.x1(:,2),'.r','LineWidth', 2);hold on;
% plot(result.x2(:,1),result.x2(:,2),'g');hold on;
% plot(goal(1),goal(2),'*r', 'DisplayName','Goal');hold on;
% plot(obstacle(:,1),obstacle(:,2),'*k', 'DisplayName','Obstacle');hold on;

% ==== Save results ====
csvwrite(sprintf('dwa_map%d.csv', map_id),    result.x(:,1:2));
csvwrite(sprintf('crane1_map%d.csv', map_id), result.x1);
csvwrite(sprintf('obs_map%d.csv', map_id),    obstacle);

% ==== Final figure, styled after Figs. 7-8 ====
figure('Name', sprintf('Map %d', map_id));
plot(result.x1(:,1), result.x1(:,2), 'b', 'LineWidth', 2.5, 'DisplayName', 'Overhead Crane'); hold on;
plot(result.x(:,1),  result.x(:,2),  'Color', [0.55 0 0], 'LineWidth', 2, 'DisplayName', 'Modified DWA');
plot(goal(1), goal(2), 'p', 'MarkerSize', 12, 'MarkerEdgeColor', 'k', ...
     'MarkerFaceColor', [0 0.5 0], 'DisplayName', 'Goal');
plot(obstacle(:,1), obstacle(:,2), '*r', 'MarkerSize', 8, 'DisplayName', 'Obstacle');
axis([0 22 0 22]); grid on;
xlabel('X (m)'); ylabel('Y (m)');
legend('Location', 'northwest');


function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
% Function to calculate input values using DWA

% Calculate Dynamic Window[vmin,vmax,ωmin,ωmax]
Vr=CalcDynamicWindow(x,model);
% Calculate evaluation function
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

% Normalize each evaluation function
evalDB=NormalizeEval(evalDB);

% Calculate final evaluation value
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[~,ind]=max(feval);% Calculate the index of the input value with the highest evaluation value
u=evalDB(ind,1:2)';% Return the input value with a high evaluation value

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
% Function to calculate evaluation values for each path
evalDB=[];
trajDB=[];

for vx = Vr(1):model(3):Vr(2)
    for vy = Vr(3):model(4):Vr(4)
        u_candidate = [vx; vy];
        [x_traj, traj] = GenerateTrajectory(x, u_candidate, evalParam(4));
        % Goal evaluation: higher if the final position is closer to the goal.
        goal_eval = CalcGoalEval(x, x_traj, goal, u_candidate);
        % Clearance evaluation: minimum distance to obstacles along the trajectory.
        clearance_eval = CalcClearanceEval(x_traj, ob, R);
        % Speed evaluation: simply the norm of the candidate velocity.
        speed_eval = norm(u_candidate);
        % Append candidate evaluation: [vx, vy, goal, clearance, speed]
        evalDB = [evalDB; vx, vy, goal_eval, clearance_eval, speed_eval];
        trajDB = [trajDB, traj];
    end
end

function EvalDB=NormalizeEval(EvalDB)
% Function to normalize evaluation values
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end

function [x_traj, traj] = GenerateTrajectory(x, u, predictTime)
% Generate a predicted trajectory for candidate input u over predictTime seconds.
global dt;
traj = x(1:2);
x_traj = x;
time = 0;
while time <= predictTime
    time = time + dt;
    x_traj = f(x_traj, u);
    traj = [traj, x_traj(1:2)];
end

function Vr=CalcDynamicWindow(x,model)
% Calculate Dynamic Window based on the model and current state
global dt;
max_speed = model(1);
max_acc = model(2);
vx = x(3); vy = x(4);
vx_min = max(-max_speed, vx - max_acc*dt);
vx_max = min(max_speed, vx + max_acc*dt);
vy_min = max(-max_speed, vy - max_acc*dt);
vy_max = min(max_speed, vy + max_acc*dt);
Vr = [vx_min, vx_max, vy_min, vy_max];
%[vmin,vmax,ωmin,ωmax]

function goal_eval = CalcGoalEval(x, x_traj, goal,u_candidate)
desired_dir = (goal' - x(1:2));
desired_dir = desired_dir / norm(desired_dir);
candidate_dir = u_candidate / (norm(u_candidate)+1e-6);
heading_score = dot(desired_dir, candidate_dir);
% Combine heading_score with the distance-based score
goal_eval = heading_score * (1 / (norm(x_traj(1:2) - goal') + 1e-6));


function clearance_val = CalcClearanceEval(x_traj, ob, R)
% Calculate the minimum clearance from the trajectory to any obstacle.
clearance = inf;
for i = 1:size(x_traj,2)
    pos = x_traj(1:2, i);
    for j = 1:size(ob,1)
        d = norm(pos - ob(j,:)') - R;
        if d < clearance
            clearance = d;
        end
    end
end
clearance_val = clearance;

function [x, crane_pos1] = f(x, u)
global dt;

[crane_pos1] = g(x(1:2));
x = [x(1)+dt*u(1);%x(1) + dt * u(1);
              x(2)+dt*u(2);%x(2) + dt * u(2);
              u(1);
              u(2)];



function [crane_pos1] = g(pose_desired)
global z01 x01; 
Ax = [0, 1, 0, 0;
      0, -384.15, 1.59, 0;
      0, 0, 0, 1;
      0, 0, -130.63, 0];
Bx = [0; 0.16; 0; 0.06];

Ay = [0, 1, 0, 0;
      0, -640.25, 18.99, 0;
      0, 0, 0, 1;
      0, 0, -217.72, 0];
By = [0; 0.27; 0; 0.11];

% Combined system
Aa = blkdiag(Ax, Ay);
Ba = blkdiag(Bx, By);
Ca = eye(8);

% manual-optimized
% Q = diag([750, 1, 1, 1, 750, 1, 1, 1]);
% R = diag([1e-3 1e-3]);

% deo-optimized
Q = diag([1000 1.0000e-05 1.0000e-05 1.3936 1000 1.0000e-05 23.6795 1.0000e-05]);
R = diag([1e-5 1e-5]);


[crane_pos1, z01, x01] = crane(Aa, Ba, Ca,Q,R, z01, x01,pose_desired);
% [crane_pos2, z02, x02] = crane(Aa, Ba, Ca,Q2,R2, z02, x02,pose_desired);

function [crane_pos, zz0, xx0] = crane(Aa, Ba, Ca,Q,R, zz0, xx0,pose_desired)
global dt; 
K = lqr(Aa, Ba, Q, R);
Ac = Aa - Ba * K;

% Reference gain pre-compensation
N_bar = [1 / (-Ca(1,:) / Ac * Ba(:,1));
         1 / (-Ca(5,:) / Ac * Ba(:,2))];

u_des = pose_desired;        % Desired input

X0 = [zz0; xx0];         % Combined initial state
tspan = [0, dt];

odefun = @(t, X) augmentedODE(t, X, Aa, Ba, Ca, N_bar, K, u_des);

[~, XSol] = ode45(odefun, tspan, X0);
xx0 = XSol(end, 3:end)';  % Extract state trajectories
zz0 = XSol(end, 1:2)';     % integrator part

crane_pos= [ xx0(1); xx0(5)];

function dXdt = augmentedODE(~, X, A, B, C, k1, k2, u_des)
    z = X(1:2);        % Integrator state
    x = X(3:10);       % System state

    y = C([1,5], :) * x;         % Outputs x and y
    dz = k1 .* (u_des - y);      % Integrator dynamics
    dx = B * (z - k2 * x) + A * x;  % Plant dynamics

    dXdt = [dz; dx];

