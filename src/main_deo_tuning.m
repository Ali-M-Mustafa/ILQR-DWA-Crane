% main_deo_tuning.m
% Section III-B: tunes the integral-LQR weights Q and R with Differential
% Evolution (Yarpiz YPEA107, de.m) by minimizing the ITAE cost, eq. (10).
%
% Search space (inferred from the published result, where every value
% not in the interior sits exactly on 1e-5 or 1000, i.e. was clipped by
% de.m at VarMin/VarMax):
%   p = [Q11..Q88, R11, R22],  1e-5 <= p <= 1000
%
% Published result (Table I), also used in dwa_crane_optimized.m:
%   Q = diag([1000 1e-5 1e-5 1.3936 1000 1e-5 23.6795 1e-5]), R = 1e-5*eye(2)
% DE is random, so a new run will not give exactly these numbers.

clc
clear
close all

%% Model and test scenario (same step test as Section V-A)
[A, B, C] = crane_model();
ref    = [20; 15];          % desired (x, y)
Tend   = 30;                % [s]
dt_sim = 0.01;              % [s]
t      = (0:dt_sim:Tend)';

J_limit   = 1e3;            % cost above this is treated as invalid
J_penalty = 1e6;            % cost given to invalid candidates

%% DE settings
nVar   = 10;
VarMin = 1e-5;
VarMax = 1000;
MaxIt  = 1000;              % Yarpiz default
nPop   = 50;                % Yarpiz default
rng(0);                     % remove for a different random run each time

CostFunction = @(p) lqr_itae_cost(p, A, B, C, ref, t, J_limit, J_penalty);

%% Reference costs
p_conv  = [750 1 1 1 750 1 1 1, 1e-3 1e-3];
p_paper = [1000 1e-5 1e-5 1.3936 1000 1e-5 23.6795 1e-5, 1e-5 1e-5];
fprintf('ITAE, conventional tuning : %.4f\n', CostFunction(p_conv));
fprintf('ITAE, published DEO tuning: %.4f\n\n', CostFunction(p_paper));

%% Run DE
[BestSol, BestCost] = de(CostFunction, nVar, VarMin, VarMax, MaxIt, nPop);

Q_deo = diag(BestSol.Position(1:8));
R_deo = diag(BestSol.Position(9:10));

fprintf('\nBest ITAE = %.4f\n', BestSol.Cost);
fprintf('Q_deo = diag([%s])\n', num2str(diag(Q_deo).', '%.4g '));
fprintf('R_deo = diag([%s])\n', num2str(diag(R_deo).', '%.4g '));

save('deo_result.mat', 'Q_deo', 'R_deo', 'BestSol', 'BestCost');
disp('Saved to deo_result.mat (compare_controllers.m can load it).');
