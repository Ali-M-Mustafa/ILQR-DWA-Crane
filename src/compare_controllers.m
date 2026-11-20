% compare_controllers.m
% Section V-A: conventional vs. DEO-tuned integral LQR.
% Reproduces Table I (K, Ki), the Lyapunov eigenvalues, Figs. 4-6 and
% Table II for a step to (x, y) = (20, 15).

clc
clear
close all

%% Model and test scenario
[A, B, C] = crane_model();
ref    = [20; 15];
Tend   = 30;
dt_sim = 0.01;
t      = (0:dt_sim:Tend)';

%% Controllers
% Conventional ("manual-optimized" in dwa_crane_optimized.m)
ctrl(1).name = 'Conv';
ctrl(1).Q    = diag([750, 1, 1, 1, 750, 1, 1, 1]);
ctrl(1).R    = diag([1e-3 1e-3]);

% Optimal ("deo-optimized" in dwa_crane_optimized.m, Table I)
ctrl(2).name = 'Optimal';
ctrl(2).Q    = diag([1000 1.0000e-05 1.0000e-05 1.3936 1000 1.0000e-05 23.6795 1.0000e-05]);
ctrl(2).R    = diag([1e-5 1e-5]);

% Set true to use the result of a new main_deo_tuning.m run instead
use_new_deo = false;
if use_new_deo && exist('deo_result.mat', 'file')
    S = load('deo_result.mat');
    ctrl(2).Q = S.Q_deo;
    ctrl(2).R = S.R_deo;
end

%% Gains, Lyapunov check, simulation
for k = 1:numel(ctrl)
    [K, Ki, Acl] = integral_lqr_gains(A, B, C, ctrl(k).Q, ctrl(k).R);
    ctrl(k).K  = K;
    ctrl(k).Ki = Ki;

    % Lyapunov (eq. 24) with T = eye(8). MATLAB's lyap(Acl, T) solves
    % Acl*W + W*Acl' + T = 0; this form reproduces the published
    % eigenvalues. lyap(Acl', T) is eq. 24 as written; both forms prove
    % stability, but their eigenvalues differ.
    W = lyap(Acl, eye(8));
    ctrl(k).eigW = sort(real(eig(W)));

    [ctrl(k).y, ctrl(k).u, ctrl(k).e] = simulate_integral_lqr(A, B, C, K, Ki, ref, t);
end

%% Table I
fprintf('==================== TABLE I ====================\n');
for k = 1:numel(ctrl)
    fprintf('\n--- %s tuning ---\n', ctrl(k).name);
    fprintf('Q    = diag([%s])\n', num2str(diag(ctrl(k).Q).', '%.4g  '));
    fprintf('R    = diag([%s])\n', num2str(diag(ctrl(k).R).', '%.4g  '));
    fprintf('Klqr =\n'); disp(round(ctrl(k).K * 100) / 100);
    fprintf('Ki   = [%s]''\n', num2str(ctrl(k).Ki.', '%.4e  '));
end

%% Lyapunov
fprintf('\n============ LYAPUNOV EIGENVALUES OF W ============\n');
for k = 1:numel(ctrl)
    fprintf('%-8s: [%s]\n', ctrl(k).name, num2str(ctrl(k).eigW.', '%.4g  '));
    if all(ctrl(k).eigW > 0)
        fprintf('          W is positive definite -> closed loop stable\n');
    end
end

%% Table II
fprintf('\n==================== TABLE II ====================\n');
fprintf('%-12s %-6s %10s %14s %14s\n', 'Controller', 'State', 'Rise (s)', 'Settling (s)', 'Overshoot (%)');
lbl = {'x', 'y'};
for k = 1:numel(ctrl)
    for j = 1:2
        info = stepinfo(ctrl(k).y(:, j), t, ref(j));
        fprintf('%-12s %-6s %10.2f %14.2f %14.2f\n', ctrl(k).name, lbl{j}, ...
            info.RiseTime, info.SettlingTime, info.Overshoot);
    end
end

%% Figures (styled after Figs. 4-6)
c_x = [0 0.447 0.741];          % blue   (x)
c_y = [0.850 0.325 0.098];      % orange (y)
opt = 2; conv = 1;

% Fig. 4: state tracking
figure('Name', 'Fig. 4 - State tracking');
plot(t, ctrl(opt).y(:,1),  '-',  'Color', c_x, 'LineWidth', 1.5); hold on;
plot(t, ctrl(conv).y(:,1), '-.', 'Color', c_x, 'LineWidth', 1.5);
plot(t, ref(1)*ones(size(t)), '--', 'Color', c_x, 'LineWidth', 1.5);
plot(t, ctrl(opt).y(:,2),  '-',  'Color', c_y, 'LineWidth', 1.5);
plot(t, ctrl(conv).y(:,2), '-.', 'Color', c_y, 'LineWidth', 1.5);
plot(t, ref(2)*ones(size(t)), '--', 'Color', c_y, 'LineWidth', 1.5);
grid on; xlabel('Time (sec)'); ylabel('States (m)');
legend('Optimal: x', 'Conv: x', 'Desired: x', 'Optimal: y', 'Conv: y', 'Desired: y');

% Fig. 5: error
figure('Name', 'Fig. 5 - Error');
plot(t, ctrl(opt).e(:,1),  '-',  'Color', c_x, 'LineWidth', 1.5); hold on;
plot(t, ctrl(opt).e(:,2),  '--', 'Color', c_x, 'LineWidth', 1.5);
plot(t, ctrl(conv).e(:,1), '-',  'Color', c_y, 'LineWidth', 1.5);
plot(t, ctrl(conv).e(:,2), '--', 'Color', c_y, 'LineWidth', 1.5);
grid on; xlabel('Time (sec)'); ylabel('State error');
legend('Optimal: x', 'Optimal: y', 'Conv: x', 'Conv: y');

% Fig. 6: control input
figure('Name', 'Fig. 6 - Input');
plot(t, ctrl(opt).u(:,1),  '-',  'Color', c_x, 'LineWidth', 1.5); hold on;
plot(t, ctrl(opt).u(:,2),  '--', 'Color', c_x, 'LineWidth', 1.5);
plot(t, ctrl(conv).u(:,1), '-',  'Color', c_y, 'LineWidth', 1.5);
plot(t, ctrl(conv).u(:,2), '--', 'Color', c_y, 'LineWidth', 1.5);
grid on; xlabel('Time (sec)'); ylabel('State input');
legend('Optimal: x', 'Optimal: y', 'Conv: x', 'Conv: y');
