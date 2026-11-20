function [y, u, e] = simulate_integral_lqr(A, B, C, K, Ki, ref, t)
% simulate_integral_lqr.m
% Step response of the crane under integral LQR (Fig. 2, eq. 8).
% The closed loop is the same as augmentedODE() in dwa_crane_optimized.m:
%     dz = Ki .* (ref - C*x)
%     u  = z - K*x
%     dx = A*x + B*u
% Being linear, it is simulated exactly with lsim (zero initial state).
%
% ref : 2x1 desired position [x; y]
% t   : time vector (column), uniform spacing
% y,u,e : N x 2 outputs, inputs and errors (ref - y)

n  = size(A, 1);
Aaug = [zeros(2),        -diag(Ki) * C;
        B,               A - B * K];
Baug = [diag(Ki);
        zeros(n, 2)];
Caug = [zeros(2), C;          % y = C x
        eye(2),   -K];        % u = z - K x
sys = ss(Aaug, Baug, Caug, zeros(4, 2));

r   = repmat(ref(:).', numel(t), 1);
out = lsim(sys, r, t(:));

y = out(:, 1:2);
u = out(:, 3:4);
e = r - y;
end
