function J = lqr_itae_cost(p, A, B, C, ref, t, J_limit, J_penalty)
% lqr_itae_cost.m
% DEO cost function for tuning the integral LQR, paper eq. (10).
%
% p = [Q11 ... Q88  R11 R22]   (10 decision variables)
%
% J = ITAE of the x and y tracking errors for a step to ref,
%     J = integral( t * (|e_x| + |e_y|) dt ).
% Candidates that are unstable, give NaN/Inf, or give J > J_limit are
% rejected with J = J_penalty (paper: limit e.g. 1e3, penalty 1e6).

if nargin < 7 || isempty(J_limit),   J_limit   = 1e3; end
if nargin < 8 || isempty(J_penalty), J_penalty = 1e6; end

try
    Q = diag(p(1:8));
    R = diag(p(9:10));

    [K, Ki, Acl] = integral_lqr_gains(A, B, C, Q, R);
    if any(~isfinite(K(:))) || any(~isfinite(Ki)) || any(real(eig(Acl)) >= 0)
        J = J_penalty;
        return;
    end

    [~, ~, e] = simulate_integral_lqr(A, B, C, K, Ki, ref, t);
    J = trapz(t, t(:) .* sum(abs(e), 2));

    if ~isfinite(J) || J > J_limit
        J = J_penalty;
    end
catch
    J = J_penalty;
end
end
