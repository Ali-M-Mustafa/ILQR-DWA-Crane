function [K, Ki, Acl] = integral_lqr_gains(A, B, C, Q, R)
% integral_lqr_gains.m
% LQR gain (eqs. 6-7) and integral gain (eq. 9).
% Same computation as crane() in dwa_crane_optimized.m, where Ki is
% called N_bar.

K   = lqr(A, B, Q, R);
Acl = A - B * K;

Ki = [1 / (-C(1,:) / Acl * B(:,1));
      1 / (-C(2,:) / Acl * B(:,2))];
end
