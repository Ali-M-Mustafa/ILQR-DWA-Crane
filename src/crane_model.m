function [A, B, C] = crane_model()
% crane_model.m
% Decoupled linearized 3D overhead crane model, paper eqs. (19)-(23).
% Values are identical to the matrices hardcoded in g() of
% dwa_crane_optimized.m. Note: the paper prints -384.15 / -640.25 in
% column 1 of Ax / Ay; that is a typo. They belong in column 2 (velocity
% term), which is the only arrangement that reproduces Table I.
%
% State order (eq. 3): q = [x dx thx dthx  y dy thy dthy]'
% Inputs:              u = [f_x; f_y]
% Outputs (eq. 23):    y = [x; y]

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

A = blkdiag(Ax, Ay);
B = blkdiag(Bx, By);
C = [1 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0];
end
