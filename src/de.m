%
% Copyright (c) 2015, Mostapha Kalami Heris & Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "LICENSE" file for license terms.
% (In this repository the Yarpiz MIT license is in LICENSE-YARPIZ.
%  Source: https://yarpiz.com/differential-evolution-ypea107
%  Modified for the crane project: problem size and bounds are passed as
%  arguments.)
%
% Project Code: YPEA107
% Project Title: Implementation of Differential Evolution (DE) in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Cite as:
% Mostapha Kalami Heris, Differential Evolution (DE) in MATLAB (URL: https://yarpiz.com/231/ypea107-differential-evolution), Yarpiz, 2015.
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

function [BestSol, BestCost] = de(CostFunction, nVar, VarMin, VarMax, MaxIt, nPop)
% Crane project change: the problem size, bounds, iterations and population
% can now be passed in. Omitted arguments keep the original Yarpiz values,
% so main.m / Sphere.m behave exactly as before.
%   LQR tuning call: de(@(p) lqr_itae_cost(...), 10, 1e-5, 1000, MaxIt, nPop)

%% Problem Definition

% CostFunction = @(x) Sphere(x);    % Cost Function

if nargin < 2 || isempty(nVar),   nVar = 20;     end   % Number of Decision Variables

VarSize = [1 nVar];   % Decision Variables Matrix Size

if nargin < 3 || isempty(VarMin), VarMin = -5;   end   % Lower Bound of Decision Variables
if nargin < 4 || isempty(VarMax), VarMax = 5;    end   % Upper Bound of Decision Variables

%% DE Parameters

if nargin < 5 || isempty(MaxIt),  MaxIt = 1000;  end   % Maximum Number of Iterations

if nargin < 6 || isempty(nPop),   nPop = 50;     end   % Population Size

beta_min = 0.2;   % Lower Bound of Scaling Factor
beta_max = 0.8;   % Upper Bound of Scaling Factor

pCR = 0.2;        % Crossover Probability

%% Initialization

empty_individual.Position = [];
empty_individual.Cost = [];

BestSol.Cost = inf;

pop = repmat(empty_individual, nPop, 1);

for i = 1:nPop

    pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
    
    pop(i).Cost = CostFunction(pop(i).Position);
    
    if pop(i).Cost<BestSol.Cost
        BestSol = pop(i);
    end
    
end

BestCost = zeros(MaxIt, 1);

%% DE Main Loop

for it = 1:MaxIt
    
    for i = 1:nPop
        
        x = pop(i).Position;
        
        A = randperm(nPop);
        
        A(A == i) = [];
        
        a = A(1);
        b = A(2);
        c = A(3);
        
        % Mutation
        %beta = unifrnd(beta_min, beta_max);
        beta = unifrnd(beta_min, beta_max, VarSize);
        y = pop(a).Position+beta.*(pop(b).Position-pop(c).Position);
        y = max(y, VarMin);
		y = min(y, VarMax);
		
        % Crossover
        z = zeros(size(x));
        j0 = randi([1 numel(x)]);
        for j = 1:numel(x)
            if j == j0 || rand <= pCR
                z(j) = y(j);
            else
                z(j) = x(j);
            end
        end
        
        NewSol.Position = z;
        NewSol.Cost = CostFunction(NewSol.Position);
        
        if NewSol.Cost<pop(i).Cost
            pop(i) = NewSol;
            
            if pop(i).Cost<BestSol.Cost
               BestSol = pop(i);
            end
        end
        
    end
    
    % Update Best Cost
    BestCost(it) = BestSol.Cost;
    
    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
end
figure;
%plot(BestCost);
semilogy(BestCost, 'LineWidth', 2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
end

%% Show Results


