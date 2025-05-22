%% Clear and Addpath
close all;
clear all;
addpath functions;
addpath matlab2tikz;
clc;

%% General
T = 0.1;
T_sim = 10;
steps = floor(T_sim/T);

%% prediction horizon
N  = 10;

%% Ship parameters:
K_ship = 2.0640;
T_ship = -4.4563;

% Guidance order:
Heading_Aimed = 15*(pi/180);

%% Model - Disretized double integrater
A = [1,T;0,1-T/T_ship];             % System Matirx
B = T* [0;K_ship/T_ship];          % Input Matrix
n = size(A,2);          % dimention of state space
m = size(B,2);          % dimention of input space
%x0 = [7.3;10];             % initial state
x0 = [0;0];             % initial state

%% Constraints A*x < b
U_set.A = [1;-1];
U_set.b = [35*(pi/180);-35*(pi/180)];
X_set.A = [-1,0;1,0;0,-1;0,1];
X_set.b = [2*pi;0;5*(pi/180);-5*(pi/180)];
% X_set = Polyhedron('A',[-1,0;1,0;0,-1;0,1],'b',[10;10;10;10]);
% terminal constraint
X_f   = X_set;

%% Optimization weights
Q  = eye(n);
R  = eye(m);
[Qf,~,~] = idare(A,B,Q,R,[],[]);

%% Preprocessing
Uad = admissibleInputs(A,B,X_set,U_set,X_f,N);

%% Variable declaration for logging
X_log = zeros(n,steps);
U_log = zeros(m,steps);
X_pre = zeros(n,N,steps);

%% Simulation
x=x0;
for i = 1:steps
    % Control Method: MPC
    U = MPC_opt(x,A,B,Q,Qf,R,Uad,N);
    u = U(1:m);
    
    % Prediction
    [A_,B_] = liftedModel(A,B,N);
    X =reshape( A_*x + B_*U, 2 ,N);
    
    % Log
    X_log(:,i) = x;
    U_log(:,i) = u;
    X_pre(:,:,i) = X;
    
    % System
    x=A*x+B*u;
end


%% plot
figure(1)
subplot(2,2,1)
hold on
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (fix(0.1*i), X_log,X_pre)
subplot(2,2,2)
hold on
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (fix(0.25*i), X_log,X_pre)
subplot(2,2,3)
hold on
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (fix(0.5*i), X_log,X_pre)
subplot(2,2,4)
hold on
X_set.plot('alpha',0.05,'color','gray')
plotStateSpace (i, X_log,X_pre)


matlab2tikz('Example1.tex')


