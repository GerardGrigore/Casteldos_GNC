clear all;
close all;
clc;

% Plant to control:
K = 0.604;
T = -5.5;
s = tf('s');
Discretization_Method = char('tustin');
T_s = 0.1; % Tuning parameter.
Rudder_To_Heading = K/(s*(1 + T*s));
Rudder_To_Heading_Discrete = c2d(Rudder_To_Heading,T_s,Discretization_Method);

% State-space matrix:
A_Ship_Cont = [0 1;0 -1/T];
B_Ship_Cont = [0;K/T];
C_Ship_Cont = [1 0];
D_Ship_Cont = 0;

% Discrete time model:
Phi = [1 T_s;0 1-(T_s/T)];
Gamma = [0;T_s*(K/T)];
C_Ship_Discrete = [1 0];
D_Ship_Discrete = 0;

% Augmented/Lifted state:
Phi_Aug = [Phi zeros(2,1);C_Ship_Discrete*Phi 1];
Gamma_Aug = [Gamma;C_Ship_Discrete*Gamma];
C_Aug = [0 0 1];
D_Aug = 0;

% Parameters for the MPC:
N_p = 3;
W = [C_Aug*Phi_Aug;C_Aug*Phi_Aug^2;C_Aug*Phi_Aug^N_p];
Z = [C_Aug*Gamma_Aug 0 0;C_Aug*Phi_Aug*Gamma_Aug C_Aug*Gamma_Aug 0;...
    C_Aug*Phi_Aug^(N_p-1)*Gamma_Aug C_Aug*Phi_Aug*Gamma_Aug C_Aug*Gamma_Aug];

% Simple constraint definition for the rate of change in the control
% action:
A_Matrix_QP_Cons = [-1 0 0;1 0 0];
b_Matrix_QP_Cons = [-0.0000001;0.0000001];

% Weight matrices:
Q = eye(3);
R = 2*eye(3);









