% This script serves as a mean to tune the free parameters of the H
% Infinity Controller tuning problem done in the other associated scripts.

clear all;
close all;
clc;

global Gain_Static_Filter_1;
global Gain_Infinity_Filter_1;
global Gain_Static_Filter_2;
global Gain_Infinity_Filter_2;
global Gain_Static_Filter_3;
global Gain_Infinity_Filter_3;
global Pulsation_Unit_Gain_Filter_1;
global Pulsation_Unit_Gain_Filter_2;
global Pulsation_Unit_Gain_Filter_3;
global Gain_Static_Filter_4;
global Number_Iterations_Count;
global State_Matrix_Ship_Wave_Nominal;
global Input_Matrix_Nominal_Global;
global Measurement_Matrix_Ship_Wave_Nominal;
global Input_Measurement_Matrix_Global_Nominal;

% The fixed ponderation paraemeters are the following:
Gain_Static_Filter_1 = 1000;
Gain_Infinity_Filter_1 = 0.5;
Gain_Static_Filter_2 = 0.01;
Gain_Infinity_Filter_2 = 10000;

% Bounds on the parameters pulsations Omega_Filter_1 to 3 and
% Gain_Static_Filter_3, Gain_Infinity_Filter_3 and Gain_Static_Filter_4 (log10):
% Omega_Filter_1:
Lower_Bound_Omega_Filter_1 = 0.1;
Upper_Bound_Omega_Filter_1 = 10;
Log_Lower_Bound_Omega_Filter_1 = log10(Lower_Bound_Omega_Filter_1);
Log_Upper_Bound_Omega_Filter_1 = log10(Upper_Bound_Omega_Filter_1);
% Omega_Filter_2:
Lower_Bound_Omega_Filter_2 = 0.1;
Upper_Bound_Omega_Filter_2 = 1000;
Log_Lower_Bound_Omega_Filter_2 = log10(Lower_Bound_Omega_Filter_2);
Log_Upper_Bound_Omega_Filter_2 = log10(Upper_Bound_Omega_Filter_2);
% Omega_Filter_3:
Lower_Bound_Omega_Filter_3 = 1;
Upper_Bound_Omega_Filter_3 = 1000;
Log_Lower_Bound_Omega_Filter_3 = log10(Lower_Bound_Omega_Filter_3);
Log_Upper_Bound_Omega_Filter_3 = log10(Upper_Bound_Omega_Filter_3);
% Gain_Static_Filter_3:
Lower_Bound_Gain_Static_Filter_3 = 0.01;
Upper_Bound_Gain_Static_Filter_3 = 100; % Not the one taken by the initial study.
Log_Lower_Bound_Gain_Static_Filter_3 = log10(Lower_Bound_Gain_Static_Filter_3);
Log_Upper_Bound_Gain_Static_Filter_3 = log10(Upper_Bound_Gain_Static_Filter_3);
% Gain_Infinity_Filter_3:
Lower_Bound_Gain_Infinity_Filter_3 = 1;
Upper_Bound_Gain_Infinity_Filter_3 = 100;
Log_Lower_Bound_Gain_Infinity_Filter_3 = log10(Lower_Bound_Gain_Infinity_Filter_3);
Log_Upper_Bound_Gain_Infinity_Filter_3 = log10(Upper_Bound_Gain_Infinity_Filter_3);
% Gain_Static_Filter_4:
Lower_Bound_Gain_Static_Filter_4 = 0.001;
Upper_Bound_Gain_Static_Filter_4 = 1000; % Not the one taken by the initial study.
Log_Lower_Bound_Gain_Static_Filter_4 = log10(Lower_Bound_Gain_Static_Filter_4);
Log_Upper_Bound_Gain_Static_Filter_4 = log10(Upper_Bound_Gain_Static_Filter_4);
% Bounds:
Lower_Bounds_Parameters_Old = [Log_Lower_Bound_Omega_Filter_1  Log_Lower_Bound_Omega_Filter_2  Log_Lower_Bound_Omega_Filter_3 Log_Lower_Bound_Gain_Static_Filter_3  Log_Lower_Bound_Gain_Infinity_Filter_3  Log_Lower_Bound_Gain_Static_Filter_4];
Upper_Bounds_Parameters_Old = [Log_Upper_Bound_Omega_Filter_1  Log_Upper_Bound_Omega_Filter_2  Log_Upper_Bound_Omega_Filter_3  Log_Upper_Bound_Gain_Static_Filter_3  Log_Upper_Bound_Gain_Infinity_Filter_3   Log_Upper_Bound_Gain_Static_Filter_4];

% Initial values for the parameters:
Pulsation_Unit_Gain_Filter_1 = 3/20;
Pulsation_Unit_Gain_Filter_2 = 0.8;
Pulsation_Unit_Gain_Filter_3 = 70;
Gain_Static_Filter_3 = 0.1;
Gain_Infinity_Filter_3 = 10;
Gain_Static_Filter_4 = 0.1;
% Bounds fixed to be in a certain interval associated with the initial
% values:
Lower_Bounds_Parameters = log10(0.1*[Pulsation_Unit_Gain_Filter_1 Pulsation_Unit_Gain_Filter_2 Pulsation_Unit_Gain_Filter_3 Gain_Static_Filter_3 Gain_Infinity_Filter_3 Gain_Static_Filter_4]);
Upper_Bounds_Parameters = log10(10*[0.4*Pulsation_Unit_Gain_Filter_1 Pulsation_Unit_Gain_Filter_2 Pulsation_Unit_Gain_Filter_3 Gain_Static_Filter_3 Gain_Infinity_Filter_3 Gain_Static_Filter_4]);
Number_Of_Variables = size(Upper_Bounds_Parameters,2);

% Search for optimal parameters:
Tolerance_Function = 0.01;
Number_Iteration_Maximal_Stall = 10;

% State-Space Ship and wave declaration:
Static_Gain_Ship_Nominal = 0.6043;
Time_Constant_Ship_Nominal = -5.2097;
Omega_Wave_Nominal = 0.6;
Damping_Wave_Nominal = 0.1;
State_Matrix_Ship_Wave_Nominal = [0 1 0 0;
                                  -Omega_Wave_Nominal^2 -2*Omega_Wave_Nominal*Damping_Wave_Nominal 0 0;
                                  0 0 0 1;
                                  0 0 0 -1/Time_Constant_Ship_Nominal];
Perturbation_Input_Matrix_Nominal = [0;Omega_Wave_Nominal^2;0;0];
Input_Matrix_Nominal = [0;0;0;Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal];
Input_Matrix_Nominal_Global = [Input_Matrix_Nominal Perturbation_Input_Matrix_Nominal];
Input_Measurement_Matrix_Global_Nominal = [0 0];
Measurement_Matrix_Ship_Wave_Nominal = [0 1 1 0];

% Considered Heading Aimed value:
Step_Heading_Aimed = 30*(pi/180);
% Perturbation step considered:
Perturbation_Step = 1*(pi/180);
% Heading measurement noise:
Heading_Variance_Noise = 0.5*(pi/180); % Used for linmod of the Simulink.
Number_Iterations_Count = 0;
Options_Optimization = optimoptions('particleswarm','FunctionTolerance',Tolerance_Function,'MaxStallIterations',Number_Iteration_Maximal_Stall);
[Optimal_Vector,...
 Cost,...
 Exitflag,...
 Output_Optimization] = particleswarm(@Cost_Function_Optimal_Tuning_Controller,...
                                      Number_Of_Variables,...
                                      Lower_Bounds_Parameters,...
                                      Upper_Bounds_Parameters,...
                                      Options_Optimization);

  Pulsation_Unit_Gain_Filter_1 = 10^(Optimal_Vector(1)); 
  Pulsation_Unit_Gain_Filter_2 = 10^(Optimal_Vector(2)); 
  Pulsation_Unit_Gain_Filter_3 = 10^(Optimal_Vector(3));  
  Gain_Static_Filter_3 = 10^(Optimal_Vector(4));
  Gain_Infinity_Filter_3 = 10^(Optimal_Vector(5));
  Gain_Static_Filter_4 = 10^(Optimal_Vector(6)); 
  disp(sprintf('Parameters :%6.4g %6.4g %6.4g %6.4g %6.4g %6.4g %6.4g',...
                Pulsation_Unit_Gain_Filter_1,...
                Pulsation_Unit_Gain_Filter_2,...
                Pulsation_Unit_Gain_Filter_3,...
                Gain_Static_Filter_3,...
                Gain_Infinity_Filter_3,...
                Gain_Static_Filter_4));
  
% Calculation of the corrector with the optimal parameters:
[State_Matrix_Standard_P_After_Optimization,...
 Input_Matrix_Standard_P_After_Optimization,...
 Measurement_Matrix_Standard_P_After_Optimization,...
 Input_Measurement_Matrix_Standard_P_After_Optimization] = linmod('Standard_H_Infinity_Full_Order_Synthesis');

% mise au format 'state-space'
State_Space_Plan = ss(State_Matrix_Standard_P_After_Optimization,...
                      Input_Matrix_Standard_P_After_Optimization,...
                      Measurement_Matrix_Standard_P_After_Optimization,...
                      Input_Measurement_Matrix_Standard_P_After_Optimization);

Numer_Of_Inputs = 2;  
Number_Of_Commands = 1;  
[Controller_After_Optimization,...
 Closed_Loop_After_Optimization,...
 Gamma_Optimal_Value_After_Optimization] = hinfsyn(State_Space_Plan,Numer_Of_Inputs,Number_Of_Commands,'display','off');
disp(sprintf('Gamma after Optimization = %6.4g',Gamma_Optimal_Value_After_Optimization));

[Sate_Matrix_After_Optimization_Controller,...
 Input_Matrix_After_Optimization_Controller,...
 Measurement_Matrix_After_Optimization_Controller,...
 Input_Measurement_Matrix_After_Optimization_Controller] = ssdata(Controller_After_Optimization);

% State-Space reprezentation of the controller:
Controller_After_Optimization = ss(Sate_Matrix_After_Optimization_Controller,...
                                   Input_Matrix_After_Optimization_Controller,...
                                   Measurement_Matrix_After_Optimization_Controller,...
                                   Input_Measurement_Matrix_After_Optimization_Controller);

% Save of the parameters optimization:
save Stochastic_Optimization_Parameters_Results_2 Controller_After_Optimization Gamma_Optimal_Value_After_Optimization Gain_Static_Filter_1 Gain_Static_Filter_2 Gain_Static_Filter_3 Gain_Static_Filter_4 Gain_Infinity_Filter_1 Gain_Infinity_Filter_2 Gain_Infinity_Filter_3 Pulsation_Unit_Gain_Filter_1 Pulsation_Unit_Gain_Filter_2 Pulsation_Unit_Gain_Filter_3;











