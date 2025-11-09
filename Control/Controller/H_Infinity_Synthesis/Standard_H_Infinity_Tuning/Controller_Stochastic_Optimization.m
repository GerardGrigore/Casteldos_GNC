% This script serves as a mean to tune the free parameters of the H-Infinity
% Controller tuning problem done in the other associated scripts.
% Some filters are used to shape the frequency responses of the several
% transfers of the loop.

clear all;
close all;
clc;

% Global parameters definition:
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
global Is_Uncertain_Model_Synthesis;
global Step_Heading_Aimed;
global Perturbation_Step;
global Heading_Variance_Noise;
global Static_Gain_Ship_Uncertain;
global Time_Constant_Ship_Uncertain;
global Omega_Wave_Pulsation_Uncertain;
global Damping_Wave_Uncertain;

% Fixed parameters of the filters:
Gain_Static_Filter_1 = 1000;
Gain_Infinity_Filter_1 = 0.5;
Gain_Static_Filter_2 = 0.01;
Gain_Infinity_Filter_2 = 10000;

% Initialization of the filter parameters to optimize:
Pulsation_Unit_Gain_Filter_1 = 0.03275;
Pulsation_Unit_Gain_Filter_2 = 1.741;
Pulsation_Unit_Gain_Filter_3 = 348;
Gain_Static_Filter_3 = 0.01501;
Gain_Infinity_Filter_3 = 0.5025;
Gain_Static_Filter_4 = 0.5167;

% Bounds on the searching for the parameters:
Lower_Bound_Coefficient = 0.1;
Upper_Bound_Coefficient = 10;
Lower_Bounds_Parameters = log10(Lower_Bound_Coefficient*[Pulsation_Unit_Gain_Filter_1 Pulsation_Unit_Gain_Filter_2 Pulsation_Unit_Gain_Filter_3 Gain_Static_Filter_3 Gain_Infinity_Filter_3 Gain_Static_Filter_4]);
Upper_Bounds_Parameters = log10(Upper_Bound_Coefficient*[Pulsation_Unit_Gain_Filter_1 Pulsation_Unit_Gain_Filter_2 Pulsation_Unit_Gain_Filter_3 Gain_Static_Filter_3 Gain_Infinity_Filter_3 Gain_Static_Filter_4]);
Number_Of_Variables = size(Upper_Bounds_Parameters,2);

% Optimization features:
Tolerance_Function = 0.01;
Number_Iteration_Maximal_Stall = 10;
Options_Optimization = optimoptions('particleswarm','FunctionTolerance',Tolerance_Function,'MaxStallIterations',Number_Iteration_Maximal_Stall);

% Select the type of model to be used for the synthesis:
Is_Uncertain_Model_Synthesis = 1;
% Uncertain parameters declaration:
Static_Gain_Ship_Nominal_Uncertainty = 0.6043;
Static_Gain_Ship_Uncertain_Interval = [0.6 5];
Time_Constant_Ship_Nominal_Uncertainty = -5.2097;
Time_Constant_Ship_Uncertain_Interval = [-387 -0.3208];
Omega_Wave_Nominal_Uncertainty = 0.6;
Omega_Wave_Uncertain_Interval = [0.3 1.3];
Damping_Wave_Nominal_Uncertainty = 0.1;
Damping_Wave_Uncertainty_Interval = [0.08 0.2];
Static_Gain_Ship_Uncertain = ureal("Static_Gain_Ship_Uncertain",Static_Gain_Ship_Nominal_Uncertainty,"Range",Static_Gain_Ship_Uncertain_Interval);
Time_Constant_Ship_Uncertain = ureal("Time_Constant_Ship_Uncertain",Time_Constant_Ship_Nominal_Uncertainty,"Range",Time_Constant_Ship_Uncertain_Interval);
Omega_Wave_Pulsation_Uncertain = ureal("Omega_Wave_Pulsation_Uncertain",Omega_Wave_Nominal_Uncertainty,"Range",Omega_Wave_Uncertain_Interval);
Damping_Wave_Uncertain = ureal("Damping_Wave_Uncertain",Damping_Wave_Nominal_Uncertainty,"Range",Damping_Wave_Uncertainty_Interval);
    

% Perfect Guidance parameters for the synthesis:
% Heading step aimed:
Step_Heading_Aimed = 30*(pi/180);
% Perturbation step:
Perturbation_Step = 10*(pi/180);
% Heading measurement noise:
Heading_Variance_Noise = 10*(pi/180);

% Stochastic optimization:
Number_Iterations_Count = 0;
[Optimal_Vector,...
 Cost,...
 Exitflag,...
 Output_Optimization] = particleswarm(@Cost_Function_Optimal_Tuning_Controller,...
                                      Number_Of_Variables,...
                                      Lower_Bounds_Parameters,...
                                      Upper_Bounds_Parameters,...
                                      Options_Optimization);

% Determination and extraction of the final obtained parameters:
Pulsation_Unit_Gain_Filter_1 = 10^(Optimal_Vector(1)); 
Pulsation_Unit_Gain_Filter_2 = 10^(Optimal_Vector(2)); 
Pulsation_Unit_Gain_Filter_3 = 10^(Optimal_Vector(3));  
Gain_Static_Filter_3 = 10^(Optimal_Vector(4));
Gain_Infinity_Filter_3 = 10^(Optimal_Vector(5));
Gain_Static_Filter_4 = 10^(Optimal_Vector(6)); 

% Monitoring:
disp(sprintf('Parameters :%6.4g %6.4g %6.4g %6.4g %6.4g %6.4g %6.4g',...
              Pulsation_Unit_Gain_Filter_1,...
              Pulsation_Unit_Gain_Filter_2,...
              Pulsation_Unit_Gain_Filter_3,...
              Gain_Static_Filter_3,...
              Gain_Infinity_Filter_3,...
              Gain_Static_Filter_4));

% Controller calculation in function of the considered models:  
if ~Is_Uncertain_Model_Synthesis
    % Calculation of the corrector with the optimal parameters:
    [State_Matrix_Standard_P_After_Optimization,...
     Input_Matrix_Standard_P_After_Optimization,...
     Measurement_Matrix_Standard_P_After_Optimization,...
     Input_Measurement_Matrix_Standard_P_After_Optimization] = linmod('Standard_H_Infinity_Full_Order_Synthesis');
else
    [Final_State_Matrix,...
     Final_Input_Matrix,...
     Final_Measurement_Matrix,...
     Final_Input_Measurement_Matrix,...
     Final_State_Space_Reprezentation,...
     Final_State_Matrix_Analytical,...
     Final_Input_Matrix_Analytical,...
     Final_Measurement_Matrix_Analytical,...
     Final_Input_Measurement_Matrix_Analytical,...
     Final_State_Space_Reprezentation_Analytical] = State_Space_Uncertain_Calculation(Omega_Wave_Pulsation_Uncertain,...
                                                                                      Damping_Wave_Uncertain,...
                                                                                      Static_Gain_Ship_Uncertain,...
                                                                                      Time_Constant_Ship_Uncertain,...
                                                                                      Gain_Static_Filter_1,...
                                                                                      Gain_Infinity_Filter_1,...
                                                                                      Pulsation_Unit_Gain_Filter_1,...
                                                                                      Gain_Static_Filter_2,...
                                                                                      Gain_Infinity_Filter_2,...
                                                                                      Pulsation_Unit_Gain_Filter_2,...
                                                                                      Gain_Static_Filter_3,...
                                                                                      Gain_Infinity_Filter_3,...
                                                                                      Pulsation_Unit_Gain_Filter_3,...
                                                                                      Gain_Static_Filter_4);

    State_Matrix_Standard_P_After_Optimization = Final_State_Matrix_Analytical;
    Input_Matrix_Standard_P_After_Optimization = Final_Input_Matrix_Analytical;
    Measurement_Matrix_Standard_P_After_Optimization = Final_Measurement_Matrix_Analytical;
    Input_Measurement_Matrix_Standard_P_After_Optimization = Final_Input_Measurement_Matrix_Analytical;
end

% State-space format:
State_Space_Plan = ss(State_Matrix_Standard_P_After_Optimization,...
                      Input_Matrix_Standard_P_After_Optimization,...
                      Measurement_Matrix_Standard_P_After_Optimization,...
                      Input_Measurement_Matrix_Standard_P_After_Optimization);
% Final controller determination:
Numer_Of_Inputs = 2;  
Number_Of_Commands = 1;  
[Controller_After_Optimization,...
 Closed_Loop_After_Optimization,...
 Gamma_Optimal_Value_After_Optimization] = hinfsyn(State_Space_Plan,Numer_Of_Inputs,Number_Of_Commands,'display','off');
disp(sprintf('Gamma after Optimization = %6.4g',Gamma_Optimal_Value_After_Optimization));

% State-space information determination:
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
Storage_Path_Of_Synthesis_Results = 'C:\Users\gerar\Documents\GitHub\Casteldos_GNC\Control\Controller\H_Infinity_Synthesis\Standard_H_Infinity_Tuning\Synthesis_Numerical_Results';
Number_Existent_Synthesis = length(dir(fullfile(Storage_Path_Of_Synthesis_Results,'**','*.mat')));
Name_Of_The_Result_File = string(sprintf('Stochastic_Optimization_Parameters_Results_%i',Number_Existent_Synthesis + 1));
cd 'C:\Users\gerar\Documents\GitHub\Casteldos_GNC\Control\Controller\H_Infinity_Synthesis\Standard_H_Infinity_Tuning\Synthesis_Numerical_Results';
Elements_To_Save = string([Gamma_Optimal_Value_After_Optimization Gain_Static_Filter_1...
                    Gain_Static_Filter_2 Gain_Static_Filter_3 Gain_Static_Filter_4 Gain_Infinity_Filter_1...
                    Gain_Infinity_Filter_2 Gain_Infinity_Filter_3 Pulsation_Unit_Gain_Filter_1...
                    Pulsation_Unit_Gain_Filter_2 Pulsation_Unit_Gain_Filter_3]);
save(Name_Of_The_Result_File);
cd 'C:\Users\gerar\Documents\GitHub\Casteldos_GNC\Control\Controller\H_Infinity_Synthesis\Standard_H_Infinity_Tuning';











