function Cost = Cost_Function_Optimal_Tuning_Controller(Optimal_Vector)

global Pulsation_Unit_Gain_Filter_1;
global Pulsation_Unit_Gain_Filter_2;
global Pulsation_Unit_Gain_Filter_3;
global Gain_Static_Filter_3;
global Gain_Infinity_Filter_3;
global Gain_Static_Filter_4;
global Number_Iterations_Count;
global Gamma_Optimal_Optimization;
global State_Matrix_Controller_Nominal;
global Input_Matrix_Controller_Nominal;
global Measurement_Matrix_Controller_Nominal;
global Input_Measurement_Matrix_Controller_Nominal;
global State_Matrix_Ship_Wave_Nominal;
global Input_Matrix_Nominal_Global;
global Measurement_Matrix_Ship_Wave_Nominal;
global Input_Measurement_Matrix_Global_Nominal;

Pulsation_Unit_Gain_Filter_1 = 10^(Optimal_Vector(1));
Pulsation_Unit_Gain_Filter_2 = 10^(Optimal_Vector(2));
Pulsation_Unit_Gain_Filter_3 = 10^(Optimal_Vector(3));
Gain_Static_Filter_3 = 10^(Optimal_Vector(4));
Gain_Infinity_Filter_3 = 10^(Optimal_Vector(5));
Gain_Static_Filter_4 = 10^(Optimal_Vector(6));
Number_Iterations_Count = Number_Iterations_Count + 1;

% Considered Heading Aimed value:
Step_Heading_Aimed = 30*(pi/180);
% Perturbation step considered:
Perturbation_Step = 1*(pi/180);
% Heading measurement noise:
Heading_Perturbed_Minimum = -0.5*(pi/180);
Heading_Perturbed_Maximum = +0.5*(pi/180);
Heading_Variance_Noise = Heading_Perturbed_Minimum; % Used for linmod of the Simulink.

disp(sprintf('Evaluation - Iteration number %d',Number_Iterations_Count))
disp(sprintf('Parameters :%6.4g %6.4g %6.4g %6.4g %6.4g %6.4g %6.4g',Pulsation_Unit_Gain_Filter_1,...
     Pulsation_Unit_Gain_Filter_2,Pulsation_Unit_Gain_Filter_3,Gain_Static_Filter_3,...
     Gain_Infinity_Filter_3,Gain_Static_Filter_4));

% State reprezentation of the plant P(s) in Simulink model 'Standard_H_Infinity_Full_Order_Synthesis': 
[State_Matrix_Standard_P_Optimization,...
 Input_Matrix_Standard_P_Optimization,...
 Measurement_Matrix_Standard_P_Optimization,...
 Input_Measurement_Matrix_Standard_P_Optimization] = linmod('Standard_H_Infinity_Full_Order_Synthesis');

% State-Space reprezentation:
Pland_Standard_P_Nominal_State_Space_Optimization = ss(State_Matrix_Standard_P_Optimization,...
                                                       Input_Matrix_Standard_P_Optimization,...
                                                       Measurement_Matrix_Standard_P_Optimization,...
                                                       Input_Measurement_Matrix_Standard_P_Optimization);

% Use of hinfsyn function for Hinfinity calculation of the corrector:
Number_Inputs_Controller_Fixed = 2;
Number_Commands_To_Be_Produced_Fixed = 1;
[Controller_Optimization,...
 Closed_Loop_Optimization,...
 Gamma_Optimal_Optimization] = hinfsyn(Pland_Standard_P_Nominal_State_Space_Optimization,...
                                       Number_Inputs_Controller_Fixed,...
                                       Number_Commands_To_Be_Produced_Fixed,'display','off');
disp(sprintf('Gamma Optimization = %6.4g',Gamma_Optimal_Optimization));
[State_Matrix_Controller_Nominal,...
 Input_Matrix_Controller_Nominal,...
 Measurement_Matrix_Controller_Nominal,...
 Input_Measurement_Matrix_Controller_Nominal] = ssdata(Controller_Optimization);

% State reprezentation of the looped systems:
% Force Simulink to have the state-space matrices:
assignin('base', 'State_Matrix_Controller_Nominal', State_Matrix_Controller_Nominal);
assignin('base', 'Input_Matrix_Controller_Nominal', Input_Matrix_Controller_Nominal);
assignin('base', 'Measurement_Matrix_Controller_Nominal', Measurement_Matrix_Controller_Nominal);
assignin('base', 'Input_Measurement_Matrix_Controller_Nominal', Input_Measurement_Matrix_Controller_Nominal);

[State_Matrix_Closed_Loop_Nominal_Optimization,...
 Input_Matrix_Closed_Loop_Nominal_Optimization,...
 Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
 Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization] = linmod('Ship_Control_Loop_H_Infinity_Standard');

System_Closed_Loop_Reference = ss(State_Matrix_Closed_Loop_Nominal_Optimization,...
                                  Input_Matrix_Closed_Loop_Nominal_Optimization(:,1),...
                                  Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
                                  Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization(:,1));

System_Closed_Loop_Perturbation = ss(State_Matrix_Closed_Loop_Nominal_Optimization,...
                                     Input_Matrix_Closed_Loop_Nominal_Optimization(:,2),...
                                     Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
                                     Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization(:,2));
[Pulsation_Natural_Frequency,...
 Damping_System_Closed_Loop_Perturbation] = damp(System_Closed_Loop_Perturbation);

System_Closed_Loop_Noise = ss(State_Matrix_Closed_Loop_Nominal_Optimization,...
                              Input_Matrix_Closed_Loop_Nominal_Optimization(:,3),...
                              Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
                              Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization(:,3));
  
% System simulation:
Step_Time_Optimization = 0.1; 
Time_Horizon_Optimization = 150;
Time_Vector_Optimization = 0:Step_Time_Optimization:Time_Horizon_Optimization;

Size_Noise = length(Time_Vector_Optimization);
Noise_Generated_Optimization = Heading_Perturbed_Minimum + (Heading_Perturbed_Maximum - Heading_Perturbed_Minimum) * rand(1, Size_Noise);

Output_Simulated_Heading = Step_Heading_Aimed*step(System_Closed_Loop_Reference,Time_Vector_Optimization);
Heading_Reference = Output_Simulated_Heading(:,3); 
Rudder_Angle_Input_Reference = Output_Simulated_Heading(:,2); 

Output_Simulated_Heading = Perturbation_Step*step(System_Closed_Loop_Perturbation,Time_Vector_Optimization);
Output_Perturbation_Heading = Output_Simulated_Heading(:,3);

Output_Simulated_Heading = lsim(System_Closed_Loop_Noise,Noise_Generated_Optimization,Time_Vector_Optimization);
Input_Response_Dynamic_System_Noise = Output_Simulated_Heading(:,2);

% Frequencial features:
[State_Matrix_System,...
 Input_Matrix_System,...
 Measurement_Matrix_System,...
 Input_Measurement_Matrix_System] = linmod('Standard_H_Infinity_Full_Order_Synthesis');
Open_Loop_Non_Corrected = ss(State_Matrix_System,...
                             Input_Matrix_System(:,4),...
                             Measurement_Matrix_System(3:4,:),...
                             Input_Measurement_Matrix_System(3:4,4));
Open_Loop_Corrector = -Controller_Optimization*Open_Loop_Non_Corrected; 
All_Margins = allmargin(Open_Loop_Corrector);
Pulsation_Frequency = min(All_Margins.PMFrequency);

Feedback_Loop = feedback(1,Open_Loop_Corrector);
Magnitude_Optimization = bode(Feedback_Loop);
Delta_Margin_Optimization = 1/max(Magnitude_Optimization);

% Criteria to be respected:
% Overshoot of the heading:
% Overshoot_Maximal = 50/100; % 10% Overshoot allowed.
Overshoot_Maximal = 10/100; % 10% Overshoot allowed.
Reference_Function_1 = (max(Heading_Reference)-Step_Heading_Aimed*(1 + Overshoot_Maximal))/(Overshoot_Maximal);
% Time response at 10% lower than 9 seconds:
% Overshoot_Maximal_Time = 100;
% Overshoot_Maximal_Time_Percent = 50/100;
Overshoot_Maximal_Time = 15;
Overshoot_Maximal_Time_Percent = 10/100;
Reference_Function_3 = max(abs(Heading_Reference(Overshoot_Maximal_Time/Step_Time_Optimization:Time_Horizon_Optimization/Step_Time_Optimization)-Step_Heading_Aimed))/(Step_Heading_Aimed*Overshoot_Maximal_Time_Percent)-1;
% Command amplitude:
% Lower than 35°:
Delta_Rudder_Limit = 35*(pi/180);
Reference_Function_4 = max(abs(Rudder_Angle_Input_Reference))/Delta_Rudder_Limit-1;                     
% Perturbation response:
% Time_Constraint_Perturbation = 100;
Time_Constraint_Perturbation = 15;
Perturbation_Function_1 = max(abs(Output_Perturbation_Heading(Time_Constraint_Perturbation/Step_Time_Optimization:Time_Horizon_Optimization/Step_Time_Optimization)))/Perturbation_Step-1;
% Frequencies and pulsations:
Delta_Pulsation_Unit_Gain = 5;
Frequency_1 = Pulsation_Frequency/Delta_Pulsation_Unit_Gain-1;   
Delta_Margin_Specified = 0.5;
Frequency_2 = 1-Delta_Margin_Optimization/Delta_Margin_Specified;  
Damping_Aimed_Specified = 0.1;
Damping = 1-min(Damping_System_Closed_Loop_Perturbation)/Damping_Aimed_Specified;
% Limitations on the rudder rate:
Rudder_Rate_Maximal = 10*(pi/180); % Very brutal and tight constraint.
Function_Constraint_Rudder_Rate = (max((diff(Rudder_Angle_Input_Reference))/Step_Time_Optimization)-Rudder_Rate_Maximal)/Rudder_Rate_Maximal;

Cost = Gamma_Optimal_Optimization + exp(1000*Reference_Function_1) + exp(1000*Reference_Function_3) ...
       + exp(1000*Reference_Function_4) + exp(1000*Perturbation_Function_1) + exp(1000*Damping) ...
       + exp(1000*Frequency_1) + exp(1000*Frequency_2) + exp(1000*Function_Constraint_Rudder_Rate) + std(Input_Response_Dynamic_System_Noise);
% Cost = Gamma_Optimal_Optimization + exp(1000*Reference_Function_1) + exp(1000*Reference_Function_3) ...
%        + exp(1000*Reference_Function_4) + exp(1000*Perturbation_Function_1) + exp(1000*Damping) ...
%        + exp(1000*Frequency_1) + exp(1000*Frequency_2) + std(Input_Response_Dynamic_System_Noise);

disp(sprintf('Criterion value : %6.4g',Cost));
disp(sprintf('Reference indicators : %6.4g %6.4g %6.4g',Reference_Function_1,Reference_Function_3,Reference_Function_4));
disp(sprintf('Perturbation indicators, ksi, w0, DM, std(ub) : %6.4g %6.4g %6.4g %6.4g %6.4g',Perturbation_Function_1,min(Damping_System_Closed_Loop_Perturbation),Pulsation_Frequency,Delta_Margin_Optimization,std(Input_Response_Dynamic_System_Noise)));


return

