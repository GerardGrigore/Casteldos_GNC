function Cost = Cost_Function_Optimal_Tuning_Controller(Optimal_Vector)

% This function serves to create the cost function used by the PSO
% algorithm in order to find optimal parameters for the synthetized
% controller.

% Global parameters definition:
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
global Is_Uncertain_Model_Synthesis;
global Gain_Static_Filter_1;
global Gain_Infinity_Filter_1;
global Gain_Static_Filter_2;
global Gain_Infinity_Filter_2;
global Step_Heading_Aimed;
global Perturbation_Step;
global Heading_Variance_Noise;
global Static_Gain_Ship_Uncertain;
global Time_Constant_Ship_Uncertain;
global Omega_Wave_Pulsation_Uncertain;
global Damping_Wave_Uncertain;
global Is_Reduced_Model;

% Initialization of the parameters of the filter:
Pulsation_Unit_Gain_Filter_1 = 10^(Optimal_Vector(1));
Pulsation_Unit_Gain_Filter_2 = 10^(Optimal_Vector(2));
Pulsation_Unit_Gain_Filter_3 = 10^(Optimal_Vector(3));
Gain_Static_Filter_3 = 10^(Optimal_Vector(4));
Gain_Infinity_Filter_3 = 10^(Optimal_Vector(5));
Gain_Static_Filter_4 = 10^(Optimal_Vector(6));
Number_Iterations_Count = Number_Iterations_Count + 1;

% Monitoring the convergence of the algorithm:
disp(sprintf('Evaluation - Iteration number %d',Number_Iterations_Count))
disp(sprintf('Parameters :%6.4g %6.4g %6.4g %6.4g %6.4g %6.4g %6.4g',Pulsation_Unit_Gain_Filter_1,...
     Pulsation_Unit_Gain_Filter_2,Pulsation_Unit_Gain_Filter_3,Gain_Static_Filter_3,...
     Gain_Infinity_Filter_3,Gain_Static_Filter_4));

% Choice of the model used for the synthesis:
if ~Is_Uncertain_Model_Synthesis
    % Nominal model:
    % State reprezentation of the plant P(s) in Simulink model 'Standard_H_Infinity_Full_Order_Synthesis':
    % Nominal model:
    if ~Is_Reduced_Model
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
        % Measurement_Matrix_Ship_Wave_Nominal = [0 1 1 0];
        Measurement_Matrix_Ship_Wave_Nominal = [0 0 1 0];
        Ship_Wave_State_Space = ss(State_Matrix_Ship_Wave_Nominal,...
            Input_Matrix_Nominal_Global,...
            Measurement_Matrix_Ship_Wave_Nominal,...
            Input_Measurement_Matrix_Global_Nominal);
    else
        Static_Gain_Ship_Nominal = 0.6043;
        Time_Constant_Ship_Nominal = -5.2097;
        State_Matrix_Ship_Wave_Nominal = [0 1;
                                          0 -1/Time_Constant_Ship_Nominal];
        Perturbation_Input_Matrix_Nominal = [0;1/Time_Constant_Ship_Nominal];
        Input_Matrix_Nominal = [0;Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal];
        Input_Matrix_Nominal_Global = [Input_Matrix_Nominal Perturbation_Input_Matrix_Nominal];
        Measurement_Matrix_Ship_Wave_Nominal = [1 0];
        Input_Measurement_Matrix_Global_Nominal = [0 0];
    end
    assignin('base', 'Ship_Wave_State_Space', Ship_Wave_State_Space);
    [State_Matrix_Standard_P_Optimization,...
     Input_Matrix_Standard_P_Optimization,...
     Measurement_Matrix_Standard_P_Optimization,...
     Input_Measurement_Matrix_Standard_P_Optimization] = linmod('Standard_H_Infinity_Full_Order_Synthesis');
else
    % State-space analytical determination under uncertainty:
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

    % Forced to use the analytical determined state-space reprezentation
    % since the automatic one uses ssdata, which does not embed uss
    % features.
    State_Matrix_Standard_P_Optimization        = Final_State_Matrix_Analytical;
    Input_Matrix_Standard_P_Optimization        = Final_Input_Matrix_Analytical;
    Measurement_Matrix_Standard_P_Optimization  = Final_Measurement_Matrix_Analytical;
    Input_Measurement_Matrix_Standard_P_Optimization = Final_Input_Measurement_Matrix_Analytical;
    % Nominal term used in the following due to the fact that the global
    % variables with "nominal" terms in it are used in the Simulink model.
    State_Matrix_Ship_Wave_Nominal = [0 1 0 0;
                                     -Omega_Wave_Pulsation_Uncertain^2 -2*Omega_Wave_Pulsation_Uncertain*Damping_Wave_Uncertain 0 0;
                                     0 0 0 1;
                                     0 0 0 -1/Time_Constant_Ship_Uncertain];
    Perturbation_Input_Matrix_Nominal = [0;Omega_Wave_Pulsation_Uncertain^2;0;0];
    Input_Matrix_Nominal = [0;0;0;Static_Gain_Ship_Uncertain/Time_Constant_Ship_Uncertain];
    Input_Matrix_Nominal_Global = [Input_Matrix_Nominal Perturbation_Input_Matrix_Nominal];
    Input_Measurement_Matrix_Global_Nominal = [0 0];
    % Measurement_Matrix_Ship_Wave_Nominal = [0 1 1 0];
    Measurement_Matrix_Ship_Wave_Nominal = [0 0 1 0];
    Ship_Wave_State_Space = ss(State_Matrix_Ship_Wave_Nominal,...
                               Input_Matrix_Nominal_Global,...
                               Measurement_Matrix_Ship_Wave_Nominal,...
                               Input_Measurement_Matrix_Global_Nominal);
    assignin('base', 'Ship_Wave_State_Space', Ship_Wave_State_Space);
end

% State-Space reprezentation of the plant considering the System 
% to be controlled (ship and wave) as well as the filters:
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

if ~Is_Uncertain_Model_Synthesis
    [State_Matrix_Closed_Loop_Nominal_Optimization,...
     Input_Matrix_Closed_Loop_Nominal_Optimization,...
     Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
     Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization] = linmod('Ship_Control_Loop_H_Infinity_Standard');
else
    load_system('Ship_Control_Loop_H_Infinity_Standard.slx');
    Uncertain_Linearized_Global_State_Space = ulinearize('Ship_Control_Loop_H_Infinity_Standard');
    State_Matrix_Closed_Loop_Nominal_Optimization = Uncertain_Linearized_Global_State_Space.A;
    Input_Matrix_Closed_Loop_Nominal_Optimization = Uncertain_Linearized_Global_State_Space.B;
    Measurement_Matrix_Closed_Loop_Nominal_Optimization = Uncertain_Linearized_Global_State_Space.C;
    Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization = Uncertain_Linearized_Global_State_Space.D;
end

% Closed loop System state-space considering the reference Heading Aimed as
% the input:
System_Closed_Loop_Reference = ss(State_Matrix_Closed_Loop_Nominal_Optimization,...
                                  Input_Matrix_Closed_Loop_Nominal_Optimization(:,1),...
                                  Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
                                  Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization(:,1));

% Closed loop System state-space considering the perturbation Step Wave as
% the input:
System_Closed_Loop_Perturbation = ss(State_Matrix_Closed_Loop_Nominal_Optimization,...
                                     Input_Matrix_Closed_Loop_Nominal_Optimization(:,2),...
                                     Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
                                     Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization(:,2));

% Frequency features of the closed loop perturbed System:
[Pulsation_Natural_Frequency,...
 Damping_System_Closed_Loop_Perturbation] = damp(System_Closed_Loop_Perturbation);

% Closed loop System state-space considering the Heading Noise as
% the input:
System_Closed_Loop_Noise = ss(State_Matrix_Closed_Loop_Nominal_Optimization,...
                              Input_Matrix_Closed_Loop_Nominal_Optimization(:,3),...
                              Measurement_Matrix_Closed_Loop_Nominal_Optimization,...
                              Input_Measurement_Matrix_Closed_Loop_Nominal_Optimization(:,3));
  
% System simulation:
Step_Time_Optimization = 0.1; 
Time_Horizon_Optimization = 50;
Time_Vector_Optimization = 0:Step_Time_Optimization:Time_Horizon_Optimization;

% Signal noise generation:
Size_Noise = length(Time_Vector_Optimization);
Heading_Perturbed_Minimum = -Heading_Variance_Noise;
Heading_Perturbed_Maximum = Heading_Variance_Noise;
Noise_Generated_Optimization = Heading_Perturbed_Minimum + (Heading_Perturbed_Maximum - Heading_Perturbed_Minimum)*rand(1, Size_Noise);

Output_Simulated_Heading = Step_Heading_Aimed*step(System_Closed_Loop_Reference,Time_Vector_Optimization);
Heading_Reference = Output_Simulated_Heading(:,3); 
Rudder_Angle_Input_Reference = Output_Simulated_Heading(:,2); 

Output_Simulated_Heading = Perturbation_Step*step(System_Closed_Loop_Perturbation,Time_Vector_Optimization);
Output_Perturbation_Heading = Output_Simulated_Heading(:,3);

Output_Simulated_Heading = lsim(System_Closed_Loop_Noise,Noise_Generated_Optimization,Time_Vector_Optimization);
Input_Response_Dynamic_System_Noise = Output_Simulated_Heading(:,2);

% Frequencial features:
if ~Is_Uncertain_Model_Synthesis
    [State_Matrix_System,...
     Input_Matrix_System,...
     Measurement_Matrix_System,...
     Input_Measurement_Matrix_System] = linmod('Standard_H_Infinity_Full_Order_Synthesis');
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
    State_Matrix_System = Final_State_Matrix_Analytical;
    Input_Matrix_System = Final_Input_Matrix_Analytical;
    Measurement_Matrix_System = Final_Measurement_Matrix_Analytical;
    Input_Measurement_Matrix_System = Final_Input_Measurement_Matrix_Analytical;
end
Open_Loop_Non_Corrected = ss(State_Matrix_System,...
                             Input_Matrix_System(:,4),...
                             Measurement_Matrix_System(3:4,:),...
                             Input_Measurement_Matrix_System(3:4,4));
Open_Loop_Corrector = -Controller_Optimization*Open_Loop_Non_Corrected; 
All_Margins = allmargin(Open_Loop_Corrector);
Pulsation_Frequency = min(All_Margins.PMFrequency);
% Frequencial features of the closed-loop:
Feedback_Loop = feedback(1,Open_Loop_Corrector);
Magnitude_Optimization = bode(Feedback_Loop);
Delta_Margin_Optimization = 1/max(Magnitude_Optimization);

% Deifnition of the criteria to be respected:
% Overshoot of the Heading in output:
Overshoot_Maximal = 3/100;
Reference_Function_1 = (max(Heading_Reference)-Step_Heading_Aimed*(1 + Overshoot_Maximal))/(Overshoot_Maximal);
% Time response at 4% lower than 15 seconds:
Overshoot_Maximal_Time = 15;
Overshoot_Maximal_Time_Percent = 2/100;
Reference_Function_3 = max(abs(Heading_Reference(Overshoot_Maximal_Time/Step_Time_Optimization:Time_Horizon_Optimization/Step_Time_Optimization)-Step_Heading_Aimed))/(Step_Heading_Aimed*Overshoot_Maximal_Time_Percent)-1;
% Command Rudder amplitude:
% Lower than 35°:
Delta_Rudder_Limit = 35*(pi/180);
Reference_Function_4 = max(abs(Rudder_Angle_Input_Reference))/Delta_Rudder_Limit-1;                     
% Perturbation response:
Time_Constraint_Perturbation = 30; 
Perturbation_Function_1 = max(abs(Output_Perturbation_Heading(Time_Constraint_Perturbation/Step_Time_Optimization:Time_Horizon_Optimization/Step_Time_Optimization)))/Perturbation_Step-1;
% Frequencies and pulsations:
Delta_Pulsation_Unit_Gain = 5;
Frequency_1 = Pulsation_Frequency/Delta_Pulsation_Unit_Gain-1;   
Delta_Margin_Specified = 0.5;
Frequency_2 = 1-Delta_Margin_Optimization/Delta_Margin_Specified;  
Damping_Aimed_Specified = 0.1;
Damping = 1-min(Damping_System_Closed_Loop_Perturbation)/Damping_Aimed_Specified;
% Limitations on the rudder rate:
Rudder_Rate_Maximal = 10*(pi/180);
Function_Constraint_Rudder_Rate = (max((diff(Rudder_Angle_Input_Reference))/Step_Time_Optimization)-Rudder_Rate_Maximal)/Rudder_Rate_Maximal;

Cost = Gamma_Optimal_Optimization + exp(1000*Reference_Function_1) + exp(1000*Reference_Function_3) ...
       + exp(1000*Reference_Function_4) + exp(1000*Perturbation_Function_1) + exp(1000*Damping) ...
       + exp(1000*Frequency_1) + exp(1000*Frequency_2) + exp(1000*Function_Constraint_Rudder_Rate) + std(Input_Response_Dynamic_System_Noise);

disp(sprintf('Criterion value : %6.4g',Cost));
disp(sprintf('Reference indicators : %6.4g %6.4g %6.4g',Reference_Function_1,Reference_Function_3,Reference_Function_4));
disp(sprintf('Perturbation indicators, ksi, w0, DM, std(ub) : %6.4g %6.4g %6.4g %6.4g %6.4g',Perturbation_Function_1,min(Damping_System_Closed_Loop_Perturbation),Pulsation_Frequency,Delta_Margin_Optimization,std(Input_Response_Dynamic_System_Noise)));

return

