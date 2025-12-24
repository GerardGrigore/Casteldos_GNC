% This script serves as a mean to tune the standard H infinity controller.
% Standard H infinity control synthesis is applied to the ship plant augmented 
% by an additive wave perturbation. All the tuning will be done considering the
% continuous models of the ship and wave.

% clear all
% close all
% clc

% Plant and System definition:
% ----------------------------
% Physical parameters:
Static_Gain_Ship_Nominal = 0.6043;
Time_Constant_Ship_Nominal = -5.2097;
Omega_Wave_Nominal = 0.6;
Damping_Wave_Nominal = 0.1;

% State-Space System definition:
State_Matrix_Ship_Wave_Nominal = [0 1 0 0;
                                  -Omega_Wave_Nominal^2 -2*Omega_Wave_Nominal*Damping_Wave_Nominal 0 0;
                                  0 0 0 1;
                                  0 0 0 -1/Time_Constant_Ship_Nominal];
Perturbation_Input_Matrix_Nominal = [0;Omega_Wave_Nominal^2;0;0];
Input_Matrix_Nominal = [0;0;0;Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal];
Input_Matrix_Nominal_Global = [Input_Matrix_Nominal Perturbation_Input_Matrix_Nominal];
% Measurement_Matrix_Ship_Wave_Nominal = [0 1 1 0];
Measurement_Matrix_Ship_Wave_Nominal = [0 0 1 0];
Input_Measurement_Matrix_Global_Nominal = [0 0];
Ship_Wave_State_Space = ss(State_Matrix_Ship_Wave_Nominal,...
                           Input_Matrix_Nominal_Global,...
                           Measurement_Matrix_Ship_Wave_Nominal,...
                           Input_Measurement_Matrix_Global_Nominal);

% H_infinity synthesis:
% ---------------------
Number_Of_Synthetized_Controllers = 11;
Selected_Synthesis_Tuning_Number = 11; 

if Selected_Synthesis_Tuning_Number == 1
    % Manual tuning of the filters - Tuning n° 1:
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 3/20;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000; % Increased value in order to impose a low
    % gain for the reference to input command transfer.
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 0.8; % Lowest value possible to participate
    % in the lowering of the gain of the transfer from input reference heading
    % aimed to the commanded rudder => to impose the lowest rudder amplitude
    % command possible. However, imposing such a low pulsation and high gain
    % results in a very high gamma synthesis (around 30).
    % Filter 3:
    Gain_Infinity_Filter_3 = 10;
    Gain_Static_Filter_3 = 0.1;
    Pulsation_Unit_Gain_Filter_3 = 70;
    % Filter 4:
    Gain_Static_Filter_4 = 0.1;
end

if Selected_Synthesis_Tuning_Number == 2
    % Stochastic optimization for tuning
    % Tuning n°2:
    % Features: large possibilities in terms of overshoot and no stringent
    % constraint in terms of response time:
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.015;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 7.625;
    % Filter 3:
    Gain_Infinity_Filter_3 = 1;
    Gain_Static_Filter_3 = 0.01;
    Pulsation_Unit_Gain_Filter_3 = 700;
    % Filter 4:
    Gain_Static_Filter_4 = 0.05359;
end

if Selected_Synthesis_Tuning_Number == 3
    % Tuning n°3:
    % Features: tighter bounds on the response time.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.02616;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 3.295;
    % Filter 3:
    Gain_Infinity_Filter_3 = 17.51;
    Gain_Static_Filter_3 = 0.02635;
    Pulsation_Unit_Gain_Filter_3 = 220.9;
    % Filter 4:
    Gain_Static_Filter_4 = 0.552;
end

if Selected_Synthesis_Tuning_Number == 4
    % Tuning n°4:
    % Features: allowing the response to a higher step command and higher
    % perturbation.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.03369;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 1.553;
    % Filter 3:
    Gain_Infinity_Filter_3 = 7.973;
    Gain_Static_Filter_3 = 0.0104;
    Pulsation_Unit_Gain_Filter_3 = 45.92;
    % Filter 4:
    Gain_Static_Filter_4 = 0.4555;
end

if Selected_Synthesis_Tuning_Number == 5
    % Tuning n°5:
    % Features: allowing the response to an even higher step command and higher
    % perturbation.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.148;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 0.4861;
    % Filter 3:
    Gain_Infinity_Filter_3 = 175.1;
    Gain_Static_Filter_3 = 0.003067;
    Pulsation_Unit_Gain_Filter_3 = 484;
    % Filter 4:
    Gain_Static_Filter_4 = 0.05542;
end

if Selected_Synthesis_Tuning_Number == 6
    % Tuning n°6:
    % Features: Sysnthetized controller using uss model.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.01423;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 1.319;
    % Filter 3:
    Gain_Infinity_Filter_3 = 0.6187;
    Gain_Static_Filter_3 = 0.006533;
    Pulsation_Unit_Gain_Filter_3 = 787.9;
    % Filter 4:
    Gain_Static_Filter_4 = 0.4485;
end

if Selected_Synthesis_Tuning_Number == 7
    % Tuning n°7:
    % Features: Sysnthetized controller using uss model.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.01423;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 1.489;
    % Filter 3:
    Gain_Infinity_Filter_3 = 0.373;
    Gain_Static_Filter_3 = 0.01287;
    Pulsation_Unit_Gain_Filter_3 = 1094;
    % Filter 4:
    Gain_Static_Filter_4 = 0.5722;
end

if Selected_Synthesis_Tuning_Number == 8
    % Tuning n°8:
    % Features: Sysnthetized controller using uss model.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.03601;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 2.745;
    % Filter 3:
    Gain_Infinity_Filter_3 = 5.025;
    Gain_Static_Filter_3 = 0.002517;
    Pulsation_Unit_Gain_Filter_3 = 1012;
    % Filter 4:
    Gain_Static_Filter_4 = 0.5529;
end

if Selected_Synthesis_Tuning_Number == 9
    % Tuning n°9:
    % Features: Sysnthetized controller using uss model and allowing
    % higher response time/overshoot, but aiming for higher damping factor
    % (around 0.3 minimum).
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 3.275;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 0.01741;
    % Filter 3:
    Gain_Infinity_Filter_3 = 0.008225;
    Gain_Static_Filter_3 = 0.07177;
    Pulsation_Unit_Gain_Filter_3 = 3.48;
    % Filter 4:
    Gain_Static_Filter_4 = 0.005167;
end

if Selected_Synthesis_Tuning_Number == 10
    % Tuning n°10:
    % Features: Sysnthetized controller using uss model and allowing
    % lower response time/overshoot, but aiming for higher damping factor
    % (around 0.3 minimum).
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 13.47;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 0.02949;
    % Filter 3:
    Gain_Infinity_Filter_3 = 0.0005295;
    Gain_Static_Filter_3 = 0.006914;
    Pulsation_Unit_Gain_Filter_3 = 103.1;
    % Filter 4:
    Gain_Static_Filter_4 = 5.103e-05;
end

if Selected_Synthesis_Tuning_Number == 11
    % Tuning n°11:
    % Features: Sysnthetized controller using nominal ss model bu another
    % state-space reprezentation involving only a measurement on the low
    % frequency heading.
    % Filter 1:
    Gain_Infinity_Filter_1 = 0.5;
    Gain_Static_Filter_1 = 1000;
    Pulsation_Unit_Gain_Filter_1 = 0.07752;
    % Filter 2:
    Gain_Infinity_Filter_2 = 10000;
    Gain_Static_Filter_2 = 0.01;
    Pulsation_Unit_Gain_Filter_2 = 2.528;
    % Filter 3:
    Gain_Infinity_Filter_3 = 0.1199;
    Gain_Static_Filter_3 = 0.01205;
    Pulsation_Unit_Gain_Filter_3 = 238.4;
    % Filter 4:
    Gain_Static_Filter_4 = 0.367;
end

% Standard P(s) plant reprezentation in the Model named
% 'Standard_H_Infinity_Full_Order_Synthesis'.
[State_Matrix_Standard_P,...
 Input_Matrix_Standard_P,...
 Measurement_Matrix_Standard_P,...
 Input_Measurement_Matrix_Standard_P] = linmod('Standard_H_Infinity_Full_Order_Synthesis');
% State-Space reprezentation:
Pland_Standard_P_Nominal_State_Space = ss(State_Matrix_Standard_P,...
                                          Input_Matrix_Standard_P,...
                                          Measurement_Matrix_Standard_P,...
                                          Input_Measurement_Matrix_Standard_P);
% Observability and Controllability checks:
Observability_Nominal_Plant_Standard = obsv(Pland_Standard_P_Nominal_State_Space);
Controlability_Nominal_Plant_Standard = ctrb(Pland_Standard_P_Nominal_State_Space);
Number_Uncontrolable_States_Nominal_Plant_Standard = length(State_Matrix_Standard_P) - rank(Controlability_Nominal_Plant_Standard);
Number_Unobservable_States_Nominal_Plant_Standard = length(State_Matrix_Standard_P) - rank(Observability_Nominal_Plant_Standard);
if Number_Uncontrolable_States_Nominal_Plant_Standard == 0
    sprintf('The System Nominal Ship, Waves and Filters transformed in Standard Plant has no uncotrolable states.')
else
    warning('The System Nominal Ship, Waves and Filters transformed in Standard Plant has %i uncotrolable states.',Number_Uncontrolable_States_Nominal_Plant_Standard);
end
if Number_Unobservable_States_Nominal_Plant_Standard == 0
    sprintf('The System Nominal Ship, Waves and Filters transformed in Standard Plant has no unobservable states.')
else
    warning('The System Nominal Ship, Waves and Filters transformed in Standard Plant has %i unobservable states.',Number_Unobservable_States_Nominal_Plant_Standard);
end

% Use of hinfsyn function for Hinfinity calculation of the corrector:
Number_Inputs_Controller = 2;
Number_Commands_To_Be_Produced = 1;
[Controller,...
 Closed_Loop,...
 Gamma_Optimal] = hinfsyn(Pland_Standard_P_Nominal_State_Space,...
                          Number_Inputs_Controller,...
                          Number_Commands_To_Be_Produced,'display','on');

% Minimal reprezentation:
Controller = minreal(Controller);
[State_Matrix_Controller_Nominal,...
 Input_Matrix_Controller_Nominal,...
 Measurement_Matrix_Controller_Nominal,...
 Input_Measurement_Matrix_Controller_Nominal] = ssdata(Controller);










