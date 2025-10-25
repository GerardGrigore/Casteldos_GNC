clear all;
close all;
clc;

% This script serves as a mean to tune the standard H infinity controller.
% Standard H infinity control synthesis is applied to the ship plant.
% All the tuning will be done considering the continuous model of the ship.

% Initialization of the ship's transfer functions:
% Nominal:
% --------
Static_Gain_Ship_Nominal = 0.6043;
Time_Constant_Ship_Nominal = -5.2097;
State_Matrix_Ship_Nominal = [0 1;
                             0 -1/Time_Constant_Ship_Nominal];
Input_Matrix_Ship_Nominal = [0;Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal];
Measurement_Matrix_Ship_Nominal = [1 1]; % System considered to be observable.
Input_Measurement_Matrix_Ship_Nominal = 0;
Ship_Heading_Rate_State_Space_Nominal = ss(State_Matrix_Ship_Nominal,...
                                           Input_Matrix_Ship_Nominal,...
                                           Measurement_Matrix_Ship_Nominal,...
                                           Input_Measurement_Matrix_Ship_Nominal);
% Poles of the ship system:
Poles_Ship_Nominal = pole(Ship_Heading_Rate_State_Space_Nominal);
% As the state matrix is triangular superior: the poles are readable on its
% diagonal: 0 and -1/Time_Constant_Ship_Nominal leading to an unstable
% system.
% Transfer function definition:
s = tf('s');
Ship_Heading_Rate_Transfer_Nominal = Static_Gain_Ship_Nominal/(s*(1 + Time_Constant_Ship_Nominal*s));

% Minimal:
% --------
Static_Gain_Ship_Minimal = 4.9000;
Time_Constant_Ship_Minimal = -385.9724;
State_Matrix_Ship_Minimal = [0 1;
                             0 -1/Time_Constant_Ship_Minimal];
Input_Matrix_Ship_Minimal = [0;Static_Gain_Ship_Minimal/Time_Constant_Ship_Minimal];
Measurement_Matrix_Ship_Minimal = [1 1]; % System considered to be observable.
Input_Measurement_Matrix_Ship_Minimal = 0;
Ship_Heading_Rate_State_Space_Minimal = ss(State_Matrix_Ship_Minimal,...
                                           Input_Matrix_Ship_Minimal,...
                                           Measurement_Matrix_Ship_Minimal,...
                                           Input_Measurement_Matrix_Ship_Minimal);

% Maximal:
% --------
Static_Gain_Ship_Maximal = 0.8384;
Time_Constant_Ship_Maximal = -0.3209;
State_Matrix_Ship_Maximal = [0 1;
                             0 -1/Time_Constant_Ship_Maximal];
Input_Matrix_Ship_Maximal = [0;Static_Gain_Ship_Maximal/Time_Constant_Ship_Maximal];
Measurement_Matrix_Ship_Maximal = [1 1]; % System considered to be observable.
Input_Measurement_Matrix_Ship_Maximal = 0;
Ship_Heading_Rate_State_Space_Maximal = ss(State_Matrix_Ship_Maximal,...
                                           Input_Matrix_Ship_Maximal,...
                                           Measurement_Matrix_Ship_Maximal,...
                                           Input_Measurement_Matrix_Ship_Maximal);

% Dispersion definition of the static gain and the time constant to
% observe, on the Bode diagram, the frequency response of the system:
Number_Dispersed_Functions = 10;
Static_Gain_Ship_Dispersed_Vector = linspace(Static_Gain_Ship_Nominal,Static_Gain_Ship_Minimal,Number_Dispersed_Functions);
Time_Constant_Ship_Dispersed_Vector = linspace(Time_Constant_Ship_Minimal,Time_Constant_Ship_Maximal,Number_Dispersed_Functions);
figure;
for index_dispersions = 1:Number_Dispersed_Functions
    hold on;
    Local_Dispersed_Ship_Transfer_Function = Static_Gain_Ship_Dispersed_Vector(index_dispersions)/(s*(1 + Time_Constant_Ship_Dispersed_Vector(index_dispersions)*s));
    bode(Local_Dispersed_Ship_Transfer_Function);
end
title('Frequency response of the ship (no waves) to be controlled - Bode diagram.');
% The Bode diagram shows that the response of the system is affected by the
% variation of parameters in LF, MF and HF (all the spectrum).

% Same for the nichols plot:
figure;
for index_dispersions = 1:Number_Dispersed_Functions
    hold on;
    Local_Dispersed_Ship_Transfer_Function = Static_Gain_Ship_Dispersed_Vector(index_dispersions)/(s*(1 + Time_Constant_Ship_Dispersed_Vector(index_dispersions)*s));
    nichols(Local_Dispersed_Ship_Transfer_Function);
end
title('Frequency response of the ship (no waves) to be controlled - Nichols diagram.');

% Augmented system with the perturbations due to waves:
Omega_Wave_Nominal = 0.6;
Omega_Wave_Minimal = 0.3;
Omega_Wave_Maximal = 1.3;
Damping_Wave_Nominal = 0.1;
Damping_Wave_Minimal = 0.09;
Damping_Wave_Maximal = 0.25;

% State-space and transfer function definitions:
% Nominal:
% --------
State_Matrix_Ship_Wave_Nominal = [0 1 0 0;
                                  -Omega_Wave_Nominal^2 -2*Omega_Wave_Nominal*Damping_Wave_Nominal 0 0;
                                  0 0 0 1;
                                  0 0 0 -1/Time_Constant_Ship_Nominal];
Input_Matrix_Ship_Wave_Nominal = [0 0;0 Omega_Wave_Nominal^2;0 0;Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal 0];
Measurement_Matrix_Ship_Wave_Nominal = [0 1 1 0];
% Measurement_Matrix_Ship_Wave_Nominal = [0 0 1 0];
Input_Measurement_Matrix_Ship_Wave_Nominal = [0 0];
Ship_Wave_Nominal_State_Space = ss(State_Matrix_Ship_Wave_Nominal,...
                                   Input_Matrix_Ship_Wave_Nominal,...
                                   Measurement_Matrix_Ship_Wave_Nominal,...
                                   Input_Measurement_Matrix_Ship_Wave_Nominal);
Ship_Wave_Nominal_Transfer_Function = tf(Ship_Wave_Nominal_State_Space);
% Observability and commandability check on the model:
Observability_Nominal = obsv(Ship_Wave_Nominal_State_Space);
Controlability_Nominal = ctrb(Ship_Wave_Nominal_State_Space);
Number_Uncontrolable_States_Nominal = length(State_Matrix_Ship_Wave_Nominal) - rank(Controlability_Nominal);
Number_Unobservable_States_Nominal = length(State_Matrix_Ship_Wave_Nominal) - rank(Observability_Nominal);
if Number_Uncontrolable_States_Nominal == 0
    sprintf('The System Nominal Ship and Waves has no uncotrolable states.')
else
    warning('The System Nominal Ship and Waves has %i uncotrolable states.',Number_Uncontrolable_States_Nominal);
end
if Number_Unobservable_States_Nominal == 0
    sprintf('The System Nominal Ship and Waves has no unobservable states.')
else
    warning('The System Nominal Ship and Waves has %i unobservable states.',Number_Unobservable_States_Nominal);
end

% Minimal:
% --------
State_Matrix_Ship_Wave_Minimal = [0 1 0 0;
                                  -Omega_Wave_Minimal^2 -2*Omega_Wave_Minimal*Damping_Wave_Minimal 0 0;
                                  0 0 0 1;
                                  0 0 0 -1/Time_Constant_Ship_Minimal];
Input_Matrix_Ship_Wave_Minimal = [0 0;0 Omega_Wave_Minimal^2;0 0;Static_Gain_Ship_Minimal/Time_Constant_Ship_Minimal 0];
Measurement_Matrix_Ship_Wave_Minimal = [0 1 1 0];
Input_Measurement_Matrix_Ship_Wave_Minimal = [0 0];
Ship_Wave_Minimal_State_Space = ss(State_Matrix_Ship_Wave_Minimal,...
                                   Input_Matrix_Ship_Wave_Minimal,...
                                   Measurement_Matrix_Ship_Wave_Minimal,...
                                   Input_Measurement_Matrix_Ship_Wave_Minimal);
Ship_Wave_Minimal_Transfer_Function = tf(Ship_Wave_Minimal_State_Space);

% Maximal:
% --------
State_Matrix_Ship_Wave_Maximal = [0 1 0 0;
                                  -Omega_Wave_Maximal^2 -2*Omega_Wave_Maximal*Damping_Wave_Maximal 0 0;
                                  0 0 0 1;
                                  0 0 0 -1/Time_Constant_Ship_Maximal];
Input_Matrix_Ship_Wave_Maximal = [0 0;0 Omega_Wave_Maximal^2;0 0;Static_Gain_Ship_Maximal/Time_Constant_Ship_Maximal 0];
Measurement_Matrix_Ship_Wave_Maximal = [0 1 1 0];
Input_Measurement_Matrix_Ship_Wave_Maximal = [0 0];
Ship_Wave_Maximal_State_Space = ss(State_Matrix_Ship_Wave_Maximal,...
                                   Input_Matrix_Ship_Wave_Maximal,...
                                   Measurement_Matrix_Ship_Wave_Maximal,...
                                   Input_Measurement_Matrix_Ship_Wave_Maximal);
Ship_Wave_Maximal_Transfer_Function = tf(Ship_Wave_Maximal_State_Space);

% Plot of the bodes:
figure;
hold on;
bode(Ship_Wave_Nominal_Transfer_Function);
hold on;
bode(Ship_Wave_Minimal_Transfer_Function);
hold on;
bode(Ship_Wave_Maximal_Transfer_Function);
title('Bode frequency response of the augmented system of the ship and the waves.');

% Dispersion of the transfer functions:
% For the ship, see the previous lines.
% For the waves:
Omega_Wave_Dispersions = linspace(Omega_Wave_Minimal,Omega_Wave_Maximal,Number_Dispersed_Functions);
Damping_Wave_Dispersions = linspace(Damping_Wave_Minimal,Damping_Wave_Maximal,Number_Dispersed_Functions);
figure;
for index_wave_dispersion = 1:Number_Dispersed_Functions
    hold on;
    Wave_Transfer_Function = (Omega_Wave_Dispersions(index_wave_dispersion)^2)/(s^2 + 2*Omega_Wave_Dispersions(index_wave_dispersion)*Damping_Wave_Dispersions(index_wave_dispersion)*s + Omega_Wave_Dispersions(index_wave_dispersion)^2);
    bode(Wave_Transfer_Function);
end
title('Wave transfer functions dispersed.');

% A third state-space system can be chosen as a one with an additive
% perturbation AND additive input, and not the two inputs in the same
% vector.
% Our data in input is as follows:
% Nominal:
% --------
Perturbation_Input_Matrix_Nominal = [0;Omega_Wave_Nominal^2;0;0];
Input_Matrix_Nominal = [0;0;0;Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal];
Input_Matrix_Nominal_Global = [Input_Matrix_Nominal Perturbation_Input_Matrix_Nominal];
Input_Measurement_Matrix_Global_Nominal = [0 0];
Ship_Wave_Nominal_State_Space_Dynamics = ss(State_Matrix_Ship_Wave_Nominal,...
                                            Input_Matrix_Nominal_Global,...
                                            Measurement_Matrix_Ship_Wave_Nominal,...
                                            Input_Measurement_Matrix_Global_Nominal);
% Observability and Controllability checks:
Observability_Nominal_Dynamic_Model = obsv(Ship_Wave_Nominal_State_Space_Dynamics);
Controlability_Nominal_Dynamic_Model = ctrb(Ship_Wave_Nominal_State_Space_Dynamics);
Number_Uncontrolable_States_Nominal_Dynamic_Model = length(State_Matrix_Ship_Wave_Nominal) - rank(Controlability_Nominal_Dynamic_Model);
Number_Unobservable_States_Nominal_Dynamic_Model = length(State_Matrix_Ship_Wave_Nominal) - rank(Observability_Nominal_Dynamic_Model);
if Number_Uncontrolable_States_Nominal_Dynamic_Model == 0
    sprintf('The System Nominal Ship and Waves has no uncotrolable states.')
else
    warning('The System Nominal Ship and Waves has %i uncotrolable states.',Number_Uncontrolable_States_Nominal_Dynamic_Model);
end
if Number_Unobservable_States_Nominal_Dynamic_Model == 0
    sprintf('The System Nominal Ship and Waves has no unobservable states.')
else
    warning('The System Nominal Ship and Waves has %i unobservable states.',Number_Unobservable_States_Nominal_Dynamic_Model);
end

% Minimal:
% --------
Perturbation_Input_Matrix_Minimal = [0;Omega_Wave_Minimal^2;0;0];
Input_Matrix_Minimal = [0;0;0;Static_Gain_Ship_Minimal/Time_Constant_Ship_Minimal];
Input_Matrix_Minimal_Global = [Input_Matrix_Minimal Perturbation_Input_Matrix_Minimal];
Input_Measurement_Matrix_Global_Minimal = [0 0];
Ship_Wave_Minimal_State_Space_Dynamics = ss(State_Matrix_Ship_Wave_Minimal,...
                                            Input_Matrix_Minimal_Global,...
                                            Measurement_Matrix_Ship_Wave_Minimal,...
                                            Input_Measurement_Matrix_Global_Minimal);

% Maximal:
% --------
Perturbation_Input_Matrix_Maximal = [0;Omega_Wave_Maximal^2;0;0];
Input_Matrix_Maximal = [0;0;0;Static_Gain_Ship_Maximal/Time_Constant_Ship_Maximal];
Input_Matrix_Maximal_Global = [Input_Matrix_Maximal Perturbation_Input_Matrix_Maximal];
Input_Measurement_Matrix_Global_Maximal = [0 0];
Ship_Wave_Maxiaml_State_Space_Dynamics = ss(State_Matrix_Ship_Wave_Maximal,...
                                            Input_Matrix_Maximal_Global,...
                                            Measurement_Matrix_Ship_Wave_Maximal,...
                                            Input_Measurement_Matrix_Global_Maximal);

figure;
bode(Ship_Wave_Nominal_State_Space_Dynamics);
hold on;
bode(Ship_Wave_Minimal_State_Space_Dynamics);
hold on;
bode(Ship_Wave_Maxiaml_State_Space_Dynamics);
title('Frequency response of the system composed by the ship and the waves - Bode diagram.');

% H_infinity synthesis:
% ---------------------
% Filter choice:
% I) Manually tuning the parameters of the filter:
% ------------------------------------------------
% % Filter 1:
% Gain_Infinity_Filter_1 = 0.5; % Initially 0.5
% Gain_Static_Filter_1 = 1000; % Initially 100. <=
% Pulsation_Unit_Gain_Filter_1 = 3/20; % Initially 1.5.
% % Filter 2:
% Gain_Infinity_Filter_2 = 10000; % Initially 100. Increased value in order to impose a low gain for the reference to input command transfer.
% Gain_Static_Filter_2 = 0.01;
% Pulsation_Unit_Gain_Filter_2 = 0.8; % To be tuned, try 100 rad/s. No, instead, lowest value possible to participate
% % in the lowering of the gain of the transfer from input reference heading
% % aimed to the commanded rudder => to impose the lowest rudder amplitude
% % command possible. However, imposing such a low pulsation and high gain
% % results in a very high gamma synthesis (around 30).
% % Filter 3:
% Gain_Infinity_Filter_3 = 10; % To be tuned. 10 initially.
% Gain_Static_Filter_3 = 0.1; % Initially 0.1.
% Pulsation_Unit_Gain_Filter_3 = 70; % To be tuned, try 70 rad/s.
% % Filter 4:
% Gain_Static_Filter_4 = 0.1; % Initially 0.1.

% II) Stochastic optimization for tuning the parameters of the filter:
% --------------------------------------------------------------------
% 1) Tuning n°1:
% --------------
% Features: large possibilities in terms of overshoot and no stringent
% constraint in terms of response time:
% % Filter 1:
% Gain_Infinity_Filter_1 = 0.5; 
% Gain_Static_Filter_1 = 1000;
% Pulsation_Unit_Gain_Filter_1 = 0.015;
% % Filter 2:
% Gain_Infinity_Filter_2 = 10000;
% Gain_Static_Filter_2 = 0.01;
% Pulsation_Unit_Gain_Filter_2 = 7.625; 
% % Filter 3:
% Gain_Infinity_Filter_3 = 1;
% Gain_Static_Filter_3 = 0.01;
% Pulsation_Unit_Gain_Filter_3 = 700; 
% % Filter 4:
% Gain_Static_Filter_4 = 0.05359; 
% 1) Tuning n°2:
% --------------
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










