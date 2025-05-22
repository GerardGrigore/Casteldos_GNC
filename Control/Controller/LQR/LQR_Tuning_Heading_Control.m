% This script aims at tuning the parameters of the LQR controller based on
% continuous and discrete time models of the Plant.
% For a complete and detailed analysis of the choice of the parameter,
% refer to the concerned section of the attached technical performance
% note.

% The simple model 'LQR_Prototype_Tuning' was used to observe the
% influence of the parameters tuning for the LQR.

% Model of the plant to control:
Static_Gain_Ship_LQ = Static_Gain_Nomoto;
Time_Constant_LQ = Time_Constant_Total;

% Transfer function definition:
Rudder_To_Heading = Static_Gain_Ship_LQ/(s*(1 + Time_Constant_LQ*s));
Rudder_To_Heading_Discrete = c2d(Rudder_To_Heading,Time_Period_Sampling,Discretization_Method);

% LQ weight parameters tuning:
State_Matrix_Weigth = [45 0;0 0.01];
Input_Matrix_Weight = 15000;

% State-space matrices reprezentation:
State_Matrix_Ship_Continuous = [0 1;0 -1/Time_Constant_LQ];
Input_Matrix_Ship_Continuous = [0;Static_Gain_Ship_LQ/Time_Constant_LQ];
Measurement_Matrix_Ship_Continuous = [1 0];
Input_Measurement_Matrix_Ship_Continuous = 0;
Ship_Heading_To_Rudder_State_Space_Continuous = ss(State_Matrix_Ship_Continuous,...
                                                   Input_Matrix_Ship_Continuous,...
                                                   Measurement_Matrix_Ship_Continuous,...
                                                   Input_Measurement_Matrix_Ship_Continuous);
Ship_Heading_To_Rudder_State_Space_Discrete = ss(State_Matrix_Ship_Continuous,...
                                                 Input_Matrix_Ship_Continuous,...
                                                 Measurement_Matrix_Ship_Continuous,...
                                                 Input_Measurement_Matrix_Ship_Continuous,...
                                                 Time_Period_Sampling);

% Continuous LQ tuning:
[Optimal_Gain_Continuous,...
 Solution_Riccati_Continuous,...
 Poles_Closed_Loop_Continuous] = lqr(Ship_Heading_To_Rudder_State_Space_Continuous,...
                                     State_Matrix_Weigth,...
                                     Input_Matrix_Weight);
% Discrete LQ tuning:
[Optimal_Gain_Discrete,...
 Solution_Riccati_Discrete,...
 Poles_Closed_Loop_Discrete] = dlqr(State_Matrix_Ship_Continuous,...
                                    Input_Matrix_Ship_Continuous,...
                                    State_Matrix_Weigth,...
                                    Input_Matrix_Weight);

% Constant for controller:
Controller_Continuous = -inv(Input_Matrix_Weight)*Input_Matrix_Ship_Continuous'*Solution_Riccati_Continuous;
Controller_Discrete = -inv(Input_Matrix_Weight)*Ship_Heading_To_Rudder_State_Space_Discrete.B'*Solution_Riccati_Discrete;
Optimal_Gain_Heading = -inv(Measurement_Matrix_Ship_Continuous*inv(State_Matrix_Ship_Continuous - ...
                       Input_Matrix_Ship_Continuous*Optimal_Gain_Continuous)*Input_Matrix_Ship_Continuous);

% Optimal control gains:
Component_1_Optimal_Gain = Optimal_Gain_Continuous(1);
Component_2_Optimal_Gain = Optimal_Gain_Continuous(2);
State_Matrix_Tracking = State_Matrix_Ship_Continuous - Input_Matrix_Ship_Continuous*Optimal_Gain_Continuous;
Input_Matrix_Tracking = Input_Matrix_Ship_Continuous*Component_1_Optimal_Gain;











