% This script serves as a mean to callibrate the MPC controller in order to
% pilot the motion of the ship.

clear all;
close all;
clc;

% Initialization parameters:
Static_Gain_Ship_Nominal = 0.6043;
Time_Constant_Ship_Nominal = -5.2097;
Time_Sampling = 0.1;
State_Matrix_Ship_Nominal = [1 Time_Sampling;
                             0 1-(Time_Sampling/Time_Constant_Ship_Nominal)];
Input_Matrix_Ship_Nominal = [0;Time_Sampling*(Static_Gain_Ship_Nominal/Time_Constant_Ship_Nominal)];
Measurement_Matrix_Ship_Nominal = [1 0;0 1];
Input_Measurement_Matrix_Ship_Nominal = [0;0];
State_Space_Model_Ship_Nominal_LTI = ss(State_Matrix_Ship_Nominal,...
                                        Input_Matrix_Ship_Nominal,...
                                        Measurement_Matrix_Ship_Nominal,...
                                        Input_Measurement_Matrix_Ship_Nominal);


















