clear all;
close all;
clc;

% Parametrization script associated with the 'Simulator_Case_3' Simulink
% model. The main features of this case are:

% * Ideal Guidance: simple step orders.
% * Ideal Controller based only on a PIDF tuning: no actuators.
% * Actuator (anticipated) saturation in angular position.
% * First-order Nomoto ship model relating the rudder angle to the heading.
% * Ideal Navigation: no sensors.
% * Environment: waves considered.
% * Kalman Filter wave observer for high frequency filtering.
% * Discretized functions and discrete time environement.

% ---Enter the path of the root Casteldos_GNC_Software folder:-------
Path_Casteldos_GNC_Software = "C:\Users\gerar\Documents\GitHub\";
% -------------------------------------------------------------------

% Discrete time features:
Time_Period_Sampling = 0.1;
Discretization_Method = char('tustin');

% The script containing the parameters of the ship, the hydrodynamics
% coefficients and the Nomoto's transfer functions:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Ship_Modeling\Ship_Parameters_Modeling\Ship_Model_Nomoto.m");
% PIDF Synthesis launch:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\PID\PID_Tuning_Case_2.m");
% Waves generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Environment\Waves\Waves_Model.m");
% Trajectory generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Trajectories\Ile_De_La_Nadiere\Mission_Waypoint_Generation_In_ENU.m");

