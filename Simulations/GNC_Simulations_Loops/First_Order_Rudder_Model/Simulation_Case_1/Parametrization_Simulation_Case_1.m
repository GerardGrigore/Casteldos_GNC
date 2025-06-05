clear all;
close all;
clc;

% Parametrization script associated with the 'Simulator_Case_1' Simulink
% model. The main features of this case are:

% * Guidance Waypoints-based algorithm.
% * Ideal Controller based only on a PIDF tuning: no actuators.
% * First-order Nomoto ship model relating the rudder angle to the heading.
% * Ideal Navigation: no sensors.
% * Ideal environment: no perturbation considered.

% ---Enter the path of the root Casteldos_GNC_Software folder:-------
Path_Casteldos_GNC_Software = "C:\Users\gerar\Documents\GitHub\";
% -------------------------------------------------------------------

% Parameters importation:
% Parameters of the ship, hydrodynamics coefficients and Nomoto's transfer functions:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Ship_Modeling\Ship_Parameters_Modeling\Ship_Model_Nomoto.m");
% PIDF Synthesis launch:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\PID\PID_Tuning_Case_0.m");
% PIDF Synthesis launch:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\PID\PID_Tuning_Case_1.m");
% Trajectory generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Trajectories\Ile_De_La_Nadiere\Mission_Waypoint_Generation_In_ENU.m");

% Repositories sourcing for algorithms:
% Guidance function:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Guidance_Algorithms\Waypoints_Based"));
% LLA conversion to ENU coordinates function:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Trajectories\Ile_De_La_Nadiere"));

% Initial states declaration:
Position_Initial_X = Waypoints_X(1);
Position_Initial_Y = Waypoints_Y(1);



