clear all;
close all;
clc;

% Parametrization script associated with the 'Simulator_Case_6' Simulink
% model. The main features of this case are:

% * Guidance Waypoints-based algorithm.
% * LQ Controller tuned to track a commanded heading attitude.
% * Actuator (anticipated) saturation in angular position and angular rate limitations.
% * First-order Nomoto ship model relating the rudder angle to the heading.
% * Realistic Navigation: two sensors modeling GPS and Magnetometer.
% * Sensor Fusion and Extended Kalman Filter algorithm for noise reduction
%   and data fusion.
% * Environment: waves considered as a perturbation.
% * Kalman Filter wave observer for high frequency filtering.
% * High-Pass filter and ARMA Online Optimized identification algorithm to
%   estimate the wave frequency and damping factor.
% * Discretized functions and discrete time environement.
% * 2 DoF added: positions of the ship.

% ---Enter the path of the root Casteldos_GNC_Software folder:-------
Path_Casteldos_GNC_Software = "C:\Users\gerar\Desktop\Bureau bis\";
% -------------------------------------------------------------------

% Discrete time features:
Time_Period_Sampling = 0.1;
Discretization_Method = char('tustin');
Time_Guidance_Sampling = 300; % Guidance algorithm sampling time.

% Parameters importation:
% Parameters of the ship, hydrodynamics coefficients and Nomoto's transfer functions:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Ship_Modeling\Ship_Parameters_Modeling\Ship_Model_Nomoto.m");
% PIDF Synthesis launch:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Control\Controller\PID\PID_Tuning_Case_2.m");
% Wind generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Environment\Wind\Wind_Model.m");
% Waves generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Environment\Waves\Waves_Model.m");
% Trajectory generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Guidance\Trajectories\Ile_De_La_Nadiere\Mission_Waypoint_Generation_In_ENU.m");
% LQ Controller synthesis:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Control\Controller\LQR\LQR_Tuning_Heading_Control.m");

% Repositories sourcing for algorithms:
% Guidance function:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Guidance\Guidance_Algorithms\Waypoints_Based"));
% LLA conversion to ENU coordinates function:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Guidance\Trajectories\Ile_De_La_Nadiere"));
% Wind generator and estimator:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Environment\Wind"));
% ARMA identification, Kalman Sensor fusion and Wave observer algorithms:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Navigation"));
% Online LQ Tracking controller:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Control\Controller\LQR"));
% Icons & pictures importation for masks:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC_Software\Icons"));

% Initial states declaration:
Reference_Point_Port_Mahon = [43.059023,3.002915,0];
Position_Initial_X = Waypoints_X(1);
Position_Initial_Y = Waypoints_Y(1);
Heading_Initial = -65*pi/180;

% Desired decimal year study:
Desired_Year_IGRF = decyear('31-December-2024','dd-mmm-yyyy');

% Declination of Sigean's zone:
Declination_Sigean = 2.116*(pi/180);