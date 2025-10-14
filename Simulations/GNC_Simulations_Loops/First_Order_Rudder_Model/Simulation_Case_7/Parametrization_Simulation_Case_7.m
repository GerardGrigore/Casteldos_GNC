clear all;
close all;
clc;

% Parametrization script associated with the 'Simulator_Case_7' Simulink
% model. The main features of this case are:

% * Guidance Waypoints-based algorithm.
% * Controller-PDF and staturations modelling the actuator.
% * LTI Ship first order dynamics.
% * Simple dynamics EKF with constant velocity assumptions.
% * ARMA Identification online algorithm for wave estimation.

% ---Enter the path of the root Casteldos_GNC_Software folder:-------
Path_Casteldos_GNC_Software = "C:\Users\gerar\Documents\GitHub\";
% -------------------------------------------------------------------

% Discrete time features:
Time_Period_Sampling = 0.1;
Discretization_Method = char('tustin');
Time_Guidance_Sampling = 20; % Guidance algorithm sampling time.

% Parameters importation:
% Parameters of the ship, hydrodynamics coefficients and Nomoto's transfer functions:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Ship_Modeling\Ship_Parameters_Modeling\Ship_Model_Nomoto.m");
% PIDF Synthesis launch:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\PID\Synthesis_Tuning\PID_Tuning_Case_2.m");
% Wind generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Environment\Wind\Wind_Model.m");
% Waves generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Environment\Waves\Waves_Model.m");
% Trajectory generation:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Trajectories\Ile_De_La_Nadiere\Mission_Waypoint_Generation_In_ENU.m");
% Linear actuator features:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Actuators\Linear_Actuator\Linear_Actuator_Features.m");


% Repositories sourcing for algorithms:
% Guidance function:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Guidance_Algorithms\Waypoints_Based"));
% LLA conversion to ENU coordinates function:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Trajectories\Ile_De_La_Nadiere"));
% Wind generator and estimator:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Environment\Wind"));
% ARMA identification, Sensor fusion and Wave observer algorithms:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Navigation"));
% Anti Wind-Up discretized PIDF control algorithm:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Saturation_Control"));
% Icons & pictures importation for masks:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Icons"));
% Embedded control algorithms:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\PID\Embedded_Algorithms"));
% Actuators models:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Actuators\Linear_Actuator"));


% Initial states declaration:
Reference_Point_Port_Mahon = [43.059023,3.002915,0];
Position_Initial_X = Waypoints_X(1);
Position_Initial_Y = Waypoints_Y(1);
Heading_Initial = -65*pi/180;

% Desired decimal year study:
Desired_Year_IGRF = decyear('31-December-2024','dd-mmm-yyyy');

% Declination of Sigean zone:
Declination_Sigean = 2.116*(pi/180);


