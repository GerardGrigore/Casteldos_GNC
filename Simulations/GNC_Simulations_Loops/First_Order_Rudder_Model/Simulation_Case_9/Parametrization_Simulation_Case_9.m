clear all;
close all;
clc;

% Parametrization script associated with the 'Simulator_Case_9' Simulink
% model. The main features of this case are:

% * Guidance Line of Sight (LOS) Waypoints-based algorithm.
% * Wave environnement switched to continuous instead of discrete (physics
%   is continuous).
% * Creation of an online hydrodynamics coefficients estimation due to
%   velocity LPV model adoption.
% * Implementation of an order three controller synthetized using the
%   standard H_Infinity method and stochastic optimization.
% * LPV in velocity ship model implementation.
% * Profile of nom-velocity creation.
% * Simplification of the EKF Algorithm in order to just incorporate the
%   velocity and not accelations anymore. Simplification of the model of
%   the sensors, using only additive noise generation on the data.
% * Modification of the Kalman Filter Observer covariance matrices.

% ---Enter the path of the root Casteldos_GNC_Software folder:-------
Path_Casteldos_GNC_Software = "C:\Users\gerar\Documents\GitHub\";
% -------------------------------------------------------------------

% Discrete time features:
Time_Period_Sampling = 0.1;
Discretization_Method = char('tustin');

% Parameters importation:
% Parameters of the ship, hydrodynamics coefficients and Nomoto's transfer functions:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Ship_Modeling\Ship_Parameters_Modeling\Ship_Model_Nomoto.m");
% H_Infinity synthesis data:
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\H_Infinity_Synthesis\Standard_H_Infinity_Tuning\Standard_H_Infinity_Controller_Tuning.m");
run(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\H_Infinity_Synthesis\Standard_H_Infinity_Tuning\Aposteriori_Corrector_Order_Reduction.m");
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
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Guidance\Guidance_Algorithms\Integrated_Line_Of_Sight"));
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
% Ship LPV functions:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Ship_Modeling\Ship_Parameters_Modeling"));
% H_Infinity synthesis data:
addpath(genpath(Path_Casteldos_GNC_Software + "Casteldos_GNC\Control\Controller\H_Infinity_Synthesis\Standard_H_Infinity_Tuning"));

% Initial states declaration:
Position_Initial_X = Waypoints_X(1);
Position_Initial_Y = Waypoints_Y(1);
Velocity_Norm_Ship_Initial = 6/3.6;

% Declination of Sigean zone:
Declination_Sigean = 2.116*(pi/180);
