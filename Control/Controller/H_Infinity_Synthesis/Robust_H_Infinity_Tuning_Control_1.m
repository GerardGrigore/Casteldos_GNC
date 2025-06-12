clear all;
close all;
clc;

% This script serves as a mean to tune a structured PIDF controller using
% the H-Infinity method. Exploration of the use of systune and hinfstruct
% will be done. In this first case, only the use if systune has been
% explored.

% Define the Nomoto's first order transfer function:
Static_Gain_Ship = 0.604;
Time_Constant_Ship = -5.5;
s = tf('s');
Time_Period_Sampling = 0.1;
Rudder_To_Heading_Continuous = Static_Gain_Ship/(s*(1 + Time_Constant_Ship*s));
Rudder_To_Heading_Discrete = c2d(Rudder_To_Heading_Continuous,Time_Period_Sampling,'tustin');

% Define the structure of the controller:
Controller_PIDF_Structure = tunablePID('Controller_PIDF','pid');

% Define the analysis point:
% An analysis point are the locations where the loop can be opened. For
% example, for signal injection about requirements on the system.
Analysis_Point_Direct_Chain = AnalysisPoint('Analysis_Point_Direct_Chain');

% Connection of the components to build an entire feedback controlled
% system:
Closed_Loop = feedback(Rudder_To_Heading_Continuous*Controller_PIDF_Structure,Analysis_Point_Direct_Chain);
Closed_Loop.InputName = 'Heading_Aimed';
Closed_Loop.OutputName = 'Heading_Observed';

% Specification of several requirements:
% Tracking requirement:
% The response time of the system:
Time_Step_Response = 10;
% The error of the output:
Heading_Error_Percent = 0.1;
Requirement_Tracking = TuningGoal.Tracking('Heading_Aimed','Heading_Observed',Time_Step_Response,Heading_Error_Percent);
% Rejection requirement:
% The attenuation of a disturbance injected at Analysis_Point_Direct_Chain 
% must be suppressed by a factor of Factor_Suppression_Disturbance.
Factor_Suppression_Disturbance = 5;
Requirement_Rejection = TuningGoal.Gain('Analysis_Point_Direct_Chain','Heading_Observed',Factor_Suppression_Disturbance);

% Tuning of the control system:
Number_Random_Start_Points = 10;
Termination_Relative_Tolerance = 0.001;
Synthesis_Options = systuneOptions('RandomStart',Number_Random_Start_Points,'Display','iter',...
                                    'SoftTol',Termination_Relative_Tolerance);
[Closed_Loop_Optimized,Best_Achieved_Soft_Contraint] = systune(Closed_Loop,[Requirement_Tracking,Requirement_Rejection],...
                                                               [],Synthesis_Options);

% Temporal validation of the synthetized controller:
figure;
stepplot(Closed_Loop_Optimized);

% Check the rejection requirement:
Closed_Loop_Disturbance = getIOTransfer(Closed_Loop_Optimized,'Analysis_Point_Direct_Chain','Heading_Observed');
figure;
stepplot(Closed_Loop_Disturbance);








