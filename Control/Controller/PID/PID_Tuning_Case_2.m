% This script aims at tuning the PID/PIDF controllers to control the
% heading of the ship. The controllers are tuned in the continuous time
% domain but are converted in discrete time using the Tustin bilinear
% transformation.
% For a complete and detailed analysis of the choice of the parameter,
% refer to the concerned section of the attached technical performance
% note.

% Convert the plant to the discrete time domain:
Rudder_To_Heading_Discretized = c2d(Heading_On_Rudder_Transfer_First_Order,Time_Period_Sampling,Discretization_Method);

% Retreive the pulsation associated with the ship's transfer function:
Pulsation_Ship = 1/Time_Constant_Total;

% Define the natural pulsation of the closed-loop system:
Pulsation_Natural = 1.5;
% Pulsation_Natural = 1.1;

% Define the damping factor of the closed-loop controlled system:
Damping_Coefficient = 1;
% Damping_Coefficient = 0.7;

% Check the Bandwidth value and establish numerical protection:
Pulsation_Bandwidth = Pulsation_Natural*(sqrt(1 - 2*Damping_Coefficient^2 + sqrt(4*Damping_Coefficient^4 - 4*Damping_Coefficient^2 + 2)));
if Pulsation_Bandwidth < Pulsation_Ship
    error('The pulsation bandwidth of the overall loop must be greater than the natural pulsation');
end

% Controller design:
% Adding a filter to the PID/PD controllers to make them causal in LTI block of Simulink:
Filter_Coefficient = 1000;
Filter_Derivative = (Filter_Coefficient*s)/(s + Filter_Coefficient);

% Adding a filter for the overall PID:
Time_Filter = 0.1;
Filter_Continuous = 1/(1 + Time_Filter*s);

% PD Controllers design:
% ----------------------
% Continuous time domain:
% -----------------------
% Determine the gains:
Gain_Proportional = (Time_Constant_Total*Pulsation_Natural^2)/Static_Gain_Nomoto;
Gain_Derivative = (2*Damping_Coefficient*Pulsation_Natural*Time_Constant_Total - 1)/Static_Gain_Nomoto;
Time_Derivative = Gain_Derivative/Gain_Proportional;
% Establish the transfer functions:
Controller_PD = Gain_Proportional + Gain_Derivative*s;
Controller_PDF_Continuous = Gain_Proportional + Gain_Derivative*Filter_Derivative;
% Discrete time domain:
% ---------------------
Controller_PD_Discrete = c2d(Controller_PD,Time_Period_Sampling,Discretization_Method);
Controller_PDF_Discrete = c2d(Controller_PDF_Continuous,Time_Period_Sampling,Discretization_Method);

% PID Controllers design:
% -----------------------
% Continuous time domain:
% -----------------------
% Determine the gains:
% The Proportional and Derivative gains have been previously determined.
Gain_Integral = (Pulsation_Natural^3/10)*(Time_Constant_Total/Static_Gain_Nomoto);
Time_Integral = Gain_Proportional/Gain_Integral;
% Transfer functions:
Controller_PID = Gain_Proportional + Gain_Derivative*s + Gain_Integral/s;
Controller_PIDF_Continuous = Gain_Proportional + Gain_Integral/s + Gain_Derivative*Filter_Derivative;
Controller_PIDF_Filtered_Continuous = Gain_Proportional + Gain_Integral/s + Gain_Derivative*Filter_Derivative + Filter_Continuous;
% Discrete time domain:
% ---------------------
Controller_PID_Discrete = c2d(Controller_PID,Time_Period_Sampling,Discretization_Method);
Controller_PIDF_Discrete = c2d(Controller_PIDF_Continuous,Time_Period_Sampling,Discretization_Method);
Controller_PIDF_Discrete_Filtered = c2d(Controller_PIDF_Filtered_Continuous,Time_Period_Sampling,Discretization_Method);

% Bode & Nichols plots for freqeuncy stability analysis:
% ------------------------------------------------------
% Continuous analysis:
% --------------------
% PD-Controller:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PD*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous PD-Controlled ship');
title('Open-loop composed by continuous PD-Controller and Ship transfer function');
% PDF-Controller:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PDF_Continuous*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous PDF-Controlled ship');
title('Open-loop composed by continuous PDF-Controller and Ship transfer function');
% PID-Controller:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PID*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous PID-Controlled ship');
title('Open-loop composed by continuous PID-Controller and Ship trasnfer function');
% PIDF-Controller:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PIDF_Continuous*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous PID-Controlled ship');
title('Open-loop composed by continuous PID-Controller and Ship trasnfer function');
% PIDF-Controller + Filter:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PIDF_Filtered_Continuous*Heading_On_Rudder_Transfer_First_Order);
legend('Continuous ship transfert function','Continuous PIDF-Controlled + Filter ship');
title('Open-loop composed by Continuous PIDF-Controller & Filter and Ship transfer function');

% Discrete analysis:
% ------------------
% PD-Controller:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PD_Discrete*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PD-Controlled ship');
title('Open-loop composed by discrete PD-Controller and Ship transfer function');
% PDF-Controller:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PDF_Discrete*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PDF-Controlled ship');
title('Open-loop composed by discrete PDF-Controller and Ship transfer function');
% PID-Controller:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PID_Discrete*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PID-Controlled ship');
title('Open-loop composed by discrete PID-Controller and Ship transfer function');
% PIDF-Controller:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PIDF_Discrete*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PIDF-Controlled ship');
title('Open-loop composed by Discrete PIDF-Controller and Ship trasnfer function');
% PIDF-Controller + Filter:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PIDF_Discrete_Filtered*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PIDF-Controlled + Filter ship');
title('Open-loop composed by Discrete PIDF-Controller & Filter and Ship trasnfer function');






















