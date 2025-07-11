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
% Pulsation_Natural = 1.5;
Pulsation_Natural = 0.3;

% Define the damping factor of the closed-loop controlled system:
Damping_Coefficient = 1.5;
% Damping_Coefficient = 0.7;

% Check the Bandwidth value and establish numerical protection:
Pulsation_Bandwidth = Pulsation_Natural*(sqrt(1 - 2*Damping_Coefficient^2 + sqrt(4*Damping_Coefficient^4 - 4*Damping_Coefficient^2 + 2)));
if Pulsation_Bandwidth < Pulsation_Ship
    error('The pulsation bandwidth of the overall loop must be greater than the natural pulsation');
end

% Controller design:
% Adding a filter to the PID/PD controllers to make them causal in LTI block of Simulink:
Filter_Coefficient = 100000;
s_LTI = (Filter_Coefficient*s)/(s + Filter_Coefficient); % Equivalent to Laplace's function 's'.
Gain_Filter = 0.01;

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
Controller_PDF_Continuous = Gain_Proportional + Gain_Derivative*s_LTI;
Controller_PDF_Classical_Continuous = Gain_Proportional + ((Gain_Derivative*s)/(1 + Gain_Filter*s));
% Discrete time domain:
% ---------------------
Controller_PD_Discrete = c2d(Controller_PD,Time_Period_Sampling,Discretization_Method);
Controller_PDF_Discrete = c2d(Controller_PDF_Continuous,Time_Period_Sampling,Discretization_Method);
Controller_PDF_Classical_Discrete = c2d(Controller_PDF_Classical_Continuous,Time_Period_Sampling,Discretization_Method);

% PID Controllers design:
% -----------------------
% Continuous time domain:
% -----------------------
% Determine the gains:
% The Proportional and Derivative gains have been previously determined.
Gain_Integral = (Pulsation_Natural^3/2)*(Time_Constant_Total/Static_Gain_Nomoto);
Time_Integral = Gain_Proportional/Gain_Integral;
% Transfer functions:
Controller_PID = Gain_Proportional + Gain_Derivative*s + Gain_Integral/s;
Controller_PIDF_Continuous = Gain_Proportional + Gain_Integral/s + Gain_Derivative*s_LTI;
Controller_PIDF_Classical_Continuous = Gain_Proportional + Gain_Integral/s + ((Gain_Derivative*s)/...
                                       (1 + Gain_Filter*s));
% Discrete time domain:
% ---------------------
Controller_PID_Discrete = c2d(Controller_PID,Time_Period_Sampling,Discretization_Method);
Controller_PIDF_Discrete = c2d(Controller_PIDF_Continuous,Time_Period_Sampling,Discretization_Method);
Controller_PIDF_Classical_Discrete = c2d(Controller_PIDF_Classical_Continuous,Time_Period_Sampling,Discretization_Method);

% Bode & Nichols plots for freqeuncy stability analysis:
% ------------------------------------------------------
% Continuous analysis:
% --------------------
% PD-Controller:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PD*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous PD Controlled ship');
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
title('Open-loop composed by continuous PID-Controller and Ship transfer function');
% PIDF-Controller:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PIDF_Continuous*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous PID-Controlled ship');
title('Open-loop composed by continuous PIDF-Controller and Ship transfer function');
% PIFD Classical:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PIDF_Classical_Continuous*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous classical PIDF-Controlled ship');
title('Open-loop composed by continuous classical PID Controller and Ship transfer function');
% PDF Classical:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Controller_PDF_Classical_Continuous*Heading_On_Rudder_Transfer_First_Order);
legend('Ship transfert function','Continuous classical PDF-Controlled ship');
title('Open-loop composed by continuous classical PD Controller and Ship transfer function');

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
% PIDF-Classical Controller:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PIDF_Classical_Discrete*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PIDF-Controlled ship');
title('Open-loop composed by Discrete PIDF-Controller classical and Ship trasnfer function');
% PDF-Classical Controller:
figure;
nichols(Rudder_To_Heading_Discretized);
hold on;
nichols(Controller_PDF_Classical_Discrete*Rudder_To_Heading_Discretized);
legend('Discrete ship transfert function','Discrete PDF-Controlled ship');
title('Open-loop composed by Discrete PDF-Controller classical and Ship trasnfer function');




















