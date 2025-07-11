% This script serves as a mean to tune a PIDF controller providing with the
% control rudder angle input to enslave the heading of the ship. This
% tuning concerns the control of the ship's plant only.
% For a complete and detailed analysis of the choice of the parameter,
% refer to the concerned section of the attached technical performance
% note.

% Import the transfer function of the Ship;
Ship_Transfer_Function = Heading_On_Rudder_Transfer_First_Order;

% Plot the Bode and Nichols graphs to assess initial stability of the
% uncorrected plant:
figure;
bode(Ship_Transfer_Function);
title('Bode of the ship Open-Loop transfer function');
figure;
nichols(Ship_Transfer_Function);
title('Nichols of the ship Open-Loop transfer function');

% Manually retreive the frequency at zero dB gain:
Frequency_Zero_Decibels = 0.662;

% Check the values:
[Gain_Ship_Zero_Initial,Phase_Ship_Zero_Initial] = bode(Ship_Transfer_Function,Frequency_Zero_Decibels);

% Determine the initial phase margin:
Phase_Margin_Initial_Radians = (180 + Phase_Ship_Zero_Initial)*(pi/180); % In radians.
Phase_Margin_Initial_Degrees = (180 + Phase_Ship_Zero_Initial); % In degrees.

% Constant correction:
Pulsation_Ratio = 10; % Usually chosen between 2 and 10;

% The objective is to add some phase at the zero-gain frequency of the ship. Hence, the
% lead-lag frequency part of the controller (real 'PD' controller) is equals to the zero-gain
% frequency of the plant:
Frequency_Lead_Lag = Frequency_Zero_Decibels;

% The PI integral action must be active at low frequency only.
% Force it to act before the Critical Point.
Frequency_Proportional_Integral = Frequency_Zero_Decibels/Pulsation_Ratio;

% For the filtering part, there is a need to filter the high frequency
% since the controller adds some gain at these frequencies:
Frequency_Filter = Pulsation_Ratio*Frequency_Zero_Decibels;

% Calculate the grain and the phase of the ship at the Frequency_Zero_Decibels
% point:
[Ship_Gain_Transfer,Ship_Phase_Transfer] = bode(-Ship_Transfer_Function,Frequency_Zero_Decibels); % Minus because the static
% gain of the transfer function under study is negative.

% Fix the aimed phase margin at Frequency_Zero_Decibels for the system:
Phase_Margin_Aimed = 40*(pi/180);

% Retreive the last parameters of the PIDF controller:
Phase_Integer_Filter = -2*atan(1/Pulsation_Ratio);
Lead_Lag_Gain = tan((Phase_Margin_Aimed - (pi/2) - Phase_Integer_Filter - Ship_Phase_Transfer*(pi/180))/2);
Gain_Proportional = -1/(Lead_Lag_Gain*Ship_Gain_Transfer); % Minus to add here because the transfer function of the
% studied system is negative.

% Determine the transfer function of the controller:
Controller_Proportional_Integral = (1 + s/Frequency_Proportional_Integral)/(s/Frequency_Proportional_Integral);
Controller_Lead_Lag_Derivative = (1 + Lead_Lag_Gain*(s/Frequency_Lead_Lag))/(1 + (s/(Lead_Lag_Gain*Frequency_Lead_Lag)));
Controller_Filter = 1/(1 + s/Frequency_Filter);
Controller_PIDF = Gain_Proportional*Controller_Proportional_Integral*Controller_Lead_Lag_Derivative*Controller_Filter;

% Check the margin:
figure;
bode(Controller_PIDF*Ship_Transfer_Function);
title('Bode of the Open-Loop with the controller and the plant');
figure;
margin(Controller_PIDF*Ship_Transfer_Function);

% Nichols diagram of the open loop system:
figure;
nichols(Controller_PIDF*Ship_Transfer_Function);
title('Nichols of the Open-Loop with the controller and the plant');

% Nichols diagram of the closed loop system:
figure;
nichols(feedback(Controller_PIDF*Ship_Transfer_Function,1));
title('Nichols of the Closed-Loop with the controller and the plant');