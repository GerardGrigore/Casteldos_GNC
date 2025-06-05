clear all;
close all;
clc;

% This script serves as a rapid prototype to tune a several kinds of
% controller by an equivalent of pole placement techniques. The goal is to
% study the stability of the ship control by observing simulated heading.

% Global parameters:
Static_Gain_Ship = 0.604;
Time_Constant_Ship = -5.5;
s = tf('s');
Time_Period_Sampling = 0.1;

% Transfer function:
Rudder_To_Heading_Continuous = Static_Gain_Ship/(s*(1 + Time_Constant_Ship*s));
Rudder_To_Heading_Discrete = c2d(Rudder_To_Heading_Continuous,Time_Period_Sampling,'tustin');

% Pulsation_Natural = 0.5;
% Damping_Factor = 1;
Pulsation_Natural = 0.3;
Damping_Factor = 1.5;
Pulsation_Ship = 1/Time_Constant_Ship;

% Bandwidth condition checking:
Pulsation_Bandwidth = Pulsation_Natural*(sqrt(1 - 2*Damping_Factor^2 + sqrt(4*Damping_Factor^4 - 4*Damping_Factor^2 + 2)));
if Pulsation_Bandwidth < Pulsation_Ship
    error('Carefull, error of pulsation tuning for the desired closed-loop.');
end

% Controller gains definition:
Gain_Proportional = (Time_Constant_Ship*Pulsation_Natural^2)/Static_Gain_Ship;
Gain_Derivative = (2*Damping_Factor*Pulsation_Natural*Time_Constant_Ship - 1)/Static_Gain_Ship;
Gain_Integral = (Pulsation_Natural^3/2)*(Time_Constant_Ship/Static_Gain_Ship);
Gain_Filter = 0.01;

% Continuous and discrete controllers definition:
Controller_PID_Continuous = Gain_Proportional + Gain_Integral/s + Gain_Derivative*s;
Controller_PIDF_Continuous = Gain_Proportional + Gain_Integral/s + ((Gain_Derivative*s)/(1 + Gain_Filter*s));
Controller_PID_Discrete = c2d(Controller_PID_Continuous,Time_Period_Sampling,'tustin');
Controller_PIDF_Discrete = c2d(Controller_PIDF_Continuous,Time_Period_Sampling,'tustin');
Controller_PDF_Continuous = Gain_Proportional + ((Gain_Derivative*s)/(1 + Gain_Filter*s));
Controller_PDF_Discrete =  c2d(Controller_PDF_Continuous,Time_Period_Sampling,'tustin');

% Stability evaluation through Nichols/Bode charts:
figure;
nichols(Rudder_To_Heading_Continuous);
hold on;
nichols(Controller_PIDF_Continuous*Rudder_To_Heading_Continuous);
legend('Countinuous ship','Open loop controlled ship using PIDF-Controller');
title('Continuous PIDF control');

figure;
nichols(Rudder_To_Heading_Continuous);
hold on;
nichols(Controller_PDF_Continuous*Rudder_To_Heading_Continuous);
legend('Countinuous ship','Open loop controlled ship using PDF-Controller');
title('Continuous PDF control');

figure;
nichols(Rudder_To_Heading_Discrete);
hold on;
nichols(Controller_PIDF_Discrete*Rudder_To_Heading_Discrete);
hold on;
nichols(Controller_PDF_Discrete*Rudder_To_Heading_Discrete);
legend('Discrete ship','Open loop controlled ship using PIDF discrete Controller',...
       'Open loop controlled ship using PDF discrete Controller');
title('Discrete PIDF/PDF control');

figure;
nichols(Rudder_To_Heading_Continuous);
hold on;
nichols(Controller_PIDF_Continuous*Rudder_To_Heading_Continuous);
hold on;
nichols(Controller_PID_Continuous*Rudder_To_Heading_Continuous);
hold on;
nichols(Controller_PDF_Continuous*Rudder_To_Heading_Continuous);
legend('Plant','PIDF discrete control','PID discrete control','PDF discrete control');
title('Continuous PID/F control');

figure;
bode(Controller_PID_Continuous*Rudder_To_Heading_Continuous);
hold on;
bode(Controller_PIDF_Continuous*Rudder_To_Heading_Continuous);
legend('PID','PIDF');




















