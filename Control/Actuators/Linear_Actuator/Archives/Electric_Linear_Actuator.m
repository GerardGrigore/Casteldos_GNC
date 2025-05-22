%-----------------------------------------------------------
% CONTROL ACTUATOR: Internal parameters definition:
% ----------------------------------------------------------

% To be done: need to try all the values to be tested and for several
% parameters. See the influence of each terms on the time & frequential
% responses. Check, in terms of response time & stability, if just a
% proportional action is enough for the control.

% Adaptator parameters to convert a linear position in a tension:
Tension_Maximal = 12;
Pitch_Screw = 12; % /!\ Must try several values /!\.
Amplifier = 5.44; % /!\ Must try several values /!\.

% Gear reduction/transmission values:
Ratio_Screw_Sensor = 10; % /!\ Must try several values /!\. Done, this parameter, if increased, increases the 
% time response of the closed loop system.
Ratio_Motor_Screw = 30; % /!\ Must try several values /!\. Done, this parameter, if decreased, increases the
% time response of the closed loop system.

% Motor parameters:
Static_Gain_Motor = 37; % /!\ Must try several values /!\. Done, this parameter, if increased, decreases the
% time response of the closed loop system.
Time_Response_Motor = 11e-4; % /!\ Must try several values /!\. Done, no significant impact detected.

% Command values:
Position_Aimed = 0.2;

% Overall equivalent transfert function:
Linear_Term = (2*pi*Ratio_Screw_Sensor*Ratio_Motor_Screw)/(Amplifier*Static_Gain_Motor*Tension_Maximal);
Quadratic_Term = (2*pi*Ratio_Screw_Sensor*Ratio_Motor_Screw*Time_Response_Motor)/(Amplifier*Static_Gain_Motor*Tension_Maximal);
s = tf('s');
Linear_Actuator_Transfer_Function = 1/(1 + Linear_Term*s + Quadratic_Term*s^2);

% Controller:
Controller_Linear_Actuator = 100;

% Movement from linear to rotating:
Steering_Motor = 0.3;

% Stability assessment:
figure(1)
bode(Controller_Linear_Actuator*Linear_Actuator_Transfer_Function); % Assessment shows that there is a stable closed loop response with the actual parameters.
figure(2)
nichols(Controller_Linear_Actuator*Linear_Actuator_Transfer_Function);