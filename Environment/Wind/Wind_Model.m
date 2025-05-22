% This script serves as a mean to model the contribution of the wind to the
% perturbation model.
% See Guidance & Control of ships page 91/494.

% Air density:
Density_Air = 1.225;

% Lateral projected wind area:
Area_Lateral_Projected_Wind = 3; % Meters, to be justified.

% Turbulance factor definition:
Gain_Turbulence = 0.05;

% Relative wind velocity 10 meters above the ship:
Velocity_Wind_Ten_Meters_Above = 2;

% Gains of the trasfer function:
Gain_Wind = sqrt(5286*Gain_Turbulence*Velocity_Wind_Ten_Meters_Above);
Time_Constant_Wind = sqrt(286/Velocity_Wind_Ten_Meters_Above);

% Laplace parameter:
s = tf('s');

% Transfer function:
Wind_Transfer_Function = Gain_Wind/(1 + Time_Constant_Wind*s);

% The wind can have relatively high influence on the ship's heading and
% occurs at very high frequency:
Frequency_Wind_Lake = 100;
Pulsation_Wind_Lake = 2*pi*Frequency_Wind_Lake;
Amplitude_Wind_Lake = 4*(pi/180);

% Creation of a corresponding initial not interpolated table containing the
% values of the Yaw Coefficient of Wind in function of the Relative Wind
% Angle:
Structure_Wind_Coefficient(1:15,1) = [0,20,40,50,60,90,130,150,170,...
                                      210,240,270,300,340,345]';
Structure_Wind_Coefficient(1:15,2) = [0,-0.1,-0.18,-0.12,0.01,0.1,...
                                      0.06,0,-0.08,-0.095,-0.001,...
                                      0.12,0.13,0.1,0]';

















