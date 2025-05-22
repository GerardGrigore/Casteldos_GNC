% This script serves as a mean to model the contribution of the waves to the
% perturbation model.
% See Guidance & Control of ships page 86,87/494. (p.238 also)

% Initial pulsation wave: /!\ must be taken between 0.3 & 1.3 rad/s to be realistic /!\.
Pulsation_Wave = 0.7;
% Pulsation_Wave = 1.2;

% Gravitational acceleration:
Gravitational_Acceleration = 9.81;

% Sea waves angle:
Sea_Waves_Angle = -45*pi/180; 

% Encounter pulsation:
Pulsation_Encounter = Pulsation_Wave - ((Pulsation_Wave^2)/Gravitational_Acceleration)*Velocity_Ship_Mean*cos(Sea_Waves_Angle);

% Damping factor that shall be taken in between 0.01 & 0.1 (but shall stay fixed even online):
Damping_Factor_Waves = 0.1;

% Constant describing wave intensity:
Waves_Intensity_Factor = sqrt(10);

% Gain wave transfer function:
% Static_Gain_Waves = 2*Damping_Factor_Waves*Pulsation_Wave*Waves_Intensity_Factor;
Static_Gain_Waves = 2*Damping_Factor_Waves*Pulsation_Encounter*Waves_Intensity_Factor;

% Transfer function:
% Wave_Transfer_Function = (Static_Gain_Waves*s)/(s^2 + 2*Damping_Factor_Waves*Pulsation_Wave*s + Pulsation_Wave^2);
Wave_Transfer_Function = (Static_Gain_Waves*s)/(s^2 + 2*Damping_Factor_Waves*Pulsation_Encounter*s + Pulsation_Encounter^2);

% Settle the transfer function to canonical terms:
Factor_Wave_1 = Pulsation_Wave^2;
Factor_Wave_2 = 2*Damping_Factor_Waves*Pulsation_Wave;
Wave_Transfer_Function = ((Pulsation_Wave^2)*s)/(s^2 + Factor_Wave_2*s + Factor_Wave_1);

% Frequency description for zero-mean white Gaussian noise generation:
% On the lake, relatively low frequency but very low amplitude variations.
Frequency_Wave_Lake = 1;
Pulsation_Wave_Lake = 2*pi*Frequency_Wave_Lake;
Amplitude_Wave_Lake = 10*(pi/180);

% Discretization of the model to have realistic effect:
Wave_Transfer_Function_Discrete = c2d(Wave_Transfer_Function,Time_Period_Sampling,Discretization_Method);

% Consider a high-pass filter:
Pulsation_Filter_Wave = 0.6;
Time_Filter_Wave = 1/Pulsation_Filter_Wave;
High_Pass_Filter_Wave = (Time_Filter_Wave*s)/(1 + Time_Filter_Wave*s);
High_Pass_Filter_Wave_Discrete = c2d(High_Pass_Filter_Wave,Time_Period_Sampling,Discretization_Method);





