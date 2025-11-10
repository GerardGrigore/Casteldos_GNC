% This script serves as a mean to model the contribution of the waves as the
% perturbation model acting on the heading of the ship.

% Initial pulsation wave: must be taken between 0.3 & 1.3 rad/s to be realistic.
Pulsation_Wave = 1.2;
% Pulsation_Wave = 1.2;

% Gravitational acceleration:
Gravitational_Acceleration = 9.81;

% Sea waves angle:
Sea_Waves_Angle = -45*pi/180; 

% Encounter pulsation:
Pulsation_Encounter = Pulsation_Wave - ((Pulsation_Wave^2)/Gravitational_Acceleration)*Velocity_Ship_Mean*cos(Sea_Waves_Angle);

% Damping factor that shall be taken in between 0.01 & 0.1:
Damping_Factor_Waves = 0.1;

% Constant describing wave intensity:
Waves_Intensity_Factor = sqrt(10);

% Gain wave transfer function:
% Static_Gain_Waves = 2*Damping_Factor_Waves*Pulsation_Wave*Waves_Intensity_Factor;
Static_Gain_Waves = 2*Damping_Factor_Waves*Pulsation_Encounter*Waves_Intensity_Factor;

% Settle the transfer function to canonical terms:
Factor_Wave_1 = Pulsation_Wave^2;
Factor_Wave_2 = 2*Damping_Factor_Waves*Pulsation_Wave;
Wave_Transfer_Function = ((Pulsation_Wave^2)*s)/(s^2 + Factor_Wave_2*s + Factor_Wave_1);

% Amplitude of the heading wave perturbation:
Heading_Wave_Perturbation_Input = 10*(pi/180);



