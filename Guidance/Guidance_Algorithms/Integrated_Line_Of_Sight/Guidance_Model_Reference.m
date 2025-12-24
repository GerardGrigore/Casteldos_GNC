function [Heading_Desired_Current,...
          Angular_Rate_Desired_Current,...
          Angular_Acceleration_Desired_Current] = Guidance_Model_Reference(Heading_Raw_Aimed,...
                                                                           Time_Sampling)

% Initialization:
% Persistents for reference model used:
persistent Angular_Acceleration_Desired_Previous;
persistent Rudder_Rate_Desired_Previous;
persistent Heading_Desired_Previous;
persistent Heading_Aimed_Previous_Raw;
if isempty(Heading_Desired_Previous)
    Heading_Desired_Previous = 0;
    Rudder_Rate_Desired_Previous = 0;
    Angular_Acceleration_Desired_Previous = 0;
    Heading_Aimed_Previous_Raw = 0;
end
Rudder_Rate_Saturation = 8*(pi/180);
Rudder_Acceleration_Saturation = 10*(pi/180);
Damping_Reference_Model = 1; % Smooth.
Pulsation_Reference_Model = 0.2; % Smaller thant the one of the closed-loop bandwidth.

Jerk_Desired_Current = -(2*Damping_Reference_Model + 1)*Pulsation_Reference_Model*Angular_Acceleration_Desired_Previous - (2*Damping_Reference_Model + 1)*Pulsation_Reference_Model^2*...
                       Rudder_Rate_Desired_Previous + Pulsation_Reference_Model^3*(Heading_Aimed_Previous_Raw - Heading_Desired_Previous);
Angular_Acceleration_Desired_Current = Angular_Acceleration_Desired_Previous + Time_Sampling*Jerk_Desired_Current;
% Saturation:
Angular_Acceleration_Desired_Current = max(min(Angular_Acceleration_Desired_Current, Rudder_Acceleration_Saturation), -Rudder_Acceleration_Saturation);

% Angular rate iteration:
% Angular_Rate_Desired_Current = Rudder_Rate_Desired_Previous + Time_Sampling*Acceleration_Desired_Previous;
Angular_Rate_Desired_Current = Rudder_Rate_Desired_Previous + Time_Sampling*Angular_Acceleration_Desired_Current;
% Saturation:
Angular_Rate_Desired_Current = max(min(Angular_Rate_Desired_Current, Rudder_Rate_Saturation), -Rudder_Rate_Saturation);

% Heading desired:
% Heading_Desired_Current = Heading_Desired_Previous + Time_Sampling*Rate_Desired_Previous;
Heading_Desired_Current = Heading_Desired_Previous + Time_Sampling*Angular_Rate_Desired_Current;

% Persistent update:
Heading_Aimed_Previous_Raw = Heading_Raw_Aimed;
Rudder_Rate_Desired_Previous = Angular_Rate_Desired_Current;
Angular_Acceleration_Desired_Previous = Angular_Acceleration_Desired_Current;
Heading_Desired_Previous = Heading_Desired_Current;

end

