function Rudder_Angle_Elaborated_Current = Controller_PDF_Algorithm(Heading_Aimed_Current,...
                                                                    Heading_Observed_Current,...
                                                                    Gain_Proportional,...
                                                                    Gain_Filter,...
                                                                    Gain_Derivative,...
                                                                    Time_Sampling)

% This is the Control algorithm written in the form of a simple discretized
% Proportional Derivative Filter (PDF). The discretization method used is
% the Tustin Bilinear Transform.

% Persistents initialization:
persistent Rudder_Angle_Commanded_Previous;
persistent Heading_Error_Previous;
if isempty(Rudder_Angle_Commanded_Previous)
    Rudder_Angle_Commanded_Previous = 0;
end
if isempty(Heading_Error_Previous)
    Heading_Error_Previous = 0;
end

% Current heading error:
Heading_Error_Current = Heading_Aimed_Current - Heading_Observed_Current;

% Current rudder angle command:
Rudder_Angle_Elaborated_Current = Rudder_Angle_Commanded_Previous*((2*Gain_Filter - Time_Sampling)/(Time_Sampling + 2*Gain_Filter)) + ...
                                  Heading_Error_Current*((Gain_Proportional*Time_Sampling + 2*Gain_Proportional*Gain_Filter + 2*Gain_Derivative)/(Time_Sampling + 2*Gain_Filter)) + ...
                                  Heading_Error_Previous*((Gain_Proportional*Time_Sampling - 2*Gain_Proportional*Gain_Filter - 2*Gain_Derivative)/(Time_Sampling + 2*Gain_Filter));

% Then persistents update:
Rudder_Angle_Commanded_Previous = Rudder_Angle_Elaborated_Current;
Heading_Error_Previous = Heading_Error_Current;

end
