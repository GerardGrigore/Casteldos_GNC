function Rudder_Angle_Elaborated_Current = Controller_PDF_Synthesis_Online_Algorithm(Velocity_Current,...
                                                                                     Pulsation_Aimed,...
                                                                                     Damping_Aimed,...
                                                                                     Gain_Filter,...
                                                                                     Length_Ship_Current,...
                                                                                     Density_Water_Current,...
                                                                                     Drag_Coefficient_Current,...
                                                                                     Draft_Depth_Current,...
                                                                                     Longeron_Area_Current,...
                                                                                     Mass_Of_The_Ship_Current,...
                                                                                     Heading_Aimed_Current,...
                                                                                     Heading_Observed_Current,...
                                                                                     Time_Sampling)

% Due to the fact that the ship is an LPV in velocity, the PDF synthesis
% and coefficients tuning of the PD controller depends on the velocity of
% the ship.

% Persistents initialization:
persistent Rudder_Angle_Commanded_Previous;
persistent Heading_Error_Previous;
if isempty(Rudder_Angle_Commanded_Previous)
    Rudder_Angle_Commanded_Previous = 0;
end
if isempty(Heading_Error_Previous)
    Heading_Error_Previous = 0;
end

% Protection for null velocity declaration:
if Velocity_Current == 0
    Velocity_Current = 5/3.6;
end

% Compute the static gain and the time constant of the model of the ship in
% function of the velocity:
[Static_Gain_Nomoto_LPV,Time_Constant_Total_LPV] = Static_Gain_Time_Constant_Varying_Velocity(Velocity_Current,...
                                                                                              Length_Ship_Current,...
                                                                                              Density_Water_Current,...
                                                                                              Drag_Coefficient_Current,...
                                                                                              Draft_Depth_Current,...
                                                                                              Longeron_Area_Current,...
                                                                                              Mass_Of_The_Ship_Current);

% Check the Bandwidth value and establish numerical protection:
Pulsation_Ship_LPV = 1/Time_Constant_Total_LPV;
Pulsation_Bandwidth = Pulsation_Aimed*(sqrt(1 - 2*Damping_Aimed^2 + sqrt(4*Damping_Aimed^4 - 4*Damping_Aimed^2 + 2)));
if Pulsation_Bandwidth < Pulsation_Ship_LPV
    error('The pulsation bandwidth of the overall loop must be greater than the natural pulsation');
end

% Determine the coefficients of the filter in concequence:
Gain_Proportional_LPV = (Time_Constant_Total_LPV*Pulsation_Aimed^2)/Static_Gain_Nomoto_LPV;
Gain_Derivative_LPV = (2*Damping_Aimed*Pulsation_Aimed*Time_Constant_Total_LPV - 1)/Static_Gain_Nomoto_LPV;

% Controller elaboration:
% Current heading error:
Heading_Error_Current = Heading_Aimed_Current - Heading_Observed_Current;

% Current rudder angle command:
Rudder_Angle_Elaborated_Current = Rudder_Angle_Commanded_Previous*((2*Gain_Filter - Time_Sampling)/(Time_Sampling + 2*Gain_Filter)) + ...
                                  Heading_Error_Current*((Gain_Proportional_LPV*Time_Sampling + 2*Gain_Proportional_LPV*Gain_Filter + 2*Gain_Derivative_LPV)/(Time_Sampling + 2*Gain_Filter)) + ...
                                  Heading_Error_Previous*((Gain_Proportional_LPV*Time_Sampling - 2*Gain_Proportional_LPV*Gain_Filter - 2*Gain_Derivative_LPV)/(Time_Sampling + 2*Gain_Filter));

% Then persistents update:
Rudder_Angle_Commanded_Previous = Rudder_Angle_Elaborated_Current;
Heading_Error_Previous = Heading_Error_Current;

end

