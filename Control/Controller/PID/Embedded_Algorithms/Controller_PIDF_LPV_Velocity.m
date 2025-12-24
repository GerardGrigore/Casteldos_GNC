function Rudder_Angle_Elaborated_Current = Controller_PIDF_LPV_Velocity(Heading_Aimed_Current,...
                                                                        Heading_Observed_Current,...
                                                                        Rudder_Rate_Desired_Current,...
                                                                        Rudder_Acceleration_Rate_Desired_Current,...
                                                                        Velocity_Current,...
                                                                        Damping_Aimed,...
                                                                        Pulsation_Aimed,...
                                                                        Time_Sampling,...
                                                                        Length_Ship_Current,...
                                                                        Density_Water_Current,...
                                                                        Drag_Coefficient_Current,...
                                                                        Draft_Depth_Current,...
                                                                        Longeron_Area_Current,...
                                                                        Mass_Of_The_Ship_Current,...
                                                                        Velocity_Initial)


% Adaptative control algorithm in function of the velocity:
% Due to the fact that the ship is an LPV in velocity, the PIDF synthesis
% and coefficients tuning of the PIDF controller depends on the velocity of
% the ship.

% Persistents declaration:
persistent Heading_Error_Previous;
persistent Heading_Error_Previous_Previous;
persistent Heading_Filter_Error_Previous;
persistent Heading_Filter_Error_Previous_Previous;
persistent Rudder_Commanded_Previous;
if isempty(Heading_Error_Previous)
    Heading_Error_Previous = 0;
    Heading_Error_Previous_Previous = 0;
    Rudder_Commanded_Previous = 0;
    Heading_Filter_Error_Previous = 0;
    Heading_Filter_Error_Previous_Previous = 0;
end

% Protection for null velocity declaration:
if Velocity_Current == 0
    Velocity_Current = Velocity_Initial;
end

% Selection of filter error or classical error:
Is_Filter_Error = 1;

% Compute the static gain and the time constant of the model of the ship in
% function of the velocity:
[Static_Gain_Nomoto_LPV,...
 Time_Constant_Total_LPV,...
 Time_Constant_1,...
 Time_Constant_2,...
 Time_Constant_3,...
 Time_Constant_Sway,...
 Static_Gain_Sway] = Static_Gain_Time_Constant_Varying_Velocity_Embedded(Velocity_Current,...
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
    warning('The pulsation bandwidth of the overall loop must be greater than the natural pulsation.');
end

% Determine the coefficients of the filter in concequence:
Gain_Proportional_LPV = (Time_Constant_Total_LPV*Pulsation_Aimed^2)/Static_Gain_Nomoto_LPV;
Gain_Derivative_LPV = (2*Damping_Aimed*Pulsation_Aimed*Time_Constant_Total_LPV - 1)/Static_Gain_Nomoto_LPV;
Integral_Parameter_Tuning = 10;
Gain_Integral_LPV = (Pulsation_Aimed^3/Integral_Parameter_Tuning)*(Time_Constant_Total_LPV/Static_Gain_Nomoto_LPV);

% Controller parameters definition:
Time_Integral = Gain_Proportional_LPV/Gain_Integral_LPV;
Time_Derivative = Gain_Derivative_LPV/Gain_Proportional_LPV;
Gain_Filter = 0.1;
Time_Filter = Gain_Filter*Time_Derivative;

% Errors definition:
Heading_Error_Current = Heading_Aimed_Current - Heading_Observed_Current;
Heading_Filter_Error_Current = Heading_Filter_Error_Previous*(1 - Time_Sampling/Time_Filter) + ...
                               Time_Sampling*(Heading_Error_Previous/Time_Filter);

% Controller terms elaboration:
if Is_Filter_Error
    Controller_PID_Term_Current = Rudder_Commanded_Previous + Gain_Proportional_LPV*(Heading_Error_Current - Heading_Error_Previous) +...
                                  ((Gain_Proportional_LPV*Time_Sampling)/Time_Integral)*Heading_Error_Current +...
                                  ((Gain_Proportional_LPV*Time_Derivative)/Time_Sampling)*(Heading_Filter_Error_Current -...
                                  2*Heading_Filter_Error_Previous + Heading_Filter_Error_Previous_Previous);
else
    Controller_PID_Term_Current = Rudder_Commanded_Previous + Gain_Proportional_LPV*(Heading_Error_Current - Heading_Error_Previous) +...
                                  ((Gain_Proportional_LPV*Time_Sampling)/Time_Integral)*Heading_Error_Current +...
                                  ((Gain_Proportional_LPV*Time_Derivative)/Time_Sampling)*(Heading_Error_Current -...
                                  2*Heading_Error_Previous + Heading_Error_Previous_Previous);
end

% Feed forward controller term based on Nomoto's model:
% Hydrodynamics derivative 'N_delta':
Yaw_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Current*Length_Ship_Current^2*Draft_Depth_Current*Velocity_Current^2*(-0.5*...
                                       ((pi/4)*(Longeron_Area_Current/(Length_Ship_Current*Draft_Depth_Current))));
Coefficient_Feed_Forward = (Time_Constant_Total_LPV*Yaw_Control_Hydrodynamics_Derivative)/Static_Gain_Nomoto_LPV;
Controller_Feed_Forward_Term_Current = Coefficient_Feed_Forward*(Rudder_Acceleration_Rate_Desired_Current + (Rudder_Rate_Desired_Current/Time_Constant_Total_LPV));

% Output update:
Rudder_Angle_Elaborated_Current = Controller_Feed_Forward_Term_Current - Controller_PID_Term_Current;

% Persistent update:
Rudder_Commanded_Previous = Rudder_Angle_Elaborated_Current;
Heading_Filter_Error_Previous_Previous = Heading_Filter_Error_Previous;
Heading_Filter_Error_Previous = Heading_Filter_Error_Current;
Heading_Error_Previous_Previous = Heading_Error_Previous;
Heading_Error_Previous = Heading_Error_Current;

end

