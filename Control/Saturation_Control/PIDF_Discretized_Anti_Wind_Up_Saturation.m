function Rudder_Angle_Command_Current = PIDF_Discretized_Anti_Wind_Up_Saturation(Heading_Error_Current,...
                                                                                 Gain_Proportional,...
                                                                                 Time_Integral,...
                                                                                 Time_Derivative,...
                                                                                 Time_Sampling)

% Persistents definition:
persistent Heading_Error_Previous;
persistent Heading_Error_Filtered_Previous;
persistent Heading_Error_Filtered_Previous_Previous;
persistent Rudder_Angle_Command_Previous;
persistent Count_Persistent_Update;

% Threshold:
Rudder_Angle_Minimal = -35*(pi/180);
Rudder_Angle_Maximal = 35*(pi/180);

% Condition of persistent valorization:
if isempty(Count_Persistent_Update)
    Count_Persistent_Update = 0;
end
if isempty(Heading_Error_Previous)
    Heading_Error_Previous = 0;
end
if isempty(Heading_Error_Filtered_Previous)
    Heading_Error_Filtered_Previous = 0;
end
if isempty(Heading_Error_Filtered_Previous_Previous)
    Heading_Error_Filtered_Previous_Previous = 0;
end
if isempty(Rudder_Angle_Command_Previous)
    Rudder_Angle_Command_Previous = 0;
end

% Calculate the current heading error filtered:
Heading_Error_Filtered_Current = Heading_Error_Filtered_Previous*(1 - (Time_Sampling/(0.1*Time_Derivative))) + Heading_Error_Previous*(Time_Sampling/(0.1*Time_Derivative));

% Define the increment of the command:
Delta_Increment_Command_Current = Gain_Proportional*(Heading_Error_Current - Heading_Error_Previous) + ((Gain_Proportional*Time_Sampling)/Time_Integral)*Heading_Error_Current + ...
                                  ((Gain_Proportional*Time_Derivative)/Time_Sampling)*(Heading_Error_Filtered_Current - 2*Heading_Error_Filtered_Previous + ...
                                  Heading_Error_Filtered_Previous_Previous);

% Update the output:
% Anti wind-up condition:
if ((Rudder_Angle_Command_Previous + Delta_Increment_Command_Current) > Rudder_Angle_Maximal) ||...
        ((Rudder_Angle_Command_Previous + Delta_Increment_Command_Current) < Rudder_Angle_Minimal)
    Rudder_Angle_Command_Current = Rudder_Angle_Command_Previous;
else
    Rudder_Angle_Command_Current = Rudder_Angle_Command_Previous + Delta_Increment_Command_Current;
end

% Update the persistents:
Rudder_Angle_Command_Previous = Rudder_Angle_Command_Current;
Heading_Error_Previous = Heading_Error_Current;
Count_Persistent_Update = Count_Persistent_Update + 1;
if Count_Persistent_Update == 1
    Heading_Error_Filtered_Previous_Previous = 0;
else
    Heading_Error_Filtered_Previous_Previous = Heading_Error_Filtered_Previous;
end
Heading_Error_Filtered_Previous = Heading_Error_Filtered_Current;

end

