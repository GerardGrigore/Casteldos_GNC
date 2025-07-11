function Rudder_Commanded_Elaborated_Current = Rudder_Angle_Rate_Saturations(Rudder_Commanded_High_Level_Loop_Current,...
                                                                             Rudder_Angle_Maximal,...
                                                                             Rudder_Angle_Minimal,...
                                                                             Rudder_Rate_Maximal,...
                                                                             Rudder_Rate_Minimal,...
                                                                             Time_Sampling)

% This function acts as a simplified model of a first Actuator version. It
% consists in a first saturation of the commanded rudder angle in terms of
% maximal and minimal angles value. And a second saturation in terms of maximal and
% minimal rudder angle rate.

% Persistent initialization:
persistent Rudder_Commanded_Actuator_Previous;
persistent Rudder_Aimed_Saturated_Angular_Fictional_Previous;
if isempty(Rudder_Commanded_Actuator_Previous)
    Rudder_Commanded_Actuator_Previous = 0;
end
if isempty(Rudder_Aimed_Saturated_Angular_Fictional_Previous)
    Rudder_Aimed_Saturated_Angular_Fictional_Previous = 0;
end

% Intermediate temporal parameter:
Rudder_Aimed_Saturated_Angular_Fictional_Current = 0;
% First rudder angle saturation:
if Rudder_Commanded_High_Level_Loop_Current > Rudder_Angle_Maximal
    % Saturation to the upper bound:
    Rudder_Aimed_Saturated_Angular_Fictional_Current = Rudder_Angle_Maximal;
elseif Rudder_Commanded_High_Level_Loop_Current < Rudder_Angle_Minimal
    % Saturation to the lower bound:
    Rudder_Aimed_Saturated_Angular_Fictional_Current = Rudder_Angle_Minimal;
else
    % No saturation:
    Rudder_Aimed_Saturated_Angular_Fictional_Current = Rudder_Commanded_High_Level_Loop_Current;
end

% Compute the current fictional delta of rudder angle:
Delta_Rudder_Angle_Previous_Fictional = Rudder_Aimed_Saturated_Angular_Fictional_Previous - Rudder_Commanded_Actuator_Previous;
Saturated_Delta_Rudder_Angle_Previous_Fictional = 0;
% Then second saturation on the rudder rate:
if Delta_Rudder_Angle_Previous_Fictional > Rudder_Rate_Maximal
    % Upper bound saturation:
    Saturated_Delta_Rudder_Angle_Previous_Fictional = Rudder_Rate_Maximal;
elseif Delta_Rudder_Angle_Previous_Fictional < Rudder_Rate_Minimal
    % Lower bound saturation:
    Saturated_Delta_Rudder_Angle_Previous_Fictional = Rudder_Rate_Minimal;
else
    Saturated_Delta_Rudder_Angle_Previous_Fictional = Delta_Rudder_Angle_Previous_Fictional;
end

% Define the current variation of delta:
Variation_Delta_Rudder_Previous = Saturated_Delta_Rudder_Angle_Previous_Fictional;

% Then update the rudder angle in output:
Rudder_Commanded_Elaborated_Current = Rudder_Commanded_Actuator_Previous + Time_Sampling*Variation_Delta_Rudder_Previous;

% Persistents update:
Rudder_Commanded_Actuator_Previous = Rudder_Commanded_Elaborated_Current;
Rudder_Aimed_Saturated_Angular_Fictional_Previous = Rudder_Aimed_Saturated_Angular_Fictional_Current;

end
