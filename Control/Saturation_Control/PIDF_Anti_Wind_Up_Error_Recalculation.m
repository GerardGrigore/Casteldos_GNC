function Rudder_Commanded_Current = PIDF_Anti_Wind_Up_Error_Recalculation(Rudder_Saturated_Commanded_Current,...
                                                                          Gain_Proportional,...
                                                                          Gain_Derivative,...
                                                                          Gain_Integral,...
                                                                          Gain_Filter,...
                                                                          Time_Sampling,...
                                                                          Heading_Aimed_Guidance,...
                                                                          Heading_Observed_Estimated_Current)

% This online embedded algorithm allows to tackle the anti wind-up problem
% when using a PIDF to control a plant subject to saturation.
% The problem with the integral term of the controller is that the error
% continues to accumulate even when the output is saturated. This can lead
% to important overshoot and sometimes, divergence.

% The goal is then to avoid or limit the useless accumulation of the
% integral term. The error recalculation method relies on the following
% principle: when the commanded input is saturated, the error is modified
% to avoind integrating an error that cannot be corrected by the system.

% Persistents definition:
persistent Heading_Error_Previous;
persistent Heading_Error_Previous_Previous;
persistent Count_Iteration;
persistent Rudder_Saturated_Commanded_Previous;
persistent Rudder_Saturated_Commanded_Previous_Previous;
persistent Expected_Heading_Error_Previous;
persistent Expected_Heading_Error_Previous_Previous;

% Persistents initialization:
if isempty(Heading_Error_Previous)
	Heading_Error_Previous = 0;
end
if isempty(Heading_Error_Previous_Previous)
	Heading_Error_Previous_Previous = 0;
end
if isempty(Rudder_Saturated_Commanded_Previous)
	Rudder_Saturated_Commanded_Previous = 0;
end
if isempty(Rudder_Saturated_Commanded_Previous_Previous)
	Rudder_Saturated_Commanded_Previous_Previous = 0;
end
if isempty(Expected_Heading_Error_Previous)
	Expected_Heading_Error_Previous = 0;
end
if isempty(Expected_Heading_Error_Previous_Previous)
	Expected_Heading_Error_Previous_Previous = 0;
end
if isempty(Count_Iteration)
	Count_Iteration = 0;
end

% Controller gains declaration:
Controller_Zero_Gain_0 = (Gain_Proportional*(Time_Sampling + Gain_Filter) + Gain_Integral*(Time_Sampling^2 + Time_Sampling*Gain_Filter) + ...
						  Gain_Derivative)/(Time_Sampling + Gain_Filter);
Controller_Zero_Gain_1 = (Gain_Proportional*(-2*Gain_Filter - Time_Sampling) - Gain_Integral*Time_Sampling*Gain_Filter - 2*Gain_Derivative)/...
					     (Time_Sampling + Gain_Filter);
Controller_Zero_Gain_2 = (Gain_Proportional*Gain_Filter + Gain_Derivative)/(Time_Sampling + Gain_Filter);
Controller_Pole_Gain_Beta = Gain_Filter/(Time_Sampling + Gain_Filter);
Controller_Pole_Gain_1 = -Controller_Pole_Gain_Beta - 1;
Controller_Pole_Gain_2 = Controller_Pole_Gain_Beta;

% Calculation of the current heading error:
Heading_Error_Current = Heading_Aimed_Guidance - Heading_Observed_Estimated_Current;

% Commanded input:
Rudder_Commanded_Current = Controller_Zero_Gain_0*Heading_Error_Current + Controller_Zero_Gain_1*Expected_Heading_Error_Previous + ...
                           Controller_Zero_Gain_2*Expected_Heading_Error_Previous_Previous - Controller_Pole_Gain_1*Rudder_Saturated_Commanded_Previous - ...
                           Controller_Pole_Gain_2*Rudder_Saturated_Commanded_Previous_Previous;

% Persistents update:																			
Count_Iteration = Count_Iteration + 1;
if Count_Iteration == 1
    Heading_Error_Previous_Previous = 0;
    Rudder_Saturated_Commanded_Previous_Previous = 0;
    Expected_Heading_Error_Previous_Previous = 0;
else
    Heading_Error_Previous_Previous = Heading_Error_Previous;
    Rudder_Saturated_Commanded_Previous_Previous = Rudder_Saturated_Commanded_Previous;
    Expected_Heading_Error_Previous_Previous = Expected_Heading_Error_Previous;
end
Heading_Error_Previous = Heading_Error_Current;	
Rudder_Saturated_Commanded_Previous = Rudder_Saturated_Commanded_Current;
Expected_Heading_Error_Previous = Heading_Error_Current - ((Rudder_Commanded_Current - Rudder_Saturated_Commanded_Current)/...
                                  Controller_Zero_Gain_0);

end

