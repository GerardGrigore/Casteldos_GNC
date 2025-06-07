function [Rudder_Angle_Commanded_Current] = PIDF_Anti_Wind_Up_Incremental(Gain_Proportional,...
																			Gain_Derivative,...
																			Gain_Integral,...
																			Gain_Filter,...
																			Time_Sampling,...
																			Time_Tracking_Constant_Anti_Wind_Up,...
																			Heading_Aimed_Guidance,...
																			Heading_Observed_Estimated_Current,...
																			Rudder_Commanded_Output_Saturation,...
																			Time_Current)

% Persistents definition:
persistent Control_Increment_Previous;
persistent Heading_Error_Previous;
persistent Heading_Error_Previous_Previous;
persistent Count_Iteration;
persistent Control_Signal_Previous;

% Persistents initialization:
if isempty(Control_Increment_Previous)
	Control_Increment_Previous = 0;
end
if isempty(Heading_Error_Previous)
	Heading_Error_Previous = 0;
end
if isempty(Heading_Error_Previous_Previous)
	Heading_Error_Previous_Previous = 0;
end
if isempty(Count_Iteration)
	Count_Iteration = 0;
end
if isempty(Control_Signal_Previous)
	Control_Signal_Previous = 0;
end

% Saturation limits:
Rudder_Angle_Maximal_Saturated = 35*(pi/180);
Rudder_Angle_Minimal_Saturated = -35*(pi/180);

% Controller gains declaration:
Controller_Zero_Gain_0 = (Gain_Proportional*(Time_Sampling + Gain_Filter) + Gain_Integral*(Time_Sampling^2 + Time_Sampling*Gain_Filter) + ...
						  Gain_Derivative)/(Time_Sampling + Gain_Filter);
						  
Controller_Zero_Gain_1 = (Gain_Proportional*(-2*Gain_Filter - Time_Sampling) - Gain_Integral*Time_Sampling*Gain_Filter - 2*Gain_Derivative)/...
					     (Time_Sampling + Gain_Filter);
						 
Controller_Zero_Gain_2 = (Gain_Proportional*Gain_Filter + Gain_Derivative)/(Time_Sampling + Gain_Filter);

Controller_Pole_Gain_Beta = Gain_Filter/(Time_Sampling + Gain_Filter);

% Calculation of the current heading error:
Heading_Error_Current = Heading_Aimed_Guidance - Heading_Observed_Estimated_Current;

% Calculate the control increment:
Control_Increment_Current = Controller_Pole_Gain_Beta*Control_Increment_Previous + Controller_Zero_Gain_0*Heading_Error_Current + ...
							Controller_Zero_Gain_1*Heading_Error_Previous + Controller_Zero_Gain_2*Heading_Error_Previous_Previous;
							
% Then calculate the control signal:
Control_Signal_Current = Control_Increment_Current + Control_Signal_Previous;

% Safe definition of the output of the saturation at the first time step:
if Time_Current == 0
	Rudder_Commanded_Output_Saturation = 0;
end

% Conditional output update:
if Control_Signal_Current > Rudder_Angle_Maximal_Saturated || ...
	Control_Signal_Current < Rudder_Angle_Minimal_Saturated
	Control_Increment_Current = Rudder_Commanded_Output_Saturation - Control_Signal_Previous;
	% Output:
	Rudder_Angle_Commanded_Current = Rudder_Commanded_Output_Saturation;
else
	Rudder_Angle_Commanded_Current = Control_Signal_Current;
end

% Persistents update:																			
Control_Increment_Previous = Control_Increment_Current;
Count_Iteration = Count_Iteration + 1;
if Count_Iteration == 1
    Heading_Error_Previous_Previous = 0;
else
    Heading_Error_Previous_Previous = Heading_Error_Previous;
end
Heading_Error_Previous = Heading_Error_Current;	
Control_Signal_Previous = Rudder_Angle_Commanded_Current;	

end

