function [Rudder_Angle_Commanded_Current] = PIDF_Anti_Wind_Up_Back_Calculation(Gain_Proportional,...
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
persistent Input_Command_Integral_Previous;
persistent Input_Command_Derivative_Previous;
persistent Heading_Error_Previous

% Persistents initialization:
if isempty(Input_Command_Integral_Previous)
	Input_Command_Integral_Previous = 0;
end
if isempty(Input_Command_Derivative_Previous)
	Input_Command_Derivative_Previous = 0;
end
if isempty(Heading_Error_Previous)
	Heading_Error_Previous = 0;
end

% Calculation of the current heading error:
Heading_Error_Current = Heading_Aimed_Guidance - Heading_Observed_Estimated_Current;

% Update the commanded inputs for each actions:
% Proportional:
Input_Command_Proportional_Current = Gain_Proportional*Heading_Error_Current;

% Integral:
Input_Command_Integral_Current = Input_Command_Integral_Previous + Gain_Integral*Time_Sampling*Heading_Error_Current;

% Derivative:
Input_Command_Derivative_Current = (Gain_Derivative*(Heading_Error_Current - Heading_Error_Previous) + ...
									Gain_Filter*Input_Command_Derivative_Previous)/(Gain_Filter + Time_Sampling);
									
% Establish the current control signal:
Control_Input_Signal_No_Wind_Up = Input_Command_Proportional_Current + Input_Command_Integral_Current + Input_Command_Derivative_Current;

% Current error between output of saturation and output of the controller:
if Time_Current == 0
	% Then first time step point:
	Rudder_Commanded_Output_Saturation = 0; % Protection for the first time step.
end
Error_Anti_Wind_Up_Current = Rudder_Commanded_Output_Saturation - Control_Input_Signal_No_Wind_Up;

% Back-calculation integral term anti wind-up protection:
Input_Command_Integral_Anti_Wind_Up_Current = Input_Command_Integral_Previous + (Gain_Integral*Heading_Error_Current + ...
										      (1/Time_Tracking_Constant_Anti_Wind_Up)*Error_Anti_Wind_Up_Current)*Time_Sampling;
											  
% Then calculate the new input signal if the saturation is activated: 
Control_Input_Signal_Wind_Up = Input_Command_Proportional_Current + Input_Command_Integral_Anti_Wind_Up_Current + Input_Command_Derivative_Current;

% Output of the algorithm:
Rudder_Angle_Commanded_Current = Control_Input_Signal_Wind_Up;

% Persistents update:
Input_Command_Integral_Previous = Input_Command_Integral_Anti_Wind_Up_Current; % But also try Input_Command_Integral_Current.
Input_Command_Derivative_Previous = Input_Command_Derivative_Current;
Heading_Error_Previous = Heading_Error_Current;

end

