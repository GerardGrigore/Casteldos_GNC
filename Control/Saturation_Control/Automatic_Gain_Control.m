function [Rudder_Angle_Desired_Current] = Automatic_Gain_Control(Rudder_Angle_Controller_Current,...
                                                                 Time_Period_Sampling)

% Check of the sign of the sampling period:
if Time_Period_Sampling <= 0
    error('The sampling time shall be strictly positive.');
end
                                          
% Define persistents:
persistent Rudder_Angle_Rate_Previous;
persistent Rudder_Angle_Controller_Previous;
persistent Intermediate_Sate_Previous;
persistent Rudder_Rate_Maximal_Reference_Previous;
persistent Gain_Filter_Previous;

% Initialize persistents:
if isempty(Rudder_Angle_Rate_Previous)
    Rudder_Angle_Rate_Previous = 0;
end
if isempty(Rudder_Angle_Controller_Previous)
    Rudder_Angle_Controller_Previous = 0;
end
if isempty(Intermediate_Sate_Previous)
    Intermediate_Sate_Previous = 0;
end
if isempty(Rudder_Rate_Maximal_Reference_Previous)
    Rudder_Rate_Maximal_Reference_Previous = 0;
end
if isempty(Gain_Filter_Previous)
    Gain_Filter_Previous = 1;
end
% Protection of the persistents:
if ~isfinite(Rudder_Angle_Rate_Previous)
    Rudder_Angle_Rate_Previous = 0;
end
if ~isfinite(Rudder_Angle_Controller_Previous)
    Rudder_Angle_Controller_Previous = 0;
end
if ~isfinite(Intermediate_Sate_Previous)
    Intermediate_Sate_Previous = 0;
end
if ~isfinite(Rudder_Rate_Maximal_Reference_Previous)
    Rudder_Rate_Maximal_Reference_Previous = 0;
end

% Initialization of important parameters:
Time_Gain_Filter = 1;
Pulsation_Filter = 1/Time_Gain_Filter;
Rudder_Rate_Maximal = 8*(pi/80);
Forgetting_Factor = 0.8;
Rudder_Angle_Maximal = 35*(pi/180);
Rudder_Angle_Minimal = -35*(pi/180);
Gain_Clamping_Minimal = 0.1;
Gain_Filter_Anti_Chattering = 0.9;

% Determine the derivative of the rudder angle:
% Using the discrete time relation:
Rudder_Angle_Rate_Current = ((2 - Pulsation_Filter*Time_Period_Sampling)/(Pulsation_Filter*Time_Period_Sampling + 2))*...
                            Rudder_Angle_Rate_Previous + 2*((Rudder_Angle_Controller_Current - Rudder_Angle_Controller_Previous)/...
                            (Pulsation_Filter*Time_Period_Sampling + 2));
% Using exact calculation based on exponential:
Intermediate_Sate_Current = exp(-Pulsation_Filter*Time_Period_Sampling)*Intermediate_Sate_Previous + (exp(-Pulsation_Filter*Time_Period_Sampling) - ...
                            1)*Rudder_Angle_Controller_Previous;
Rudder_Angle_Rate_Exact_Current = Intermediate_Sate_Current + Rudder_Angle_Controller_Current;

% Select the chosen value:
Rudder_Rate_Current = Rudder_Angle_Rate_Current;

% Then compute the absolute value of the rudder rate:
Rudder_Rate_Absolute_Current = abs(Rudder_Rate_Current);

% Determine the maximal rudder rate reference signal:
Rudder_Rate_Maximal_Reference_Current = max([Rudder_Rate_Maximal,Rudder_Rate_Absolute_Current,Forgetting_Factor*Rudder_Rate_Maximal_Reference_Previous]);

% Compute the gain adjustment controller:
Gain_Adjustment_Controller = Rudder_Rate_Maximal/Rudder_Rate_Maximal_Reference_Current;
% Clamping the gain for division protection:----------------------------------------------------------
Gain_Adjustment_Controller = max(Gain_Adjustment_Controller,Gain_Clamping_Minimal);
Gain_Adjustment_Controller = min(Gain_Adjustment_Controller,1);
% Anti-Chattering to avoid too high variation of the gain from one step to
% the next:
Gain_Adjustment_Controller = Gain_Filter_Anti_Chattering*Gain_Filter_Previous + (1 - Gain_Filter_Anti_Chattering)*Gain_Adjustment_Controller;
% ----------------------------------------------------------------------------------------------------

% Update the output:
% Protection of the rudder angle:
if ~isfinite(Rudder_Angle_Controller_Current)
    warning('Input rudder angle is not finite. Holding previous value');
    Rudder_Angle_Desired_Current = Rudder_Angle_Controller_Previous;
else
    Rudder_Angle_Desired_Current = Gain_Adjustment_Controller*Rudder_Angle_Controller_Current;
    % Physical limitation of the rudder angle:---------------------------------------------------------------------
    Rudder_Angle_Desired_Current = min(max(Rudder_Angle_Desired_Current, Rudder_Angle_Minimal),Rudder_Angle_Maximal);
end

% Update the persistents:
Rudder_Angle_Rate_Previous = Rudder_Angle_Rate_Current;
% Rudder_Angle_Rate_Previous = Rudder_Angle_Rate_Exact_Current;
Rudder_Angle_Controller_Previous = Rudder_Angle_Controller_Current;
Intermediate_Sate_Previous = Intermediate_Sate_Current;
Rudder_Rate_Maximal_Reference_Previous = Rudder_Rate_Maximal_Reference_Current;
Gain_Filter_Previous = Gain_Adjustment_Controller;

end

