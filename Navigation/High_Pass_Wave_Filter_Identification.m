function Heading_Filtered_Ouput_Current = High_Pass_Wave_Filter_Identification(Heading_Unfiltered_Input_Current,...
                                                                               Time_Sampling)

% Numerical High pass filter for filtering the heading for the ARMA Identification
% algorithm.

% Filter parameters:
Pulsation_Filter_Wave = 0.6;
Time_Filter_Wave = 1/Pulsation_Filter_Wave;

% Persistents definition:
persistent Heading_Unfiltered_Input_Previous;
persistent Heading_Filtered_Ouput_Previous;

% Persistent initialization:
if isempty(Heading_Unfiltered_Input_Previous)
    Heading_Unfiltered_Input_Previous = 0;
end
if isempty(Heading_Filtered_Ouput_Previous)
    Heading_Filtered_Ouput_Previous = 0;
end

% Update the next filtered:
Heading_Filtered_Ouput_Current = ((2*Time_Filter_Wave)/(Time_Sampling + 2*Time_Filter_Wave))*(Heading_Unfiltered_Input_Current - ...
                                  Heading_Unfiltered_Input_Previous) + ((2*Time_Filter_Wave - Time_Sampling)/(Time_Sampling + 2*Time_Filter_Wave))*...
                                  Heading_Filtered_Ouput_Previous;

% Update of the persistents:
Heading_Filtered_Ouput_Previous = Heading_Filtered_Ouput_Current;
Heading_Unfiltered_Input_Previous = Heading_Unfiltered_Input_Current;

end

