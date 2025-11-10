function [Heading_Reference_Aimed_Raw,...
          Cross_Track_Error_Current] = Guidance_Integrated_Line_Of_Sight(Waypoints,...
                                                                         Position_X_Ship_Current,...
                                                                         Position_Y_Ship_Current,...
                                                                         Time_Sampling,...
                                                                         Position_X_Ship_Initial,...
                                                                         Position_Y_Ship_Initial,...
                                                                         Time_Current)

% Initialization:
Look_Ahead_Distance_Horizon = 100;
Gain_Integrator_ILOS_Kappa = 0.01;
Time_Integration = Time_Sampling;
Switching_Radius = 20;
Waypoint.Position.Longitudinal_X = Waypoints(:,1);
Waypoint.Position.Lateral_Y = Waypoints(:,2);
Heading_Aimplitude_Limitation = 5*(pi/180);

% Persistents definition:
persistent Index_Active_Waypoint;
persistent Longitudinal_Current_Waypoint_X Lateral_Current_Waypoint_Y;
persistent Cross_Track_Error_Integral_Term_Current;
persistent Heading_Aimed_Previous;
if isempty(Heading_Aimed_Previous)
    Heading_Aimed_Previous = 0;
end

% Position initialization:
if Time_Current == 0
    % Position initialization:
    Position_X_Ship_Current = Position_X_Ship_Initial;
    Position_Y_Ship_Current = Position_Y_Ship_Initial;
end

% Initialization of the waypoints:
if isempty(Index_Active_Waypoint)

    % Check if R_switch is smaller than the minimum distance between the
    % waypoints:
    if Switching_Radius > min( sqrt( diff(Waypoint.Position.Longitudinal_X).^2 + diff(Waypoint.Position.Lateral_Y).^2 ) )
        error("The distances between the waypoints must be larger than R_switch.");
    end

    % Check input parameters:
    if (Switching_Radius < 0)
        error("R_switch must be larger than zero.");
    end
    if (Look_Ahead_Distance_Horizon < 0)
        error("Delta must be larger than zero.");
    end
    % Initialization of the parameters of the Guidance algorithm:
    Cross_Track_Error_Integral_Term_Current = 0;
    Index_Active_Waypoint = 1;               
    Longitudinal_Current_Waypoint_X = Waypoint.Position.Longitudinal_X(Index_Active_Waypoint);
    Lateral_Current_Waypoint_Y = Waypoint.Position.Lateral_Y(Index_Active_Waypoint);
end

% Reading of the next waypoint:
Number_Of_Waypoints = length(Waypoint.Position.Longitudinal_X);
% If the current index of the active waypoints is lower than the total
% number of waypoints, select the next ones:
if Index_Active_Waypoint < Number_Of_Waypoints                       
    Longitudinal_Next_Wapypoint_X = Waypoint.Position.Longitudinal_X(Index_Active_Waypoint+1);  
    Lateral_Next_Waypoint_Y = Waypoint.Position.Lateral_Y(Index_Active_Waypoint+1);    
else                           
    Bearing_Waypoints = atan2((Waypoint.Position.Lateral_Y(Number_Of_Waypoints)- Waypoint.Position.Lateral_Y(Number_Of_Waypoints-1)),...
                              (Waypoint.Position.Longitudinal_X(Number_Of_Waypoints)-Waypoint.Position.Longitudinal_X(Number_Of_Waypoints-1)));
    Radius_Termination = 1e10;
    Longitudinal_Next_Wapypoint_X = Waypoint.Position.Longitudinal_X(Number_Of_Waypoints) + Radius_Termination*sin(Bearing_Waypoints);
    Lateral_Next_Waypoint_Y = Waypoint.Position.Lateral_Y(Number_Of_Waypoints) + Radius_Termination*cos(Bearing_Waypoints); 
end

% Compute the path-tangnetial angle with respect to the lateral North
% direction:
Path_Tangential_Current_Next_Waypoint_Angle = atan2((Longitudinal_Next_Wapypoint_X-Longitudinal_Current_Waypoint_X),(Lateral_Next_Waypoint_Y-Lateral_Current_Waypoint_Y)); 

% Along-track and cross-track errors (Along_Track_Error_Current,
% Cross_Track_Error_Current):
Along_Track_Error_Current =  (Position_X_Ship_Current-Longitudinal_Current_Waypoint_X)*sin(Path_Tangential_Current_Next_Waypoint_Angle) +...
                             (Position_Y_Ship_Current-Lateral_Current_Waypoint_Y)*cos(Path_Tangential_Current_Next_Waypoint_Angle);
Cross_Track_Error_Current = (Position_X_Ship_Current-Longitudinal_Current_Waypoint_X)*cos(Path_Tangential_Current_Next_Waypoint_Angle) -...
                            (Position_Y_Ship_Current-Lateral_Current_Waypoint_Y)*sin(Path_Tangential_Current_Next_Waypoint_Angle);

% Iteration of the active current waypoint if the switching criterion is
% realized:
Distance_Switching_Criterion = sqrt((Longitudinal_Next_Wapypoint_X-Longitudinal_Current_Waypoint_X)^2 + (Lateral_Next_Waypoint_Y-Lateral_Current_Waypoint_Y)^2);
if ((Distance_Switching_Criterion - Along_Track_Error_Current < Switching_Radius) && (Index_Active_Waypoint < Number_Of_Waypoints))
    Index_Active_Waypoint = Index_Active_Waypoint + 1;
    Longitudinal_Current_Waypoint_X = Longitudinal_Next_Wapypoint_X;
    Lateral_Current_Waypoint_Y = Lateral_Next_Waypoint_Y; 
end

% Integration LOS Guidance law computation:
Gain_Proportional_ILOS = 1/Look_Ahead_Distance_Horizon;
Heading_Reference_Aimed_Raw = Path_Tangential_Current_Next_Waypoint_Angle - atan(Gain_Proportional_ILOS*(Cross_Track_Error_Current + ...
                              Gain_Integrator_ILOS_Kappa*Cross_Track_Error_Integral_Term_Current));     
% Saturation if the heading variation is too high:
if Heading_Reference_Aimed_Raw - Heading_Aimed_Previous > Heading_Aimplitude_Limitation
    Heading_Reference_Aimed_Raw = Heading_Aimed_Previous + Heading_Aimplitude_Limitation;
end

% Update of the persistents:
Cross_Track_Error_Integral_Term_Current = Cross_Track_Error_Integral_Term_Current + Time_Integration*Look_Ahead_Distance_Horizon*Cross_Track_Error_Current/...
                                          (Look_Ahead_Distance_Horizon^2 + (Cross_Track_Error_Current + Gain_Integrator_ILOS_Kappa * Cross_Track_Error_Integral_Term_Current)^2);
Heading_Aimed_Previous = Heading_Reference_Aimed_Raw;

end

