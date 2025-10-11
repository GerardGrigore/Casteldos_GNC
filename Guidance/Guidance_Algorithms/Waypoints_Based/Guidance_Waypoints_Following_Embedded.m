function [Heading_Aimed_Next,Time_End_Guidance] = Guidance_Waypoints_Following_Embedded(Velocity_Current,...
                                                                                        Waypoints,...
                                                                                        Position_X_Ship_Current,...
                                                                                        Position_Y_Ship_Current,...
                                                                                        Heading_Ship_Current,...
                                                                                        Position_X_Final_Waypoint,...
                                                                                        Position_Y_Final_Waypoint,...
                                                                                        Time_Current)

% Initialization:
if Time_Current == 0
    Position_X_Ship_Current = Waypoints(1,1);
    Position_Y_Ship_Current = Waypoints(1,2);
end

% Terminal point features:
Position_X_Final_Point = Position_X_Final_Waypoint;
Position_Y_Final_Point = Position_Y_Final_Waypoint;
Radius_Final_Waypoint = 50;

% Define persistent for heading protection:
persistent Heading_Previous;
if isempty(Heading_Previous)
    Heading_Previous = Heading_Ship_Current;
end

% Enable Guidance to work if the ship is not close to the final waypoint:
if ~((Position_X_Ship_Current - Position_X_Final_Point)^2 + (Position_Y_Ship_Current - Position_Y_Final_Point)^2 <= Radius_Final_Waypoint^2)

    % Then settle the time of end Guidance:
    Time_End_Guidance = 0;

    % Retreive the aimed heading angle that must be performed at this step time
    % of the simulation to follow the Waypoints generated trajectory:
    [Heading_Aimed_Unit,Heading_Aimed_Compass,...
     Heading_Error_Unit,Heading_Error_Compass,...
     Selected_Waypoints,Heading_Current_Unit] = Guidance_Heading_Rudder_Angle(Waypoints,...
                                                                              Position_X_Ship_Current,...
                                                                              Position_Y_Ship_Current, ...
                                                                              Heading_Ship_Current);

    % Update the heading aimed:
    Heading_Aimed_Next = Heading_Aimed_Compass;

    % Then update the persistent:
    Heading_Previous = Heading_Aimed_Next;       

else
    
    [Heading_Aimed_Unit,Heading_Aimed_Compass,...
     Heading_Error_Unit,Heading_Error_Compass,...
     Selected_Waypoints,Heading_Current_Unit] = Guidance_Heading_Rudder_Angle(Waypoints,...
                                                                              Position_X_Ship_Current,...
                                                                              Position_Y_Ship_Current, ...
                                                                              Heading_Ship_Current);

    % Update the heading aimed:
    Heading_Aimed_Next = Heading_Aimed_Compass;

    % Then update the persistent:
    Heading_Previous = Heading_Aimed_Next; 
    
    % Then determine the end time of the Guidance scheme:
    Time_End_Guidance = Time_Current;

end
end
