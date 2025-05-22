function [Heading_Aimed_Unit,Heading_Aimed_Compass,...
          Heading_Error_Unit,Heading_Error_Compass,...
          Selected_Waypoints,Heading_Current_Unit] = Guidance_Heading_Rudder_Angle(Waypoints,...
                                                                                   Position_Ship_X,...
                                                                                   Position_Ship_Y, ...
                                                                                   Heading_Current_Compass_Estimated)

% We suppose that the current estimated Heading in input is given in the
% bearing compass format clock-wise circle. Conversion to unit circle:
Heading_Current_Unit = -Heading_Current_Compass_Estimated + (pi/2);

% Retreive the coordinates of the waypoints:
Position_X_Waypoints = [Waypoints(:,1)];
Position_Y_Waypoints = [Waypoints(:,2)];

% Waypoints selection initialization:
Waypoint_X_Current_Previous = 0;
Waypoint_X_Current_Next = 0;
Waypoint_Y_Current_Previous = 0;
Waypoint_Y_Current_Next = 0;

% First select the waypoints in between which the ship is:
Number_Waypoints = length(Waypoints);
for index_waypoints = 2:Number_Waypoints
    % Search where the ship is in terms of waypoints surrounding.
    if Waypoint_X_Current_Previous == 0 && Waypoint_X_Current_Next == 0 &&...
            Waypoint_Y_Current_Previous == 0 && Waypoint_Y_Current_Next == 0

        % Then locate the horizontal component of the ship:
        if (Position_Ship_X <= Position_X_Waypoints(index_waypoints)) &&...
                (Position_Ship_X >= Position_X_Waypoints(index_waypoints - 1))

            % When found, then select the correct next waypoint to select:
            Waypoint_X_Current_Previous = Position_X_Waypoints(index_waypoints - 1);
            Waypoint_X_Current_Next = Position_X_Waypoints(index_waypoints);
            Waypoint_Y_Current_Previous = Position_Y_Waypoints(index_waypoints - 1);
            Waypoint_Y_Current_Next = Position_Y_Waypoints(index_waypoints);

        end
    end
end

% Then save the selected waypoints:
Selected_Waypoints = [[Waypoint_X_Current_Previous Waypoint_Y_Current_Previous];
                      [Waypoint_X_Current_Next Waypoint_Y_Current_Next]];

% Determine the deviation angle in between a horizontal line to the newly
% aimed heading:
Deviation_Heading = atan2(Waypoint_Y_Current_Next - Position_Ship_Y,Waypoint_X_Current_Next - Position_Ship_X);

% Provide the heading in both unit and compass circles:
Heading_Aimed_Unit = Deviation_Heading;
Heading_Aimed_Compass = -Heading_Aimed_Unit + (pi/2);

% Rudder angle to apply in order to correct the heading error:
Heading_Error_Unit = Heading_Aimed_Unit - Heading_Current_Unit;
Heading_Error_Compass = Heading_Aimed_Compass - Heading_Current_Compass_Estimated;

end

