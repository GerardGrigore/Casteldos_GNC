% Wyapoint trajectory generation for Île-De-La-Nadière mission:

% Waypoints in LLA frame near Port-Mahon's location to Île-De-La-Nadière:
Waypoint_1 = [43.0618927,3.0062474,0];
Waypoint_2 = [43.0633876,3.0100340,0];
Waypoint_3 = [43.0614052,3.0174674,0];
Waypoint_4 = [43.0553345,3.0241387,0];
Waypoint_5 = [43.0474289,3.0330359,0];
Waypoint_6 = [43.0406640,3.0391810,0];
Waypoints_In_LLA = [Waypoint_1;
                    Waypoint_2;
                    Waypoint_3;
                    Waypoint_4;
                    Waypoint_5;
                    Waypoint_6];

% The reference point is taken at Port-Mahon location:
Reference_Point_Port_Mahon = [43.059023,3.002915,0];

% Transformation LLA to ENU:
Semi_Major_Axis_Earth = 6378137.0;
Flatterning_Earth = 1/298.25;
for index_waypoints = 1:length(Waypoints_In_LLA)
    Waypoints_In_ENU(index_waypoints,:) = LLA_To_ENU(Waypoints_In_LLA(index_waypoints,1),Waypoints_In_LLA(index_waypoints,2),...
                                                     Waypoints_In_LLA(index_waypoints,3),...
                                                     Reference_Point_Port_Mahon(1),Reference_Point_Port_Mahon(2),Reference_Point_Port_Mahon(3),...
                                                     Semi_Major_Axis_Earth,Flatterning_Earth);
end

% Main waypoints along the path:
Waypoints_X = [Waypoints_In_ENU(:,1)'];
Waypoints_Y = [Waypoints_In_ENU(:,2)'];

% Waypoint object creation:
Waypoints = [Waypoints_In_ENU(1,1:2);
             Waypoints_In_ENU(2,1:2);
             Waypoints_In_ENU(3,1:2);
             Waypoints_In_ENU(4,1:2);
             Waypoints_In_ENU(5,1:2);
             Waypoints_In_ENU(6,1:2)];
















