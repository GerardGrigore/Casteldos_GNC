% This script serves as a mean to get a visual image on the aimed
% trajectory for the Île-De-La-Nadière starting from the reference point
% specified.

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

% Trajectory plotting:
geoplot([Reference_Point_Port_Mahon(1) Waypoint_1(1)],[Reference_Point_Port_Mahon(2) Waypoint_1(2)],'r-*','LineWidth',2);
hold on;
geoplot([Waypoint_1(1) Waypoint_2(1)],[Waypoint_1(2) Waypoint_2(2)],'r-*','LineWidth',2);
hold on;
geoplot([Waypoint_2(1) Waypoint_3(1)],[Waypoint_2(2) Waypoint_3(2)],'r-*','LineWidth',2);
hold on;
geoplot([Waypoint_3(1) Waypoint_4(1)],[Waypoint_3(2) Waypoint_4(2)],'r-*','LineWidth',2);
hold on;
geoplot([Waypoint_4(1) Waypoint_5(1)],[Waypoint_4(2) Waypoint_5(2)],'r-*','LineWidth',2);
hold on;
geoplot([Waypoint_5(1) Waypoint_6(1)],[Waypoint_5(2) Waypoint_6(2)],'r-*','LineWidth',2);
geolimits([43 43.1],[2.9 3.2]);
geobasemap colorterrain;















