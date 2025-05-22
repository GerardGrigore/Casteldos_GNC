% This script serves as a validation of the function that converts
% coordinates from LLA to ENU coordinate frames.

Waypoint_Initial = [43.021340,2.981993,50];
Reference_Point = [43.021774,2.981880,50];
Semi_Major_Axis = 6378137.0;
Flattening = 1/298.25;

% Using the internal Matlab function:
ENU_Vector_Matlab = lla2enu(Waypoint_Initial,Reference_Point,'ellipsoid');

% Created function for conversion to compare:
ENU_Vector = LLA_To_ENU(Waypoint_Initial(1),Waypoint_Initial(2),Waypoint_Initial(3),...
                        Reference_Point(1),Reference_Point(2),Reference_Point(3),...
                        Semi_Major_Axis,Flattening);
