%-----------------------------------------------------------------------
%***********************************************************************
%   Trajectory generation V0 - Aiming: Île de la Nadière
%-----------------------------------------------------------------------
%***********************************************************************

%***********************************************************************
%   I) Creation of the trajectory:
%***********************************************************************

% This script provided the user with the LLA coordiantes of the aimed
% nominal trajectory to reach the Île-de-la-Nadière location. Those
% coordinates will be stored in waypoints. 0 Altitude selected since
% navigation done at sea surface level. The retreived waypoints were taken
% from Google_Maps coordinates indications.

% Add to path the validated Guidance function:
addpath(genpath("C:\Users\gerar\Desktop\Bureau bis\Cours - Projets - Travail\GNC_Casteldos_Save\Guidance\Guidance_Algorithms\Waypoints_Based"));

% Waypoints in LLA frame:
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

% Waypoint objects creation:
Waypoints = [Waypoints_In_ENU(1,1:2);
             Waypoints_In_ENU(2,1:2);
             Waypoints_In_ENU(3,1:2);
             Waypoints_In_ENU(4,1:2);
             Waypoints_In_ENU(5,1:2);
             Waypoints_In_ENU(6,1:2)];

Number_Waypoints = length(Waypoints);

%***********************************************************************
%   II) LOS Guidance algorithm validation:
%***********************************************************************

% Need to simulate some estimated measured headings & ship positions:
% Based on the specified waypoints (in ENU):
Position_Ship_0 = [272,319];
Position_Ship_1 = [300,470];
Position_Ship_2 = [580,300];
Position_Ship_3 = [800,290];
Position_Ship_4 = [1200,102];
Position_Ship_5 = [1400,12];
Position_Ship_6 = [1800,-700];
Position_Ship_7 = [2100,-1000];
Position_Ship_8 = [2500,-1300];
Position_Ship_9 = [2600,-1400];
Position_Ship_10 = [2700,-1900];
Position_Ship_Vector = [Position_Ship_0;
                        Position_Ship_1;
                        Position_Ship_2;
                        Position_Ship_3;
                        Position_Ship_4;
                        Position_Ship_5;
                        Position_Ship_6;
                        Position_Ship_7;
                        Position_Ship_8;
                        Position_Ship_9;
                        Position_Ship_10];
Number_Of_Positions = length(Position_Ship_Vector);

Position_Ship_X = Position_Ship_Vector(:,1)';
Position_Ship_Y = Position_Ship_Vector(:,2)';

% Estimated headings according to ship position (taken in the compass circle fomat):
Estimated_Heading_0 = 65*pi/180;
Estimated_Heading_1 = 64*pi/180;
Estimated_Heading_2 = 90*pi/180;
Estimated_Heading_3 = 120*pi/180;
Estimated_Heading_4 = 125*pi/180;
Estimated_Heading_5 = 130*pi/180;
Estimated_Heading_6 = 180*pi/180;
Estimated_Heading_7 = 131*pi/180;
Estimated_Heading_8 = 125*pi/180;
Estimated_Heading_9 = 127*pi/180;
Estimated_Heading_10 = 128*pi/180;
Heading_Estimated_Vector = [Estimated_Heading_0;
                            Estimated_Heading_1;
                            Estimated_Heading_2;
                            Estimated_Heading_3;
                            Estimated_Heading_4;
                            Estimated_Heading_5;
                            Estimated_Heading_6;
                            Estimated_Heading_7;
                            Estimated_Heading_8;
                            Estimated_Heading_9;
                            Estimated_Heading_10];

% Guidance function called:
% Call the function to determine, for each positions of the ship and
% heading estimated, the heading to provide:

% Initialization of paramters:
Heading_Current_Unit_Vector = [];
Heading_Aimed_Unit_Vector = [];
Heading_Aimed_Compass_Vector = [];
Heading_Error_Unit_Vector = [];
Heading_Error_Compass_Vector = [];
Waypoints_Selected_For_Each_Heading = [];

for index_positions = 1:Number_Of_Positions
    
    % Guidance function calling:
    [Heading_Aimed_Unit,Heading_Aimed_Compass,...
     Heading_Error_Unit,Heading_Error_Compass,...
     Selected_Waypoints,Heading_Current_Unit] = Guidance_Heading_Rudder_Angle(Waypoints,...
                                                                              Position_Ship_X(index_positions),...
                                                                              Position_Ship_Y(index_positions), ...
                                                                              Heading_Estimated_Vector(index_positions));

    % Store the heading aimed in unit and compass formats:
    Heading_Aimed_Unit_Vector = [Heading_Aimed_Unit_Vector;Heading_Aimed_Unit];
    Heading_Aimed_Compass_Vector = [Heading_Aimed_Compass_Vector;Heading_Aimed_Compass];
    Heading_Current_Unit_Vector = [Heading_Current_Unit_Vector;Heading_Current_Unit];

    % Store the error of the heading hence the rudder angle to correct:
    Heading_Error_Unit_Vector = [Heading_Error_Unit_Vector;Heading_Error_Unit];
    Heading_Error_Compass_Vector = [Heading_Error_Compass_Vector;Heading_Error_Compass];

    % Store the selected waypoints for each headings if needed for
    % observation:
    Waypoints_Selected_For_Each_Heading = [Waypoints_Selected_For_Each_Heading;Selected_Waypoints];
end

% For validation purpose, plot all the aimed headings. Plot the two heading
% (compass and circle) to see the difference. Also plot the rudder angle
% aimed.

% Definition of a norm to visualize the direction on the plots:
Norm_Visual_Heading = 100;

% Retreive the number of headings:
Number_Of_Headings = length(Heading_Aimed_Unit_Vector);

% Plots:
figure;
plot(Waypoints_X,Waypoints_Y,'red'); % Trajectory.
hold on;
Plots_Waypoints = scatter(Waypoints_X,Waypoints_Y,'+'); % Waypoints.
Plots_Waypoints.LineWidth = 2;
hold on;
scatter(Position_Ship_X,Position_Ship_Y,'blue','filled'); % Estimated ship positions.

for index_headings = 1:Number_Of_Headings

    % Select the current heading aimed unit corrected to plot:
    Heading_Aimed_Unit_Current = Heading_Aimed_Unit_Vector(index_headings);
    % Select the current heading aimed compass corrected to plot:
    Heading_Aimed_Compass_Current = Heading_Aimed_Compass_Vector(index_headings);
    % Select the current Rudder heading error unit corrected to plot:
    Rudder_Error_Unit_Current = Heading_Error_Unit_Vector(index_headings);
    % Select the current heading aimed compass corrected to plot:
    Rudder_Error_Compass_Current = Heading_Error_Compass_Vector(index_headings);
    % plot of the current heading and go to see the sign of
    % Heading_Error_Unit_Vector to see if logical.
    Heading_Current_Unit_Current = Heading_Current_Unit_Vector(index_headings);

    % Retreive the components of the plotted 2D vector:
    % Here there is the difference for the plots: inversion of the use of sin and
    % cos in function of the conventions (unit or compass).
    Heading_Aimed_Unit_Current_East = Norm_Visual_Heading*cos(Heading_Aimed_Unit_Current);
    Heading_Aimed_Unit_Current_North = Norm_Visual_Heading*sin(Heading_Aimed_Unit_Current);
    Heading_Aimed_Compass_Current_East = Norm_Visual_Heading*sin(Heading_Aimed_Compass_Current);
    Heading_Aimed_Compass_Current_North = Norm_Visual_Heading*cos(Heading_Aimed_Compass_Current);
    Rudder_Error_Unit_Current_East = Norm_Visual_Heading*cos(Rudder_Error_Unit_Current + Heading_Current_Unit_Vector(index_headings));
    Rudder_Error_Unit_Current_North = Norm_Visual_Heading*sin(Rudder_Error_Unit_Current + Heading_Current_Unit_Vector(index_headings));
    Heading_Current_Unit_Current_East = Norm_Visual_Heading*cos(Heading_Current_Unit_Current);
    Heading_Current_Unit_Current_North = Norm_Visual_Heading*sin(Heading_Current_Unit_Current);

    % Then plot on the previous figure each headings aimed for all the
    % estimated ship position:
    hold on;
    plot([Position_Ship_X(index_headings) Position_Ship_X(index_headings) + Heading_Aimed_Unit_Current_East],...
         [Position_Ship_Y(index_headings) Position_Ship_Y(index_headings) + Heading_Aimed_Unit_Current_North],'green');
    plot([Position_Ship_X(index_headings) Position_Ship_X(index_headings) + Heading_Aimed_Compass_Current_East],...
         [Position_Ship_Y(index_headings) Position_Ship_Y(index_headings) + Heading_Aimed_Compass_Current_North],'magenta');

    % Then plot also the rudder angle according to the heading error and the initial unit heading current:
    plot([Position_Ship_X(index_headings) Position_Ship_X(index_headings) - Rudder_Error_Unit_Current_East],...
         [Position_Ship_Y(index_headings) Position_Ship_Y(index_headings) - Rudder_Error_Unit_Current_North],'black');
    plot([Position_Ship_X(index_headings) Position_Ship_X(index_headings) + Heading_Current_Unit_Current_East],...
         [Position_Ship_Y(index_headings) Position_Ship_Y(index_headings) + Heading_Current_Unit_Current_North],'blue');

end
legend('Trajectory Waypoints connexion','Waypoints alone','Ship Position alone',...
       'Heading to aim Unit circle','Heading to aim Compass Circle',...
       'Direction of Rudder to correct the heading','Heading initial current')
xlabel('East - Horizontal position of the ship (in meters)');
ylabel('North - Vertical position of the ship (in meters)');
title('Trajectory, desired waypoints, estimated position and corrected headings for Île-de-la-Nadière');








