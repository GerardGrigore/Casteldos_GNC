% Post Mission Analysis of the 'Simulator_Case_9' model:
close all;

% Simulated positions along the X-Axis and Y-Axis of the Body-Fixed
% frame (positions of the ship):
Position_Along_X_Axis_Simulator = out.Position_Along_X_Axis.Data;
Position_Along_Y_Axis_Simulator = out.Position_Along_Y_Axis.Data;

% Simulated velocities:
Velocity_Along_X_Axis_Simulator = out.Velocity_Along_X_Axis_Local.Data;
Velocity_Along_Y_Axis_Simulator = out.Velocity_Along_Y_Axis_Local.Data;

% Simulated time:
Time_Velocity_Plots = out.Velocity_Along_X_Axis_Local.Time;

% Simulated headings and respective times:
Heading_GPS_Noisy_Processed = out.Heading_GPS.Data;
Time_Heading_GPS = out.Heading_GPS.Time;                                           
Heading_Magnetometer_Noisy = out.Heading_Noisy_Magnetometer.Data;
Time_Heading_Magnetometer = out.Heading_Noisy_Magnetometer.Time;
Heading_Filtered_Fused = out.Heading_Kalman_Filtered.Data;
Time_Heading_Filtered = out.Heading_Kalman_Filtered.Time;
Heading_Filtered_Low_Frequency = out.Heading_Filtered_Waves.Data;
Time_Heading_Filtered_Low_Frequency = out.Heading_Filtered_Waves.Time;

% Estimated pulsation and damping of the waves:
Pulsation_Wave_Estimated = out.Pulsation_Wave_Estimated.Data;
Time_Pulsation_Wave_Estimated = out.Pulsation_Wave_Estimated.Time;
Damping_Wave_Estimated = out.Damping_Wave_Estimated.Data;
Time_Damping_Wave_Estimated = out.Damping_Wave_Estimated.Time;

% Wave heading estimation:
Heading_Wave_Input = out.Heading_Wave_Simulated_Input.Data;
Time_Heading_Wave_Input = out.Heading_Wave_Simulated_Input.Time;
Heading_Wave_Estimated = out.Heading_Wave_Estimated.Data;
Time_Heading_Wave_Estimated = out.Heading_Wave_Estimated.Time;

% Initially aimed trajectory:
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
Semi_Major_Axis = 6378137.0;
Flattening = 1/298.25;
for index_waypoints = 1:length(Waypoints_In_LLA)
    Waypoints_In_ENU(index_waypoints,:) = LLA_To_ENU(Waypoints_In_LLA(index_waypoints,1),Waypoints_In_LLA(index_waypoints,2),...
                                                     Waypoints_In_LLA(index_waypoints,3),...
                                                     Reference_Point_Port_Mahon(1),Reference_Point_Port_Mahon(2),Reference_Point_Port_Mahon(3),...
                                                     Semi_Major_Axis,Flattening);
end

% Main waypoints along the path:
Waypoints_X = [Waypoints_In_ENU(:,1)'];
Waypoints_Y = [Waypoints_In_ENU(:,2)'];

% Plots:
figure;
plot(Waypoints_X,Waypoints_Y,'red'); % Trajectory.
hold on;
PLOTS_WAYPOINTS = scatter(Waypoints_X,Waypoints_Y,'+'); % Waypoints.
PLOTS_WAYPOINTS.LineWidth = 2;
hold on;
Position = plot(Position_Along_X_Axis_Simulator,Position_Along_Y_Axis_Simulator,'blue'); % Simulated trajectory.
Position.LineWidth = 1.5;
legend('Waypoints','Waypoints Points','Propagated positions');
xlabel('Horizontal East position (m)');
ylabel('Vertical North position (m)');
title('Simulation of a trajectory following');

figure;
plot(Time_Velocity_Plots,Velocity_Along_X_Axis_Simulator);
hold on;
plot(Time_Velocity_Plots,Velocity_Along_Y_Axis_Simulator);
hold on;
plot(Time_Velocity_Plots,sqrt(Velocity_Along_X_Axis_Simulator.^2 + Velocity_Along_Y_Axis_Simulator.^2));
legend('Velocity along X-Axis','Velocity along Y-Axis','Norm of the velocity');
xlabel('Simulation time (s)');
ylabel('Velocities (m/s)');
title('Velocity in function of time');

figure;
% Headings:
plot(Time_Heading_GPS,Heading_GPS_Noisy_Processed,'magenta');
hold on;
plot(Time_Heading_Magnetometer,Heading_Magnetometer_Noisy,'red');
hold on;
plot(Time_Heading_Filtered,Heading_Filtered_Fused,'cyan');
hold on;
plot(Time_Heading_Filtered_Low_Frequency,Heading_Filtered_Low_Frequency,'black');
legend('Heading from noisy GPS','Heading from noisy Magnetometer',...
       'Heading filtered through Kalman','Heading without wave contribution estimated');
xlabel('Time of measurements (s)');
ylabel('Headings (rad)');
title('Headings in function of time');

figure;
% Pulsation and damping of waves:
plot(Time_Pulsation_Wave_Estimated,Pulsation_Wave_Estimated,'red');
hold on;
yline(Pulsation_Wave,'black');
legend('Estimated pulsation of the waves','Simulated pulsation of the waves');
xlabel('Time (seconds)');
ylabel('Pulsation (rad/s)');
title('Estimation of the pulsation of the waves and comparison with simulation input.');

figure;
% Pulsation and damping of waves:
plot(Time_Damping_Wave_Estimated,Damping_Wave_Estimated,'red');
hold on;
yline(Damping_Factor_Waves,'black');
legend('Estimated damping of the waves','Simulated damping of the waves');
xlabel('Time (seconds)');
ylabel('Damping coefficient (rad/s)');
title('Estimation of the damping of the waves and comparison with simulation input.');

figure;
% Wave heading estimation:
plot(Time_Heading_Wave_Input,Heading_Wave_Input);
hold on;
plot(Time_Heading_Wave_Estimated,Heading_Wave_Estimated);
legend('Generated input environnement heading wave','Estimated heading due to waves');
xlabel('Time (seconds)');
ylabel('Heading of the waves (rad/s)');
title('Estimation of the heading of the waves and comparison with simulation input.');


















