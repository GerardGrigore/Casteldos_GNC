% -----------------------------------------------------------------------------
% Heading simplified estimation based on GNSS, IMU & Magnetometer measurements.
% -----------------------------------------------------------------------------

clear all;
clc;

% ----------------------------------------------------------------
% 1) Definition of the process model:
% ----------------------------------------------------------------
% We define the following process model state vector:
% STATES_INITIAL = [EAST_CURRENT;...
%                   NORTH_CURRENT;...
%                   PSI_HEADING_GPS;...
%                   VELOCITY_CURRENT_GPS]

% The associated Jacobians and first order Taylor Expansion are:
% * FUNCTION_PROCESS_MODEL_HEADING_POSITION_AZIMUTH;
% * JACOBIAN_STATES_PROCESS_MODEL_HEADING_POSITION_AZIMUTH;

% The process model covariance matrix is the following ('Q' matrix):
SIGMA_EAST_GPS = 25;
SIGMA_NORTH_GPS = 25;
SIGMA_PSI_HEADING_GPS = (2*pi/180)^2;
SIGMA_VELOCITY_GPS = 10e-4;
PROCESS_MODEL_COVARIANCE_MATRIX = [SIGMA_EAST_GPS 0 0 0;
                                   0 SIGMA_NORTH_GPS 0 0;
                                   0 0 SIGMA_PSI_HEADING_GPS 0;
                                   0 0 0 SIGMA_VELOCITY_GPS];

% The control input matrix 'B' matrix:
CONTROL_INPUTS_INITIAL = 0; % Process model considered as without input influence.

% Chose the step time, it shall be aligned with HW specification:
TIME_STEP_HW = 1;

% ----------------------------------------------------------------
% 2) Definition of the measurement & innovation model:
% ----------------------------------------------------------------
% We define the following measurement model state vector:
% MEASUREMENT_VECTOR = [EAST_CURRENT_GPS;...
%                       NORTH_CURRENT_GPS;...
%                       PSI_HEADING_CURRENT_GPS;...
%                       VELOCITY_CURRENT_GPS;...
%                       PSI_HEADING_CURRENT_MAGNETOMETER;...
%                       AZIMUTH_MAGNETOMETER];

% The associated Jacobians and first order Taylor Expansion are:
% * FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH;
% * JACOBIAN_STATES_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH;

% However, in our case, the JACOBIAN_STATES_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH
% can be took as a constant:
JACOBIAN_STATES_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH_CST = [1 0 0 0;
                                                                  0 1 0 0;
                                                                  0 0 1 0;
                                                                  0 0 0 1;
                                                                  0 0 1 0;
                                                                  0 0 1 0];

% Definition of the measurement noise covariance matrix refered as 'R' matrix:
% We define here the used covariance values from the Magnetometer:
SIGMA_PSI_HEADING_MAGNETOMETER = (2*pi/180)^2;
SIGMA_AZIMUTH_MAGNETOMETER = (2*pi/180)^2;
% MEASUREMENT_COVARIANCE_MATRIX = [SIGMA_EAST_GPS 0 0 0 0 0;
%                                  0 SIGMA_NORTH_GPS 0 0 0 0;
%                                  0 0 SIGMA_PSI_HEADING_GPS 0 0 0;
%                                  0 0 0 SIGMA_VELOCITY_GPS 0 0;
%                                  0 0 0 0 SIGMA_PSI_HEADING_MAGNETOMETER SIGMA_PSI_HEADING_MAGNETOMETER;
%                                  0 0 0 0 SIGMA_PSI_HEADING_MAGNETOMETER SIGMA_AZIMUTH_MAGNETOMETER];
MEASUREMENT_COVARIANCE_MATRIX = [SIGMA_EAST_GPS 0 0 0 0 0;
                                 0 SIGMA_NORTH_GPS 0 0 0 0;
                                 0 0 SIGMA_PSI_HEADING_GPS 0 0 0;
                                 0 0 0 SIGMA_VELOCITY_GPS 0 0;
                                 0 0 0 0 SIGMA_PSI_HEADING_MAGNETOMETER 0;
                                 0 0 0 0 0 SIGMA_AZIMUTH_MAGNETOMETER];

% ----------------------------------------------------------------
% 3) Simulated measurements block:
% ----------------------------------------------------------------

% Define some measurements along time to simulate the sensor fusion done by
% the EKF. Then the MEASUREMENT_VECTOR will has to be called in the loop
% calculation:
VELOCITY_CURRENT_GPS = 5/3.6;
% Re-use the constant velocity declared in the process model section.
% Definition of step times:
EAST_GPS = [0,1,2,3,4,5,6,7,8,9,10];
% NORTH_GPS = [0,-0.5,-1,-1.5,-2,-2.5,-3,-3.5,-4,-4.5,-5];
NORTH_GPS = -EAST_GPS;
% PSI_HEADING_GPS = [-65,-64,-66,-67,-69,-65,-63,-62,-61,-60,-59]*(pi/180);
PSI_HEADING_GPS = [-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45]*(pi/180);
VELOCITY_GPS = [VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,...
            VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,VELOCITY_CURRENT_GPS,...
            VELOCITY_CURRENT_GPS];
% Magnetometer measurements generation:
PSI_HEADING_MAGNETOMETER = [-44.5,-44.8,-44.9,-45.2,-45.1,-44.99,-45,-45,-44.78,-44,-45.01]*(pi/180);
AZIMUTH_MAGNETOMETER = [-225,-224,-223,-224.3,-224.1,-225,-225.3,-224.8,-223,-225,-224]*(pi/180);

% Define the initial values for each of the measurements:
EAST_INIT_GPS = EAST_GPS(1);
NORTH_INIT_GPS = NORTH_GPS(1);
PSI_HEADING_INIT_GPS = PSI_HEADING_GPS(1);
VELOCITY_INIT_GPS = VELOCITY_GPS(1);
PSI_HEADING_INIT_MAGNETOMETER = PSI_HEADING_MAGNETOMETER(1);
AZIMUTH_INIT_MAGNETOMETER = AZIMUTH_MAGNETOMETER(1);

% According to the measurement, initialize the state of the process model:
STATES_INITIAL = [EAST_INIT_GPS;NORTH_INIT_GPS;PSI_HEADING_INIT_GPS;VELOCITY_INIT_GPS];


% ----------------------------------------------------------------
% 4) Extended Kalman Filter loop calculations:
% ----------------------------------------------------------------

% We define the covariance matrix of the states at the initial step refered as the 'P0|0':
COVARIANCE_STATES_INITIAL = [SIGMA_EAST_GPS 0 0 0;
                             0 SIGMA_NORTH_GPS 0 0;
                             0 0 SIGMA_PSI_HEADING_GPS 0;
                             0 0 0 SIGMA_VELOCITY_GPS];

NUMBER_MEASUREMENTS = length(EAST_GPS);
% Store the results:
STATES_ESTIMATED_EKF = [STATES_INITIAL];
COVARIANCE_ESTIMATED_EKF = [COVARIANCE_STATES_INITIAL];

% Initialize the non-empty covariance vectors:
COVARIANCE_EAST = [COVARIANCE_STATES_INITIAL(1,1)];
COVARIANCE_NORTH = [COVARIANCE_STATES_INITIAL(2,2)];
COVARIANCE_PSI_HEADING = [COVARIANCE_STATES_INITIAL(3,3)];
COVARIANCE_VELOCITY = [COVARIANCE_STATES_INITIAL(4,4)];

for time_indexes = 1:(NUMBER_MEASUREMENTS-1)

    % Initialization:
    % ---------------
    STATES_PREV = STATES_INITIAL; % The 'X' state vector.
    COVARIANCE_STATE_PREV = COVARIANCE_STATES_INITIAL; % The 'P0|0'.
    CONTROL_INPUTS_PREV = CONTROL_INPUTS_INITIAL; % The usual 'B' matrix but unused here.
    PROCESS_MODEL_COVARIANCE_MATRIX_PREV = PROCESS_MODEL_COVARIANCE_MATRIX; % The 'Q' process model matrix.
    MEASUREMENT_COVARIANCE_MATRIX_PREV = MEASUREMENT_COVARIANCE_MATRIX; % The 'R' measurement matrix.
    MEASUREMENT_CURRENT = [EAST_GPS(time_indexes),NORTH_GPS(time_indexes),PSI_HEADING_GPS(time_indexes),VELOCITY_GPS(time_indexes),...
                           PSI_HEADING_MAGNETOMETER(time_indexes),AZIMUTH_MAGNETOMETER(time_indexes)];
    
    % EKF loop calculation:
    % ---------------------
    [STATE_APOSTERIORI_CURRENT,COVARIANCE_APOSTERIORI_CURRENT] = EKF_HEADING_POSITION_HEADING_AZIMUTH(STATES_PREV,COVARIANCE_STATE_PREV,CONTROL_INPUTS_PREV,...
                                                                             JACOBIAN_STATES_PROCESS_MODEL_HEADING_POSITION_AZIMUTH(STATES_PREV(4),STATES_PREV(3),TIME_STEP_HW),...
                                                                             PROCESS_MODEL_COVARIANCE_MATRIX_PREV,JACOBIAN_STATES_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH_CST,...
                                                                             MEASUREMENT_COVARIANCE_MATRIX_PREV,...
                                                                             FUNCTION_PROCESS_MODEL_HEADING_POSITION_AZIMUTH(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),TIME_STEP_HW),...
                                                                             MEASUREMENT_CURRENT',...
                                                                             eye(size(PROCESS_MODEL_COVARIANCE_MATRIX,1)),...
                                                                             eye(size(MEASUREMENT_COVARIANCE_MATRIX,1)),...
                                                                             FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),STATES_PREV(3),TIME_STEP_HW));


    % Inputs update:
    % --------------
    STATES_INITIAL = STATE_APOSTERIORI_CURRENT;
    COVARIANCE_STATES_INITIAL = COVARIANCE_APOSTERIORI_CURRENT;
    % Store the results:
    STATES_ESTIMATED_EKF = [STATES_ESTIMATED_EKF,STATE_APOSTERIORI_CURRENT];
    COVARIANCE_ESTIMATED_EKF = [COVARIANCE_ESTIMATED_EKF,COVARIANCE_APOSTERIORI_CURRENT];

    % Store the interesting components of the covariance matrix (diagonal
    % terms):
    COVARIANCE_EAST = [COVARIANCE_EAST,COVARIANCE_APOSTERIORI_CURRENT(1,1)];
    COVARIANCE_NORTH = [COVARIANCE_NORTH,COVARIANCE_APOSTERIORI_CURRENT(2,2)];
    COVARIANCE_PSI_HEADING = [COVARIANCE_PSI_HEADING,COVARIANCE_APOSTERIORI_CURRENT(3,3)];
    COVARIANCE_VELOCITY = [COVARIANCE_VELOCITY,COVARIANCE_APOSTERIORI_CURRENT(4,4)];

end


% ----------------------------------------------------------------
% 5) Plot simulation results to conclude on the efficiency:
% ----------------------------------------------------------------

% Plot all the states on several respective figure:
% Plot the East estimation:
figure(1);
plot([1:NUMBER_MEASUREMENTS],EAST_GPS,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(1,:),'blue');
legend('Raw sensor position along East-axis','Estimated position along East-axis with heading measurements');
xlabel('Time simulation (number of measurements)');
ylabel('Position along East-axis (in meters)');
title('Position along East-axis in function of time (simulation number)');

% Plot the North estimation:
figure(2);
plot([1:NUMBER_MEASUREMENTS],NORTH_GPS,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(2,:),'blue');
legend('Raw sensor position along North-axis','Estimated position along North-axis with heading measurements');
xlabel('Time simulation (number of measurements)');
ylabel('Position along North-axis (in meters)');
title('Position along North-axis in function of time (simulation number)');

% Plot the heading angle:
figure(3);
plot([1:NUMBER_MEASUREMENTS],PSI_HEADING_GPS,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],PSI_HEADING_MAGNETOMETER,'blue');
hold on
plot([1:NUMBER_MEASUREMENTS],AZIMUTH_MAGNETOMETER + pi,'black');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(3,:),'green');
legend('Raw sensor heading GNSS','Raw sensor Magnetometer Heading','Raw sensor Magnetometer Azimuth','Estimated heading');
xlabel('Time simulation (number of measurements)');
ylabel('Raw sensors measurements (in rad)');
title('Sensor headings and fused headings in function of time (simulation number)');

% No need to plot the velocity as it is considered being time invariant.

% ----------------------------------------------------------------
% 6) Plot the covariance matrices evolution:
% ----------------------------------------------------------------

figure(6);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_EAST,'red');
legend('Covariance of the position along East-axis');
xlabel('Time simulation (number of measurements)');
title('Covariance along East-axis in function of time (simulation number)');

figure(7);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_NORTH,'red');
legend('Covariance of the position along North-axis');
xlabel('Time simulation (number of measurements)');
title('Covariance along North-axis in function of time (simulation number)');

figure(8);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_PSI_HEADING,'red');
legend('Covariance of the PHI angle heading');
xlabel('Time simulation (number of measurements)');
title('Covariance of the heading in function of time (simulation number)');

figure(9);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_VELOCITY,'red');
legend('Covariance of the velocity');
xlabel('Time simulation (number of measurements)');
title('Covariance of the velocity in function of time (simulation number)');





