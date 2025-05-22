% ------------------------------------------------------------------
% Heading simplified estimation based on GNSS and IMU measurements.
% ------------------------------------------------------------------

clear all;
clc;

% ----------------------------------------------------------------
% 1) Definition of the process model:
% ----------------------------------------------------------------
% We define the following process model state vector:
% STATES_INITIAL = [EAST_CURRENT;...
%                   NORTH_CURRENT;...
%                   PHI_HEADING_CURRENT;...
%                   VELOCITY_CURRENT;...
%                   ANGULAR_RATE_PHI_POINT_CURRENT;...
%                   ACCELERATION_LONGITUDINAL_CURRENT];

% The associated Jacobians and first order Taylor Expansion are:
% * FUNCTION_PROCESS_MODEL_HEADING_POSITION_SIMPLIFIED;
% * JACOBIAN_STATES_PROCESS_MODEL_SIMPLIFIED;

% The process model covariance matrix is the following ('Q' matrix):
SIGMA_EAST = 25;
SIGMA_NORTH = 25;
SIGMA_PHI_HEADING = (10*pi/180)^2;
SIGMA_VELOCITY = 10e-4;
SIGMA_ANGULAR_RATE_PHI_POINT = (0.1*pi/180)^2;
SIGMA_ACCELERATION_LONGITUDINAL = 0.01;
SIGMA_ACCELERATION_LATERAL = 1;
PROCESS_MODEL_COVARIANCE_MATRIX = [SIGMA_EAST 0 0 0 0 0;
                                   0 SIGMA_NORTH 0 0 0 0;
                                   0 0 SIGMA_PHI_HEADING 0 0 0;
                                   0 0 0 SIGMA_VELOCITY 0 0;
                                   0 0 0 0 SIGMA_ANGULAR_RATE_PHI_POINT 0;
                                   0 0 0 0 0 SIGMA_ACCELERATION_LONGITUDINAL];

% The control input matrix 'B' matrix:
CONTROL_INPUTS_INITIAL = 0; % Process model considered as without input influence.

% Chose the step time, it shall be aligned with HW specification:
TIME_STEP_HW = 1;

% ----------------------------------------------------------------
% 2) Definition of the measurement & innovation model:
% ----------------------------------------------------------------
% We define the following measurement model state vector:
% MEASUREMENT_VECTOR = [EAST_CURRENT;...
%                       NORTH_CURRENT;...
%                       PHI_HEADING_CURRENT;...
%                       VELOCITY_CURRENT;...
%                       ANGULAR_RATE_PHI_POINT_CURRENT;...
%                       ACCELERATION_LONGITUDINAL_CURRENT;...
%                       ACCELERATION_LATERAL_CURRENT];

% The associated Jacobians and first order Taylor Expansion are:
% * FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_SIMPLIFIED;
% * JACOBIAN_STATES_MEASUREMENT_MODEL_SIMPLIFIED;

% Definition of the measurement noise covariance matrix refered as 'R' matrix:
MEASUREMENT_COVARIANCE_MATRIX = [SIGMA_EAST 0 0 0 0 0 0;
                                 0 SIGMA_NORTH 0 0 0 0 0;
                                 0 0 SIGMA_PHI_HEADING 0 0 0 0;
                                 0 0 0 SIGMA_VELOCITY 0 0 0;
                                 0 0 0 0 SIGMA_ANGULAR_RATE_PHI_POINT 0 0;
                                 0 0 0 0 0 SIGMA_ACCELERATION_LONGITUDINAL 0;
                                 0 0 0 0 0 0 SIGMA_ACCELERATION_LATERAL];

% ----------------------------------------------------------------
% 3) Simulated measurements block:
% ----------------------------------------------------------------

% Define some measurements along time to simulate the sensor fusion done by
% the EKF. Then the MEASUREMENT_VECTOR will has to be called in the loop
% calculation:
VELOCITY_CURRENT = 5/3.6;
% Re-use the constant velocity declared in the process model section.
% Definition of step times:
EAST = [0,1,2,3,4,5,6,7,8,9,10];
% NORTH = [0,-0.5,-1,-1.5,-2,-2.5,-3,-3.5,-4,-4.5,-5];
NORTH = -EAST;
% PHI_HEADING = [-65,-64,-66,-67,-69,-65,-63,-62,-61,-60,-59]*(pi/180);
PHI_HEADING = [-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45]*(pi/180);
VELOCITY = [VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,...
            VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,...
            VELOCITY_CURRENT];
% ANGULAR_RATE_PHI_POINT = [1,2,1,2,4,2,1,1,1,1,1]*(pi/180); % In rad/s.
ANGULAR_RATE_PHI_POINT = zeros(1,11)*(pi/180); % In rad/s.
% /!\ Be careful for the simulation of the two following: /!\.
ACCELERATION_LONGITUDINAL = zeros(1,11); % In m/s².
ACCELERATION_LATERAL = ANGULAR_RATE_PHI_POINT.*VELOCITY;
% Define the initial values for each of the measurements:
EAST_INIT = EAST(1);
NORTH_INIT = NORTH(1);
PHI_HEADING_INIT = PHI_HEADING(1);
VELOCITY_INIT = VELOCITY(1);
ANGULAR_RATE_PHI_POINT_INIT = ANGULAR_RATE_PHI_POINT(1);
ACCELERATION_LONGITUDINAL_INIT = ACCELERATION_LONGITUDINAL(1);
ACCELERATION_LATERAL_INIT = ACCELERATION_LATERAL(1);
% According to the measurement, initialize the state of the process model:
STATES_INITIAL = [EAST_INIT;NORTH_INIT;PHI_HEADING_INIT;VELOCITY_INIT;
                  ANGULAR_RATE_PHI_POINT_INIT;ACCELERATION_LONGITUDINAL_INIT];


% ----------------------------------------------------------------
% 4) Extended Kalman Filter loop calculations:
% ----------------------------------------------------------------

% We define the covariance matrix of the states at the initial step refered as the 'P0|0':
COVARIANCE_STATES_INITIAL = [SIGMA_EAST 0 0 0 0 0;
                             0 SIGMA_NORTH 0 0 0 0;
                             0 0 SIGMA_PHI_HEADING 0 0 0;
                             0 0 0 SIGMA_VELOCITY 0 0;
                             0 0 0 0 SIGMA_ANGULAR_RATE_PHI_POINT 0;
                             0 0 0 0 0 SIGMA_ACCELERATION_LATERAL];

NUMBER_MEASUREMENTS = length(EAST);
% Store the results:
STATES_ESTIMATED_EKF = [STATES_INITIAL];
COVARIANCE_ESTIMATED_EKF = [COVARIANCE_STATES_INITIAL];

% Initialize the non-empty covariance vectors:
COVARIANCE_EAST = [COVARIANCE_STATES_INITIAL(1,1)];
COVARIANCE_NORTH = [COVARIANCE_STATES_INITIAL(2,2)];
COVARIANCE_PHI_HEADING = [COVARIANCE_STATES_INITIAL(3,3)];
COVARIANCE_VELOCITY = [COVARIANCE_STATES_INITIAL(4,4)];
COVARIANCE_ANGULAR_RATE_PHI_POINT = [COVARIANCE_STATES_INITIAL(5,5)];
COVARIANCE_ACCELERATION_LONGITUDINAL = [COVARIANCE_STATES_INITIAL(6,6)];

for time_indexes = 1:(NUMBER_MEASUREMENTS-1)

    % Initialization:
    % ---------------
    STATES_PREV = STATES_INITIAL; % The 'X' state vector.
    COVARIANCE_STATE_PREV = COVARIANCE_STATES_INITIAL; % The 'P0|0'.
    CONTROL_INPUTS_PREV = CONTROL_INPUTS_INITIAL; % The usual 'B' matrix but unused here.
    PROCESS_MODEL_COVARIANCE_MATRIX_PREV = PROCESS_MODEL_COVARIANCE_MATRIX; % The 'Q' process model matrix.
    MEASUREMENT_COVARIANCE_MATRIX_PREV = MEASUREMENT_COVARIANCE_MATRIX; % The 'R' measurement matrix.
    MEASUREMENT_CURRENT = [EAST(time_indexes),NORTH(time_indexes),PHI_HEADING(time_indexes),VELOCITY(time_indexes),...
                           ANGULAR_RATE_PHI_POINT(time_indexes),ACCELERATION_LONGITUDINAL(time_indexes),...
                           ACCELERATION_LATERAL(time_indexes)];
    
    % EKF loop calculation:
    % ---------------------
    [STATE_APOSTERIORI_CURRENT,COVARIANCE_APOSTERIORI_CURRENT] = EKF_HEADING_POSITION_UNBIASED_SIMPLIFIED(STATES_PREV,COVARIANCE_STATE_PREV,CONTROL_INPUTS_PREV,...
                                                                             JACOBIAN_STATES_PROCESS_MODEL_SIMPLIFIED(STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),STATES_PREV(6),TIME_STEP_HW),...
                                                                             PROCESS_MODEL_COVARIANCE_MATRIX_PREV,JACOBIAN_STATES_MEASUREMENT_MODEL_SIMPLIFIED(STATES_PREV(4),STATES_PREV(5)),...
                                                                             MEASUREMENT_COVARIANCE_MATRIX_PREV,...
                                                                             FUNCTION_PROCESS_MODEL_HEADING_POSITION_SIMPLIFIED(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),STATES_PREV(6),TIME_STEP_HW),...
                                                                             MEASUREMENT_CURRENT',...
                                                                             JACOBIAN_NOISES_PROCESS_MODEL_SIMPLIFIED(),...
                                                                             JACOBIAN_NOISES_MEASUREMENT_MODEL_SIMPLIFIED(),...
                                                                             FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_SIMPLIFIED(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),STATES_PREV(6),TIME_STEP_HW));


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
    COVARIANCE_PHI_HEADING = [COVARIANCE_PHI_HEADING,COVARIANCE_APOSTERIORI_CURRENT(3,3)];
    COVARIANCE_VELOCITY = [COVARIANCE_VELOCITY,COVARIANCE_APOSTERIORI_CURRENT(4,4)];
    COVARIANCE_ANGULAR_RATE_PHI_POINT = [COVARIANCE_ANGULAR_RATE_PHI_POINT,COVARIANCE_APOSTERIORI_CURRENT(5,5)];
    COVARIANCE_ACCELERATION_LONGITUDINAL = [COVARIANCE_ACCELERATION_LONGITUDINAL,COVARIANCE_APOSTERIORI_CURRENT(6,6)];

end


% ----------------------------------------------------------------
% 5) Plot simulation results to conclude on the efficiency:
% ----------------------------------------------------------------

% Plot all the states on several respective figure:
% Plot the East estimation:
figure(1);
plot([1:NUMBER_MEASUREMENTS],EAST,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(1,:),'blue');
legend('Raw sensor position along East-axis','Estimated position along East-axis with heading measurements');
xlabel('Time simulation (number of measurements)');
ylabel('Position along East-axis (in meters)');
title('Position along East-axis in function of time (simulation number)');

% Plot the North estimation:
figure(2);
plot([1:NUMBER_MEASUREMENTS],NORTH,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(2,:),'blue');
legend('Raw sensor position along North-axis','Estimated position along North-axis with heading measurements');
xlabel('Time simulation (number of measurements)');
ylabel('Position along North-axis (in meters)');
title('Position along North-axis in function of time (simulation number)');

% Plot the heading angle:
figure(3);
plot([1:NUMBER_MEASUREMENTS],PHI_HEADING,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(3,:),'green');
legend('Raw sensor heading GNSS','Estimated heading');
xlabel('Time simulation (number of measurements)');
ylabel('Raw sensors measurements (in rad)');
title('Sensor headings and fused headings in function of time (simulation number)');

% No need to plot the velocity as it is considered being time invariant.

% Plot the Angular rate estimation:
figure(4);
plot([1:NUMBER_MEASUREMENTS],ANGULAR_RATE_PHI_POINT,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(5,:),'blue');
legend('Raw sensor angular rate along Up-axis','Estimated angular rate along up-axis');
xlabel('Time simulation (number of measurements)');
ylabel('Angular rate along Up-axis');
title('Angular rate along Up-axis in function of time (simulation number)');

% Plot the acceleration longitudinal estimation:
figure(5);
plot([1:NUMBER_MEASUREMENTS],ACCELERATION_LONGITUDINAL,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(6,:),'blue');
legend('Raw sensor longitudinal acceleration','Estimated longitudinal acceleation');
xlabel('Time simulation (number of measurements)');
ylabel('Longitudinal acceleration');
title('Longitudinal acceleration in function of time (simulation number)');

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
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_PHI_HEADING,'red');
legend('Covariance of the PHI angle heading');
xlabel('Time simulation (number of measurements)');
title('Covariance of the heading in function of time (simulation number)');

figure(9);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_VELOCITY,'red');
legend('Covariance of the velocity');
xlabel('Time simulation (number of measurements)');
title('Covariance of the velocity in function of time (simulation number)');

figure(10);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_ANGULAR_RATE_PHI_POINT,'red');
legend('Covariance of the angular rate');
xlabel('Time simulation (number of measurements)');
title('Covariance of the angular rate in function of time (simulation number)');

figure(11);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_ACCELERATION_LONGITUDINAL,'red');
legend('Covariance of the longitudinal acceleration');
xlabel('Time simulation (number of measurements)');
title('Covariance of the longitudinal acceleration in function of time (simulation number)');





