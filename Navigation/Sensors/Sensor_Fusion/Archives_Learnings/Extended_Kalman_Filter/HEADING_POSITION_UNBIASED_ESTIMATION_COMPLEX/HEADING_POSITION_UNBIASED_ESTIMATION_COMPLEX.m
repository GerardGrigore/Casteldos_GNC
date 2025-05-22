% ----------------------------------------------------------------
% Heading complex estimation based on GNSS and IMU measurements.
% ----------------------------------------------------------------

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

% The used Jacobian functions for the process model are:
% * JACOBIAN_STATES_PROCESS_MODEL;
% * JACOBIAN_NOISES_PROCESS_MODEL;
% * FUNCTION_PROCESS_MODEL_HEADING_POSITION;

% We then define the process noise covariance matrix refered as the 'Q' matrix in estimation:
% Here composed by the JERK and the ANGULAR_ACCELERATION.
SIGMA_ANGULAR_ACCELERATION_YAW = 10*(20*pi/180)^2;
SIGMA_JERK = 1000;
PROCESS_MODEL_COVARIANCE_MATRIX = [SIGMA_ANGULAR_ACCELERATION_YAW 0;0 SIGMA_JERK]; % Search for the influence of this term.

% Define the control input 'B' matrix:
CONTROL_INPUTS_INITIAL = 0; 

% Define the time step:
TIME_STEP_HW = 1;

% ----------------------------------------------------------------
% 2) Measurement & Innovation model:
% ----------------------------------------------------------------
% We define the following measurement model state vectors:
% MEASUREMENT_VECTOR = [EAST_CURRENT;...
%                       NORTH_CURRENT;...
%                       PHI_HEADING_CURRENT;...
%                       VELOCITY_CURRENT;...
%                       ANGULAR_RATE_PHI_POINT_CURRENT;...
%                       ACCELERATION_LONGITUDINAL_CURRENT;...
%                       ACCELERATION_LATERAL_CURRENT];
% Or:
% MEASUREMENT_VECTOR = [EAST_CURRENT;...
%                       NORTH_CURRENT;...
%                       PHI_HEADING_CURRENT;...
%                       VELOCITY_CURRENT;...
%                       ANGULAR_RATE_PHI_POINT_CURRENT;...
%                       ACCELERATION_LONGITUDINAL_CURRENT;...
%                       ACCELERATION_LATERAL_CURRENT;...
%                       PHI_HEADING_CURRENT_IMU;...
%                       PHI_HEADING_CURRENT_GYRO];
% Considering that PHI_HEADING_CURRENT corresponds to
% PHI_HEADING_CURRENT_GNSS.

% Activate a flag indicating the used model. If the flag is activated, it means
% that the heading angle will be fused using 3 sensors:
% * GNSS;
% * IMU;
% * GYROCOMPASS (Magnetometer);
IS_HEADING_IMU_GYRO_AVAILABLE = 0;

% The used Jacobian functions for the measurement model are:
% * JACOBIAN_STATES_MEASUREMENT_MODEL;
% * JACOBIAN_NOISES_MEASUREMENT_MODEL;
% * FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION;

% Definition of the measurement noise covariance matrix refered as 'R' matrix:
SIGMA_EAST = 25;
SIGMA_NORTH = 25;
SIGMA_PHI_HEADING = (10*pi/180)^2;
SIGMA_VELOCITY = 10e-4;
SIGMA_ANGULAR_RATE_PHI_POINT = (0.1*pi/180)^2;
SIGMA_ACCELERATION_LONGITUDINAL = 0.01;
SIGMA_ACCELERATION_LATERAL = 1;
SIGMA_PHI_HEADING_IMU = (12*pi/180)^2;
SIGMA_PHI_HEADING_GYRO = (11*pi/180)^2;

if ~IS_HEADING_IMU_GYRO_AVAILABLE
    MEASUREMENT_COVARIANCE_MATRIX = [SIGMA_EAST 0 0 0 0 0 0;
                                     0 SIGMA_NORTH 0 0 0 0 0;
                                     0 0 SIGMA_PHI_HEADING 0 0 0 0;
                                     0 0 0 SIGMA_VELOCITY 0 0 0;
                                     0 0 0 0 SIGMA_ANGULAR_RATE_PHI_POINT 0 0;
                                     0 0 0 0 0 SIGMA_ACCELERATION_LONGITUDINAL 0;
                                     0 0 0 0 0 0 SIGMA_ACCELERATION_LATERAL];
else
    MEASUREMENT_COVARIANCE_MATRIX = [SIGMA_EAST 0 0 0 0 0 0 0 0;
                                     0 SIGMA_NORTH 0 0 0 0 0 0 0;
                                     0 0 SIGMA_PHI_HEADING 0 0 0 0 0 0;
                                     0 0 0 SIGMA_VELOCITY 0 0 0 0 0;
                                     0 0 0 0 SIGMA_ANGULAR_RATE_PHI_POINT 0 0 0 0;
                                     0 0 0 0 0 SIGMA_ACCELERATION_LONGITUDINAL 0 0 0;
                                     0 0 0 0 0 0 SIGMA_ACCELERATION_LATERAL 0 0;
                                     0 0 0 0 0 0 0 SIGMA_PHI_HEADING_IMU 0;
                                     0 0 0 0 0 0 0 0 SIGMA_PHI_HEADING_GYRO];
end

% ----------------------------------------------------------------
% 3) Simulated measurements block:
% ----------------------------------------------------------------

% NOTA - For the measurement generation, use the rand function:
% NUMBER_MEASUREMENTS_GENERATED = 11;
% PHI_MIN = -45.99;
% PHI_MAX = -44.99;
% PHI_HEADING_RANGE = (PHI_MAX - PHI_MIN)*rand(NUMBER_MEASUREMENTS,1) + PHI_MIN;
% PHI_HEADING_RANGE = PHI_HEADING_RANGE';

% Definition of the current velocity aimed:
VELOCITY_CURRENT = 5/3.6; % Constant velocity approximation.
% Simple East vector:
EAST = [0,1,2,3,4,5,6,7,8,9,10];
% Simple North vector:
NORTH = -EAST;
% Constant heading angle:
PHI_HEADING = [-45,-45,-45,-45,-45,-45,-45,-45,-45,-45,-45]*(pi/180);
% Propagation of the velocity vector:
VELOCITY = [VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,...
            VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,...
            VELOCITY_CURRENT];
% Angular rate:
ANGULAR_RATE_PHI_POINT = zeros(1,11)*(pi/180); % In rad/s.
% Longitudinal acceleration:
ACCELERATION_LONGITUDINAL = zeros(1,11); % In m/s².
% Lateral acceleration:
ACCELERATION_LATERAL = ANGULAR_RATE_PHI_POINT.*VELOCITY;
% Define anyway the measurements heading of the IMU and the GYROCOMPASS:
PHI_HEADING_IMU = [-44.8,-44.9,-44.78,-44.98,-44.99,-45.01,-45.2,-44.87,-44.965,-45.21,-44.75]*(pi/180);
PHI_HEADING_GYROCOMPASS = [-44.9,-45.0,-44.799,-44.945,-44.56,-45.001,-45.22,-44.77,-44.92,-45.25,-44.88]*(pi/180);
% Define the initial values for each of the measurements:
EAST_INIT = EAST(1);
NORTH_INIT = NORTH(1);
PHI_HEADING_INIT = PHI_HEADING(1);
VELOCITY_INIT = VELOCITY(1);
ANGULAR_RATE_PHI_POINT_INIT = ANGULAR_RATE_PHI_POINT(1);
ACCELERATION_LONGITUDINAL_INIT = ACCELERATION_LONGITUDINAL(1);
ACCELERATION_LATERAL_INIT = ACCELERATION_LATERAL(1);
PHI_HEADING_IMU_INIT = PHI_HEADING_IMU(1);
PHI_HEADING_GYROCOMPASS_INIT = PHI_HEADING_GYROCOMPASS(1);
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
POSITION_COVARIANCE_MATRIX = [[SIGMA_EAST 0;
                               0 SIGMA_NORTH]];

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
    if ~IS_HEADING_IMU_GYRO_AVAILABLE
        MEASUREMENT_CURRENT = [EAST(time_indexes),NORTH(time_indexes),PHI_HEADING(time_indexes),VELOCITY(time_indexes),...
                               ANGULAR_RATE_PHI_POINT(time_indexes),ACCELERATION_LONGITUDINAL(time_indexes),...
                               ACCELERATION_LATERAL(time_indexes)];
    else
        MEASUREMENT_CURRENT = [EAST(time_indexes),NORTH(time_indexes),PHI_HEADING(time_indexes),VELOCITY(time_indexes),...
                               ANGULAR_RATE_PHI_POINT(time_indexes),ACCELERATION_LONGITUDINAL(time_indexes),...
                               ACCELERATION_LATERAL(time_indexes),PHI_HEADING_IMU(time_indexes),PHI_HEADING_GYROCOMPASS(time_indexes)];
    end
    SIZE_MEASUREMENT_VECTOR = length(MEASUREMENT_CURRENT);
    
    % EKF loop calculation:
    % ---------------------
    [STATE_APOSTERIORI_CURRENT,COVARIANCE_APOSTERIORI_CURRENT] = EKF_HEADING_POSITION_UNBIASED(COVARIANCE_STATE_PREV,JACOBIAN_STATES_PROCESS_MODEL(STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),STATES_PREV(6),TIME_STEP_HW),...
                                                                             PROCESS_MODEL_COVARIANCE_MATRIX_PREV,JACOBIAN_STATES_MEASUREMENT_MODEL(STATES_PREV(4),STATES_PREV(5),IS_HEADING_IMU_GYRO_AVAILABLE),...
                                                                             MEASUREMENT_COVARIANCE_MATRIX_PREV,FUNCTION_PROCESS_MODEL_HEADING_POSITION(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),STATES_PREV(6),TIME_STEP_HW),...
                                                                             MEASUREMENT_CURRENT', JACOBIAN_NOISES_PROCESS_MODEL(STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),STATES_PREV(6),0,0,TIME_STEP_HW),...
                                                                             JACOBIAN_NOISES_MEASUREMENT_MODEL(SIZE_MEASUREMENT_VECTOR),FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),STATES_PREV(5),...
                                                                             STATES_PREV(6),TIME_STEP_HW,IS_HEADING_IMU_GYRO_AVAILABLE));


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

    % In particular, for ellipse error on position, store the position
    % covariance matrix:
    POSITION_COVARIANCE_MATRIX = [POSITION_COVARIANCE_MATRIX;COVARIANCE_APOSTERIORI_CURRENT(1:2,1:2)];

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
if ~IS_HEADING_IMU_GYRO_AVAILABLE
    figure(3);
    plot([1:NUMBER_MEASUREMENTS],PHI_HEADING,'red');
    hold on
    plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(3,:),'green');
    legend('Raw sensor heading GNSS','Estimated heading');
    xlabel('Time simulation (number of measurements)');
    ylabel('Raw sensors measurements (in rad)');
    title('Sensor headings and fused headings in function of time (simulation number)');
else
    figure(3);
    plot([1:NUMBER_MEASUREMENTS],PHI_HEADING,'red');
    hold on
    plot([1:NUMBER_MEASUREMENTS],PHI_HEADING_IMU,'blue');
    hold on
    plot([1:NUMBER_MEASUREMENTS],PHI_HEADING_GYROCOMPASS,'black');
    hold on
    plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(3,:),'green');
    legend('Raw sensor heading GNSS','Raw sensor heading IMU','Raw sensor heading Gyrocompass','Estimated heading');
    xlabel('Time simulation (number of measurements)');
    ylabel('Raw sensors measurements (in rad)');
    title('Sensor headings and fused headings in function of time (simulation number)');
end

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

% ----------------------------------------------------------------
% 7) Plot the confidence ellipses:
% ----------------------------------------------------------------

% Initialize important numerical values:
NUMBER_OF_STATES = length(STATES_INITIAL);
NUMBER_COLUMS_COVARIANCE_ESTIMATED_EKF = NUMBER_OF_STATES*NUMBER_MEASUREMENTS;
% The position covariance matrices (2-by-2) are available in the following
% parameter POSITION_COVARIANCE_MATRIX.
% Therefore, for each position measurements, one shall define the
% CONFIDENCE_INTERVAL.
% Since all the estimates are available, extract the estimated positions:
EAST_ESTIMATED = STATES_ESTIMATED_EKF(1,:);
NORTH_ESTIMATED = STATES_ESTIMATED_EKF(2,:);
% Compute the standard deviation of the vectors:
EAST_ESTIMATED_STANDARD_DEVIATION = std(EAST_ESTIMATED);
NORTH_ESTIMATED_STANDARD_DEVIATION = std(NORTH_ESTIMATED);
% For each estimates, determine its confidence interval at 95%:
for index_positions = 1:length(EAST_ESTIMATED)
    % For east positions:
    EAST_CONFIDENCE_INTERVAL(index_positions,1:2) = [EAST_ESTIMATED(index_positions) - 1.96*EAST_ESTIMATED_STANDARD_DEVIATION,...
                                                     EAST_ESTIMATED(index_positions) + 1.96*EAST_ESTIMATED_STANDARD_DEVIATION];
    % For north positions:
    NORTH_CONFIDENCE_INTERVAL(index_positions,1:2) = [NORTH_ESTIMATED(index_positions) - 1.96*NORTH_ESTIMATED_STANDARD_DEVIATION,...
                                                     NORTH_ESTIMATED(index_positions) + 1.96*NORTH_ESTIMATED_STANDARD_DEVIATION];
end
% Therefore, possible to use the created function POSITION_ERROR_ELLIPSE in
% order to plot the positions and the associated ellipses:
figure(12);
for index_estimation = 1:length(EAST_ESTIMATED)
    % Determine the current position point:
    EAST_ORIGIN_ELLIPSE = EAST_ESTIMATED(index_estimation);
    NORTH_ORIGIN_ELLIPSE = NORTH_ESTIMATED(index_estimation);
    % Determine the current POSITION_COVARIANCE_MATRIX:
    POSITION_COVARIANCE_MATRIX_CURRENT = POSITION_COVARIANCE_MATRIX(2*index_estimation - 1:2*index_estimation,1:2);
    % Since every intervall calulated was at 95%:
    CONFIDENCE_INTERVAL_CURRENT = 95;
    [POSITION_EAST_ELLIPSE,POSITION_NORTH_ELLIPSE] = POSITION_ERROR_ELLIPSES(EAST_ORIGIN_ELLIPSE,...
                                                                             NORTH_ORIGIN_ELLIPSE,...
                                                                             POSITION_COVARIANCE_MATRIX_CURRENT,...
                                                                             CONFIDENCE_INTERVAL_CURRENT);
    % Then plot the ellipse and the current position on the figure:
    plot(EAST_ORIGIN_ELLIPSE,NORTH_ORIGIN_ELLIPSE,'o');
    hold on;
    plot(POSITION_EAST_ELLIPSE,POSITION_NORTH_ELLIPSE,'red');
    hold on;
end
xlabel('East position (in meters)');
ylabel('North position (in meters)');
title('North position in function of East position with error ellipses');












