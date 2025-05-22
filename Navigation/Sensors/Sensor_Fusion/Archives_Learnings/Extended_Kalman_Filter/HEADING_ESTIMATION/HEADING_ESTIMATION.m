% ----------------------------------------------------------------
% Heading estimation based on GNSS, IMU, GYROCOMPASS measurements.
% ----------------------------------------------------------------

clear all;
clc;

% ----------------------------------------------------------------
% 1) Definition of the process model:
% ----------------------------------------------------------------
% We define the following process model state vector:
% STATES_INITIAL = [POSITION_X_CURRENT;...
%                   POSITION_Y_CURRENT;...
%                   PSI_HEADING_CURRENT;...
%                   VELOCITY_CURRENT];

% Initiate the variables:
POSITION_X_CURRENT = 0;
POSITION_Y_CURRENT = 0;
PSI_HEADING_CURRENT = pi/4;
VELOCITY_CURRENT = 5/3.6;
STATES_INITIAL = [POSITION_X_CURRENT;POSITION_Y_CURRENT;PSI_HEADING_CURRENT;VELOCITY_CURRENT];

% We then define the process noise covariance matrix refered as the 'Q' matrix in estimation:
VARIANCE_POSITION_X = 5;
VARIANCE_POSITION_Y = 5;
VARIANCE_HEADING = (12*pi/180);
VARIANCE_VELOCITY = 10e-4;
PROCESS_MODEL_COVARIANCE_MATRIX = [VARIANCE_POSITION_X^2 0 0 0;0 VARIANCE_POSITION_Y^2 0 0;...
                                   0 0 VARIANCE_HEADING^2 0;0 0 0 VARIANCE_VELOCITY^2]; % Search for the influence of this term.

% We define the covariance matrix of the states at the initial step refered as the 'P0|0':
% We just take COVARIANCE_MATRIX_STATES_INITIAL =
% PROCESS_MODEL_COVARIANCE_MATRIX:
COVARIANCE_STATES_INITIAL = PROCESS_MODEL_COVARIANCE_MATRIX; % Search for the influence of this term.

% Define the control input matrix refered as the 'B' matrix in the usual
% description of linear state-space system reprezentation:
CONTROL_INPUTS_INITIAL = 0; % Process model considered as without input influence.

% Chose the step time, it shall be aligned with HW specification:
TIME_STEP_HW = 1;

% NOTA:
% When calling the EKF function such that ESTIMATES = EKF(), to get the
% proper evaluated functions used inside (JACOBIANS for example), the use
% the notation ESTIMAES = EKF(JACOBIAN(VARIABLE1,VARIABLE2)).
% If it's not working, the delete the function as an input of the EKF,
% call it directly inside the EKF function. But pay attention to get the
% good inputs inside the EKF function.
% /!\ Same thing for the FUNCTION_PROCESS_MODEL inside the EKF /!\.

% ----------------------------------------------------------------
% 2) Measurement & Innovation model:
% ----------------------------------------------------------------
% We define the following measurement model state vector:
% MEASUREMENT_VECTOR = [POSITION_X_CURRENT;...
%                       POSITION_Y_CURRENT;...
%                       PSI_HEADING_GNSS_CURRENT;...
%                       PSI_HEADING_MAG_CURRENT;...
%                       PSI_HEADING_IMU_CURRENT
%                       VELOCITY_CURRENT];

% Definition of the measurement noise covariance matrix refered as 'R' matrix:
% Retake the VARIANCE_POSITION_X, VARIANCE_POSITION_Y, VARIANCE_VELOCITY
% and VARIANCE_HEADING for VARIANCE_HEADING_GNSS used in the process model section:
VARIANCE_HEADING_GNSS = VARIANCE_HEADING;
VARIANCE_HEADING_MAG = 20*(pi/180);
VARIANCE_HEADING_IMU = 6*(pi/180);
MEASUREMENT_COVARIANCE_MATRIX = [VARIANCE_POSITION_X 0 0 0 0 0;
                                 0 VARIANCE_POSITION_Y 0 0 0 0;
                                 0 0 VARIANCE_HEADING_GNSS 0 0 0;
                                 0 0 0 VARIANCE_HEADING_MAG 0 0;
                                 0 0 0 0 VARIANCE_HEADING_IMU 0;
                                 0 0 0 0 0 VARIANCE_VELOCITY];

% Define the measurement model matrix: (ofter refered as 'H' or 'Gx'
% matrices):
MEASUREMENT_MODEL_MATRIX = [1 0 0 0;
                            0 1 0 0;
                            0 0 1 0;
                            0 0 1 0;
                            0 0 1 0;
                            0 0 0 1];

% Define some measurements along time to simulate the sensor fusion done by
% the EKF. Then the MEASUREMENT_VECTOR will has to be called in the loop
% calculation:
% Re-use the constant velocity declared in the process model section.
% Definition of step times:
POSITION_X = [0,1,2,3,4,5,6,7,8,9,10];
POSITION_Y = -POSITION_X;
% POSITION_X = [0,200,310,500,850,1000,1600,1750,2200,2290,2320];
% POSITION_Y = [0,-150,-200,-190,-90,-20,-10,-1300,-2000,-2050,-2090];
PSI_HEADING_GNSS = [-65,-64,-90,-120,-125,-130,-180,-131,-125,-127,-128]*(pi/180);
PSI_HEADING_MAG = [-65.8,-64.3,-93,-130,-125.9,-135,-182,-131.2,-127,-127.5,-128.1]*(pi/180);
PSI_HEADING_IMU = [-66,-64.7,-90.5,-125,-130.4,-182,-132,-125.9,-127.8,-129,-128.7]*(pi/180);
VELOCITY = [VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,...
            VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,VELOCITY_CURRENT,...
            VELOCITY_CURRENT];
% Define the initial values foreach of the measurements:
POSITION_X_INIT = POSITION_X(1);
POSITION_Y_INIT = POSITION_Y(1);
PSI_HEADING_GNSS_INIT = PSI_HEADING_GNSS(1);
PSI_HEADING_MAG_INIT = PSI_HEADING_MAG(1);
PSI_HEADING_IMU_INIT = PSI_HEADING_IMU(1);
VELOCITY_INIT = VELOCITY(1);



% ----------------------------------------------------------------
% Extended Kalman Filter loop calculations:
% ----------------------------------------------------------------
NUMBER_MEASUREMENTS = length(POSITION_Y);
% Store the results:
STATES_ESTIMATED_EKF = [];
COVARIANCE_ESTIMATED_EKF = [];

for time_indexes = 1:NUMBER_MEASUREMENTS

    % Initialization:
    % ---------------
    STATES_PREV = STATES_INITIAL; % The 'X' state vector.
    COVARIANCE_STATE_PREV = COVARIANCE_STATES_INITIAL; % The 'P0|0' equal to 'Q' in this case.
    CONTROL_INPUTS_PREV = CONTROL_INPUTS_INITIAL; % The usual 'B' matrix but unused here.
    PROCESS_MODEL_COVARIANCE_MATRIX_PREV = PROCESS_MODEL_COVARIANCE_MATRIX; % The 'Q' process model matrix.
    MEASUREMENT_COVARIANCE_MATRIX_PREV = MEASUREMENT_COVARIANCE_MATRIX; % The 'R' measurement matrix.
    % MEASUREMENT_CURRENT = [POSITION_X_INIT,POSITION_Y_INIT,PSI_HEADING_GNSS_INIT,PSI_HEADING_MAG_INIT,...
    %                        PSI_HEADING_IMU_INIT,VELOCITY_INIT];
    MEASUREMENT_CURRENT = [POSITION_X(time_indexes),POSITION_Y(time_indexes),PSI_HEADING_GNSS(time_indexes),PSI_HEADING_MAG(time_indexes),...
                           PSI_HEADING_IMU(time_indexes),VELOCITY(time_indexes)];
    
    % EKF loop calculation:
    % ---------------------
    [STATE_APOSTERIORI_CURRENT,COVARIANCE_APOSTERIORI_CURRENT] = EKF_HEADING(STATES_PREV,COVARIANCE_STATE_PREV,CONTROL_INPUTS_PREV,...
                                                                             JACOBIAN_STATE_PROCESS_MODEL(STATES_PREV(4),STATES_PREV(3),TIME_STEP_HW),...
                                                                             PROCESS_MODEL_COVARIANCE_MATRIX_PREV,MEASUREMENT_MODEL_MATRIX,...
                                                                             MEASUREMENT_COVARIANCE_MATRIX_PREV,...
                                                                             FUNCTION_PROCESS_MODEL(STATES_PREV(1),STATES_PREV(2),STATES_PREV(3),STATES_PREV(4),TIME_STEP_HW),...
                                                                             MEASUREMENT_CURRENT',...
                                                                             TIME_STEP_HW);


    % Inputs update:
    % --------------
    STATES_INITIAL = STATE_APOSTERIORI_CURRENT;
    COVARIANCE_STATES_INITIAL = COVARIANCE_APOSTERIORI_CURRENT;
    % Measurement simulated update:
    % POSITION_X_INIT = POSITION_X(time_indexes);
    % POSITION_Y_INIT = POSITION_Y(time_indexes);
    % PSI_HEADING_GNSS_INIT = PSI_HEADING_GNSS(time_indexes);
    % PSI_HEADING_MAG_INIT = PSI_HEADING_MAG(time_indexes);
    % PSI_HEADING_IMU_INIT = PSI_HEADING_IMU(time_indexes);
    % Store the results:
    STATES_ESTIMATED_EKF = [STATES_ESTIMATED_EKF,STATE_APOSTERIORI_CURRENT];
    COVARIANCE_ESTIMATED_EKF = [COVARIANCE_ESTIMATED_EKF,COVARIANCE_APOSTERIORI_CURRENT];

    % Store the interesting components of the covariance matrix (diagonal
    % terms):
    COVARIANCE_POSITION_X(time_indexes) = COVARIANCE_APOSTERIORI_CURRENT(1,1);
    COVARIANCE_POSITION_Y(time_indexes) = COVARIANCE_APOSTERIORI_CURRENT(2,2);
    COVARIANCE_PSI_HEADING(time_indexes) = COVARIANCE_APOSTERIORI_CURRENT(3,3);
    COVARIANCE_VELOCITY(time_indexes) = COVARIANCE_APOSTERIORI_CURRENT(4,4);

end


% ----------------------------------------------------------------
% Plot simulation results to conclude on the efficiency:
% ----------------------------------------------------------------

% Plot all the states on several respective figure:
% Plot the POSITION_X estimation:
figure(1);
plot([1:NUMBER_MEASUREMENTS],POSITION_X,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(1,:),'blue');
legend('Raw sensor position along X-axis','Estimated position along X-axis with heading measurements');
xlabel('Time simulation (number of measurements)');
ylabel('Position along X-axis (in meters)');
title('Position along X-axis in function of time (simulation number)');

% Plot the POSITION_Y estimation:
figure(2);
plot([1:NUMBER_MEASUREMENTS],POSITION_Y,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(2,:),'blue');
legend('Raw sensor position along Y-axis','Estimated position along Y-axis with heading measurements');
xlabel('Time simulation (number of measurements)');
ylabel('Position along Y-axis (in meters)');
title('Position along Y-axis in function of time (simulation number)');

% Plot the fused heading angle:
figure(3);
plot([1:NUMBER_MEASUREMENTS],PSI_HEADING_GNSS,'red');
hold on
plot([1:NUMBER_MEASUREMENTS],PSI_HEADING_MAG,'blue');
hold on 
plot([1:NUMBER_MEASUREMENTS],PSI_HEADING_IMU,'black');
hold on
plot([1:NUMBER_MEASUREMENTS],STATES_ESTIMATED_EKF(3,:),'green');
legend('Raw sensor heading GNSS','Raw sensor heading Gyrocompass','Raw sensor heading IMU','Fused heading');
xlabel('Time simulation (number of measurements)');
ylabel('Raw sensors measurements (in rad)');
title('Sensor headings and fused headings in function of time (simulation number)');

% No need to plot the velocity as it is considered being time invariant.

% ----------------------------------------------------------------
% Plot the covariance matrices evolution:
% ----------------------------------------------------------------

figure(4);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_POSITION_X,'red');
legend('Covariance of the position along X-axis');
xlabel('Time simulation (number of measurements)');
title('Covariance along X-axis in function of time (simulation number)');

figure(5);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_POSITION_Y,'red');
legend('Covariance of the position along Y-axis');
xlabel('Time simulation (number of measurements)');
title('Covariance along Y-axis in function of time (simulation number)');

figure(6);
plot([1:NUMBER_MEASUREMENTS],COVARIANCE_PSI_HEADING,'red');
legend('Covariance of the PSI angle heading');
xlabel('Time simulation (number of measurements)');
title('Covariance of the heading in function of time (simulation number)');






