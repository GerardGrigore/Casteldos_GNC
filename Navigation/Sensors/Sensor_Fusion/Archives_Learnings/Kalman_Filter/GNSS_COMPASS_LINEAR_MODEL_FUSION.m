clear all;
clc;

% This script serves as an attempt to validate the Kalman Filter designed.
% The idea is to realize a sensor fusion based on 2 available measurements
% of a heading:

%           1/- Psi_Heading_GNSS
%           2/- Psi_Heading_Compass

% The process model:
% ------------------
% We will base our analysis on the following process model:
% States_Current = A*States_Previous + B*Inputs_Control_Previous + Process_Noise

% As one studies the direction heading, our state vector will only be
% composed of Psi_Heading. Hence, the process model can be summurized such
% as:
% Psi_Heading_Current = 1*Psi_Heading_Previous + 1*Rudder_Angle_Previous + Heading_Noise

% Having that in mind, it is a 1D simplified situation. Hence, in this
% case, the state and control matrix are reals (integers).

% Points of attention:
% 1) Generate a logical Rudder_Angle sequence for simulation.
% 2) Model the Heading_Noise in a realistic way.

% The measurement model:
% ----------------------
% We will base our analysis on the following measurement model:
% Measures_Current = H*States_Current + Measurement_Noise.

% As the sensors allow us to obtain the following measurement vector:
% Measures_Current = [Psi_Heading_Current_GNSS;Psi_Heading_Current_Compass];
% and that the state vector is as: States_Current = [Psi_Heading], then one
% can concludes that:

% H = Measurement_Matrix_Model_H = [1;1], as Psi_Heading_Current_GNSS =
% Psi_Heading and Psi_Heading_Current_Compass = Psi_Heading.

% And, Measurement_Noise = [Psi_Heading_Current_GNSS_Noise;Psi_Heading_Current_Compass_Noise].

% Points of attention:
% 1) Generate a sequence of measurements headings: use simulink blocks.
% 2) Model the Measurement_Noise in a realistic way.

% In order to simulate a generation of measurements coming from the GPS and
% the Gyrocompass, one must have input points Positions, Velocity,
% Acceleration and Magnetic_Field.

% -------------------------------------------------------------------------
% I) Data generation using Simulink blocks GPS & Ecompass:
% -------------------------------------------------------------------------

% To be done.

% Position, Velocity & Acceleration - Trajectory points:
% ------------------------------------------------------
% Positions:
% /!\ OPENWORK - Must define the positions in ENU referential frame.
Position_Ship_0 = [0,0,0];
POSITION_SHIP_1 = [200,150,0];
POSITION_SHIP_2 = [310,200,0];
POSITION_SHIP_3 = [500,190,0];
POSITION_SHIP_4 = [850,90,0];
POSITION_SHIP_5 = [1000,-20,0];
POSITION_SHIP_6 = [1600,10,0];
POSITION_SHIP_7 = [1750,-1300,0];
POSITION_SHIP_8 = [2200,-2000,0];
POSITION_SHIP_9 = [2290,-2050,0];
POSITION_SHIP_10 = [2320,-2090,0];
POSITION_SHIP_VECTOR = [Position_Ship_0
                        POSITION_SHIP_1;
                        POSITION_SHIP_2;
                        POSITION_SHIP_3;
                        POSITION_SHIP_4;
                        POSITION_SHIP_5;
                        POSITION_SHIP_6;
                        POSITION_SHIP_7;
                        POSITION_SHIP_8;
                        POSITION_SHIP_9
                        POSITION_SHIP_10];

NUMBER_POINTS = length(POSITION_SHIP_VECTOR);

% Velocity:
% One shall consider a basic constant velocity of the ship. Approximately 5
% km/h (5.6567) at first approximation:
% VELOCITY_MEAN = 5/3.6;
VELOCITY_X = 4/3.6;
VELOCITY_Y = 4/3.6;
VELOCITY_Z = 0;
% Vector creation:
VELOCITY_VECTOR(1:NUMBER_POINTS,1) = VELOCITY_X;
VELOCITY_VECTOR(1:NUMBER_POINTS,2) = VELOCITY_Y;
VELOCITY_VECTOR(1:NUMBER_POINTS,3) = VELOCITY_Z;

% Acceleration:
% We will consider, as the velocity is constant, a null acceleration for
% our first order approximation:
ACCELERATION_X = 0;
ACCELERATION_Y = 0;
ACCELERATION_Z = 9.81;
% Vector creation:
ACCELERATION_VECTOR(1:NUMBER_POINTS,1) = ACCELERATION_X;
ACCELERATION_VECTOR(1:NUMBER_POINTS,2) = ACCELERATION_Y;
ACCELERATION_VECTOR(1:NUMBER_POINTS,3) = ACCELERATION_Z;

% Magnetic field generation:
% --------------------------
% Number of simulated points:
NUMBER_MAGNETIC_POINTS = 11;

% Mean intensity of the Earth's magnetic field (Microteslas):
MEAN_MAGNETIC_INTENSITY = 50;

% Generation of magnetic points with a random variation around the mean
% intensity.
MAGNECTIC_FIELD_X = MEAN_MAGNETIC_INTENSITY + randn(NUMBER_MAGNETIC_POINTS, 1) * 20;  % Variation of ±5 µT.
MAGNECTIC_FIELD_Y = MEAN_MAGNETIC_INTENSITY + randn(NUMBER_MAGNETIC_POINTS, 1) * 5;
MAGNECTIC_FIELD_Z = MEAN_MAGNETIC_INTENSITY + randn(NUMBER_MAGNETIC_POINTS, 1) * 78;

% Vector concatenation notation:
Magnetic_Field = [MAGNECTIC_FIELD_X,MAGNECTIC_FIELD_Y,MAGNECTIC_FIELD_Z];

% -------------------------------------------------------------------------
% II) Data generation using simple guess generation for test:
% -------------------------------------------------------------------------
% Creation of a naive GNSS heading vector measurements like:
ESTIM_HEAD_0 = -65*pi/180;
ESTIM_HEAD_1 = -64*pi/180;
ESTIM_HEAD_2 = -90*pi/180;
ESTIM_HEAD_3 = -120*pi/180;
ESTIM_HEAD_4 = -125*pi/180;
ESTIM_HEAD_5 = -130*pi/180;
ESTIM_HEAD_6 = -180*pi/180;
ESTIM_HEAD_7 = -131*pi/180;
ESTIM_HEAD_8 = -125*pi/180;
ESTIM_HEAD_9 = -127*pi/180;
ESTIM_HEAD_10 = -128*pi/180;
HEADING_ESTIMATED_VECTOR = [ESTIM_HEAD_0;ESTIM_HEAD_1;ESTIM_HEAD_2;ESTIM_HEAD_3;ESTIM_HEAD_4;ESTIM_HEAD_5;...
                            ESTIM_HEAD_6;ESTIM_HEAD_7;ESTIM_HEAD_8;ESTIM_HEAD_9;ESTIM_HEAD_10];
PSI_HEADING_GNSS_GUESS = HEADING_ESTIMATED_VECTOR;
NUMBER_MEASUREMENTS = length(HEADING_ESTIMATED_VECTOR);

% Creation of a naive GYROCOMPASS heading measurement like:
ESTIM_HEAD_0_GY = -67.6*pi/180;
ESTIM_HEAD_1_GY = -68.2*pi/180;
ESTIM_HEAD_2_GY = -91*pi/180;
ESTIM_HEAD_3_GY = -125*pi/180;
ESTIM_HEAD_4_GY = -120*pi/180;
ESTIM_HEAD_5_GY = -132.3*pi/180;
ESTIM_HEAD_6_GY = -188.7*pi/180;
ESTIM_HEAD_7_GY = -135.3*pi/180;
ESTIM_HEAD_8_GY = -126.9*pi/180;
ESTIM_HEAD_9_GY = -127.2*pi/180;
ESTIM_HEAD_10_GY = -130*pi/180;
HEADING_ESTIMATED_VECTOR_GY = [ESTIM_HEAD_0_GY;ESTIM_HEAD_1_GY;ESTIM_HEAD_2_GY;ESTIM_HEAD_3_GY;ESTIM_HEAD_4_GY;ESTIM_HEAD_5_GY;...
                               ESTIM_HEAD_6_GY;ESTIM_HEAD_7_GY;ESTIM_HEAD_8_GY;ESTIM_HEAD_9_GY;ESTIM_HEAD_10_GY];
PSI_HEADING_GYROCOMPASS_GUESS = HEADING_ESTIMATED_VECTOR_GY;

% Consider a simple error zero mean white noise:
%   1/- Either with a Matlab function.
%   2/- Either in creating 2 naives vectors with dispersion of x° for
%   example.
% Determine the process noise and measurement matrix:
% Process_Noise = (1*pi/180)^2; 
% Measurement_Noise = [(4*pi/180)^2,(1.5*pi/180)^2];
Process_Noise = 1*pi/180; 
Measurement_Noise = [4*pi/180,1.5*pi/180];
PROCESS_NOISE_MATRIX = Process_Noise;
MEASUREMENT_NOISE_MATRIX = diag(Measurement_Noise);

% Inputs simulation sequence:
% Furthermore, possible to estimate, thanks to Guidance algorithm, the
% desired rudder direction for every headings:
HEADING_AIMED_RADIANS = [0.9239;0.9202;0.6775;1.9677;2.1390;1.9622;2.8397;2.4664;2.1408;2.2191;1.8417];
HEADING_AIMED_DEGREES = HEADING_AIMED_RADIANS*180/pi;

% Definition of the measurement model:
Measurement_Matrix_Model_H = [1;1];

% State-space system definition - Process Model:
STATE_MATRIX_MODEL_A = 1;
% INPUTS_CONTROL_B = 1;
INPUTS_CONTROL_B = -1; % Due to negative definition of heading.

% Kalman Filter inputs definition for comprehension:
PSI_PREV_MODEL = 0;
STATES_INITIAL_PREV = PSI_PREV_MODEL;
COVARIANCE_STATES_PREV = 0;
CONTROL_INPUTS_PREV = HEADING_AIMED_RADIANS(1);
PROCESS_NOISE_PREV = Process_Noise;
MEASUREMENTS_NOISE_PREV = MEASUREMENT_NOISE_MATRIX;
MEASUREMENTS_CURRENT_PREV = [PSI_HEADING_GNSS_GUESS(1);PSI_HEADING_GYROCOMPASS_GUESS(1)];

% Outputs of the algorithm:
STATES_ESTIMATES = 0;
COVARIANCE_ESTIMATES = 0;

% Loop with Kalman Filter calculation:
for time_index = 1:NUMBER_MEASUREMENTS

    % Initialization:
    STATES_INITIAL = STATES_INITIAL_PREV;
    COVARIANCE_STATES_INITIAL = COVARIANCE_STATES_PREV;
    CONTROL_INPUTS_INITIAL = CONTROL_INPUTS_PREV;
    Process_Noise = PROCESS_NOISE_PREV;
    MEASUREMENTS_NOISE = MEASUREMENTS_NOISE_PREV;
    MEASUREMENTS_CURRENT = MEASUREMENTS_CURRENT_PREV;
    
    % Kalman filter calculation:
    [STATES_APOSTERIORI_ESTIMATE,COVARIANCE_APOSTERIORI_ESTIMATE] = KALMAN_FILTER(STATES_INITIAL,COVARIANCE_STATES_INITIAL,...
                                                                                  CONTROL_INPUTS_INITIAL,STATE_MATRIX_MODEL_A,...
                                                                                  INPUTS_CONTROL_B,Process_Noise,...
                                                                                  Measurement_Matrix_Model_H,MEASUREMENTS_NOISE,...
                                                                                  MEASUREMENTS_CURRENT);

    % Parameters update:
    STATES_INITIAL_PREV = STATES_APOSTERIORI_ESTIMATE;
    COVARIANCE_STATES_PREV = COVARIANCE_APOSTERIORI_ESTIMATE;
    CONTROL_INPUTS_PREV = HEADING_AIMED_RADIANS(time_index);

    % Outputs:
    STATES_ESTIMATES(time_index) = STATES_APOSTERIORI_ESTIMATE;
    COVARIANCE_ESTIMATES(time_index) = COVARIANCE_APOSTERIORI_ESTIMATE;

end

% -------------------------------------------------------------------------
% III) Plots of measurements, estimation and covariance:
% -------------------------------------------------------------------------
% Plots for comparison of noisy measurements and the estimated ones:
figure(1)
plot([0:NUMBER_MEASUREMENTS - 1],PSI_HEADING_GYROCOMPASS_GUESS,'r');
hold on
plot([0:NUMBER_MEASUREMENTS - 1],PSI_HEADING_GNSS_GUESS,'blue');
hold on 
plot([0:NUMBER_MEASUREMENTS - 1],STATES_ESTIMATES,'black');
xlabel('Measurement points - times scale');
ylabel('Measurements and estimate');
legend('GYROCOMPASS heading measurements','GNSS heading measurements',...
       'Kalman Filter estimated measurements');
title('Measurements comparison');
% Plots of the covariance and uncertainty:
figure(2)
plot([0:NUMBER_MEASUREMENTS - 1],COVARIANCE_ESTIMATES,'r');

% -------------------------------------------------------------------------
% IV) Cross-checking the results using internal Kalman Filter function:
% -------------------------------------------------------------------------
% Steady State Kalman Filter crosscheck:
% --------------------------------------
% Cross check with iternal Matlab function Falman Filter:
% Design the filter:
TIME_SAMPLE = -1; % Set as discrete.
PROCESS_NOISE_CONTROL_MATRIX = INPUTS_CONTROL_B;
PROCESS_MODEL_SYSTEM = ss(STATE_MATRIX_MODEL_A,[INPUTS_CONTROL_B PROCESS_NOISE_CONTROL_MATRIX],...
                          Measurement_Matrix_Model_H,0,TIME_SAMPLE,...
                          'InputName',{'INPUT_CONTROL_HEADING' 'Process_Noise'},...
                          'OutputName',{'HEADINGS_MEASURED_1' 'HEADINGS_MEASURED_2'});
% Kalman filter calculation:
[KALMAN_FILTER,PARAMERER_L,~,INNOVATION_GAIN,PARAMETER_Z] = kalman(PROCESS_MODEL_SYSTEM,PROCESS_NOISE_MATRIX,...
                                                                   MEASUREMENT_NOISE_MATRIX);
% Keep only the two outputs:
KALMAN_FILTER = KALMAN_FILTER(1:2,:);

PROCESS_MODEL_SYSTEM.InputName = {'INPUT_CONTROL_HEADING','Process_Noise'};
PROCESS_MODEL_SYSTEM.OutputName = {'HEADINGS_MEASURED_1','HEADINGS_MEASURED_2'};

% Sum blocks correctly created:
SENSOR_NOISE_INPUT_1 = sumblk('HEADINGS_1 = HEADINGS_MEASURED_1 + SENSOR_NOISE_1');
SENSOR_NOISE_INPUT_2 = sumblk('HEADINGS_2 = HEADINGS_MEASURED_2 + SENSOR_NOISE_2');

KALMAN_FILTER.InputName = {'INPUT_CONTROL_HEADING','HEADINGS_MEASURED_1','HEADINGS_MEASURED_2'};
KALMAN_FILTER.OutputName = {'HEADING_ESTIMATED_1','HEADING_ESTIMATED_2'};

% Then create the Simulink model:
SIMULINK_MODEL = connect(PROCESS_MODEL_SYSTEM,SENSOR_NOISE_INPUT_1,SENSOR_NOISE_INPUT_2,KALMAN_FILTER,...
                    {'INPUT_CONTROL_HEADING','Process_Noise','SENSOR_NOISE_1','SENSOR_NOISE_2'},...
                    {'HEADINGS_1','HEADINGS_2','HEADING_ESTIMATED_1','HEADING_ESTIMATED_2'});

% Then generate an input vector:
% NOTA - The time vector is the number of measurements from
% 0:NUMBER_MEASUREMENTS.
INPUT_CONTROL_HEADING = HEADING_AIMED_RADIANS;

% Generate process noise and sensor noise vectors using same values of Q
% (PROCESS_NOISE_MATRIX) and R (MEASUREMENT_NOISE_MATRIX):
rng(10,"twister");
Process_Noise = sqrt(PROCESS_NOISE_MATRIX)*randn(NUMBER_POINTS,1);
SENSOR_NOISE = randn(NUMBER_POINTS,2)*sqrt(MEASUREMENT_NOISE_MATRIX);

% Then simulate the response:
OUTPUT_SIMULATION = lsim(SIMULINK_MODEL,[INPUT_CONTROL_HEADING,Process_Noise,SENSOR_NOISE]);

% Compute the measured responses:
HEADINGS_MEASURED_1 = OUTPUT_SIMULATION(:,1);
HEADINGS_MEASURED_2 = OUTPUT_SIMULATION(:,2);
HEADING_ESTIMATED_1 = OUTPUT_SIMULATION(:,3);
HEADING_ESTIMATED_2 = OUTPUT_SIMULATION(:,4);
HEADINGS_1 = HEADINGS_MEASURED_1 + SENSOR_NOISE(:,1); % Measured response for the first output.
HEADINGS_2 = HEADINGS_MEASURED_2 + SENSOR_NOISE(:,2);

% Compare the true response with the filtered response:
figure(3)
subplot(211);
plot([0:NUMBER_MEASUREMENTS - 1], HEADINGS_MEASURED_1, 'b', [0:NUMBER_MEASUREMENTS - 1], HEADING_ESTIMATED_1, 'r--',...
     [0:NUMBER_MEASUREMENTS - 1], HEADINGS_1, 'g--');
xlabel('Number of Samples');
ylabel('Output');
title('Kalman Filter Response - Output 1');
legend('True', 'Filtered', 'Measured');

subplot(212);
plot([0:NUMBER_MEASUREMENTS - 1], HEADINGS_MEASURED_2, 'b', [0:NUMBER_MEASUREMENTS - 1], HEADING_ESTIMATED_2, 'r--',...
     [0:NUMBER_MEASUREMENTS - 1], HEADINGS_2, 'g--');
xlabel('Number of Samples');
ylabel('Output');
title('Kalman Filter Response - Output 2');
legend('True', 'Filtered', 'Measured');






