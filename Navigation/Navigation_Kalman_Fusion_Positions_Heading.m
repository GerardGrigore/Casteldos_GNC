function [Position_Ship_X_Current_Filtered,...
          Position_Ship_Y_Current_Filtered,...
          Heading_Ship_Current_Filtered,...
          Covariance_Error_Vector_Current]  = Navigation_Kalman_Fusion_Positions_Heading(Position_Ship_X_Current_GPS,...
                                                                                         Position_Ship_Y_Current_GPS,...
                                                                                         Heading_GPS,...
                                                                                         Heading_Magnetometer,...
                                                                                         Velocity_Ship_Mean,...
                                                                                         Time_Sampling)

% This function serves as a filtering and data fusion to provide the ship
% with the right heading data.

persistent State_Model_Prediction;
persistent Covariance_Model_Prediction;

% ----------------------------------------------------------------
% 1) Definition of the process model:
% ----------------------------------------------------------------
% We define the following process model state vector:
% States_Initial_Model = [Position_X_Current;...
%                   Position_Y_Current;...
%                   Psi_Heading_Current;...
%                   Velocity_Current];

% Initialization of the current models:
Position_Ship_X_Current_Model = Position_Ship_X_Current_GPS;
Position_Ship_Y_Current_Model = Position_Ship_Y_Current_GPS;
Heading_Current_Model = Heading_Magnetometer;
Velocity_Ship_Model = Velocity_Ship_Mean;
States_Initial_Model = [Position_Ship_X_Current_Model;...
                        Position_Ship_Y_Current_Model;...
                        Heading_Current_Model;...
                        Velocity_Ship_Model];

% We then define the process noise covariance matrix refered as the 'Q'
% matrix in estimation. This has a link with how one is confident about the
% used model:
Variance_Position_X = 5;
Variance_Position_Y = 5;
Variance_Heading = (2*pi/180);
Variance_Velocity = 0.5;
Process_Model_Covariance_Matrix = [Variance_Position_X^2 0 0 0;
                                   0 Variance_Position_Y^2 0 0;
                                   0 0 Variance_Heading^2 0;
                                   0 0 0 Variance_Velocity^2];

% We define the covariance matrix of the states at the initial step refered as the 'P0|0':
% We just take Covariance_Matrix_States_Initial_Model =
% Process_Model_Covariance_Matrix:
Covariance_States_Initial_Model = Process_Model_Covariance_Matrix;

% Define the control input matrix refered as the 'B' matrix in the usual
% description of linear state-space system reprezentation:
Control_Inputs_Initial = 0; % Process model considered as without input influence.

% Chose the step time, it shall be aligned with HW specification:
Time_Step_Integration = Time_Sampling;

% ----------------------------------------------------------------
% 2) Measurement & Innovation model:
% ----------------------------------------------------------------
% We define the following measurement model state vector:
% Measurement_Vector = [Position_X_Current;...
%                       Position_Y_Current;...
%                       Psi_Heading_GNSS_Current;...
%                       Psi_Heading_Mag_Current;...
%                       Psi_Heading_IMU_Current
%                       Velocity_Current];

% Definition of the measurement noise covariance matrix refered as 'R' matrix:
% Retake the Variance_Position_X, Variance_Position_Y, Variance_Velocity
% and Variance_Heading for Variance_Heading_GNSS used in the process model section:
% To be aligned with the specification from the sensors and the models used
% for the Navigation:
Variance_Position_X_GPS = 2;
Variance_Position_Y_GPS = 2;
Variance_Heading_GNSS = 0.1*(pi/180);
Variance_Heading_Magnetic = 2*(pi/180);
Variance_Velocity_Sensors = 0.1;
Measurement_Covariance_Matrix = [Variance_Position_X_GPS^2 0 0 0 0;
                                 0 Variance_Position_Y_GPS^2 0 0 0;
                                 0 0 Variance_Heading_GNSS^2 0 0;
                                 0 0 0 Variance_Heading_Magnetic^2 0;
                                 0 0 0 0 Variance_Velocity_Sensors^2];

% Define the measurement model matrix: (ofter refered as 'H' or 'Gx'
% matrices):
Measurement_Model_Matrix = [1 0 0 0;
                            0 1 0 0;
                            0 0 1 0;
                            0 0 1 0;
                            0 0 0 1];

% ----------------------------------------------------------------
% Extended Kalman Filter loop calculations:
% ----------------------------------------------------------------
% Store the results:
% States_Estimated_EKF = [];
% Covariance_Estimated_EKF = [];

% Initialization:
if isempty(State_Model_Prediction)
    State_Model_Prediction = States_Initial_Model;
end
States_Previous = State_Model_Prediction; % The 'X' state vector.
if isempty(Covariance_Model_Prediction)
    Covariance_Model_Prediction = Covariance_States_Initial_Model;
end
Covariance_State_Previous = Covariance_Model_Prediction; % The 'P0|0' equal to 'Q' in this case.
Control_Inputs_Previous = Control_Inputs_Initial; % The usual 'B' matrix but unused here.
Process_Model_Covariance_Matrix_Previous = Process_Model_Covariance_Matrix; % The 'Q' process model matrix.
Measurement_Covariance_Matrix_Previous = Measurement_Covariance_Matrix; % The 'R' measurement matrix.
Measurment_Current = [Position_Ship_X_Current_GPS,Position_Ship_Y_Current_GPS,Heading_GPS,Heading_Magnetometer,Velocity_Ship_Mean];

% EKF loop calculation:
[State_Aposteriori_Current,Covariance_Aposteriori_Current] = EKF_Heading(States_Previous,Covariance_State_Previous,Control_Inputs_Previous,...
                                                                         Jacobian_State_Process_Model(States_Previous(4),States_Previous(3),Time_Step_Integration),...
                                                                         Process_Model_Covariance_Matrix_Previous,Measurement_Model_Matrix,...
                                                                         Measurement_Covariance_Matrix_Previous,...
                                                                         Function_Process_Model(States_Previous(1),States_Previous(2),States_Previous(3),States_Previous(4),Time_Step_Integration),...
                                                                         Measurment_Current');

% Inputs update:
State_Model_Prediction = State_Aposteriori_Current; % This must be stored and reused at the next step time.
Covariance_Model_Prediction = Covariance_Aposteriori_Current;

% Store the interesting components of the covariance matrix (diagonal
% terms):
Covariance_Position_X = Covariance_Aposteriori_Current(1,1);
Covariance_Position_Y = Covariance_Aposteriori_Current(2,2);
Covariance_Psi_Heading = Covariance_Aposteriori_Current(3,3);
Covariance_Velocity = Covariance_Aposteriori_Current(4,4);
Covariance_Error_Vector_Current = [Covariance_Position_X;...
                                   Covariance_Position_Y;...
                                   Covariance_Psi_Heading;...
                                   Covariance_Velocity];

% Filtered and fused data:
Position_Ship_X_Current_Filtered = State_Aposteriori_Current(1);
Position_Ship_Y_Current_Filtered = State_Aposteriori_Current(2);
Heading_Ship_Current_Filtered = State_Aposteriori_Current(3);
Velocity_Current_Filtered = State_Aposteriori_Current(4); % The velocity is supposed constant.

end

