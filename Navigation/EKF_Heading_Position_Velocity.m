function [Position_Ship_X_Current_Filtered,...
          Position_Ship_Y_Current_Filtered,...
          Heading_Ship_Current_Filtered,...
          Heading_Rate_Current_Filtered,...
          Bias_Heading_Rate_Current_Model,...
          Velocity_Ship_Current] = EKF_Heading_Position_Velocity(Position_Ship_X_Current_GPS,...
                                                                 Position_Ship_Y_Current_GPS,...
                                                                 Heading_GPS,...
                                                                 Heading_Magnetometer,...
                                                                 Velocity_Ship_GPS,...
                                                                 Time_Sampling,...
                                                                 Rudder_Angle_Elaborated,...
                                                                 Length_Ship_Current,...
                                                                 Density_Water_Current,...
                                                                 Drag_Coefficient_Current,...
                                                                 Draft_Depth_Current,...
                                                                 Longeron_Area_Current,...
                                                                 Mass_Of_The_Ship_Current,...
                                                                 Velocity_Ship_Initial,...
                                                                 Position_X_Ship_Initial,...
                                                                 Position_Y_Ship_Initial)

% This function serves as a filtering and data fusion to provide the ship
% with the right heading data.

persistent State_Model_Prediction;
persistent Covariance_Model_Prediction;
persistent Rudder_Angle_Elaborated_Previous;
if isempty(Rudder_Angle_Elaborated_Previous)
    Rudder_Angle_Elaborated_Previous = 0;
end

% ----------------------------------------------------------------
% 1) Definition of the process model:
% ----------------------------------------------------------------
% Initialization of the current models:
Velocity_Current = Velocity_Ship_Initial; 
Heading_Current_Model = 0;
Heading_Rate_Current_Model = 0;
Bias_Heading_Rate_Current_Model = 0;
Position_Ship_X_Current_Model = Position_X_Ship_Initial;
Position_Ship_Y_Current_Model = Position_Y_Ship_Initial;
States_Initial_Model = [Velocity_Current;...
                        Heading_Current_Model;...
                        Heading_Rate_Current_Model;...
                        Bias_Heading_Rate_Current_Model;...
                        Position_Ship_X_Current_Model;...
                        Position_Ship_Y_Current_Model];

% We then define the process noise covariance matrix refered as the 'Q'
% matrix in estimation. This has a link with how one is confident about the
% used model:
% Variance_Velocity = 3;
Variance_Velocity = 0.001;
Variance_Rate_Heading = 0.01*(pi/180);
Variance_Bias = 10*(pi/180); 
Process_Model_Covariance_Matrix = diag([Variance_Velocity^2,Variance_Rate_Heading^2,Variance_Bias^2]);

% Noise model matrix 'G':
Noise_State_Matrix = [Time_Sampling 0 0;
                      0 0 0;
                      0 Time_Sampling 0;
                      0 0 Time_Sampling;
                      0 0 0;
                      0 0 0];

% Compute the covariance matrix of the states at the initial step refered as the 'P0|0':
Covariance_States_Initial_Model = Noise_State_Matrix*Process_Model_Covariance_Matrix*Noise_State_Matrix';

% Compute the control input matrix refered as the 'B' matrix:
Control_Inputs_Initial = Rudder_Angle_Elaborated_Previous; 

% Time step for numerical integration:
Time_Step_Integration = Time_Sampling;

% ----------------------------------------------------------------
% 2) Measurement & Innovation model:
% ----------------------------------------------------------------
% Definition of the measurement noise covariance matrix refered as 'R' matrix:
Variance_Heading_Magnetic = 2*(pi/180);
Variance_Position_X_GPS = 2;
Variance_Position_Y_GPS = 2;
Variance_Velocity_GPS = 0.1;
Variance_Heading_GPS = 5*(pi/180);
Measurement_Covariance_Matrix = [Variance_Heading_Magnetic^2 0 0 0 0;
                                 0 Variance_Position_X_GPS^2 0 0 0;
                                 0 0 Variance_Position_Y_GPS^2 0 0;
                                 0 0 0 Variance_Velocity_GPS^2 0;
                                 0 0 0 0 Variance_Heading_GPS^2];

% Define the measurement model matrix: (often refered as 'H' or 'Gx'
% matrices):
Measurement_Model_Matrix = [0 1 0 0 0 0;
                            0 0 0 0 1 0;
                            0 0 0 0 0 1;
                            1 0 0 0 0 0;
                            0 1 0 0 0 0];

% ----------------------------------------------------------------
% Extended Kalman Filter loop calculations:
% ----------------------------------------------------------------
% Initialization:
if isempty(State_Model_Prediction)
    State_Model_Prediction = States_Initial_Model;
end
States_Previous = State_Model_Prediction; % The 'X' state vector.
% Calculation, in function of the velocity of the ship, of the Static_Gain
% and Time_Constant value:
[Static_Gain,...
 Time_Constant,...
 Time_Constant_1,...
 Time_Constant_2,...
 Time_Constant_3,...
 Time_Constant_Sway,...
 Gain_Static_Sway] = Static_Gain_Time_Constant_Varying_Velocity_Embedded(States_Previous(1),...
                                                                         Length_Ship_Current,...
                                                                         Density_Water_Current,...
                                                                         Drag_Coefficient_Current,...
                                                                         Draft_Depth_Current,...
                                                                         Longeron_Area_Current,...
                                                                         Mass_Of_The_Ship_Current);
if isempty(Covariance_Model_Prediction)
    Covariance_Model_Prediction = Covariance_States_Initial_Model;
end
Covariance_State_Previous = Covariance_Model_Prediction; % The 'P0|0' equal to 'Q' in this case.
Control_Inputs_Previous = Control_Inputs_Initial; % The usual 'B' matrix but unused here.
Process_Model_Covariance_Matrix_Previous = Noise_State_Matrix*Process_Model_Covariance_Matrix*Noise_State_Matrix'; % The 'Q' process model matrix.
Measurement_Covariance_Matrix_Previous = Measurement_Covariance_Matrix; % The 'R' measurement matrix.
Measurment_Current = [Heading_Magnetometer,...
                      Position_Ship_X_Current_GPS,...
                      Position_Ship_Y_Current_GPS,...
                      Velocity_Ship_GPS,...
                      Heading_GPS];

% EKF loop calculation:
[State_Aposteriori_Current,Covariance_Aposteriori_Current] = EKF_Heading(States_Previous,Covariance_State_Previous,Control_Inputs_Previous,...
                                                                         Jacobian_State_Process_Heading_Position_Velocity(States_Previous(1),States_Previous(2),Time_Step_Integration,Time_Constant),...
                                                                         Process_Model_Covariance_Matrix_Previous,Measurement_Model_Matrix,...
                                                                         Measurement_Covariance_Matrix_Previous,...
                                                                         Function_Process_Heading_Position_Velocity(States_Previous(1),States_Previous(2),States_Previous(3),States_Previous(4),States_Previous(5),...
                                                                         States_Previous(6),Time_Step_Integration,Time_Constant,Control_Inputs_Previous,Static_Gain),...
                                                                         Measurment_Current');

% Inputs update:
State_Model_Prediction = State_Aposteriori_Current; % This must be stored and reused at the next step time.
Covariance_Model_Prediction = Covariance_Aposteriori_Current;
Rudder_Angle_Elaborated_Previous = Rudder_Angle_Elaborated;

% Store the interesting components of the covariance matrix (diagonal
% terms):
Covariance_Velocity = Covariance_Aposteriori_Current(1,1);
Covariance_Psi_Heading = Covariance_Aposteriori_Current(2,2);
Covariance_Heading_Rate = Covariance_Aposteriori_Current(3,3);
Covariance_Bias = Covariance_Aposteriori_Current(4,4);
Covariance_Position_X = Covariance_Aposteriori_Current(5,5);
Covariance_Position_Y = Covariance_Aposteriori_Current(6,6);
Covariance_Error_Vector_Current = [Covariance_Velocity;...
                                   Covariance_Psi_Heading;...
                                   Covariance_Heading_Rate;...
                                   Covariance_Bias;
                                   Covariance_Position_X;
                                   Covariance_Position_Y];

% Filtered and fused data:
Velocity_Ship_Current = State_Aposteriori_Current(1);
Heading_Ship_Current_Filtered = State_Aposteriori_Current(2);
Heading_Rate_Current_Filtered = State_Aposteriori_Current(3);
Bias_Heading_Rate_Current_Model = State_Aposteriori_Current(4);
Position_Ship_X_Current_Filtered = State_Aposteriori_Current(5);
Position_Ship_Y_Current_Filtered = State_Aposteriori_Current(6);

end

