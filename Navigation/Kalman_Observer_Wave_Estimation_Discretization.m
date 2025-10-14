function [Heading_Rate_Wave_Estimated_Current,...
          Heading_Wave_Estimated_Current,...
          Heading_Low_Frequency_Estimated_Current,...
          Heading_Rate_Estimated_Current,...
          Bias_Estimated_Current,...
          Covariance_Aposteriori_Estimate] = Kalman_Observer_Wave_Estimation_Discretization(Heading_Total_Observed_Current,...
                                                                                            Rudder_Angle_Commanded_Current,...
                                                                                            Wave_Pulsation_Estimated,...
                                                                                            Damping_Wave_Estimated,...
                                                                                            Time_Current,...
                                                                                            Time_Constant,...
                                                                                            Static_Gain,...
                                                                                            Time_Sampling,...
                                                                                            Noise_For_Heading_wave,...
                                                                                            Torque_Wind_Estimated_Input,...
                                                                                            Inertia_Ship_Vertical,...
                                                                                            Wave_Pulsation_Identified,...
                                                                                            Heading_Rate_Estimated_Current_Navigation)



% Persistent states definition:
persistent State_Model_Prediction;
persistent Covariance_Prediction;
persistent Rudder_Angle_Commanded_Previous;
persistent Identified_Wave_Pulsation;

% Input parameters checking:
if isempty(Time_Sampling) || Time_Sampling <= 0
    error('Time_Sampling must be positive.');
end
if isempty(Time_Constant) || Time_Constant == 0
    error('Time_Constant_Ship must be nonzero.');
end
if any(isnan([Heading_Total_Observed_Current, Rudder_Angle_Commanded_Current, Wave_Pulsation_Estimated, Damping_Wave_Estimated]))
    error('One or more inputs are NaN.');
end

% Freeze wave pulsation after identified time: take the online estimated
% value:
% Wave_Pulsation_Identified = Wave_Pulsation_Estimated;
if Time_Current > 550
    if isempty(Identified_Wave_Pulsation)
        Identified_Wave_Pulsation = Wave_Pulsation_Identified;
    end
    Wave_Pulsation_Estimated = Identified_Wave_Pulsation;
end

% Current continuous time matrix definition:
State_Matrix_Continuous = [0 1 0 0 0;
                          -Wave_Pulsation_Estimated^2 -2*Damping_Wave_Estimated*Wave_Pulsation_Estimated 0 0 0;
                           0 0 0 1 0;
                           0 0 0 -1/Time_Constant 1;
                           0 0 0 0 0];

Input_Matrix_Continuous = [0; 0; 0; Static_Gain/Time_Constant; 0];

% Noise_Matrix_Continuous = [0 0 0;
%           Wave_Pulsation_Estimated^2 0 0;
%           0 0 0;
%           0 1 0;
%           0 0 1];
Noise_Matrix_Continuous = [0 0 0;
                          1 0 0;
                          0 0 0;
                          0 1 0;
                          0 0 1];

% Block discretization to avoid matrix inversion and potential
% singularities:
Time_Sampling_Current = Time_Sampling;
Size_State_Matrix = size(State_Matrix_Continuous,1);
Size_Input_Matrix = size(Input_Matrix_Continuous,2);
Size_Noise_Matrix = size(Noise_Matrix_Continuous,2);

% Exponential block matrix:
Exponential_Block_Matrix = [State_Matrix_Continuous, Input_Matrix_Continuous, Noise_Matrix_Continuous;
                            zeros(Size_Input_Matrix, Size_State_Matrix+Size_Input_Matrix+Size_Noise_Matrix);
                            zeros(Size_Noise_Matrix, Size_State_Matrix+Size_Input_Matrix+Size_Noise_Matrix)];
Current_Exponential_Matrix_Block = expm(Exponential_Block_Matrix * Time_Sampling_Current);
State_Matrix_Discretized_Prediction = Current_Exponential_Matrix_Block(1:Size_State_Matrix, 1:Size_State_Matrix);
Input_Matrix_Discretized_Prediction = Current_Exponential_Matrix_Block(1:Size_State_Matrix, Size_State_Matrix+1 : Size_State_Matrix+Size_Input_Matrix);
Noise_Matrix_Discretized_Prediction = Current_Exponential_Matrix_Block(1:Size_State_Matrix, Size_State_Matrix+Size_Input_Matrix+1 : Size_State_Matrix+Size_Input_Matrix+Size_Noise_Matrix);

% First-call states initialization:
if isempty(State_Model_Prediction)
    State_Model_Prediction = [0;                                
                              Heading_Total_Observed_Current;   
                              Heading_Total_Observed_Current;   
                              0;                                
                              0];                               
end
if isempty(Rudder_Angle_Commanded_Previous)
    Rudder_Angle_Commanded_Previous = 0;
end
if isempty(Covariance_Prediction)
    Covariance_Prediction = diag([ (deg2rad(5))^2, ...   
                                   (deg2rad(10))^2, ...  
                                   (deg2rad(10))^2, ...  
                                   (deg2rad(1))^2, ...   
                                   (deg2rad(0.5))^2]);
end

% Covariance matrices definition:
Sigma_Heading_Wave = deg2rad(20.0);  
Sigma_Rate_Heading = deg2rad(0.5);   
Sigma_Bias = deg2rad(0.01);          
Covariance_Matrix_Model = diag([ Sigma_Heading_Wave^2, Sigma_Rate_Heading^2, Sigma_Bias^2 ]);
Covariance_Matrix_Model_Shape = Noise_Matrix_Discretized_Prediction * Covariance_Matrix_Model * Noise_Matrix_Discretized_Prediction';

% Measurements definition:
Measurement_Matrix = [0 1 1 0 0];
Measurement_Vector = Heading_Total_Observed_Current;

% Measurement covariance matrix:
Sigma_Heading_Estimated = deg2rad(2.0); 
Covariance_Matrix_Measurement = Sigma_Heading_Estimated^2;

% Prediction step of the Kalman Filter:
Input_Control = Rudder_Angle_Commanded_Current;
State_Vector_Predicted_Apriori = State_Matrix_Discretized_Prediction * State_Model_Prediction + Input_Matrix_Discretized_Prediction * Input_Control;
Covariance_Matrix_Predicted_Apriori = State_Matrix_Discretized_Prediction * Covariance_Prediction * State_Matrix_Discretized_Prediction' + Covariance_Matrix_Model_Shape;

% Basic sanity - ensure Covariance_Matrix_Predicted_Apriori is symmetric
% SPD:
Covariance_Matrix_Predicted_Apriori = (Covariance_Matrix_Predicted_Apriori + Covariance_Matrix_Predicted_Apriori')/2;
[Covariance_Eigen_Value, Covariance_Matrix_Diagonal] = eig(Covariance_Matrix_Predicted_Apriori);
if any(diag(Covariance_Matrix_Diagonal) <= 0)
    % Regularization step:
    Covariance_Matrix_Predicted_Apriori = Covariance_Matrix_Predicted_Apriori + eye(Size_State_Matrix) * 1e-9;
end

% Measurement update step of the Kalman Filter:
Pre_Innovation = Measurement_Matrix * Covariance_Matrix_Predicted_Apriori * Measurement_Matrix' + Covariance_Matrix_Measurement;
Kalman_Gain_Filter = (Covariance_Matrix_Predicted_Apriori * Measurement_Matrix') / Pre_Innovation;
Innovation_Filter = Measurement_Vector - Measurement_Matrix * State_Vector_Predicted_Apriori;
State_Vector_Aposteriori_Estimated = State_Vector_Predicted_Apriori + Kalman_Gain_Filter * Innovation_Filter;
Covariance_Matrix_Aposteriori_Estimated = (eye(Size_State_Matrix) - Kalman_Gain_Filter * Measurement_Matrix) * Covariance_Matrix_Predicted_Apriori;

% Ensure Symmetry step:
Covariance_Matrix_Aposteriori_Estimated = (Covariance_Matrix_Aposteriori_Estimated + Covariance_Matrix_Aposteriori_Estimated')/2;

% Outputs assignment:
Heading_Rate_Wave_Estimated_Current     = State_Vector_Aposteriori_Estimated(1);
Heading_Wave_Estimated_Current          = State_Vector_Aposteriori_Estimated(2);
Heading_Low_Frequency_Estimated_Current = State_Vector_Aposteriori_Estimated(3);
Heading_Rate_Estimated_Current          = State_Vector_Aposteriori_Estimated(4);
Bias_Estimated_Current                  = State_Vector_Aposteriori_Estimated(5);

Covariance_Aposteriori_Estimate = Covariance_Matrix_Aposteriori_Estimated;

% Persistents update:
State_Model_Prediction = State_Vector_Aposteriori_Estimated;
Covariance_Prediction  = Covariance_Matrix_Aposteriori_Estimated;
Rudder_Angle_Commanded_Previous = Rudder_Angle_Commanded_Current;

end

