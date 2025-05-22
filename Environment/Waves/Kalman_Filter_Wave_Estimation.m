function [Heading_Rate_Wave_Estimated_Current,...
          Heading_Wave_Estimated_Current,...
          Heading_Low_Frequency_Estimated_Current,...
          Heading_Rate_Estimated_Current,...
          Bias_Estimated_Current,...
          Covariance_Aposteriori_Estimate]= Kalman_Filter_Wave_Estimation(Heading_Total_Observed_Current,...
                                                                          Rudder_Angle_Commanded_Current,...
                                                                          Wave_Pulsation_Estimated,...
                                                                          Damping_Wave_Estimated,...
                                                                          Time_Current,...
                                                                          Time_Constant_Ship,...
                                                                          Gain_Ship,...
                                                                          Time_Sampling,...
                                                                          Noise_For_Heading_wave)

% Persistents definition:
persistent State_Model_Prediction;
persistent Covariance_Prediction;
persistent Rudder_Angle_Commanded_Prev;

% Integration step to be aligned with the sampling
% frequency:
Time_Integration = Time_Sampling;

% State model definition:
% -----------------------                          
% State matrix 'A':
State_Matrix = [1 Time_Integration 0 0 0;
                -Time_Integration*Wave_Pulsation_Estimated^2 1-2*Damping_Wave_Estimated*Wave_Pulsation_Estimated*Time_Integration 0 0 0;
                0 0 1 Time_Integration 0;
                0 0 0 1-(Time_Integration/Time_Constant_Ship) Time_Integration;
                0 0 0 0 1];
            
% Input matrix 'B':
Input_Matrix = [0;0;0;(Time_Integration*Gain_Ship)/Time_Constant_Ship;0];

% Noise state matrix 'G':
Noise_State_Matrix = [0 0 0;
                      Time_Integration*Wave_Pulsation_Estimated^2 0 0;
                      0 0 0;
                      0 Time_Integration 0;
                      0 0 Time_Integration];


% State model covariance matrix 'Q': must be better estimated.
Sigma_Heading_Wave = (270*(pi/180))/3;
%Sigma_Heading_Wave = sqrt(250);
Sigma_Rate_Heading = (5*(pi/180))/3;
Sigma_Bias = (15*(pi/180))/3;
State_Model_Covariance_Matrix = diag([Sigma_Heading_Wave^2,Sigma_Rate_Heading^2,Sigma_Bias^2]);


% Measurement model definition:
% -----------------------------
Measurement_Matrix = [0 1 1 0 0];
% Measurement model covariance matrix 'R':
Sigma_Heading_Estimated = ((25*pi/180)/3); % Must be the same amount of noise applied in the heading in input of the filter (ie: sensor noise).
Measurement_Model_Covariance_Matrix = Sigma_Heading_Estimated^2;
Measurement_Vector_Current = Heading_Total_Observed_Current;

% Kalman filter equations:
% ------------------------
% Prediction:
% -----------
% State parameters propagation through the model for estimation:
if isempty(State_Model_Prediction)
    Heading_Rate_Wave_Model_Previous = 0;
    Heading_Wave_Model_Previous = 0;
    Heading_Low_Frequency_Model_Previous = 0;
    Heading_Rate_Model_Previous = 0;
    Bias_Model_Previous = 0;
    State_Model_Prediction = [Heading_Rate_Wave_Model_Previous;
                              Heading_Wave_Model_Previous;...
                              Heading_Low_Frequency_Model_Previous;...
                              Heading_Rate_Model_Previous;
                              Bias_Model_Previous];
end
if isempty(Rudder_Angle_Commanded_Prev)
    Rudder_Angle_Commanded_Prev = 0;
end
State_Vector_Model_Apriori_Estimated_Current = State_Matrix*State_Model_Prediction + Input_Matrix*Rudder_Angle_Commanded_Current;
% Predicted apriori estimated covariance:
if isempty(Covariance_Prediction)
    Covariance_Prediction = Noise_State_Matrix*State_Model_Covariance_Matrix*Noise_State_Matrix';
end
Predicted_Apriori_Estimated_Covariance_Previous = Covariance_Prediction;
Predicted_Apriori_Estimated_Covariance_Current = State_Matrix*Predicted_Apriori_Estimated_Covariance_Previous*State_Matrix' + ...
                                                 Noise_State_Matrix*State_Model_Covariance_Matrix*Noise_State_Matrix';

% Innovation:
Measurement_Innovation = Measurement_Vector_Current - Measurement_Matrix*State_Vector_Model_Apriori_Estimated_Current;
% Covariance:
Covariance_Innovation = Measurement_Matrix*Predicted_Apriori_Estimated_Covariance_Current*Measurement_Matrix' + Measurement_Model_Covariance_Matrix;
% Optimal Kalman gain:
Kalman_Gain = Predicted_Apriori_Estimated_Covariance_Current*Measurement_Matrix'*inv(Covariance_Innovation);
% A posteriori state estimate:
State_Vector_Aposteriori_Estimate = State_Vector_Model_Apriori_Estimated_Current + Kalman_Gain*Measurement_Innovation;
% A posteriori estimate covariance:
Covariance_Aposteriori_Estimate = (eye(size(Kalman_Gain*Measurement_Matrix,1)) - Kalman_Gain*Measurement_Matrix)*Predicted_Apriori_Estimated_Covariance_Current;

% Update the outputs:
Heading_Rate_Wave_Estimated_Current = State_Vector_Aposteriori_Estimate(1);
Heading_Wave_Estimated_Current = State_Vector_Aposteriori_Estimate(2);
Heading_Low_Frequency_Estimated_Current = State_Vector_Aposteriori_Estimate(3);
Heading_Rate_Estimated_Current = State_Vector_Aposteriori_Estimate(4);
Bias_Estimated_Current = State_Vector_Aposteriori_Estimate(5);

% Update the Kalman Filter states and covariance for the next prediction:
State_Model_Prediction = State_Vector_Aposteriori_Estimate;
Covariance_Prediction = Covariance_Aposteriori_Estimate;
Rudder_Angle_Commanded_Prev = Rudder_Angle_Commanded_Current;

end
