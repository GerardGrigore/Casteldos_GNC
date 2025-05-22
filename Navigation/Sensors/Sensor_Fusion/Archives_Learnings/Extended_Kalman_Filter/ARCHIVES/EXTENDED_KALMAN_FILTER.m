function [] = EXTENDED_KALMAN_FILTER(STATES_INIT,COVARIANCE_STATES_INIT,TIME_CURRENT,FUNCTION_NON_LINEAR,...
                                     JACOBIAN_STATE_VECTOR_CURRENT_INIT,JACOBIAN_NOISE_VECTOR_CURRENT_INIT,...
                                     PROCESS_NOISE_MATRIX,MEASUREMENT_MATRIX_MODEL_H)

% The Extended Kalman Filter is used in a context of sensor fusion based
% upon non-linear process and measurement models. It allows to estimate
% some states by linearizing the non-linear functions.

% In order for the algorithm to work well, that is to say for the
% FUNCTION_NON_LINEAR to be fetch with the good inputs, one shall carefully
% declare STATES_INIT as it follows:

% STATES_PREV = [EAST_INIT;
%                NORTH_INIT;
%                PHI_INIT;
%                VELOCITY_INIT;
%                ANGULAR_RATE_INIT;
%                ACCELERATION_INIT;
%                JERK_INIT;
%                ANGULAR_ACCELERATION_INIT]

% 1) Initialization:
% ------------------
STATES_PREV = STATES_INIT;
COVARIANCE_STATES_PREV = COVARIANCE_STATES_INIT;
JACOBIAN_STATE_VECTOR_PREV = JACOBIAN_STATE_VECTOR_CURRENT_INIT;
JACOBIAN_NOISE_VECTOR_PREV = JACOBIAN_NOISE_VECTOR_CURRENT_INIT;

% 2) Time update equations - Prediction Step equations:
% -----------------------------------------------------
% Predicted mean or Estimate of the state at TIME_CURRENT given the data up
% to TIME_PREVIOUS or A priori estimate of the state:
STATES_APRIORI_ESTIMATE = FUNCTION_NON_LINEAR([STATES_PREV;TIME_CURRENT]); % The non-linear function is FUNCTION_KINEMATIC.
% Predicted covariance of the state or A priori state estimate covariance matrix error:
COVARIANCE_APRIORI_ESTIMATE = JACOBIAN_STATE_VECTOR_PREV*COVARIANCE_STATES_PREV*COVARIANCE_STATES_PREV' + ...
                              JACOBIAN_NOISE_VECTOR_PREV*PROCESS_NOISE_MATRIX*JACOBIAN_NOISE_VECTOR_PREV';

% 3) Measurement update step:
% ---------------------------
% Current Kalman Gain:
KALMAN_GAIN_CURRENT = COVARIANCE_APRIORI_ESTIMATE*MEASUREMENT_MATRIX_MODEL_H'*...
                      inv(MEASUREMENT_MATRIX_MODEL_H*COVARIANCE_APRIORI_ESTIMATE*MEASUREMENT_MATRIX_MODEL_H' + ...
                      MEASUREMENTS_NOISE);
% Estimated current value of the state or A posteriori estimate of the
% state:
STATES_APOSTERIORI_ESTIMATE = STATES_APRIORI_ESTIMATE + KALMAN_GAIN_CURRENT*(MEASUREMENTS_CURRENT - ...
                              MEASUREMENT_MATRIX_MODEL_H*STATES_APRIORI_ESTIMATE);
% Covariance of the updated estimate or A posteriori state estimate
% covariance matrix error:
COVARIANCE_APOSTERIORI_ESTIMATE = COVARIANCE_APRIORI_ESTIMATE - KALMAN_GAIN_CURRENT*(MEASUREMENT_MATRIX_MODEL_H*...
                                  COVARIANCE_APRIORI_ESTIMATE*MEASUREMENT_MATRIX_MODEL_H' + MEASUREMENTS_NOISE)*...
                                  KALMAN_GAIN_CURRENT';
% DIMENSION_EYE = size(KALMAN_GAIN_CURRENT*MEASUREMENT_MATRIX_MODEL_H,1);
% COVARIANCE_APOSTERIORI_ESTIMATE = (eye(DIMENSION_EYE) - KALMAN_GAIN_CURRENT*MEASUREMENT_MATRIX_MODEL_H)*COVARIANCE_APRIORI_ESTIMATE;

end

