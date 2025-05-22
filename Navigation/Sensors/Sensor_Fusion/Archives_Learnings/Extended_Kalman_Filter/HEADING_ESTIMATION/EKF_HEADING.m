function [STATE_APOSTERIORI_CURRENT,COVARIANCE_APOSTERIORI_CURRENT] = EKF_HEADING(STATES_INITIAL,COVARIANCE_STATES_INITIAL,CONTROL_INPUTS_INITIAL,...
                                                                                  JACOBIAN_STATE_PROCESS_MODEL,...
                                                                                  PROCESS_MODEL_COVARIANCE_MATRIX,MEASUREMENT_MODEL_MATRIX,...
                                                                                  MEASUREMENT_COVARIANCE_MATRIX,...
                                                                                  FUNCTION_PROCESS_MODEL,MEASUREMENT_CURRENT,...
                                                                                  TIME_STEP_HW)

% This function serves as the EKF function allowing to provide, given
% measurements, an estimation of an aimed state vector.

% Initialization:
% ---------------
STATES_PREVIOUS = STATES_INITIAL; % Not used because used as an input in the main script.
COVARIANCE_STATES_PREVIOUS = COVARIANCE_STATES_INITIAL; % The 'P0|0' equal to 'Q' in this case.
CONTROL_INPUTS_PREVIOUS = CONTROL_INPUTS_INITIAL; % Not used ('B' process model matrix).

% Prediction step equations:
% --------------------------
% State prediction equation:
STATE_APRIORI_PREDICTION_CURRENT = FUNCTION_PROCESS_MODEL;
% Covariance matrix prediction:
COVARIANCE_PREDICTION_CURRENT = JACOBIAN_STATE_PROCESS_MODEL*COVARIANCE_STATES_PREVIOUS*JACOBIAN_STATE_PROCESS_MODEL' + ...
                                PROCESS_MODEL_COVARIANCE_MATRIX;

% Measurement update equations:
% -----------------------------
% Kalman gain value:
% /!\ Numerical protection for inversion.
KALMAN_GAIN_CURRENT = COVARIANCE_PREDICTION_CURRENT*MEASUREMENT_MODEL_MATRIX'*inv(MEASUREMENT_MODEL_MATRIX*...
    COVARIANCE_PREDICTION_CURRENT*MEASUREMENT_MODEL_MATRIX' + MEASUREMENT_COVARIANCE_MATRIX);
% State update using measurements and apriori prediction estimate:
STATE_APOSTERIORI_CURRENT = STATE_APRIORI_PREDICTION_CURRENT + KALMAN_GAIN_CURRENT*(MEASUREMENT_CURRENT - ...
    MEASUREMENT_MODEL_MATRIX*STATE_APRIORI_PREDICTION_CURRENT);
% Update of the covariance matrix:
DIMENSION_EYE = size(KALMAN_GAIN_CURRENT*MEASUREMENT_MODEL_MATRIX,1);
COVARIANCE_APOSTERIORI_CURRENT = (eye(DIMENSION_EYE) - KALMAN_GAIN_CURRENT*MEASUREMENT_MODEL_MATRIX)*COVARIANCE_PREDICTION_CURRENT;
% Try the second calculation method:
% COVARIANCE_APOSTERIORI_CURRENT = COVARIANCE_PREDICTION_CURRENT - KALMAN_GAIN_CURRENT*(MEASUREMENT_MODEL_MATRIX*COVARIANCE_PREDICTION_CURRENT*...
%                                  MEASUREMENT_MODEL_MATRIX' + MEASUREMENT_COVARIANCE_MATRIX)*KALMAN_GAIN_CURRENT;

end

