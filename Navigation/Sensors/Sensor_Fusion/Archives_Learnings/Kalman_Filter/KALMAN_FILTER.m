function [STATES_APOSTERIORI_ESTIMATE,COVARIANCE_APOSTERIORI_ESTIMATE] = KALMAN_FILTER(STATES_INITIAL,COVARIANCE_STATES_INITIAL,...
                                                                                       CONTROL_INPUTS_INITIAL,STATE_MATRIX_MODEL_A,...
                                                                                       INPUTS_CONTROL_B,PROCESS_NOISE,...
                                                                                       MEASUREMENT_MATRIX_MODEL_H,MEASUREMENTS_NOISE,...
                                                                                       MEASUREMENTS_CURRENT)

% In the context of Navigation and Sensor Fusion, the use of a Kalman
% Filter allows the user to estimate the states of its System. The use of
% such a filter is optimal when it comes to linear state-space process and
% measurement models.

% /!\ Carefull, as designed here, this algorithm shall be called at each
% time step in a generic script containing the measurements. It is
% done in such a way that it has to be called at each sampling times /!\.

% 1) Initialization:
% ------------------
STATES_PREVIOUS = STATES_INITIAL;
COVARIANCE_STATES_PREVIOUS = COVARIANCE_STATES_INITIAL;
CONTROL_INPUTS_PREVIOUS = CONTROL_INPUTS_INITIAL;

% 2) Prediction step (time update):
% ---------------------------------
% Predicted mean or Estimate of the state at TIME_CURRENT given the data up
% to TIME_PREVIOUS or A priori estimate of the state:
STATES_APRIORI_ESTIMATE = STATE_MATRIX_MODEL_A*STATES_PREVIOUS + INPUTS_CONTROL_B*CONTROL_INPUTS_PREVIOUS;
% Predicted covariance of the state or A priori state estimate covariance matrix error:
COVARIANCE_APRIORI_ESTIMATE = STATE_MATRIX_MODEL_A*COVARIANCE_STATES_PREVIOUS*STATE_MATRIX_MODEL_A' + ...
                                PROCESS_NOISE;

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

