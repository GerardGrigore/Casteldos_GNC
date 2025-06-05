function [State_Aposteriori_Current,Covariance_Aposteriori_Current] = EKF_Heading(States_Initial,...
                                                                                  Covariance_States_Initial,...
                                                                                  Control_Inputs_Initial,...
                                                                                  Jacobian_State_Process_Model,...
                                                                                  Process_Model_Covariance_Matrix,...
                                                                                  Measurement_Model_Matrix,...
                                                                                  Measurement_Covariance_Matrix,...
                                                                                  Function_Process_Model,...
                                                                                  Measurement_Current)
                                                                                  

% This function serves as the EKF function allowing to provide, given
% measurements, an estimation of an aimed state vector.

% Initialization:
States_Previous = States_Initial; % Not used because used as an input in the main script.
Covariance_States_Previous = Covariance_States_Initial; % The 'P0|0' equal to 'Q' in this case.
Control_Inputs_Previous = Control_Inputs_Initial; % The 'u' current control input.

% Prediction step equations:
% State prediction equation:
State_Apriori_Prediction_Current = Function_Process_Model;
% Covariance matrix prediction:
Covariance_Prediction_Current = Jacobian_State_Process_Model*Covariance_States_Previous*Jacobian_State_Process_Model' + ...
    Process_Model_Covariance_Matrix;

% Measurement update equations:
% Kalman gain value:
Kalman_Gain_Current = Covariance_Prediction_Current*Measurement_Model_Matrix'*inv(Measurement_Model_Matrix*...
    Covariance_Prediction_Current*Measurement_Model_Matrix' + Measurement_Covariance_Matrix);
% State update using measurements and apriori prediction estimate:
% Innovation function:
Innovation_Current = Measurement_Current -  Measurement_Model_Matrix*State_Apriori_Prediction_Current;
% Normalization of the heading:
Innovation_Current(3) = mod(Innovation_Current(3) + pi, 2*pi) - pi;
State_Aposteriori_Current = State_Apriori_Prediction_Current + Kalman_Gain_Current*Innovation_Current;
% Final normalization of the heading angle:
State_Aposteriori_Current(3) = mod(State_Aposteriori_Current(3), 2*pi);

% Update of the covariance matrix:
Dimension_Eye = size(Kalman_Gain_Current*Measurement_Model_Matrix,1);
Covariance_Aposteriori_Current = (eye(Dimension_Eye) - Kalman_Gain_Current*Measurement_Model_Matrix)*Covariance_Prediction_Current;

end

