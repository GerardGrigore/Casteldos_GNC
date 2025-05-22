function JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = JACOBIAN_STATES_MEASUREMENT_MODEL(VELOCITY_CURRENT,ANGULAR_RATE_PHI_POINT_CURRENT,...
                                                                                      IS_HEADING_IMU_GYRO_AVAILABLE)

% This function computes in output the jacobian of the states of the measurement model
% evaluated on a specific point specified in input.
% Compute the jacobian:
if ~IS_HEADING_IMU_GYRO_AVAILABLE
    JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = [1 0 0 0 0 0;
                                                0 1 0 0 0 0;
                                                0 0 1 0 0 0;
                                                0 0 0 1 0 0;
                                                0 0 0 0 1 0;
                                                0 0 0 0 0 1;
                                                0 0 0 ANGULAR_RATE_PHI_POINT_CURRENT VELOCITY_CURRENT 0];
else
    JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = [1 0 0 0 0 0;
                                                0 1 0 0 0 0;
                                                0 0 1 0 0 0;
                                                0 0 0 1 0 0;
                                                0 0 0 0 1 0;
                                                0 0 0 0 0 1;
                                                0 0 0 ANGULAR_RATE_PHI_POINT_CURRENT VELOCITY_CURRENT 0;
                                                0 0 1 0 0 0;
                                                0 0 1 0 0 0];
end

% JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = [1 0 0 0 0 0;
%                                             0 1 0 0 0 0;
%                                             0 0 1 0 0 0;
%                                             0 0 0 1 0 0;
%                                             0 0 0 0 1 0;
%                                             0 0 0 0 0 1;
%                                             0 0 0 1 1 0];

end

