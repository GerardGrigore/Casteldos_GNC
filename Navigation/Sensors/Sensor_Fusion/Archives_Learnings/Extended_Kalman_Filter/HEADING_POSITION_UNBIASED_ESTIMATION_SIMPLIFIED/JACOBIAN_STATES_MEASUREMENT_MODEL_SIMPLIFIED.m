function JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = JACOBIAN_STATES_MEASUREMENT_MODEL_SIMPLIFIED(VELOCITY_CURRENT,ANGULAR_RATE_PHI_POINT_CURRENT)

% This function computes in output the jacobian of the states of the measurement model
% evaluated on a specific point specified in input.
% Compute the jacobian:

JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = [1 0 0 0 0 0;
                                            0 1 0 0 0 0;
                                            0 0 1 0 0 0;
                                            0 0 0 1 0 0;
                                            0 0 0 0 1 0;
                                            0 0 0 0 0 1;
                                            0 0 0 ANGULAR_RATE_PHI_POINT_CURRENT VELOCITY_CURRENT 0];

% JACOBIAN_STATES_MEASUREMENT_MODEL_OUTPUT = [1 0 0 0 0 0;
%                                             0 1 0 0 0 0;
%                                             0 0 1 0 0 0;
%                                             0 0 0 1 0 0;
%                                             0 0 0 0 1 0;
%                                             0 0 0 0 0 1;
%                                             0 0 0 1 1 0];

end

