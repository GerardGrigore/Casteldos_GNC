function JACOBIAN_STATE_PROCESS_MATRIX = JACOBIAN_STATE_PROCESS_MODEL(VELOCITY_PREVIOUS_KNOWING_PREVIOUS,...
                                                                      HEADING_PSI_PREVIOUS_KNOWING_PREVIOUS,...
                                                                      TIME_STEP_HW)

% This function serves to compute the Jacobian associated to the state
% vector of the process model.

% Compute the components of the matrix:
JACOBIAN_STATE_13 = -VELOCITY_PREVIOUS_KNOWING_PREVIOUS*TIME_STEP_HW*sin(HEADING_PSI_PREVIOUS_KNOWING_PREVIOUS);
JACOBIAN_STATE_14 = TIME_STEP_HW*cos(HEADING_PSI_PREVIOUS_KNOWING_PREVIOUS);
JACOBIAN_STATE_23 = VELOCITY_PREVIOUS_KNOWING_PREVIOUS*TIME_STEP_HW*cos(HEADING_PSI_PREVIOUS_KNOWING_PREVIOUS);
JACOBIAN_STATE_24 = TIME_STEP_HW*sin(HEADING_PSI_PREVIOUS_KNOWING_PREVIOUS);

% Compute the Jacobian state process model matrix at the prediction step:
JACOBIAN_STATE_PROCESS_MATRIX = [1 0 JACOBIAN_STATE_13 JACOBIAN_STATE_14;
    0 1 JACOBIAN_STATE_23 JACOBIAN_STATE_24;
    0 0 1 0;
    0 0 0 1];

end

