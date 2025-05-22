function JACOBIAN_NOISES_PROCESS_MODEL_OUTPUT = JACOBIAN_NOISES_PROCESS_MODEL(PHI_HEADING_CURRENT,VELOCITY_CURRENT,ANGULAR_RATE_PHI_POINT_CURRENT,...
                                                                              ACCELERATION_LONGITUDINAL_CURRENT,ANGULAR_ACCELERATION_YAW_CURRENT,...
                                                                              JERK_LONGITUDINAL_CURRENT,TIME_STEP_HW)

% This function computes in output the jacobian of the noise of the process model
% evaluated on a specific point specified in input.

% Evaluate the specific Jacobian components:
JACOBIAN_11 = -sin(PHI_HEADING_CURRENT)*((1/6)*VELOCITY_CURRENT*TIME_STEP_HW^3 + (1/8)*ACCELERATION_LONGITUDINAL_CURRENT*TIME_STEP_HW^4 +...
              (1/20)*JERK_LONGITUDINAL_CURRENT*TIME_STEP_HW^5);

JACOBIAN_12 = cos(PHI_HEADING_CURRENT)*(1/3)*TIME_STEP_HW^3 - sin(PHI_HEADING_CURRENT)*(ANGULAR_RATE_PHI_POINT_CURRENT*(1/8)*TIME_STEP_HW^4 +...
              (1/20)*ANGULAR_ACCELERATION_YAW_CURRENT*TIME_STEP_HW^5);

JACOBIAN_21 = cos(PHI_HEADING_CURRENT)*((1/6)*VELOCITY_CURRENT*TIME_STEP_HW^3 + (1/8)*ACCELERATION_LONGITUDINAL_CURRENT*TIME_STEP_HW^4 +...
              (1/20)*JERK_LONGITUDINAL_CURRENT*TIME_STEP_HW^5); 

JACOBIAN_22 = sin(PHI_HEADING_CURRENT)*(1/3)*TIME_STEP_HW^3 + cos(PHI_HEADING_CURRENT)*(ANGULAR_RATE_PHI_POINT_CURRENT*(1/8)*TIME_STEP_HW^4 +...
              (1/20)*ANGULAR_ACCELERATION_YAW_CURRENT*TIME_STEP_HW^5);

% The compute the output Jacobian:
JACOBIAN_NOISES_PROCESS_MODEL_OUTPUT = [JACOBIAN_11 JACOBIAN_12;
                                        JACOBIAN_21 JACOBIAN_22;
                                        sign(VELOCITY_CURRENT)*0.5*TIME_STEP_HW^2 0;
                                        0 0.5*TIME_STEP_HW^2;
                                        TIME_STEP_HW 0;
                                        0 TIME_STEP_HW];

end

