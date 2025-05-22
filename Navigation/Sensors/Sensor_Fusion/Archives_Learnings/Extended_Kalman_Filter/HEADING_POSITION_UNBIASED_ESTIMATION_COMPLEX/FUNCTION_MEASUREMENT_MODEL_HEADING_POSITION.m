function MEASUREMENT_TAYLOR_EXPANSION_CURRENT = FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION(EAST_CURRENT,NORTH_CURRENT,PHI_HEADING_CURRENT,...
                                                                                            VELOCITY_CURRENT,ANGULAR_RATE_PHI_POINT_CURRENT,...
                                                                                            ACCELERATION_LONGITUDINAL_CURRENT,TIME_STEP_HW,...
                                                                                            IS_HEADING_IMU_GYRO_AVAILABLE,PHI_HEADING_CURRENT_IMU,...
                                                                                            PHI_HEADING_CURRENT_GYRO)

% This function reprezents the first term of the Taylor's Series Expansion
% of the measurement equation linearized with the noise settled to 0.

% Compute directly the output:
if ~IS_HEADING_IMU_GYRO_AVAILABLE
    MEASUREMENT_TAYLOR_EXPANSION_CURRENT = [EAST_CURRENT;
                                            NORTH_CURRENT;
                                            PHI_HEADING_CURRENT;
                                            VELOCITY_CURRENT;
                                            ANGULAR_RATE_PHI_POINT_CURRENT;
                                            ACCELERATION_LONGITUDINAL_CURRENT;
                                            ANGULAR_RATE_PHI_POINT_CURRENT*VELOCITY_CURRENT];
else
    MEASUREMENT_TAYLOR_EXPANSION_CURRENT = [EAST_CURRENT;
                                            NORTH_CURRENT;
                                            PHI_HEADING_CURRENT;
                                            VELOCITY_CURRENT;
                                            ANGULAR_RATE_PHI_POINT_CURRENT;
                                            ACCELERATION_LONGITUDINAL_CURRENT;
                                            ANGULAR_RATE_PHI_POINT_CURRENT*VELOCITY_CURRENT;
                                            PHI_HEADING_CURRENT;
                                            PHI_HEADING_CURRENT];
end

% Try also this:
% Make the connection:
% EAST_PREVIOUS = EAST_CURRENT;
% NORTH_PREVIOUS = NORTH_CURRENT;
% PHI_HEADING_PREVIOUS = PHI_HEADING_CURRENT;
% VELOCITY_PREVIOUS = VELOCITY_CURRENT;
% ANGULAR_RATE_PHI_POINT_PREVIOUS = ANGULAR_RATE_PHI_POINT_CURRENT;
% ACCELERATION_LONGITUDINAL_PREVIOUS = ACCELERATION_LONGITUDINAL_CURRENT;
% 
% EAST_CURRENTp = EAST_PREVIOUS + cos(PHI_HEADING_PREVIOUS)*(VELOCITY_PREVIOUS*TIME_STEP_HW + ACCELERATION_LONGITUDINAL_PREVIOUS*0.5*TIME_STEP_HW^2) -...
%                sin(PHI_HEADING_PREVIOUS)*(VELOCITY_PREVIOUS*ANGULAR_RATE_PHI_POINT_PREVIOUS*0.5*TIME_STEP_HW^2 + (1/3)*ANGULAR_RATE_PHI_POINT_PREVIOUS*...
%                ACCELERATION_LONGITUDINAL_PREVIOUS*TIME_STEP_HW^3);
% 
% NORTH_CURRENTp = NORTH_PREVIOUS + sin(PHI_HEADING_PREVIOUS)*(VELOCITY_PREVIOUS*TIME_STEP_HW + ACCELERATION_LONGITUDINAL_PREVIOUS*0.5*TIME_STEP_HW^2) +...
%                cos(PHI_HEADING_PREVIOUS)*(VELOCITY_PREVIOUS*ANGULAR_RATE_PHI_POINT_PREVIOUS*0.5*TIME_STEP_HW^2 + (1/3)*ANGULAR_RATE_PHI_POINT_PREVIOUS*...
%                ACCELERATION_LONGITUDINAL_PREVIOUS*TIME_STEP_HW^3);
% 
% PHI_HEADING_CURRENTp = PHI_HEADING_PREVIOUS + sign(VELOCITY_PREVIOUS)*(ANGULAR_RATE_PHI_POINT_PREVIOUS*TIME_STEP_HW);
% 
% VELOCITY_CURRENTp = VELOCITY_PREVIOUS + ACCELERATION_LONGITUDINAL_PREVIOUS*TIME_STEP_HW;
% 
% ANGULAR_RATE_PHI_POINT_CURRENTp = ANGULAR_RATE_PHI_POINT_PREVIOUS;
% 
% ACCELERATION_LONGITUDINAL_CURRENTp = ACCELERATION_LONGITUDINAL_PREVIOUS;
% 
% % Then compute the output:
% 
% MEASUREMENT_TAYLOR_EXPANSION_CURRENT = [EAST_CURRENTp;NORTH_CURRENTp;PHI_HEADING_CURRENTp;VELOCITY_CURRENTp;...
%                                         ANGULAR_RATE_PHI_POINT_CURRENTp;ACCELERATION_LONGITUDINAL_CURRENTp;...
%                                         ANGULAR_RATE_PHI_POINT_CURRENTp*VELOCITY_CURRENTp];

end

