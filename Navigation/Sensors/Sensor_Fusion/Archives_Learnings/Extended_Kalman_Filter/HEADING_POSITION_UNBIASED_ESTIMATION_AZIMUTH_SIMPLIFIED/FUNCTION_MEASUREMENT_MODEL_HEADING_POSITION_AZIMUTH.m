function FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH_OUT = FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH(EAST_POSITION_X_PREVIOUS,NORTH_POSITION_Y_PREVIOUS,...
                                                                                                                       PSI_GPS_HEADING_PREVIOUS,VELOCITY_PREVIOUS,...
                                                                                                                       PSI_MAGNETOMETER_PREVIOUS,TIME_STEP_HW)

% This function reprezents the first term of the first order Taylor
% Series expantion.

FUNCTION_MEASUREMENT_MODEL_HEADING_POSITION_AZIMUTH_OUT = [EAST_POSITION_X_PREVIOUS;
                                                           NORTH_POSITION_Y_PREVIOUS;
                                                           PSI_GPS_HEADING_PREVIOUS;
                                                           VELOCITY_PREVIOUS;
                                                           PSI_MAGNETOMETER_PREVIOUS;
                                                           PSI_MAGNETOMETER_PREVIOUS - pi];

end

