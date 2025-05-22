function FUNCTION_OUTPUT_CURRENT = FUNCTION_KINEMATIC(EAST_PREV,NORTH_PREV,PHI_PREV,VELOCITY_PREV,...
                                                      ANGULAR_RATE_PREV,ACCELERATION_PREV,JERK_PREV,...
                                                      ANGULAR_ACCELERATION_PREV,TIME_STEP)

% This function is the non-linear usually called 'f' function such that:
% X(k) = f(X(k-1),U(k-1),W(k-1)). Hence here, this function is composed by
% the kinematic relations between the following states of the state vector:

% STATE_VECTOR = [EAST;             (m)
%                 NORTH;            (m)
%                 PHI;              (rad)
%                 VELOCITY;         (m/s)
%                 ANGULAR_RATE;     (rad/s)
%                 ACCELERATION;     (m/s²)

% EAST state vector component calculation:
% ----------------------------------------
EAST_CURR = EAST_PREV + cos(PHI_PREV)*(VELOCITY_PREV*TIME_STEP + ACCELERATION_PREV*0.5*TIME_STEP^2 + ...
            JERK_PREV*(1/3)*TIME_STEP^3) - sin(PHI_PREV)*(0.5*VELOCITY_PREV*ANGULAR_RATE_PREV*TIME_STEP^2 +...
            (VELOCITY_PREV*ANGULAR_ACCELERATION_PREV*0.5*TIME_STEP^2 + 2*ACCELERATION_PREV*ANGULAR_RATE_PREV)*(1/6)*TIME_STEP^3 + ...
            (JERK_PREV*ANGULAR_RATE_PREV + ACCELERATION_PREV*ANGULAR_ACCELERATION_PREV)*(1/8)*TIME_STEP^4 + ...
            JERK_PREV*ANGULAR_ACCELERATION_PREV*(1/20)*TIME_STEP^5);

% NORTH state vector component calculation:
% ----------------------------------------
NORTH_CURR = NORTH_PREV + sin(PHI_PREV)*(VELOCITY_PREV*TIME_STEP + ACCELERATION_PREV*0.5*TIME_STEP^2 + ...
            JERK_PREV*(1/3)*TIME_STEP^3) + sin(PHI_PREV)*(0.5*VELOCITY_PREV*ANGULAR_RATE_PREV*TIME_STEP^2 +...
            (VELOCITY_PREV*ANGULAR_ACCELERATION_PREV*0.5*TIME_STEP^2 + 2*ACCELERATION_PREV*ANGULAR_RATE_PREV)*(1/6)*TIME_STEP^3 + ...
            (JERK_PREV*ANGULAR_RATE_PREV + ACCELERATION_PREV*ANGULAR_ACCELERATION_PREV)*(1/8)*TIME_STEP^4 + ...
            JERK_PREV*ANGULAR_ACCELERATION_PREV*(1/20)*TIME_STEP^5);

% PHI state vector component calculation:
% ---------------------------------------
PHI_CURR = PHI_PREV + sign(VELOCITY_PREV)*(ANGULAR_RATE_PREV*TIME_STEP + ANGULAR_ACCELERATION_PREV*0.5*TIME_STEP^2);

% VELOCITY state vector component calculation:
% --------------------------------------------
VELOCITY_CURR = VELOCITY_PREV + ACCELERATION_PREV*TIME_STEP + JERK_PREV*0.5*TIME_STEP^2;

% ANGULAR_RATE state vector component calculation:
% ------------------------------------------------
ANGULAR_RATE_CURR = ANGULAR_RATE_PREV + ANGULAR_ACCELERATION_PREV*TIME_STEP;

% ACCELERATION state vector component calculation:
% ------------------------------------------------
ACCELERATION_CURR = ACCELERATION_PREV + JERK_PREV*TIME_STEP;

% Current function vector form elaboration for output:

FUNCTION_OUTPUT_CURRENT = [EAST_CURR;
                           NORTH_CURR;
                           PHI_CURR;
                           VELOCITY_CURR;
                           ANGULAR_RATE_CURR;
                           ACCELERATION_CURR];

end

