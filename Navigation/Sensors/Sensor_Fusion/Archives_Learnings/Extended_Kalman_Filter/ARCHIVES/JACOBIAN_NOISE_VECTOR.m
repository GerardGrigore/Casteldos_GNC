function JACOBIAN_NOISE_VECTOR_CURRENT = JACOBIAN_NOISE_VECTOR(PHI,ANGULAR_RATE,VELOCITY,ACCELERATION,TIME_STEP,JERK,...
                                                               ANGULAR_ACCELERATION)

% This function aims at returning the Jacobian of a non-linear function
% evaluated on a specific point and given several inputs and wrt noise process vector.

% First, the algorithm will calculate the elements (i,j) of the Jacobian.
% Then, the Jacobian will be created.

% NOTA - /!\ The equations must be verified /!\.

GAMMA_11 = -sin(PHI)*(VELOCITY*(1/6)*TIME_STEP^3 + ACCELERATION*(1/8)*TIME_STEP^4 + JERK*(1/20)*TIME_STEP^5);

GAMMA_12 = cos(PHI)*(1/3)*TIME_STEP^3 - sin(PHI)*((1/8)*ANGULAR_RATE^4 + ANGULAR_ACCELERATION*(1/20)*TIME_STEP^5);

GAMMA_21 = cos(PHI)*(VELOCITY*(1/6)*TIME_STEP^3 + ACCELERATION*(1/8)*TIME_STEP^4 + JERK*(1/20)*TIME_STEP^5);

GAMMA_22 = sin(PHI)*(1/3)*TIME_STEP^3 + cos(PHI)*((1/8)*ANGULAR_RATE^4 + ANGULAR_ACCELERATION*(1/20)*TIME_STEP^5);

% Creation of the Jacobian:
% -------------------------

JACOBIAN_NOISE_VECTOR_CURRENT = [GAMMA_11 GAMMA_12;
                                 GAMMA_21 GAMMA_22;
                                 sin(PHI)*0.5*TIME_STEP^2 0;
                                 0 0.5*TIME_STEP^2;
                                 TIME_STEP 0;
                                 0 TIME_STEP];

end

