function JACOBIAN_STATE_VECTOR_CURRENT = JACOBIAN_STATE_VECTOR(PHI,ANGULAR_RATE,VELOCITY,ACCELERATION,TIME_STEP)

% This function aims at returning the Jacobian of a non-linear function
% evaluated on a specific point and given several inputs and wrt state vector.

% First, the algorithm will calculate the elements (i,j) of the Jacobian.
% Then, the Jacobian will be created.

% NOTA - /!\ The equations must be verified /!\.

JACOBIAN_13 = -sin(PHI)*(VELOCITY*TIME_STEP + ACCELERATION*0.5*TIME_STEP^2) - cos(PHI)*...
              (VELOCITY*ANGULAR_RATE*0.5*TIME_STEP^2 + 2*ACCELERATION*ANGULAR_RATE*(1/6)*TIME_STEP^3);

JACOBIAN_14 = cos(PHI)*TIME_STEP - sin(PHI)*(ANGULAR_RATE*0.5*TIME_STEP^2);

JACOBIAN_15 = -sin(PHI)*(VELOCITY*0.5*TIME_STEP^2 + 2*ACCELERATION*(1/6)*TIME_STEP^3);

JACOBIAN_16 = cos(PHI)*0.5*TIME_STEP^2 - sin(PHI)*(2*(1/6)*ANGULAR_RATE*TIME_STEP^3);

JACOBIAN_23 = cos(PHI)*(VELOCITY*TIME_STEP + ACCELERATION*0.5*TIME_STEP^2) - sin(PHI)*...
              (VELOCITY*ANGULAR_RATE*0.5*TIME_STEP^2 + 2*ACCELERATION*ANGULAR_RATE*(1/6)*TIME_STEP^3);

JACOBIAN_24 = sin(PHI)*TIME_STEP + cos(PHI)*(ANGULAR_RATE*0.5*TIME_STEP^2);

JACOBIAN_25 = cos(PHI)*(VELOCITY*0.5*TIME_STEP^2 + 2*ACCELERATION*(1/6)*TIME_STEP^3);

JACOBIAN_26 = sin(PHI)*0.5*TIME_STEP^2 + cos(PHI)*(2*(1/6)*ANGULAR_RATE*TIME_STEP^3);

% Creation of the Jacobian:
% -------------------------

JACOBIAN_STATE_VECTOR_CURRENT = [1 0 JACOBIAN_13 JACOBIAN_14 JACOBIAN_15 JACOBIAN_16;
                               0 1 JACOBIAN_23 JACOBIAN_24 JACOBIAN_25 JACOBIAN_26;
                               0 0 1 0 sign(VELOCITY)*TIME_STEP 0;
                               0 0 0 1 0 TIME_STEP;
                               0 0 0 0 1 0;
                               0 0 0 0 0 1];

end

