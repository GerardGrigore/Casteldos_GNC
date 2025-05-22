clear all
clc

%--------------------------------------------------------------------------
% I) Process Model - Initialization:
%--------------------------------------------------------------------------

% The process model of the Kalman Filter is based on the kinematic
% relations between states that are measurables. The following state vector
% was designated to be used:

% STATE_VECTOR = [EAST;             (m)
%                 NORTH;            (m)
%                 PHI;              (rad)
%                 VELOCITY;         (m/s)
%                 ANGULAR_RATE;     (rad/s)
%                 ACCELERATION;     (m/s²)

% Everuthing will be expressed in local coordinate frame ENU. One shall
% note that the "Up" components are ignored because the considered plane is
% the EN one.

% Generation of the TIME_STEP_VECTOR:
TIME_STEP_VECTOR = linspace(0,2*60*60,200)'; % Suppose the trajectory is created to last 2 hours.

% a) Initialization of the state vector:
% --------------------------------------
EAST_INIT = 0;
NORTH_INIT = 0;
PHI_INIT = 0;
VELOCITY_INIT = 0;
ANGULAR_RATE_INIT = 0;
ACCELERATION_INIT = 0;
TIME_STEP_INIT = TIME_STEP_VECTOR(1);
JERK_INIT = 0;
ANGULAR_ACCELERATION_INIT = 0;

STATE_VECTOR_INIT = [EAST_INIT;
                     NORTH_INIT;
                     PHI_INIT;
                     VELOCITY_INIT;
                     ANGULAR_RATE_INIT;
                     ACCELERATION_INIT];

% b) Linearization of the state-space model:
% ------------------------------------------
% Non linearities found in the kinematic equations implies the use of the
% Extented Kalman Filter as well as linearization of the state equations
% such that:

% X(n) = f(X̂(n-1|n-1),0) + Fx*(X(n-1) - X̂(n-1|n-1)) + GAMMA*W(n-1)

% With:
% -----

%   * f(X̂(n-1|n-1),0): the non-linear f function evaluated at the point
%     X̂(n-1|n-1) and considering the input as 0.

%   * Fx: the Jacobian of the non-linear f function wrt state vector X and evaluated 
%     at the point X̂(n-1|n-1) and input 0. See JACOBIAN_STATE_VECTOR.m.

%   * X(n-1): State vector at previous step time.

%   * GAMMA: the Jacobian of the non-linear f function wrt noise vector W
%     at the point X̂(n-1|n-1) and input 0. See JACOBIAN_NOISE_VECTOR.m.

JACOBIAN_STATE_VECTOR_CURRENT_INIT = JACOBIAN_STATE_VECTOR(PHI_INIT,ANGULAR_RATE_INIT,VELOCITY_INIT,...
                                   ACCELERATION_INIT,TIME_STEP_INIT);

JACOBIAN_NOISE_VECTOR_CURRENT_INIT = JACOBIAN_NOISE_VECTOR(PHI_INIT,ANGULAR_RATE_INIT,VELOCITY_INIT,...
                                   ACCELERATION_INIT,TIME_STEP_INIT,JERK_INIT,ANGULAR_ACCELERATION_INIT);

MEASUREMENT_MATRIX_MODEL = [1 0 0 0 0 0;
                            0 1 0 0 0 0;
                            0 0 1 0 0 0;
                            0 0 0 1 0 0;
                            0 0 0 0 1 0;
                            0 0 0 0 0 1;
                            0 0 0 1 1 0];

%--------------------------------------------------------------------------------------
% II) Extended Kalman Filter - Measurement Model - Measurement & process noise Matrices:
%--------------------------------------------------------------------------------------

% c) Extended Kalman Filter:
% --------------------------









