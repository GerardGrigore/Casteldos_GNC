function [Heading_Rate_Wave_Estimated_Current,...
          Heading_Wave_Estimated_Current,...
          Heading_Low_Frequency_Estimated_Current,...
          Heading_Rate_Estimated_Current,...
          Bias_Estimated_Current,...
          Covariance_Aposteriori_Estimate] = Kalman_Observer_Wave_Estimation_2(Heading_Total_Observed_Current,...
                                                                               Rudder_Angle_Commanded_Current,...
                                                                               Wave_Pulsation_Estimated,...
                                                                               Damping_Wave_Estimated,...
                                                                               Time_Current,...
                                                                               Time_Constant,...
                                                                               Static_Gain,...
                                                                               Time_Sampling,...
                                                                               Noise_For_Heading_wave,...
                                                                               Torque_Wind_Estimated_Input,...
                                                                               Inertia_Ship_Vertical,...
                                                                               Wave_Pulsation_Identified,...
                                                                               Heading_Rate_Estimated_Current_Navigation)



% To be cleaned and implemented:
% Persistent states
persistent State_Model_Prediction Covariance_Prediction Rudder_Angle_Commanded_Prev Identified_Wave_Pulsation

% --------------------------
% Basic input safety checks
% --------------------------
if isempty(Time_Sampling) || Time_Sampling <= 0
    error('Time_Sampling must be > 0');
end
if isempty(Time_Constant) || Time_Constant == 0
    error('Time_Constant_Ship must be nonzero');
end
if any(isnan([Heading_Total_Observed_Current, Rudder_Angle_Commanded_Current, Wave_Pulsation_Estimated, Damping_Wave_Estimated]))
    error('One or more inputs are NaN');
end

% freeze wave pulsation after identified time
Wave_Pulsation_Identified = Wave_Pulsation_Estimated;
if Time_Current > 550
    if isempty(Identified_Wave_Pulsation)
        Identified_Wave_Pulsation = Wave_Pulsation_Identified;
    end
    Wave_Pulsation_Estimated = Identified_Wave_Pulsation;
end

% --------------------------
% Continuous-time model (ordre 5)
% --------------------------
A_cont = [0 1 0 0 0;
          -Wave_Pulsation_Estimated^2 -2*Damping_Wave_Estimated*Wave_Pulsation_Estimated 0 0 0;
           0 0 0 1 0;
           0 0 0 -1/Time_Constant 1;
           0 0 0 0 0];

B_cont = [0; 0; 0; Static_Gain/Time_Constant; 0];

% E_cont = [0 0 0;
%           Wave_Pulsation_Estimated^2 0 0;
%           0 0 0;
%           0 1 0;
%           0 0 1];
E_cont = [0 0 0;
          1 0 0;
          0 0 0;
          0 1 0;
          0 0 1];

% --------------------------
% Discretisation par bloc (évite l'inversion de A_cont)
% --------------------------
h = Time_Sampling;
n = size(A_cont,1);
m = size(B_cont,2);
p = size(E_cont,2);

% Matrice bloc [A B E; 0 0 0; 0 0 0] et exponentielle
Mblk = [A_cont, B_cont, E_cont;
        zeros(m, n+m+p);
        zeros(p, n+m+p)];
Md = expm(Mblk * h);
F = Md(1:n, 1:n);
D = Md(1:n, n+1 : n+m);
G = Md(1:n, n+m+1 : n+m+p);

% F = expm(A_cont*h);
% D = (A_cont \ (F - eye(size(F,1))))*B_cont;
% G = (A_cont \ (F - eye(size(F,1))))*E_cont;

% --------------------------
% Initialisation des états si première exécution
% --------------------------
if isempty(State_Model_Prediction)
    % initial states: small values, use measured heading as initial guess for components
    State_Model_Prediction = [0;                                % heading rate wave (rad/s)
                              Heading_Total_Observed_Current;   % heading wave (rad) (initial guess)
                              Heading_Total_Observed_Current;   % heading low freq (rad)
                              0;                                % heading rate (rad/s)
                              0];                               % bias (rad)
end

if isempty(Rudder_Angle_Commanded_Prev)
    Rudder_Angle_Commanded_Prev = 0;
end

if isempty(Covariance_Prediction)
    % initial P0 (SPD, petites valeurs sur états)
    Covariance_Prediction = diag([ (deg2rad(5))^2, ...   % state 1 (wave rate) incertaine un peu
                                   (deg2rad(10))^2, ...  % state 2 (wave heading)
                                   (deg2rad(10))^2, ...  % state 3 (low freq heading)
                                   (deg2rad(1))^2, ...   % state 4 (heading rate)
                                   (deg2rad(0.5))^2]);   % state 5 (bias)
end

% --------------------------
% Covariances Qc (continu) and discrete Qd
% --------------------------
% Qc is defined on the noise components of E_cont (p x p)
Sigma_Heading_Wave = deg2rad(20.0);    % std of wave heading process (rad)
Sigma_Rate_Heading = deg2rad(0.5);    % std of wave rate process (rad/s)
Sigma_Bias = deg2rad(0.01);           % std of slow bias

Qc = diag([ Sigma_Heading_Wave^2, Sigma_Rate_Heading^2, Sigma_Bias^2 ]); % p x p

% discrete process noise covariance on the state:
% since we used G from block expm (x_{k+1} = F x_k + D u + G * w_cont_integrated)
% an approximation consistent is:
Qd = G * Qc * G';

% --------------------------
% Measurement
% --------------------------
H = [0 1 1 0 0];   % measurement matrix
y = Heading_Total_Observed_Current;

% Measurement noise R (variance)
Sigma_Heading_Estimated = deg2rad(2.0);  % measurement std (2 deg)
R = Sigma_Heading_Estimated^2;

% --------------------------
% Kalman Prediction
% --------------------------
u = Rudder_Angle_Commanded_Current;
x_apriori = F * State_Model_Prediction + D * u;
P_apriori = F * Covariance_Prediction * F' + Qd;

% basic sanity: ensure P_apriori is symmetric SPD
P_apriori = (P_apriori + P_apriori')/2;
[eigV, eigD] = eig(P_apriori);
if any(diag(eigD) <= 0)
    % regularize
    P_apriori = P_apriori + eye(n) * 1e-9;
end

% --------------------------
% Kalman Update
% --------------------------
S = H * P_apriori * H' + R;
K = (P_apriori * H') / S;   % 5x1
innovation = y - H * x_apriori;

x_aposteriori = x_apriori + K * innovation;
P_aposteriori = (eye(n) - K * H) * P_apriori;

% ensure symmetry
P_aposteriori = (P_aposteriori + P_aposteriori')/2;

% --------------------------
% Outputs
% --------------------------
Heading_Rate_Wave_Estimated_Current     = x_aposteriori(1);
Heading_Wave_Estimated_Current          = x_aposteriori(2);
Heading_Low_Frequency_Estimated_Current = x_aposteriori(3);
Heading_Rate_Estimated_Current          = x_aposteriori(4);
Bias_Estimated_Current                  = x_aposteriori(5);

Covariance_Aposteriori_Estimate = P_aposteriori;

% --------------------------
% Store for next iteration
% --------------------------
State_Model_Prediction = x_aposteriori;
Covariance_Prediction  = P_aposteriori;
Rudder_Angle_Commanded_Prev = Rudder_Angle_Commanded_Current;



end

