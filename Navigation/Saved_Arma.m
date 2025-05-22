% Persistents definition:
persistent Theta_S_Previous_Estimate;
persistent Theta_N_Previous_Estimate;
persistent P_s_Previous_Estimated;
persistent P_n_Previous_Estimated;
persistent Measurement_Previous;
persistent Measurement_Previous_Previous;
persistent Count_Persistent_Update;
persistent v_Previous_Estimated;

% Initialization of the algorithm:
P_0 = 10e6;
if isempty(Theta_S_Previous_Estimate)
    Theta_S_Previous_Estimate = [1/P_0;1/P_0];
end
if isempty(Theta_N_Previous_Estimate)
    Theta_N_Previous_Estimate = 1/P_0;
end
if isempty(P_s_Previous_Estimated)
    P_s_Previous_Estimated = P_0*eye(2);
end
if isempty(P_n_Previous_Estimated)
    P_n_Previous_Estimated = P_0;
end
if isempty(Measurement_Previous)
    Measurement_Previous = 0;
end
if isempty(Measurement_Previous_Previous)
    Measurement_Previous_Previous = 0;
end
if isempty(v_Previous_Estimated)
    v_Previous_Estimated = 0;
end
if isempty(Count_Persistent_Update)
    Count_Persistent_Update = 0;
end

% Collect the output data:
Measurement_Current = Heading_Wave_High_Frequency_Estimated;

% Compute Phi_S and Phi_N_Estimated:
Phi_S_Transpose_Current = [-Measurement_Previous,-Measurement_Previous_Previous];
Phi_S_Current = Phi_S_Transpose_Current';
Phi_N_Current_Estimated = v_Previous_Estimated;

% Compute the gain vector L_s and the covariance matrix P_s:
L_S_Gain_Vector_Current = (P_s_Previous_Estimated*Phi_S_Current)/(1 + Phi_S_Transpose_Current*P_s_Previous_Estimated*Phi_S_Current);
P_S_Covariance_Matrix_Current = (eye(2) - L_S_Gain_Vector_Current*Phi_S_Transpose_Current)*P_s_Previous_Estimated;

% Compute the gain vector L_n and the covariance matrix P_n:
L_N_Gain_Vector_Current = (P_n_Previous_Estimated*Phi_N_Current_Estimated)/( 1 + P_n_Previous_Estimated*Phi_N_Current_Estimated^2);
P_N_Covariance_Matrix_Current = (1 - L_N_Gain_Vector_Current*Phi_N_Current_Estimated)*P_n_Previous_Estimated;

% Update the parameter estimates Theta_S and Theta_N:
Theta_S_Current_Estimate = Theta_S_Previous_Estimate + L_S_Gain_Vector_Current*(Measurement_Current - Phi_N_Current_Estimated*Theta_N_Previous_Estimate - ...
                           Phi_S_Transpose_Current*Theta_S_Previous_Estimate);
Theta_N_Current_Estimate = Theta_N_Previous_Estimate + L_N_Gain_Vector_Current*(Measurement_Current - Phi_S_Transpose_Current*Theta_S_Previous_Estimate - ...
                           Phi_N_Current_Estimated*Theta_N_Previous_Estimate);

% Compute v:
Phi_Current_Transpose_Estimated = [Phi_S_Transpose_Current,Phi_N_Current_Estimated];
Theta_Transpose_Current_Estimated = [Theta_S_Current_Estimate',Theta_N_Current_Estimate];
Theta_Current_Estimated = Theta_Transpose_Current_Estimated';
v_Current_Estimated = Measurement_Current - Phi_Current_Transpose_Estimated*Theta_Current_Estimated;

% Persistent update:
Count_Persistent_Update = Count_Persistent_Update + 1;
if Count_Persistent_Update == 1
    Measurement_Previous_Previous = 0;
else
    Measurement_Previous_Previous = Measurement_Previous;
end
Measurement_Previous = Measurement_Current;
Theta_S_Previous_Estimate = Theta_S_Current_Estimate;
Theta_N_Previous_Estimate = Theta_N_Current_Estimate;
P_s_Previous_Estimated = P_S_Covariance_Matrix_Current;
P_n_Previous_Estimated = P_N_Covariance_Matrix_Current;
v_Previous_Estimated = v_Current_Estimated;

% Compute the wave characteristics:
Theta_S_Current_Estimate_Transpose = Theta_S_Current_Estimate';
a_1 = Theta_S_Current_Estimate_Transpose(1);
a_2 = Theta_S_Current_Estimate_Transpose(2);

% Solve the second order equation:
Delta = complex(a_1^2 - 4*a_2);
Z_1 = complex(0);
Z_2 = complex(0);
if Delta > 0
    Z_1 = (-a_1 + sqrt(Delta))/(2);
    Z_2 = (-a_1 - sqrt(Delta))/(2);
elseif Delta == 0
    Z_1 = -a_1/(2);
    Z_2 = Z_1;
elseif Delta < 0
    Z_1 = (-a_1 + 1i*sqrt(abs(Delta)))/(2);
    Z_2 = (-a_1 - 1i*sqrt(abs(Delta)))/(2);
end

s_1 = (1/Time_Sampling)*log(Z_1);
s_2 = (1/Time_Sampling)*log(Z_2);

Alpha = -real(s_1);
Beta = imag(s_1);

Alphas2 = -real(s_2);
Beta2 = -imag(s_2);

Wave_Pulsation_Estimated = sqrt(Alpha^2 + Beta^2);
Wave_Damping_Factor_Estimated = Alpha/Wave_Pulsation_Estimated;