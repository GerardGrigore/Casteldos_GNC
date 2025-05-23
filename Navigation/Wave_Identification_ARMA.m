function [Wave_Pulsation_Estimated,...
          Wave_Damping_Factor_Estimated] = Wave_Identification_ARMA(Heading_Wave_High_Frequency_Estimated,...
                                                                    Time_Sampling)

% Persistents definition:
persistent Parameters_To_Estimate_Left_Previous_Estimate;
persistent Parameters_To_Estimate_Right_Previous_Estimate;
persistent Covariance_Matrix_Left_Previous_Estimate;
persistent Covariance_Matrix_Right_Previous_Estimate;
persistent Measurement_Previous;
persistent Measurement_Previous_Previous;
persistent Count_Persistent_Update;
persistent White_Noise_Process_Previous_Estimate;

% Initialization of the algorithm:
Covariance_Initial_Value = 10e6;
if isempty(Parameters_To_Estimate_Left_Previous_Estimate)
    Parameters_To_Estimate_Left_Previous_Estimate = [1/Covariance_Initial_Value;1/Covariance_Initial_Value];
end
if isempty(Parameters_To_Estimate_Right_Previous_Estimate)
    Parameters_To_Estimate_Right_Previous_Estimate = 1/Covariance_Initial_Value;
end
if isempty(Covariance_Matrix_Left_Previous_Estimate)
    Covariance_Matrix_Left_Previous_Estimate = Covariance_Initial_Value*eye(2);
end
if isempty(Covariance_Matrix_Right_Previous_Estimate)
    Covariance_Matrix_Right_Previous_Estimate = Covariance_Initial_Value;
end
if isempty(Measurement_Previous)
    Measurement_Previous = 0;
end
if isempty(Measurement_Previous_Previous)
    Measurement_Previous_Previous = 0;
end
if isempty(White_Noise_Process_Previous_Estimate)
    White_Noise_Process_Previous_Estimate = 0;
end
if isempty(Count_Persistent_Update)
    Count_Persistent_Update = 0;
end

% Collect the output data:
Measurement_Current = Heading_Wave_High_Frequency_Estimated;

% Compute the information vector and the estimated one:
Information_Left_Vector_Transpose_Current = [-Measurement_Previous,-Measurement_Previous_Previous];
Information_Left_Vector_Current = Information_Left_Vector_Transpose_Current';
Information_Right_Vector_Current_Estimated = White_Noise_Process_Previous_Estimate;

% Compute the first case gain vector and covariance matrix:
Gain_Vector_Left_Current = (Covariance_Matrix_Left_Previous_Estimate*Information_Left_Vector_Current)/(1 + Information_Left_Vector_Transpose_Current*Covariance_Matrix_Left_Previous_Estimate*Information_Left_Vector_Current);
Covariance_Matrix_Left_Current = (eye(2) - Gain_Vector_Left_Current*Information_Left_Vector_Transpose_Current)*Covariance_Matrix_Left_Previous_Estimate;

% Compute the second case gain vector covariance matrix:
Gain_Vector_Right_Current = (Covariance_Matrix_Right_Previous_Estimate*Information_Right_Vector_Current_Estimated)/( 1 + Covariance_Matrix_Right_Previous_Estimate*Information_Right_Vector_Current_Estimated^2);
Covariance_Matrix_Right_Current = (1 - Gain_Vector_Right_Current*Information_Right_Vector_Current_Estimated)*Covariance_Matrix_Right_Previous_Estimate;

% Update the parameter estimates:
Parameter_To_Estimate_Left_Current_Estimate = Parameters_To_Estimate_Left_Previous_Estimate + Gain_Vector_Left_Current*(Measurement_Current - Information_Right_Vector_Current_Estimated*Parameters_To_Estimate_Right_Previous_Estimate - ...
                                              Information_Left_Vector_Transpose_Current*Parameters_To_Estimate_Left_Previous_Estimate);
Parameter_To_Estimate_Right_Current_Estimate = Parameters_To_Estimate_Right_Previous_Estimate + Gain_Vector_Right_Current*(Measurement_Current - Information_Left_Vector_Transpose_Current*Parameters_To_Estimate_Left_Previous_Estimate - ...
                                               Information_Right_Vector_Current_Estimated*Parameters_To_Estimate_Right_Previous_Estimate);

% Compute the white-noise:
Information_Vector_Total_Transpose_Estimated = [Information_Left_Vector_Transpose_Current,Information_Right_Vector_Current_Estimated];
Parameter_To_Estimate_Total_Current_Estimated = [Parameter_To_Estimate_Left_Current_Estimate',Parameter_To_Estimate_Right_Current_Estimate];
Parameter_Total_Estimated = Parameter_To_Estimate_Total_Current_Estimated';
White_Noise_Process_Current_Estimated = Measurement_Current - Information_Vector_Total_Transpose_Estimated*Parameter_Total_Estimated;

% Persistent update:
Count_Persistent_Update = Count_Persistent_Update + 1;
if Count_Persistent_Update == 1
    Measurement_Previous_Previous = 0;
else
    Measurement_Previous_Previous = Measurement_Previous;
end
Measurement_Previous = Measurement_Current;
Parameters_To_Estimate_Left_Previous_Estimate = Parameter_To_Estimate_Left_Current_Estimate;
Parameters_To_Estimate_Right_Previous_Estimate = Parameter_To_Estimate_Right_Current_Estimate;
Covariance_Matrix_Left_Previous_Estimate = Covariance_Matrix_Left_Current;
Covariance_Matrix_Right_Previous_Estimate = Covariance_Matrix_Right_Current;
White_Noise_Process_Previous_Estimate = White_Noise_Process_Current_Estimated;

% Compute the wave characteristics:
Parameter_To_Estimate_Current_Estimate_Transpose = Parameter_To_Estimate_Left_Current_Estimate';
Identification_Parameter_1_ARMA = Parameter_To_Estimate_Current_Estimate_Transpose(1);
Identification_Parameter_2_ARMA = Parameter_To_Estimate_Current_Estimate_Transpose(2);

% Solve the second order equation:
Delta_Discriminant = complex(Identification_Parameter_1_ARMA^2 - 4*Identification_Parameter_2_ARMA);
Root_Discrete_Transform_1 = complex(0);
Root_Discrete_Transform_2 = complex(0);

% Discretization in function of the delta:
if Delta_Discriminant > 0
    Root_Discrete_Transform_1 = (-Identification_Parameter_1_ARMA + sqrt(Delta_Discriminant))/(2);
    Root_Discrete_Transform_2 = (-Identification_Parameter_1_ARMA - sqrt(Delta_Discriminant))/(2);

elseif Delta_Discriminant == 0
    Root_Discrete_Transform_1 = -Identification_Parameter_1_ARMA/(2);
    Root_Discrete_Transform_2 = Root_Discrete_Transform_1;

elseif Delta_Discriminant < 0
    Root_Discrete_Transform_1 = (-Identification_Parameter_1_ARMA + 1i*sqrt(abs(Delta_Discriminant)))/(2);
    Root_Discrete_Transform_2 = (-Identification_Parameter_1_ARMA - 1i*sqrt(abs(Delta_Discriminant)))/(2);

end

% Continuous roots of the equation:
Root_Continuous_Transform_1 = (1/Time_Sampling)*log(Root_Discrete_Transform_1);
Root_Continuous_Transform_2 = (1/Time_Sampling)*log(Root_Discrete_Transform_2);

% Extract the final coefficients:
Real_Part_Continuous_Roots = -real(Root_Continuous_Transform_1);
Imaginary_Part_Continuous_Roots = imag(Root_Continuous_Transform_1);

% Real_Part_Continuous_Roots_2 = -real(Root_Continuous_Transform_2);
% Imaginary_Part_Continuous_Roots_2 = -imag(Root_Continuous_Transform_2);

% Then update the output:
Wave_Pulsation_Estimated = sqrt(Real_Part_Continuous_Roots^2 + Imaginary_Part_Continuous_Roots^2);
Wave_Damping_Factor_Estimated = Real_Part_Continuous_Roots/Wave_Pulsation_Estimated;

end

