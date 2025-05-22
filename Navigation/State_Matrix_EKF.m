function State_Matrix_Current = State_Matrix_EKF(Heading_Low_Frequency_Previous_Apriori,...
                                                 Velocity_Ship_Previous_Apriori,...
                                                 Time_Sampling,...
                                                 Pulsation_Wave,...
                                                 Damping_Wave,...
                                                 Time_Constant_Ship)

% This function is used in Simulator_Case_4 and further versions as a mean
% to estimate the State Matrix (usually referred to as 'A' or 'F'
% linearized State Matrix) of the estimation problem.

State_Matrix_Current = [1 0 Time_Sampling*sin(Heading_Low_Frequency_Previous_Apriori) 0 0 Time_Sampling*Velocity_Ship_Previous_Apriori*cos(Heading_Low_Frequency_Previous_Apriori) 0 0;
                        0 1 Time_Sampling*cos(Heading_Low_Frequency_Previous_Apriori) 0 0 -Time_Sampling*Velocity_Ship_Previous_Apriori*sin(Heading_Low_Frequency_Previous_Apriori) 0 0;
                        0 0 1 0 0 0 0 0;
                        0 0 0 1 Time_Sampling 0 0 0;
                        0 0 0 -Time_Sampling*Pulsation_Wave^2 1-2*Pulsation_Wave*Damping_Wave*Time_Sampling 0 0 0;
                        0 0 0 0 0 1 Time_Sampling 0;
                        0 0 0 0 0 0 1-(Time_Sampling/Time_Constant_Ship) Time_Sampling;
                        0 0 0 0 0 0 0 1];

end

