function [State_Vector_Model] = EKF_Heading_Position_Waves_Model(Position_X_Ship_Previous,...
                                                                 Position_Y_Ship_Previous,...
                                                                 Velocity_Ship_Previous,...
                                                                 Heading_Rate_Wave_Previous,...
                                                                 Heading_Wave_Previous,...
                                                                 Heading_Low_Frequency_Previous,...
                                                                 Heading_Rate_Ship_Previous,...
                                                                 Bias_Perturbations_Previous,...
                                                                 Time_Sampling,...
                                                                 Pulsation_Wave,...
                                                                 Damping_wave,...
                                                                 Static_Gain_Ship,...
                                                                 Time_Constant_Ship,...
                                                                 Rudder_Angle_Input_Current)

% This function is used in Simulator_Case_4 and further versions as a mean
% to estimate the position of the ship, its heading (by applying sensor
% fusion on the headings coming from two sensors: GPS & compass) and the
% wave-induced heading perturbation.

Position_X_Ship_Current = Position_X_Ship_Previous + Time_Sampling*Velocity_Ship_Previous*sin(Heading_Low_Frequency_Previous);
Position_Y_Ship_Current = Position_Y_Ship_Previous + Time_Sampling*Velocity_Ship_Previous*cos(Heading_Low_Frequency_Previous);
Velocity_Ship_Current = Velocity_Ship_Previous;
Heading_Rate_Wave_Current = Heading_Rate_Wave_Previous + Time_Sampling*Heading_Wave_Previous;
Heading_Wave_Current = Heading_Wave_Previous + Time_Sampling*(-Pulsation_Wave^2*Heading_Rate_Wave_Previous - 2*Pulsation_Wave*...
                       Damping_wave*Heading_Wave_Previous);
Heading_Low_Frequency_Current = Heading_Low_Frequency_Previous + Time_Sampling*Heading_Rate_Ship_Previous;
Heading_Rate_Ship_Current = Heading_Rate_Ship_Previous + Time_Sampling*((-1/Time_Constant_Ship)*Heading_Rate_Ship_Previous + ...
                            (Static_Gain_Ship/Time_Constant_Ship)*Rudder_Angle_Input_Current) + Time_Sampling*Bias_Perturbations_Previous;
Bias_Perturbations_Current = Bias_Perturbations_Previous;

% Update the output vector:
State_Vector_Model = [Position_X_Ship_Current;...
                      Position_Y_Ship_Current;...
                      Velocity_Ship_Current;...
                      Heading_Rate_Wave_Current;...
                      Heading_Wave_Current;...
                      Heading_Low_Frequency_Current;...
                      Heading_Rate_Ship_Current;...
                      Bias_Perturbations_Current];

end

