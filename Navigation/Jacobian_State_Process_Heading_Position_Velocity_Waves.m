function Jacobian_State_Process_Matrix = Jacobian_State_Process_Heading_Position_Velocity_Waves(Velocity_Previous_Knowing_Previous,...
                                                                                                Heading_Previous_Knowing_Previous,...
                                                                                                Time_Integration_Euler,...
                                                                                                Time_Ship_Constant_Current,...
                                                                                                Pulsation_Wave,...
                                                                                                Damping_Wave)

% Compute the components of the matrix:
Jacobian_State_44 = 1 - Time_Integration_Euler*(1/Time_Ship_Constant_Current);
Jacobian_State_63 = Velocity_Previous_Knowing_Previous*Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);
Jacobian_State_68 = Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_73 = -Velocity_Previous_Knowing_Previous*Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_78 = Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);

% Compute the Jacobian state process model matrix at the prediction step:
Jacobian_State_Process_Matrix = [0 Time_Integration_Euler 0 0 0 0 0 0;
                                 -Time_Integration_Euler*Pulsation_Wave^2 1-2*Damping_Wave*Pulsation_Wave*Time_Integration_Euler 0 0 0 0 0 0;
                                 0 0 1 Time_Integration_Euler 0 0 0 0;
                                 0 0 0 Jacobian_State_44 Time_Integration_Euler 0 0 0;
                                 0 0 0 0 1 0 0 0;
                                 0 0 Jacobian_State_63 0 0 1 0 Jacobian_State_68;
                                 0 0 Jacobian_State_73 0 0 0 1 Jacobian_State_78;
                                 0 0 0 0 0 0 0 1];

end

