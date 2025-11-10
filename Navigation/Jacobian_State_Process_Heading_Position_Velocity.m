function Jacobian_State_Process_Matrix = Jacobian_State_Process_Heading_Position_Velocity(Velocity_Previous_Knowing_Previous,...
                                                                                          Heading_Previous_Knowing_Previous,...
                                                                                          Time_Integration_Euler,...
                                                                                          Time_Ship_Constant_Current)

% Compute the components of the matrix:
Jacobian_State_33 = 1 - Time_Integration_Euler*(1/Time_Ship_Constant_Current);
Jacobian_State_52 = Velocity_Previous_Knowing_Previous*Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);
Jacobian_State_51 = Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_62 = -Velocity_Previous_Knowing_Previous*Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_61 = Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);

% Compute the Jacobian state process model matrix at the prediction step:
Jacobian_State_Process_Matrix = [1 0 0 0 0 0;
                                 0 1 Time_Integration_Euler 0 0 0;
                                 0 0 Jacobian_State_33 Time_Integration_Euler 0 0;
                                 0 0 0 1 0 0;
                                 Jacobian_State_51 Jacobian_State_52 0 0 1 0;
                                 Jacobian_State_61 Jacobian_State_62 0 0 0 1];

end

