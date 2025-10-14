function Jacobian_State_Process_Matrix = Jacobian_State_Process_Model_Dynamics(Velocity_Previous_Knowing_Previous,...
                                                                               Heading_Previous_Knowing_Previous,...
                                                                               Time_Integration_Euler,...
                                                                               Time_Ship_Constant_Current)

% This function serves to compute the Jacobian associated to the state
% vector of the process model.

% Compute the components of the matrix:
Jacobian_State_44 = 1 - Time_Integration_Euler*(1/Time_Ship_Constant_Current);
Jacobian_State_63 = Velocity_Previous_Knowing_Previous*Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);
Jacobian_State_62 = Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_73 = -Velocity_Previous_Knowing_Previous*Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_72 = Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);

% Compute the Jacobian state process model matrix at the prediction step:
Jacobian_State_Process_Matrix = [1 0 0 0 0 0 0;
                                 Time_Integration_Euler 1 0 0 0 0 0;
                                 0 0 1 Time_Integration_Euler 0 0 0;
                                 0 0 0 Jacobian_State_44 Time_Integration_Euler 0 0;
                                 0 0 0 0 1 0 0;
                                 0 Jacobian_State_62 Jacobian_State_63 0 0 1 0;
                                 0 Jacobian_State_72 Jacobian_State_73 0 0 0 1];

end

