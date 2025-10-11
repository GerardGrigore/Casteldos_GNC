function Jacobian_State_Process_Matrix = Jacobian_State_Process_Model_2(Velocity_Previous_Knowing_Previous,...
                                                                        Heading_Previous_Knowing_Previous,...
                                                                        Time_Integration_Euler,Time_Ship_Constant)

% This function serves to compute the Jacobian associated to the state
% vector of the process model.

% Compute the components of the matrix:
Jacobian_State_13 = Velocity_Previous_Knowing_Previous*Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);
Jacobian_State_16 = Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_23 = -Velocity_Previous_Knowing_Previous*Time_Integration_Euler*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_26 = Time_Integration_Euler*cos(Heading_Previous_Knowing_Previous);

% Compute the Jacobian state process model matrix at the prediction step:
Jacobian_State_Process_Matrix = [1 0 Jacobian_State_13 0 0 Jacobian_State_16 0;
                                 0 1 Jacobian_State_23 0 0 Jacobian_State_26 0;
                                 0 0 1 Time_Integration_Euler 0 0 0;
                                 0 0 1-(Time_Integration_Euler/Time_Ship_Constant) Time_Integration_Euler 0 0 0;
                                 0 0 0 0 1 0 0;
                                 0 0 0 0 0 1 Time_Integration_Euler;
                                 0 0 0 0 0 0 1];

end

