function Jacobian_State_Process_Matrix = Jacobian_State_Process_Model(Velocity_Previous_Knowing_Previous,...
                                                                      Heading_Previous_Knowing_Previous,...
                                                                      Time_Step_HW)

% This function serves to compute the Jacobian associated to the state
% vector of the process model.

% Compute the components of the matrix:
Jacobian_State_13 = -Velocity_Previous_Knowing_Previous*Time_Step_HW*sin(Heading_Previous_Knowing_Previous);
Jacobian_State_14 = Time_Step_HW*cos(Heading_Previous_Knowing_Previous);
Jacobian_State_23 = Velocity_Previous_Knowing_Previous*Time_Step_HW*cos(Heading_Previous_Knowing_Previous);
Jacobian_State_24 = Time_Step_HW*sin(Heading_Previous_Knowing_Previous);

% Compute the Jacobian state process model matrix at the prediction step:
Jacobian_State_Process_Matrix = [1 0 Jacobian_State_13 Jacobian_State_14;
                                 0 1 Jacobian_State_23 Jacobian_State_24;
                                 0 0 1 0;
                                 0 0 0 1];

end

