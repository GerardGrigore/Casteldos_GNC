function State_Vector_Current = Function_Process_Model(Position_X_Previous,Position_Y_Previous,...
                                                       Psi_Heading_Previous,Velocity_Previous,...
                                                       Time_Step_HW)

% This function models the non linear function that reprezents the
% kinematic of a surface moving ship on a 2D plane.

Position_X_Current = Position_X_Previous + Velocity_Previous*Time_Step_HW*cos(Psi_Heading_Previous);
Position_Y_Current = Position_Y_Previous + Velocity_Previous*Time_Step_HW*sin(Psi_Heading_Previous);
Psi_Heading_Current = Psi_Heading_Previous;
Velocity_Current = Velocity_Previous;
State_Vector_Current = [Position_X_Current;Position_Y_Current;Psi_Heading_Current;Velocity_Current];

end

