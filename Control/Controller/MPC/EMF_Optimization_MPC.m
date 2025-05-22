function [Rudder_Input_Sequence_Optimized,Rudder_Input] = EMF_Optimization_MPC(Heading_Error,...
                                                                Z,...
                                                                Q,...
                                                                R,...
                                                                W,...
                                                                r_p,...
                                                                x_aug,...
                                                                A_C1,...
                                                                b_C1)

% Creation of the Optimization matrices:
H = Z'*Q*Z + R;
f_Transpose = 0.5*(W*x_aug - r_p)'*Q*Z;
f = f_Transpose';
X_0 = x_aug;

% QP Optimization:
options = optimoptions('quadprog','Display','off','Algorithm','active-set');
% min 0.5*x^T*H*x + f^T*x
[Rudder_Optimized_Sequence,~,exitflag] = quadprog(H,f',A_C1,b_C1,[],[],[],[],X_0,options);

% Output update:
Rudder_Input_Sequence_Optimized = Rudder_Optimized_Sequence;
Rudder_Input = Rudder_Input_Sequence_Optimized(1);

end

