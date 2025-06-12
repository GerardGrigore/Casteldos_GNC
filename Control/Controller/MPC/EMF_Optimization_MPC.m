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
% f_Transpose = 0.5*(W*x_aug - r_p)'*Q*Z;
% f = f_Transpose';
f = Z' * Q * (W * x_aug - r_p);

%f = reshape(f, [], 1);
% X_0 = x_aug;
disp("Size of H:"); disp(size(H))
disp("Size of f:"); disp(size(f))

disp("---- MPC Debug ----");
disp("Size H:"), disp(size(H))
disp("Size f:"), disp(size(f))
disp("Size A_C1:"), disp(size(A_C1))
disp("Size b_C1:"), disp(size(b_C1))


if isempty(f)
    disp("f est vide !")
end
if isempty(H)
    disp("H est vide !")
end




% QP Optimization:
% options = optimoptions('quadprog','Display','off','Algorithm','active-set');
% min 0.5*x^T*H*x + f^T*x
%options = optimoptions('quadprog','Display','off','Algorithm','interior-point');
X_0 = zeros(size(H,1), 1);
disp("Size X_0:"), disp(size(X_0))
options = optimoptions('quadprog','Display','off','Algorithm','active-set');
[Rudder_Optimized_Sequence, ~, exitflag] = quadprog(H, f', A_C1, b_C1, [], [], [], [], X_0, options);


% Output update:
Rudder_Input_Sequence_Optimized = Rudder_Optimized_Sequence;
Rudder_Input = Rudder_Input_Sequence_Optimized(1);

if isempty(f) || isempty(H)
    Rudder_Optimized_Sequence = zeros(size(H,1),1);
    Rudder_Input = 0;
    return;
end


end

