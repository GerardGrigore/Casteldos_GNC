% This script serves as a mean to lower the order of the corrector after
% the H infinity standard synthesis of the controller.

% Initial corrector order reduction:
Order_Corrector_Aimed = 3;
figure;
bode(Controller); 
hold on;
[Equilibred_State_Equivalent_System,Diagonal_Vector_Hankel_Singular_Values] = balreal(Controller);

% Note that the reduced controller is obtained by only keeping the states
% for which the singular Hankel values are the highest. By doing so, the
% controller keeps the most important features of the original controller.
Equilibred_State_Equivalent_System_Initial = modred(Equilibred_State_Equivalent_System,Order_Corrector_Aimed+1:size(Equilibred_State_Equivalent_System.a,1));
bode(Equilibred_State_Equivalent_System_Initial);
% Reduced controller.
disp('Reduction of order done.')

% Compagnon-form of the resulting reduced controller:
Compagnon_Form_Controller_Reduced = canon(Equilibred_State_Equivalent_System_Initial,'modal');
bode(Compagnon_Form_Controller_Reduced);
% Compagnon form of the reduced controller

% Initialization of H_Infinity hinfstruct:
Blocks_Definition = tunableSS('corst',Compagnon_Form_Controller_Reduced,'tridiag');
Controller_Initial = ss(ss(Blocks_Definition).a,ss(Blocks_Definition).b,ss(Blocks_Definition).c,ss(Blocks_Definition).d);
bode(Controller_Initial);

% Controller_Calculation:
[State_Matrix_Plant_After_Reduction,...
 Input_Matrix_Plant_After_Reduction,...
 Measurement_Matrix_Plant_After_Reduction,...
 Input_Measurement_Matrix_After_Reduction] = linmod('Standard_H_Infinity_Full_Order_Synthesis');
Plant_System = ss(State_Matrix_Plant_After_Reduction,...
                  Input_Matrix_Plant_After_Reduction,...
                  Measurement_Matrix_Plant_After_Reduction,...
                  Input_Measurement_Matrix_After_Reduction);

% Controller synthesis:
[Controller_After_Reduction,...
 Gamma_Controller_After_Reduction,...
 Information_After_Reduction] = hinfstruct(Plant_System,Blocks_Definition);

Corrector_After_Reduction = ss(Controller_After_Reduction);
bode(Corrector_After_Reduction)
legend('Full order corrector',...
       'Reduced corrector (Balanced form)',...
       'Reduced corrector (Modal form)',...
       'Initial Corrector',...
       'Final Corrector');
State_Matrix_Controller_After_Reduction = Corrector_After_Reduction.a; 
Input_Matrix_Controller_After_Reduction = Corrector_After_Reduction.b; 
Measurement_Matrix_Controller_After_Reduction=Corrector_After_Reduction.c; 
Input_Measurement_Matrix_Controller_After_Reduction = Corrector_After_Reduction.d;

% Then use of the newly reduced values in order to relaunch the script
% Frequency_Responses.m and the temporal plots simulations:
State_Matrix_Controller_Nominal = State_Matrix_Controller_After_Reduction;
Input_Matrix_Controller_Nominal = Input_Matrix_Controller_After_Reduction;
Measurement_Matrix_Controller_Nominal = Measurement_Matrix_Controller_After_Reduction;
Input_Measurement_Matrix_Controller_Nominal = Input_Measurement_Matrix_Controller_After_Reduction;
Controller = Corrector_After_Reduction;















