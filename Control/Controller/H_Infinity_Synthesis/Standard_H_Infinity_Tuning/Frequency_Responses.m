% This script serves as a mean to analyse the frequency responses of the
% various transfers of the Ship and Waves system. This will be based on the
% Simulink model 'Ship_Control_Loop_H_Infinity_Standard'.

Pulsation_Omega = logspace(-2,3,300);
Type = 'k';
% Filters definition:
% Filter 1:
Numerator_Filter_1 = Gain_Static_Filter_1*[Gain_Infinity_Filter_1 Pulsation_Unit_Gain_Filter_1];
Denominator_Filter_1 = [Gain_Static_Filter_1 Pulsation_Unit_Gain_Filter_1];
Transfer_Function_Filter_1 = tf(Numerator_Filter_1,Denominator_Filter_1);
% Filter 2:
Numerator_Filter_2 = Gain_Infinity_Filter_2*[1 Gain_Static_Filter_2*Pulsation_Unit_Gain_Filter_2];
Denominator_Filter_2 = [1 Gain_Infinity_Filter_2*Pulsation_Unit_Gain_Filter_2];
Transfer_Function_Filter_2 = tf(Numerator_Filter_2,Denominator_Filter_2);
% Filter 3:
Numerator_Filter_3 = Gain_Infinity_Filter_3*[1 Gain_Static_Filter_3*Pulsation_Unit_Gain_Filter_3];
Denominator_Filter_3 = [1 Gain_Infinity_Filter_3*Pulsation_Unit_Gain_Filter_3];
Transfer_Function_Filter_3 = tf(Numerator_Filter_3,Denominator_Filter_3);
% Plots:
figure(1); 
bodemag(Transfer_Function_Filter_1,...
        Transfer_Function_Filter_2,...
        Transfer_Function_Filter_3); 
grid
title('Frequencial ponderations.'); 
legend('Filter 1','Filter 2','Filter 3');

% Considered Heading Aimed value:
Step_Heading_Aimed = 30*(pi/180);
% Perturbation step considered:
Perturbation_Step = 1*(pi/180);
% Perturbation_Step = 10*(pi/180);
% Perturbation measure:
Heading_Variance_Noise = 0.5*(pi/180);
% Heading_Variance_Noise = 10*(pi/180);

% Frequency response of Transfer from Heading_Aimed (r/Psi_Aimed) to the error
% epsilon (Epsilon):
% NOTA - Carefully chose the inputs and outputs ports number since their
% number will be reflected in the choice of the elements of the matrices of
% the state-space reprezentation.
[State_Matrix_Closed_Loop_Nominal,...
 Input_Matrix_Closed_Loop_Nominal,...
 Measurement_Matrix_Closed_Loop_Nominal,...
 Input_Measurement_Matrix_Closed_Loop_Nominal] = linmod('Ship_Control_Loop_H_Infinity_Standard');
Sensibility_Function = ss(State_Matrix_Closed_Loop_Nominal,...
                          Input_Matrix_Closed_Loop_Nominal(:,1),...
                          Measurement_Matrix_Closed_Loop_Nominal(1,:),...
                          Input_Measurement_Matrix_Closed_Loop_Nominal(1,1));
figure(2); 
bodemag(Sensibility_Function,Type,Pulsation_Omega); 
hold on; 
bodemag(Gamma_Optimal/Transfer_Function_Filter_1,'k:',Pulsation_Omega);
legend('Transfer from the Heading Aimed to the Error (Epsilon) - Sensibility Function (S).',...
        'Pattern of the Gamma over the Filter 1.');
title('T_{\epsilonr}  and \gamma/W_1.');

% Frequency response of the transfer function from the reference heading
% aimed (r/Psi_Aimed) in input and the command rudder (u/delta):
Corrector_Times_Sensibility_Function = ss(State_Matrix_Closed_Loop_Nominal,...
                                          Input_Matrix_Closed_Loop_Nominal(:,1),...
                                          Measurement_Matrix_Closed_Loop_Nominal(2,:),...
                                          Input_Measurement_Matrix_Closed_Loop_Nominal(2,1));
figure(3);
bodemag(Corrector_Times_Sensibility_Function,Type,Pulsation_Omega);
hold on;
bodemag(Gamma_Optimal/Transfer_Function_Filter_2,'k:',Pulsation_Omega);
legend('Transfer from the Heading Aimed to the Commanded Rudder (Delta) - Corrector times Sensibility Function (KS).',...
        'Pattern of the Gamma over the Filter 2.');
title('T_{ur}  et \gamma/W_2.');
% With the initial filters tuning; one sees that the gain is important at
% high frequency, hence a very high amplitude of command response to the
% reference aimed heading and a high sensitivity to measure noises.

% Frequency response of the transfer function from the error epsilon
% (Epsilon) to the perturbation input.
Sensibility_Function_Times_Plant = ss(State_Matrix_Closed_Loop_Nominal,...
                                      Input_Matrix_Closed_Loop_Nominal(:,2),...
                                      Measurement_Matrix_Closed_Loop_Nominal(1,:),...
                                      Input_Measurement_Matrix_Closed_Loop_Nominal(1,2));
figure(4); 
bodemag(Sensibility_Function_Times_Plant,Type,Pulsation_Omega); 
hold on; 
bodemag(Gamma_Optimal/Transfer_Function_Filter_1/Transfer_Function_Filter_3,'k:',Pulsation_Omega);
legend('Transfer from the error to the perturbation input in the plant - Sensibility Function times de Plant to control (SG).',...
       'Pattern of the Gamma over the filters 1 and 3.');
title('T_{\epsilon\gamma''}  et \gamma/W_1.W_3.');
% Note that the resonance of the perturbation is very seenable on the
% previous Bode gain plot for the initial filters coefficients. Hence even
% if the commande order is sufficiently well followed, the response to the
% perturbation presents an oscillatory behavior: see the script
% 'Temporal_Responses'.

% Frequency response of the transfer function from the input delta to the
% perturbation input:
Corrector_Sensibility_Plant = ss(State_Matrix_Closed_Loop_Nominal,...
                                 Input_Matrix_Closed_Loop_Nominal(:,2),...
                                 Measurement_Matrix_Closed_Loop_Nominal(2,:),...
                                 Input_Measurement_Matrix_Closed_Loop_Nominal(2,2));
figure(5); 
bodemag(Corrector_Sensibility_Plant,Type,Pulsation_Omega); 
hold on; 
bodemag(Gamma_Optimal/Transfer_Function_Filter_2/Transfer_Function_Filter_3,'k:',Pulsation_Omega);
legend('Transfer function from the input delta to the perturbation input.',...
       'Pattern of the Gamma over the filters 2 and 3.');
title('T_{u\gamma''}  et \gamma/W_2.W_3.')

% Controller Bode diagram:
figure(6)
bode(Controller,Type,Pulsation_Omega); 
hold on; 
grid on
title('Corrector Bode diagram.');

% Open Loop Bode diagram:
figure(7)
Open_Loop_State_Space_System = ss(State_Matrix_Ship_Wave_Nominal,...
                                  Input_Matrix_Nominal,...
                                  [Measurement_Matrix_Ship_Wave_Nominal; ...
                                  0 1 1 0],...
                                  0);
Open_Loop = Controller*Open_Loop_State_Space_System;
bode(Open_Loop,Type,Pulsation_Omega); 
hold on;
title('Open Loop Bode frequency domain.');
% Check to see if algebra was well understood:
Controller_Transfer_Function = tf(Controller);
% Controller of the ship:
Controller_Ship = Controller_Transfer_Function(1);
Controller_Wave = Controller_Transfer_Function(2);
Ship_Transfer_Function_Open_Loop = tf(ss(State_Matrix_Ship_Wave_Nominal,...
                                         Input_Matrix_Nominal_Global,...
                                         [0 1 1 0],...
                                         [0 0]));
% Ship transfer function:
Ship_Transfer_Function = Ship_Transfer_Function_Open_Loop(1);
Wave_Transfer_Function = Ship_Transfer_Function_Open_Loop(2);

% Additional verification:
Feedback_Loop = feedback(1,Open_Loop);
Magnitude = bode(Feedback_Loop);
Delta_Margin = 1/max(Magnitude);












