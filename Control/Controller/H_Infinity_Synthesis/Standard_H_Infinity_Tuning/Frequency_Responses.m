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

% Frequency response of Transfer from Heading_Aimed (r) to the error
% epsilon (eps):
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
% aimed in input and the command rudder:
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
title('T_{ur}  et \gamma/W_2');
















