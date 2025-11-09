function [Final_State_Matrix,...
          Final_Input_Matrix,...
          Final_Measurement_Matrix,...
          Final_Input_Measurement_Matrix,...
          Final_State_Space_Global,...
          State_Matrix_Global_Analytical,...
          Input_Matrix_Global_Analytical,...
          Measurement_Matrix_Global_Analytical,...
          Input_Measurement_Matrix_Global_Analytical,...
          Final_Global_Analytical_State_Space] = State_Space_Uncertain_Calculation(Omega_Wave_Pulsation_Uncertain,...
                                                                                   Damping_Wave_Uncertain,...
                                                                                   Static_Gain_Ship_Uncertain,...
                                                                                   Time_Constant_Ship_Uncertain,...
                                                                                   Gain_Static_Filter_1,...
                                                                                   Gain_Infinity_Filter_1,...
                                                                                   Pulsation_Unit_Gain_Filter_1,...
                                                                                   Gain_Static_Filter_2,...
                                                                                   Gain_Infinity_Filter_2,...
                                                                                   Pulsation_Unit_Gain_Filter_2,...
                                                                                   Gain_Static_Filter_3,...
                                                                                   Gain_Infinity_Filter_3,...
                                                                                   Pulsation_Unit_Gain_Filter_3,...
                                                                                   Gain_Static_Filter_4)

% This function serves to determine analytically, the state-space form of
% the Simulink model 'Standard_H_Infinity_Full_Order_Synthesis'.

% Ship Wave System creation:
State_Matrix_Ship_Wave = [0 1 0 0;
                          -Omega_Wave_Pulsation_Uncertain^2 -2*Omega_Wave_Pulsation_Uncertain*Damping_Wave_Uncertain 0 0;
                          0 0 0 1;
                          0 0 0 -1/Time_Constant_Ship_Uncertain];
Perturbation_Input_Matrix_Nominal = [0;Omega_Wave_Pulsation_Uncertain^2;0;0];
Input_Matrix_Nominal = [0;0;0;Static_Gain_Ship_Uncertain/Time_Constant_Ship_Uncertain];
Input_Matrix_Ship_Wave = [Input_Matrix_Nominal Perturbation_Input_Matrix_Nominal];
Measurement_Matrix_Ship_Wave = [0 1 1 0];
Input_Measurement_Matrix_Ship_Wave = [0 0];

System_Ship_Wave = ss(State_Matrix_Ship_Wave,...
                      Input_Matrix_Ship_Wave,...
                      Measurement_Matrix_Ship_Wave,...
                      Input_Measurement_Matrix_Ship_Wave,...
                     'StateName',{'State_Ship_Wave_Rate_Waves','State_Ship_Wave_Heading_Waves','State_Ship_Wave_Heading_Ship','State_Ship_Wave_Rate_Heading'}, ...
                     'InputName',{'Input_Ship_Wave_1', 'Input_Ship_Wave_2'}, ...
                     'OutputName',{'Output_Heading_Total_Ship_Wave'});

% Definition of the filter n°1:
Numerator_Filter_1 = Gain_Static_Filter_1*[Gain_Infinity_Filter_1 Pulsation_Unit_Gain_Filter_1];
Denominator_Filter_1 = [Gain_Static_Filter_1 Pulsation_Unit_Gain_Filter_1];
System_Filter_1 = tf(Numerator_Filter_1, Denominator_Filter_1);
System_Filter_1 = ss(System_Filter_1,'minimal');
System_Filter_1.InputName  = {'Epsilon_Error_1'};
System_Filter_1.OutputName = {'Signal_Filter_1'};
System_Filter_1.StateName  = {'Filter_Number_1_W1'};

% Definition of the filter n°2:
Numerator_Filter_2 = Gain_Infinity_Filter_2*[1 Gain_Static_Filter_2*Pulsation_Unit_Gain_Filter_2];
Denominator_Filter_2 = [1 Gain_Infinity_Filter_2*Pulsation_Unit_Gain_Filter_2];
System_Filter_2 = tf(Numerator_Filter_2, Denominator_Filter_2);
System_Filter_2 = ss(System_Filter_2,'minimal');
System_Filter_2.InputName  = {'Input_Rudder'};     
System_Filter_2.OutputName = {'Signal_Filter_2'};
System_Filter_2.StateName  = {'Filter_Number_2_W2'};

% Definition of the filter n°3:
Numerator_Filter_3 = Gain_Infinity_Filter_3*[1 Gain_Static_Filter_3*Pulsation_Unit_Gain_Filter_3];
Denominator_Filter_3 = [1 Gain_Infinity_Filter_3*Pulsation_Unit_Gain_Filter_3];
System_Filter_3 = tf(Numerator_Filter_3, Denominator_Filter_3);
System_Filter_3 = ss(System_Filter_3,'minimal');
System_Filter_3.InputName  = {'Perturbation_Number_3'};
System_Filter_3.OutputName = {'Filtered_Perturbation_3'};
System_Filter_3.StateName  = {'Filter_Number_3_W3'};

% Definition of the filter n°4:
System_Filter_4 = ss(Gain_Static_Filter_4,'minimal');
System_Filter_4.InputName  = {'Perturbation_Number_4'};
System_Filter_4.OutputName = {'Gain4_Output'};

% Connection logic definition:
System_Ship_Wave.InputName  = {'Input_Ship_Wave_1','Input_Ship_Wave_2'};
System_Ship_Wave.OutputName = {'Output_Heading_Total_Ship_Wave'};
Sum_Epsilon_1 = sumblk('Epsilon_Error_1 = Heading_Aimed - Output_Heading_Total_Ship_Wave');
Sum_Epsilon_2 = sumblk('Epsilon_Prime_Error_2 = Gain4_Output - Output_Heading_Total_Ship_Wave');
Sum_Input_1 = sumblk('Input_Ship_Wave_1 = Input_Rudder');
Sum_Input_2 = sumblk('Input_Ship_Wave_2 = Filtered_Perturbation_3');

% Connection of the blocks:
AllBlocks = connect(System_Ship_Wave, System_Filter_1, System_Filter_2, System_Filter_3, System_Filter_4, ...
                    Sum_Input_1, Sum_Input_2, Sum_Epsilon_1, Sum_Epsilon_2, ...
                    {'Heading_Aimed', 'Perturbation_Number_3', 'Perturbation_Number_4', 'Input_Rudder'}, ...
                    {'Signal_Filter_1', 'Signal_Filter_2', 'Epsilon_Error_1', 'Epsilon_Prime_Error_2'});

% Final State-space matrices:
[State_Matrix_Global,...
 Input_Matrix_Global,...
 Measurement_Matrix_Global,...
 Input_Measurement_Matrix_Global] = ssdata(AllBlocks);

% Order to be applyed desired:
StateNames = {'State_Ship_Wave_Rate_Waves','State_Ship_Wave_Heading_Waves','State_Ship_Wave_Heading_Ship','State_Ship_Wave_Rate_Heading', ...
              'Filter_Number_1_W1','Filter_Number_2_W2','Filter_Number_3_W3'};
InputNames  = {'Heading_Aimed','Perturbation_Number_3','Perturbation_Number_4','Input_Rudder'};
OutputNames = {'Signal_Filter_1','Signal_Filter_2','Epsilon_Error_1','Epsilon_Prime_Error_2'};

% Construction of the final ss model:
Final_State_Space_Global = ss(State_Matrix_Global,...
                              Input_Matrix_Global,...
                              Measurement_Matrix_Global,...
                              Input_Measurement_Matrix_Global,...
                              'StateName', StateNames,...
                              'InputName', InputNames,...
                              'OutputName', OutputNames);

% Outputs:
Final_State_Matrix = Final_State_Space_Global.A;
Final_Input_Matrix = Final_State_Space_Global.B;
Final_Measurement_Matrix = Final_State_Space_Global.C;
Final_Input_Measurement_Matrix = Final_State_Space_Global.D;

% Global analytical state-space system:
% Filter n°1:
State_Matrix_Filter_1 = -Pulsation_Unit_Gain_Filter_1/Gain_Static_Filter_1;
Input_Matrix_Filter_1 = 1;
Measurement_Matrix_Filter_1 = Pulsation_Unit_Gain_Filter_1 - ((Pulsation_Unit_Gain_Filter_1/Gain_Static_Filter_1)*Gain_Infinity_Filter_1);
Input_Measurement_Matrix_Filter_1 = Gain_Infinity_Filter_1;
% Filter n°2:
State_Matrix_Filter_2 = -Gain_Infinity_Filter_2*Pulsation_Unit_Gain_Filter_2;
Input_Matrix_Filter_2 = 1;
Measurement_Matrix_Filter_2 = Gain_Static_Filter_2*Gain_Infinity_Filter_2*Pulsation_Unit_Gain_Filter_2 - Gain_Infinity_Filter_2*Pulsation_Unit_Gain_Filter_2*Gain_Infinity_Filter_2;
Input_Measurement_Matrix_Filter_2 = Gain_Infinity_Filter_2;
% Filter n°3:
State_Matrix_Filter_3 = -Gain_Infinity_Filter_3*Pulsation_Unit_Gain_Filter_3;
Input_Matrix_Filter_3 = 1;
Measurement_Matrix_Filter_3 = Gain_Static_Filter_3*Gain_Infinity_Filter_3*Pulsation_Unit_Gain_Filter_3 - Gain_Infinity_Filter_3*Pulsation_Unit_Gain_Filter_3*Gain_Infinity_Filter_3;
Input_Measurement_Matrix_Filter_3 = Gain_Infinity_Filter_3;
% Filter n°4:
Gain_Filter_4 = Gain_Static_Filter_4;

% State-space analytical definition:
State_Matrix_Global_Analytical = [0 1 0 0 0 0 0;
                                  -Omega_Wave_Pulsation_Uncertain^2 -2*Omega_Wave_Pulsation_Uncertain*Damping_Wave_Uncertain 0 0 0 0 Omega_Wave_Pulsation_Uncertain^2*Measurement_Matrix_Filter_3;
                                  0 0 0 1 0 0 0;
                                  0 0 0 -1/Time_Constant_Ship_Uncertain 0 0 0;
                                  0 -Input_Matrix_Filter_1 -Input_Matrix_Filter_1 0 State_Matrix_Filter_1 0 0;
                                  0 0 0 0 0 State_Matrix_Filter_2 0;
                                  0 0 0 0 0 0 State_Matrix_Filter_3];

Input_Matrix_Global_Analytical = [0 0 0 0;
                                  0 Omega_Wave_Pulsation_Uncertain^2*Input_Measurement_Matrix_Filter_3 0 0;
                                  0 0 0 0;
                                  0 0 0 Static_Gain_Ship_Uncertain/Time_Constant_Ship_Uncertain;
                                  Input_Matrix_Filter_1 0 0 0;
                                  0 0 0 Input_Matrix_Filter_2;
                                  0 Input_Matrix_Filter_3 0 0];

Measurement_Matrix_Global_Analytical = [0 -Input_Measurement_Matrix_Filter_1 -Input_Measurement_Matrix_Filter_1 0 Measurement_Matrix_Filter_1 0 0;
                                        0 0 0 0 0 Measurement_Matrix_Filter_2 0;
                                        0 -1 -1 0 0 0 0;
                                        0 -1 -1 0 0 0 0];

Input_Measurement_Matrix_Global_Analytical = [Input_Measurement_Matrix_Filter_1 0 0 0;
                                              0 0 0 Input_Measurement_Matrix_Filter_2;
                                              1 0 0 0;
                                              0 0 Gain_Filter_4 0];

% Global analytical state-space:
Final_Global_Analytical_State_Space = ss(State_Matrix_Global_Analytical,...
                                         Input_Matrix_Global_Analytical,...
                                         Measurement_Matrix_Global_Analytical,...
                                         Input_Measurement_Matrix_Global_Analytical);


end