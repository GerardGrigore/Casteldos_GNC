% This script serves as a mean to plot the various temporal responses
% associated to the synthesis of the controller using standard H infinity
% method.
% Use of the Simulink closed loop model
% 'Ship_Control_Loop_H_Infinity_Standard'.

% Choice of an horizon an time step:
Time_Final = 100;
Time_Considered = 0:0.02:Time_Final;

% Response to the Commanded Heading:
figure;
Closed_Loop_System_1 = ss(State_Matrix_Closed_Loop_Nominal,...
                          Input_Matrix_Closed_Loop_Nominal(:,1),...
                          Measurement_Matrix_Closed_Loop_Nominal([3 2],:),...
                          Input_Measurement_Matrix_Closed_Loop_Nominal([3 2],1));
set(Closed_Loop_System_1,'OutputName',{'Output Heading Total','Control Input Rudder'});
step(Step_Heading_Aimed*Closed_Loop_System_1*180/pi,Type,Time_Considered); 
hold on;
title('Response to a reference Heading Step.');

% Response to the perturbation (second input):
figure;
Closed_Loop_System_2 = ss(State_Matrix_Closed_Loop_Nominal,...
                          Perturbation_Step*Input_Matrix_Closed_Loop_Nominal(:,2),...
                          Measurement_Matrix_Closed_Loop_Nominal([3 2],:),...
                          Perturbation_Step*Input_Measurement_Matrix_Closed_Loop_Nominal([3 2],2));
set(Closed_Loop_System_2,'OutputName',{'Output Command Rudder','Perturbation input'});
step(Closed_Loop_System_2*180/pi,Type,Time_Considered); 
hold on;
title('Response to a perturbation step.');

disp('  ')
disp('Eigen values of the closed loop :')
disp('---------------------------------')
damp(eig(State_Matrix_Closed_Loop_Nominal))

% Response of the Command rudder input delta to the noise measure:
Time_Interval_Noise = 0.02; Horizon = Time_Final;
Time_Noise = out.Noise_Heading_Generated.Time;
Noise_Heading_Generated = out.Noise_Heading_Generated.Data;
figure;
Close_Loop_System_3 = ss(State_Matrix_Closed_Loop_Nominal,...
                         Input_Matrix_Closed_Loop_Nominal(:,3),...
                         Measurement_Matrix_Closed_Loop_Nominal(2,:),...
                         Input_Measurement_Matrix_Closed_Loop_Nominal(2,3));
Time_Response_Dynamic_System = lsim(Close_Loop_System_3,Noise_Heading_Generated,Time_Noise);
plot(Time_Noise,Time_Response_Dynamic_System,Type); 
hold on;
title('Response of the commanded input to the measurement noise.');
xlabel('Time (s)');














