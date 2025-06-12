clear all;
close all;
clc;

% This script serves as a mean to explore the use of H-Infinity synthesis
% to tune the PIDF controller. Here, the Matlab design function hinfstruct
% will be used to tune and optimize the controller.
% As described in the documentation, the control elements are the
% controller and the low pass filter of the indirect chain.
% Note that the Loop-Shape technique will be used here to force the
% frequencial behaviour of the open-loop.

% Definition of the plant:
Static_Gain = -0.6;
Time_Constant = -5.5;
s = tf('s');
Plant_Transfer_Function = Static_Gain/(s*(1 + Time_Constant*s));
Plant_Transfer_Function.u = 'u';
Plant_Transfer_Function.y = 'y';

% Define the structure of the wanted controller:
Controller_PIDF_Structure = tunablePID('Controller_PIDF','pid');

% Define the parameter of the first order low pass filter:
Pulsation_Filter_Initial = 1;
Pulsation_Filter = realp('Pulsation_Filter',Pulsation_Filter_Initial);
Filter_Transfer_Function = Pulsation_Filter/(s + Pulsation_Filter);

% Target loop shape for the open-loop:
Pulsation_Cross_Aimed = 1000;
Loop_Shape_Open_Loop = ((1 + 0.001*(s/Pulsation_Cross_Aimed))/(0.001 + (s/Pulsation_Cross_Aimed)));

% The design requirements are satisfied if the H-Infinity norm of this
% control structure is less than 1.
% Use of the connect command to link each blocks.
% Label of the inputs and outputs blocks:
Weight_Matrix_Noise = 1/Loop_Shape_Open_Loop;
Weight_Matrix_Noise.u = 'nw';
Weight_Matrix_Noise.y = 'n';
Weight_Matrix_Error = Loop_Shape_Open_Loop;
Weight_Matrix_Error.u = 'e';
Weight_Matrix_Error.y = 'ew';
Controller_PIDF_Structure.u = 'e';
Controller_PIDF_Structure.y = 'u';
Filter_Transfer_Function.u = 'yn';
Filter_Transfer_Function.y = 'yf';

% Summing junctions specification:
Sum_1 = sumblk('e = r - yf');
Sum_2 = sumblk('yn = y + n');

% Block connection:
Transfer_To_Optimize = connect(Plant_Transfer_Function,Weight_Matrix_Noise,Weight_Matrix_Error,Controller_PIDF_Structure,...
                               Filter_Transfer_Function,Sum_1,Sum_2,{'r','nw'},{'y','ew'});

% Then use of hinstruct to find some values for the parameters of the
% controller and the filter in order to minimize the H-Infinity norm of the transfer 
% Transfer_To_Optimize.
% Note that the risk of finding local minima can be reduced if the number
% of iterations is increased.
Number_Of_Random_Starts_Iteration = 20;
rng('default');
Optimization_H_Infinity_Options = hinfstructOptions('Display','final','RandomStart',Number_Of_Random_Starts_Iteration);
Transfer_Optimized = hinfstruct(Transfer_To_Optimize,Optimization_H_Infinity_Options);

% To validate the design, plot the tuned open-loop response
% Filter_Transfer_Function*Plant_Transfer_Function*Controller_PIDF_Structure
% and comparison with the aimed loop shape function Loop_Shape_Open_Loop.
% To compute the current Loop_Shaped_Open_Loop, use of getBlockValue to get
% the tuned values of the controller Controller_PIDF_Structure. Use of
% getValue to evaluate the filter Filter_Transfer_Function for the obtained
% and tuned value of Pulsation_Filter:
Controller_PIDF_Tuned = getBlockValue(Transfer_To_Optimize,'Controller_PIDF');
Filter_Low_Pass_Tuned = getValue(Filter_Transfer_Function,Transfer_Optimized.Blocks);
Loop_Shaped_Open_Loop = Plant_Transfer_Function*Controller_PIDF_Tuned*Filter_Low_Pass_Tuned;

% Plot of the Bode chart:
bode(Loop_Shape_Open_Loop,'r--',Loop_Shaped_Open_Loop,'b');
title('Open-loop response'), legend('Aimed loop shape','Actual loop shape')























