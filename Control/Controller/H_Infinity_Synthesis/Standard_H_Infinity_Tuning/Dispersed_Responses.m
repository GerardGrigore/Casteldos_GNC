% This script serves as a mean to check the Bode and Niquyst diagrams of
% all the dispersed systems using the tuned controller from the H Infinity
% standard synthesis.

close all;

% Dispersed parameters of the ship:
% Ship:
Static_Gain_Ship_Nominal = 0.6043;
Time_Constant_Ship_Nominal = -5.2097;
Static_Gain_Ship_Minimal = 4.9000;
Time_Constant_Ship_Minimal = -385.9724;
Static_Gain_Ship_Maximal = 0.8384;
Time_Constant_Ship_Maximal = -0.3209;
% Waves:
Omega_Wave_Nominal = 0.6;
Omega_Wave_Minimal = 0.3;
Omega_Wave_Maximal = 1.3;
Damping_Wave_Nominal = 0.1;
Damping_Wave_Minimal = 0.09;
Damping_Wave_Maximal = 0.25;
% Dispersions:
Step_Dispersions = 0.1;
Number_Points = 20;
Static_Gain_Dispersions = linspace(0.5,5,Number_Points);
Time_Constant_Ship_Dispersions = linspace(Time_Constant_Ship_Maximal,Time_Constant_Ship_Minimal,Number_Points);
Wave_Pulsation_Dispersions = linspace(Omega_Wave_Minimal,Omega_Wave_Maximal,Number_Points);
Damping_Wave_Dispersions = linspace(Damping_Wave_Minimal,Damping_Wave_Maximal,Number_Points);

s = tf('s');
% Open-loop systemp:
Controller_Function_1 = tf(Controller(1));
Controller_Function_2 = tf(Controller(2));
figure;
for index_dispersed_system = 1:Number_Points
    hold on;
    % Compute the current Ship transfer function:
    Ship_Transfer_Function_Current = Static_Gain_Dispersions(index_dispersed_system)/(s*(1 + Time_Constant_Ship_Dispersions(index_dispersed_system)*s));
    % Open-loop:
    Current_Open_Loop = Ship_Transfer_Function_Current*Controller_Function_1;
    bode(Current_Open_Loop);
end
figure;
for index_dispersed_system = 1:Number_Points
    hold on;
    % Compute the current Ship transfer function:
    Ship_Transfer_Function_Current = Static_Gain_Dispersions(index_dispersed_system)/(s*(1 + Time_Constant_Ship_Dispersions(index_dispersed_system)*s));
    % Open-loop:
    Current_Open_Loop = Ship_Transfer_Function_Current*Controller_Function_1;
    nichols(Current_Open_Loop);
end
figure;
for index_dispersed_system = 1:Number_Points
    hold on;
    % Compute the current Ship transfer function:
    Ship_Transfer_Function_Current = Static_Gain_Dispersions(index_dispersed_system)/(s*(1 + Time_Constant_Ship_Dispersions(index_dispersed_system)*s));
    % Open-loop:
    Current_Open_Loop = Ship_Transfer_Function_Current*Controller_Function_1;
    nyquist(Current_Open_Loop);
end















