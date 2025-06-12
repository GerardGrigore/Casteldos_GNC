clear all;
close all;
clc;

% This script serves as a rapid prototype to tune several kind of
% controller using the pole placement technique. The goal is to
% study the stability of the ship control by observing simulated heading.

% Global parameters:
Static_Gain_Ship = 0.604;
Time_Constant_Ship = -5.5;
s = tf('s');
Time_Period_Sampling = 0.1;
Number_Dispersed_Parameters = 20;
Static_Gain_Ship_Array = linspace(0.2,3,Number_Dispersed_Parameters);
Time_Constant_Ship_Array = linspace(-3,-10,Number_Dispersed_Parameters);

% Transfer function:
Rudder_To_Heading_Continuous = Static_Gain_Ship/(s*(1 + Time_Constant_Ship*s));
Rudder_To_Heading_Discrete = c2d(Rudder_To_Heading_Continuous,Time_Period_Sampling,'tustin');
% Array transfer functions in function of the number of wanted model:
for index_models_number = 1:Number_Dispersed_Parameters
    Rudder_To_Heading_Continuous_Array(index_models_number) = Static_Gain_Ship_Array(index_models_number)/...
                                                              (s*(1 + Time_Constant_Ship_Array(index_models_number)*s));
    Rudder_To_Heading_Discrete(index_models_number) = c2d(Rudder_To_Heading_Continuous_Array(index_models_number),Time_Period_Sampling,'tustin');
end

% Pulsation_Natural = 0.5;
% Damping_Factor = 1;
Pulsation_Natural = 0.3;
Pulsation_Natural_Array = linspace(0.3,5,Number_Dispersed_Parameters);
Damping_Factor = 1.5;
Damping_Factor_Array = linspace(0.5,1.5,Number_Dispersed_Parameters);
Pulsation_Ship = 1/Time_Constant_Ship;

% Bandwidth condition checking:
Pulsation_Bandwidth = Pulsation_Natural*(sqrt(1 - 2*Damping_Factor^2 + sqrt(4*Damping_Factor^4 - 4*Damping_Factor^2 + 2)));
if Pulsation_Bandwidth < Pulsation_Ship
    error('Carefull, error of pulsation tuning for the desired closed-loop.');
end

% Controller gains definition:
Gain_Proportional = (Time_Constant_Ship*Pulsation_Natural^2)/Static_Gain_Ship;
Gain_Proportional_Array = (Time_Constant_Ship*Pulsation_Natural_Array.^2)/Static_Gain_Ship;
Gain_Derivative = (2*Damping_Factor*Pulsation_Natural*Time_Constant_Ship - 1)/Static_Gain_Ship;
Gain_Derivative_Array = (2*Damping_Factor_Array.*Pulsation_Natural_Array*Time_Constant_Ship - 1)/Static_Gain_Ship;
Gain_Integral = (Pulsation_Natural^3/2)*(Time_Constant_Ship/Static_Gain_Ship);
Gain_Integral_Array = (Pulsation_Natural_Array.^3/2)*(Time_Constant_Ship/Static_Gain_Ship);
Gain_Filter = 0.01;
Gain_Filter_Array = linspace(0.01,1,Number_Dispersed_Parameters);

% Continuous and discrete controllers definition:
Controller_PID_Continuous = Gain_Proportional + Gain_Integral/s + Gain_Derivative*s;
Controller_PID_Continuous_Array = Gain_Proportional_Array + Gain_Integral_Array/s + Gain_Derivative_Array*s;
Controller_PIDF_Continuous = Gain_Proportional + Gain_Integral/s + ((Gain_Derivative*s)/(1 + Gain_Filter*s));
Controller_PIDF_Continuous_Array = Gain_Proportional_Array + Gain_Integral_Array/s + ((Gain_Derivative_Array*s)/(1 + Gain_Filter*s));
Controller_PID_Discrete = c2d(Controller_PID_Continuous,Time_Period_Sampling,'tustin');
Controller_PID_Discrete_Array = c2d(Controller_PID_Continuous_Array,Time_Period_Sampling,'tustin');
Controller_PIDF_Discrete = c2d(Controller_PIDF_Continuous,Time_Period_Sampling,'tustin');
Controller_PIDF_Discrete_Array = c2d(Controller_PIDF_Continuous_Array,Time_Period_Sampling,'tustin');
Controller_PDF_Continuous = Gain_Proportional + ((Gain_Derivative*s)/(1 + Gain_Filter*s));
Controller_PDF_Continuous_Array = Gain_Proportional_Array + ((Gain_Derivative_Array*s)/(1 + Gain_Filter*s));
Controller_PDF_Discrete =  c2d(Controller_PDF_Continuous,Time_Period_Sampling,'tustin');
Controller_PDF_Discrete_Array =  c2d(Controller_PDF_Continuous_Array,Time_Period_Sampling,'tustin');

% Stability evaluation through Nichols/Bode charts:
figure;
nichols(Rudder_To_Heading_Continuous);
hold on;
for index_dispersed_parameters = 1:Number_Dispersed_Parameters
    hold on;
    nichols(Controller_PIDF_Continuous_Array(index_dispersed_parameters)*Rudder_To_Heading_Continuous);
end
title('Continuous PIDF control robust analysis');

figure;
nichols(Rudder_To_Heading_Continuous);
hold on;
for index_dispersed_parameters = 1:Number_Dispersed_Parameters
    hold on;
    nichols(Controller_PDF_Continuous_Array(index_dispersed_parameters)*Rudder_To_Heading_Continuous);
end
title('Continuous PDF control');

figure;
nichols(Rudder_To_Heading_Discrete);
hold on;
for index_dispersed_parameters = 1:Number_Dispersed_Parameters
    hold on;
    nichols(Controller_PIDF_Discrete_Array(index_dispersed_parameters)*Rudder_To_Heading_Discrete);
    hold on;
    nichols(Controller_PDF_Discrete_Array(index_dispersed_parameters)*Rudder_To_Heading_Discrete);
end
title('Discrete PIDF/PDF control');

figure;
nichols(Rudder_To_Heading_Continuous);
hold on;
for index_dispersed_parameters = 1:Number_Dispersed_Parameters
    hold on;
    nichols(Controller_PIDF_Continuous_Array(index_dispersed_parameters)*Rudder_To_Heading_Continuous);
    hold on;
    nichols(Controller_PID_Continuous_Array(index_dispersed_parameters)*Rudder_To_Heading_Continuous);
    hold on;
    nichols(Controller_PDF_Continuous_Array(index_dispersed_parameters)*Rudder_To_Heading_Continuous);
end
title('Continuous PID/F control');

figure;
bode(Controller_PID_Continuous*Rudder_To_Heading_Continuous);
hold on;
bode(Controller_PIDF_Continuous*Rudder_To_Heading_Continuous);
legend('PID','PIDF');

% Stability analysis of the dispersed plant in function of the current
% tuning of the PIDF controller:
figure;
for index_dispersed_parameters = 1:Number_Dispersed_Parameters
    hold on;
    nichols(Controller_PIDF_Continuous*Rudder_To_Heading_Continuous_Array(index_dispersed_parameters))
end
title('PIDF Continuous controller and dispersed ship - Open loop');

figure;
for index_dispersed_parameters = 1:Number_Dispersed_Parameters
    hold on;
    nichols(Controller_PDF_Continuous*Rudder_To_Heading_Continuous_Array(index_dispersed_parameters))
end
title('PDF Continuous controller and dispersed ship - Open loop');


















