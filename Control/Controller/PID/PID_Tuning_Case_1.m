% This PID tuning is based on the Guidance & Control method and uses the
% usual techniques of 2-nd order system identification and parameter
% determination to make the system to be stable.
% For a complete and detailed analysis of the choice of the parameter,
% refer to the concerned section of the attached technical performance
% note.

% Choose natural pulsation to be higher than the inverse of the time constant:
Pulsation_Natural = 10;

% Define the damping factor:
% Damping_Factor = 0.82;
Damping_Factor = 0.9;

% Closed-Loop frequency condition for stability:
Pulsation_Closed_Loop_Zero_Decibels = Pulsation_Natural*sqrt(1 - 2*Damping_Factor^2 + sqrt(4*Damping_Factor^4 - ...
                                      4*Damping_Factor^2 + 2));

% Security check for the natural pulsation:
if ~(Pulsation_Closed_Loop_Zero_Decibels > 1/Time_Constant_Total)
    error('The natural pulsation shall be grater than the inverse of the time constant.');
end
warning('When there will be a rudder model, need to tune the pulsation to be lower than the rudder pulsation.');

% Define the gains:
Gain_Proportional_Controller = (Time_Constant_Total*Pulsation_Natural^2)/Static_Gain_Nomoto;
Gain_Derivative_Controller = ((2*Time_Constant_Total*Pulsation_Natural*Damping_Factor) - 1)/Static_Gain_Nomoto;

% Define the transfer function of the controller:
Controller_Proportional_Derivative = Gain_Proportional_Controller + Gain_Derivative_Controller*s;

% Then define the overall open-loop functions:
Open_Loop_Transfer_Proportional_Derivative_Ship_Model = Controller_Proportional_Derivative*Heading_On_Rudder_Transfer_First_Order;
Open_Loop_Transfer_Proportional_Ship_Model = Gain_Proportional_Controller*Heading_On_Rudder_Transfer_First_Order;

% Monitor the stability of the open-loop using this controller:
figure;
bode(Heading_On_Rudder_Transfer_First_Order);
hold on;
bode(Open_Loop_Transfer_Proportional_Derivative_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the PD Controller and the ship first order');
title('Transfer function Open-Loop comparison on Bode diagrams');
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Open_Loop_Transfer_Proportional_Derivative_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the PD Controller and the ship first order');
title('Transfer function Open-Loop comparison on Nichols diagrams');

% Additionally, observe the effect of the proportional control alone:
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Open_Loop_Transfer_Proportional_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the P Controller and the ship first order');
title('Transfer function Open-Loop comparison on Nichols diagrams');

% Add the integral action to compensate for the error due to wind, current and waves contributions:
Gain_Integral_Controller = (Pulsation_Natural*Gain_Proportional_Controller)/10;
% Gain_Integral_Controller = 0;

% Build the entire controller:
Controller_PD = Gain_Proportional_Controller + Gain_Derivative_Controller*s;
Controller_PID = Gain_Proportional_Controller + Gain_Derivative_Controller*s + Gain_Integral_Controller/s;

% Then define the overall transfer function of the open loop:
Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model = Controller_PID*Heading_On_Rudder_Transfer_First_Order;

% Plot for stability monitoring:
figure;
bode(Heading_On_Rudder_Transfer_First_Order);
hold on;
bode(Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the PID Controller and the ship first order');
title('Transfer function Open-Loop comparison on Bode diagrams');
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the PID Controller and the ship first order');
title('Transfer function Open-Loop comparison on Nichols diagrams');

% Test of adding a filtering component such that the filter frequency will
% Then define the filter:
Frequency_Filter = 10*19;
Filter_Corrector = 1/(1 + (1/Frequency_Filter)*s);

% See the stability:
figure;
bode(Heading_On_Rudder_Transfer_First_Order);
hold on;
bode(Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model);
hold on;
bode(Filter_Corrector*Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the PID Controller and the ship first order',...
       'PIDF controller and plant open-loop');
title('Transfer function Open-Loop comparison on Bode diagrams');
figure;
nichols(Heading_On_Rudder_Transfer_First_Order);
hold on;
nichols(Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model);
hold on;
nichols(Filter_Corrector*Open_Loop_Transfer_Proportional_Integral_Derivative_Ship_Model);
legend('Transfer function of the ship alone Nomoto first order',...
       'Open-Loop transfer function of the PID Controller and the ship first order',...
       'PIDF controller and plant open-loop');
title('Transfer function Open-Loop comparison on Nichols diagrams');

% Convert the PD and PID functions to discrete transfer functions:
Time_Period_Sampling = 1;
Proportional_Derivative_Controller_Tustin = c2d(Controller_PD,Time_Period_Sampling,'tustin');
Proportional_Integral_Derivative_Tustin = c2d(Controller_PID,Time_Period_Sampling,'tustin');
Ship_Discrete_Tustin = c2d(Heading_On_Rudder_Transfer_First_Order,Time_Period_Sampling,'tustin');









