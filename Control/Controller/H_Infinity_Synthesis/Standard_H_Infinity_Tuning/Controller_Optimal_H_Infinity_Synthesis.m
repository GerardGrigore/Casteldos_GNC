function Rudder_Angle_Elaborated_Current = Controller_Optimal_H_Infinity_Synthesis(Heading_Aimed_Current,...
                                                                                   Heading_Observed_Current,...
                                                                                   Time_Sampling)

% This controller is based on the stochastic optimization done to find
% optimal parameters for the reduced controller of order 3 with two degrees
% of freedom.
% Heading observed persistents:
persistent Heading_Observed_Previous;
persistent Heading_Observed_Previous_Previous;
persistent Heading_Observed_Previous_Previous_Previous;
% Heading aimed persistents:
persistent Heading_Aimed_Previous;
persistent Heading_Aimed_Previous_Previous;
persistent Heading_Aimed_Previous_Previous_Previous;
% Rudder persistents:
persistent Rudder_Transfer_1_Previous;
persistent Rudder_Transfer_1_Previous_Previous;
persistent Rudder_Transfer_1_Previous_Previous_Previous;
persistent Rudder_Transfer_2_Previous;
persistent Rudder_Transfer_2_Previous_Previous;
persistent Rudder_Transfer_2_Previous_Previous_Previous;

% Persistents initialization:
if isempty(Heading_Observed_Previous)
    Heading_Observed_Previous = 0;
    Heading_Observed_Previous_Previous = 0;
    Heading_Observed_Previous_Previous_Previous = 0;
end
if isempty(Heading_Aimed_Previous)
    Heading_Aimed_Previous = 0;
    Heading_Aimed_Previous_Previous = 0;
    Heading_Aimed_Previous_Previous_Previous = 0;
end
if isempty(Rudder_Transfer_1_Previous)
    Rudder_Transfer_1_Previous = 0;
    Rudder_Transfer_1_Previous_Previous = 0;
    Rudder_Transfer_1_Previous_Previous_Previous = 0;
end
if isempty(Rudder_Transfer_2_Previous)
    Rudder_Transfer_2_Previous = 0;
    Rudder_Transfer_2_Previous_Previous = 0;
    Rudder_Transfer_2_Previous_Previous_Previous = 0;
end

% Tustin sampling:
Tustin_Sampling = 2/Time_Sampling;

% Coefficient gains:
Numerator_Coefficient_Controller_1_1 = 4.2512e-05;
Numerator_Coefficient_Controller_1_2 = -1.0304;
Numerator_Coefficient_Controller_1_3 = -1.8663;
Numerator_Coefficient_Controller_1_4 = -0.1164;

Numerator_Coefficient_Controller_2_1 = 6.0170e-05;
Numerator_Coefficient_Controller_2_2 = -2.2543;
Numerator_Coefficient_Controller_2_3 = -3.9369;
Numerator_Coefficient_Controller_2_4 = -6.4214e-05;

Denominator_Coefficient_Controller_1_1 = 1;
Denominator_Coefficient_Controller_1_2 = 3.2588;
Denominator_Coefficient_Controller_1_3 = 3.7817;
Denominator_Coefficient_Controller_1_4 = 1.9916;

Denominator_Coefficient_Controller_2_1 = 1;
Denominator_Coefficient_Controller_2_2 = 3.2588;
Denominator_Coefficient_Controller_2_3 = 3.7817;
Denominator_Coefficient_Controller_2_4 = 1.9916;

% Coefficients calculation:
% First degree of freedom corrector:
Alpha_01 = Numerator_Coefficient_Controller_1_1*Tustin_Sampling^3 + Numerator_Coefficient_Controller_1_2*Tustin_Sampling^2 +...
           Numerator_Coefficient_Controller_1_3*Tustin_Sampling + Numerator_Coefficient_Controller_1_4;
Alpha_11 = -3*Numerator_Coefficient_Controller_1_1*Tustin_Sampling^3 - Numerator_Coefficient_Controller_1_2*Tustin_Sampling^2 +...
           Numerator_Coefficient_Controller_1_3*Tustin_Sampling + 3*Numerator_Coefficient_Controller_1_4;
Alpha_21 = 3*Numerator_Coefficient_Controller_1_1*Tustin_Sampling^3 - Numerator_Coefficient_Controller_1_2*Tustin_Sampling^2 -...
           Numerator_Coefficient_Controller_1_3*Tustin_Sampling + 3*Numerator_Coefficient_Controller_1_4;
Alpha_31 = -Numerator_Coefficient_Controller_1_1*Tustin_Sampling^3 + Numerator_Coefficient_Controller_1_2*Tustin_Sampling^2 -...
           Numerator_Coefficient_Controller_1_3*Tustin_Sampling + Numerator_Coefficient_Controller_1_4;
Beta_01 = Denominator_Coefficient_Controller_1_1*Tustin_Sampling^3 + Denominator_Coefficient_Controller_1_2*Tustin_Sampling^2 +...
          Denominator_Coefficient_Controller_1_3*Tustin_Sampling + Denominator_Coefficient_Controller_1_4;
Beta_11 = -3*Denominator_Coefficient_Controller_1_1*Tustin_Sampling^3 - Denominator_Coefficient_Controller_1_2*Tustin_Sampling^2 +...
          Denominator_Coefficient_Controller_1_3*Tustin_Sampling + 3*Denominator_Coefficient_Controller_1_4;
Beta_21 = 3*Denominator_Coefficient_Controller_1_1*Tustin_Sampling^3 - Denominator_Coefficient_Controller_1_2*Tustin_Sampling^2 -...
          Denominator_Coefficient_Controller_1_3*Tustin_Sampling + 3*Denominator_Coefficient_Controller_1_4;
Beta_31 = -Denominator_Coefficient_Controller_1_1*Tustin_Sampling^3 + Denominator_Coefficient_Controller_1_2*Tustin_Sampling^2 -...
          Denominator_Coefficient_Controller_1_3*Tustin_Sampling + Denominator_Coefficient_Controller_1_4;

% Second degree of freedom corrector:
Alpha_02 = Numerator_Coefficient_Controller_2_1*Tustin_Sampling^3 + Numerator_Coefficient_Controller_2_2*Tustin_Sampling^2 +...
           Numerator_Coefficient_Controller_2_3*Tustin_Sampling + Numerator_Coefficient_Controller_2_4;
Alpha_12 = -3*Numerator_Coefficient_Controller_2_1*Tustin_Sampling^3 - Numerator_Coefficient_Controller_2_2*Tustin_Sampling^2 +...
           Numerator_Coefficient_Controller_2_3*Tustin_Sampling + 3*Numerator_Coefficient_Controller_2_4;
Alpha_22 = 3*Numerator_Coefficient_Controller_2_1*Tustin_Sampling^3 - Numerator_Coefficient_Controller_2_2*Tustin_Sampling^2 -...
           Numerator_Coefficient_Controller_2_3*Tustin_Sampling + 3*Numerator_Coefficient_Controller_2_4;
Alpha_32 = -Numerator_Coefficient_Controller_2_1*Tustin_Sampling^3 + Numerator_Coefficient_Controller_2_2*Tustin_Sampling^2 -...
           Numerator_Coefficient_Controller_2_3*Tustin_Sampling + Numerator_Coefficient_Controller_2_4;
Beta_02 = Denominator_Coefficient_Controller_2_1*Tustin_Sampling^3 + Denominator_Coefficient_Controller_2_2*Tustin_Sampling^2 +...
          Denominator_Coefficient_Controller_2_3*Tustin_Sampling + Denominator_Coefficient_Controller_2_4;
Beta_12 = -3*Denominator_Coefficient_Controller_2_1*Tustin_Sampling^3 - Denominator_Coefficient_Controller_2_2*Tustin_Sampling^2 +...
          Denominator_Coefficient_Controller_2_3*Tustin_Sampling + 3*Denominator_Coefficient_Controller_2_4;
Beta_22 = 3*Denominator_Coefficient_Controller_2_1*Tustin_Sampling^3 - Denominator_Coefficient_Controller_2_2*Tustin_Sampling^2 -...
          Denominator_Coefficient_Controller_2_3*Tustin_Sampling + 3*Denominator_Coefficient_Controller_2_4;
Beta_32 = -Denominator_Coefficient_Controller_2_1*Tustin_Sampling^3 + Denominator_Coefficient_Controller_2_2*Tustin_Sampling^2 -...
          Denominator_Coefficient_Controller_2_3*Tustin_Sampling + Denominator_Coefficient_Controller_2_4;

% Error determination:
Error_Current = Heading_Aimed_Current - Heading_Observed_Current;
Error_Previous = Heading_Aimed_Previous - Heading_Observed_Previous;
Error_Previous_Previous = Heading_Aimed_Previous_Previous - Heading_Observed_Previous_Previous;
Error_Previous_Previous_Previous = Heading_Aimed_Previous_Previous_Previous - Heading_Observed_Previous_Previous_Previous;

% Rudder angle controller 1:
Rudder_Angle_Transfer_Error_1 = (Alpha_01/Beta_01)*Error_Current + (Alpha_11/Beta_01)*Error_Previous + (Alpha_21/Beta_01)*Error_Previous_Previous + ...
                                (Alpha_31/Beta_01)*Error_Previous_Previous_Previous - (Beta_11/Beta_01)*Rudder_Transfer_1_Previous - (Beta_21/Beta_01)*Rudder_Transfer_1_Previous_Previous -...
                                (Beta_31/Beta_01)*Rudder_Transfer_1_Previous_Previous_Previous;

% Rudder angle controller 2:
Rudder_Angle_Transfer_Heading_2 = (Alpha_02/Beta_02)*Heading_Observed_Current + (Alpha_12/Beta_02)*Heading_Observed_Previous + (Alpha_22/Beta_02)*Heading_Observed_Previous_Previous + ...
                                  (Alpha_32/Beta_02)*Heading_Observed_Previous_Previous_Previous - (Beta_12/Beta_02)*Rudder_Transfer_2_Previous - (Beta_22/Beta_02)*Rudder_Transfer_2_Previous_Previous -...
                                  (Beta_32/Beta_02)*Rudder_Transfer_2_Previous_Previous_Previous;

% Rudder angle final:
Rudder_Angle_Elaborated_Current = Rudder_Angle_Transfer_Error_1 - Rudder_Angle_Transfer_Heading_2;

% Persistents update:
Rudder_Transfer_1_Previous_Previous_Previous = Rudder_Transfer_1_Previous_Previous;
Rudder_Transfer_1_Previous_Previous = Rudder_Transfer_1_Previous;
Rudder_Transfer_1_Previous = Rudder_Angle_Transfer_Error_1;

Rudder_Transfer_2_Previous_Previous_Previous = Rudder_Transfer_2_Previous_Previous;
Rudder_Transfer_2_Previous_Previous = Rudder_Transfer_2_Previous;
Rudder_Transfer_2_Previous = Rudder_Angle_Transfer_Heading_2;

Heading_Aimed_Previous_Previous_Previous = Heading_Aimed_Previous_Previous;
Heading_Aimed_Previous_Previous = Heading_Aimed_Previous;
Heading_Aimed_Previous = Heading_Aimed_Current;

Heading_Observed_Previous_Previous_Previous = Heading_Observed_Previous_Previous;
Heading_Observed_Previous_Previous = Heading_Observed_Previous;
Heading_Observed_Previous = Heading_Observed_Current;

end

