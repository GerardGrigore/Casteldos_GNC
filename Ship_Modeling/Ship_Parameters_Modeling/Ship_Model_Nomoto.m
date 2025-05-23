%-------------------------------------------------------------------
% Nomotos's ship modeling and description
%-------------------------------------------------------------------

% This script serves as a launch script allowing to obtain several physical
% parameters about the ship as well as the hydrodynamics derivatives. This
% contains the model of the ship's yaw dynamics in function of the input
% rudder angle.

% ******************************************************************
% 1) Definition of main physical parameters of the ship:
% ******************************************************************

% Length of the ship:
Length_Ship = 5.3;

% Salted water density:
Density_Water_Salted = 1000;

% Viscous dynamics of the salted water:
Viscous_Dynamics_Salted_Water = 9.3e-5; % Must play with this value.

% Mean depth of the tarn:
Depth_Tarn_Mean = 1; % Careful, this plays on the Reynolds number and therfore one the drag coefficient

% Gravitational acceleration:
Gravitational_Acceleration = 9.81;

% The drag coefficient of the ship at zero angle of attack:
% First define the speed of the fluid in Shallow waters:
Velocity_Water_Shallow = sqrt(Gravitational_Acceleration*Depth_Tarn_Mean);
% Reynolds number:
Reynolds_Number = (Length_Ship*Velocity_Water_Shallow)/Viscous_Dynamics_Salted_Water;

% As the value of Reynolds number is lower than 2e5, consider the model of White
% to calculate the drag coefficient:
% Drag_Coefficient = (24/RENOLDS) + 0.4 + (6/(1 + sqrt(RENOLDS)));
Drag_Coefficient = 0.05; % Typical values in between 0.5 to 0.1 SI.

% Depth of the draft:
Draft_Depth = 0.35;

% Velocity of the ship:
Velocity_Ship_Mean = 5/3.6;

% Rudder area:
% No rudder but a longeron:
Longeron_Area = 0.2; % Typical values from 0.2 to 0.3 m².

% Mass of the ship:
Mass_Ship = 550;

% Position of the COG:
Position_Center_Of_Gravity = 0.5*Length_Ship; % Estimated to be at the center of the ship.

% Distance between COG and COP refered as 'x_p':
Distance_Center_Of_Gravity_To_Center_Of_Pressure = Position_Center_Of_Gravity - 0.1*Length_Ship; % Need to make this value vary +-0.1*L_SHIP.

% The radius of giration:
Radius_Of_Giration = 0.15*Length_Ship; % Must vary in between 0.15*Length_Ship to 0.3*Length_Ship.

% The inertia caused by the radius of giration & mass:
Inertia_Radius_Of_Giration = Mass_Ship*Radius_Of_Giration^2;

% Moment of inertia along Z-axis:
Inertia_Moment = Mass_Ship*Position_Center_Of_Gravity^2 + Inertia_Radius_Of_Giration;

% ******************************************************************
% 2) Hydrodynamics & added mass derivatives:
% ******************************************************************

% Hydrodynamics derivative 'Y_v':
Sway_Velocity_Hydrodynamics_Derivative = -0.5*Density_Water_Salted*Length_Ship*Draft_Depth*Velocity_Ship_Mean*...
                                         ((pi*Draft_Depth/Length_Ship) - Drag_Coefficient);

% Added mass derivative 'Y_v_point':
Sway_Acceleration_Added_Mass_Derivative = -0.6*Mass_Ship; % Must vary between -0.7*Mass_Ship to -1*Mass_Ship.

% Added mass derivative prime II system 'Y_v_point_prime':
Sway_Acceleration_Added_Mass_Derivative_Prime = Sway_Acceleration_Added_Mass_Derivative/(0.5*Density_Water_Salted*Draft_Depth*Length_Ship^2);

% Hydrodynamics derivative 'Y_v_prime':
Sway_Velocity_Hydrodynamics_Derivative_Prime = -((pi*Draft_Depth/Length_Ship) - Drag_Coefficient);

% Added mass derivative 'X_u_point':
Surge_Acceleration_Added_Mass_Derivative = -0.05*Mass_Ship; % Must vary in between -0.05*Mass_Ship to -0.10*Mass_Ship.

% Hydrodynamic derivative in prime system II 'X_u_point_prime':
Surge_Acceleration_Added_Mass_Derivative_Prime = Surge_Acceleration_Added_Mass_Derivative/(0.5*Density_Water_Salted*Draft_Depth*Length_Ship^2);

% Hydrodynamics derivative 'Y_r':
Sway_Yaw_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Draft_Depth*Velocity_Ship_Mean*Length_Ship^2*...
                                    (Surge_Acceleration_Added_Mass_Derivative_Prime + (Distance_Center_Of_Gravity_To_Center_Of_Pressure/Length_Ship)*...
                                    Sway_Velocity_Hydrodynamics_Derivative_Prime);

% Added mass derivative 'Y_r_point':
Sway_Yaw_Added_Mass_Derivative = 0;

% Hydrodynamics derivative 'N_v':
Yaw_Velocity_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Draft_Depth*Velocity_Ship_Mean*Length_Ship^2*((Distance_Center_Of_Gravity_To_Center_Of_Pressure/Length_Ship)*Sway_Velocity_Hydrodynamics_Derivative_Prime - ...
                                        (Surge_Acceleration_Added_Mass_Derivative_Prime - Sway_Acceleration_Added_Mass_Derivative_Prime));

% Added mass derivative 'N_v_point':
Yaw_Velocity_Added_Mass_Derivative = 0;

% Hydrodynamics derivative 'N_r':
Yaw_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Draft_Depth*Velocity_Ship_Mean*Length_Ship^3*(1/4)*Sway_Velocity_Hydrodynamics_Derivative_Prime;

% Added mass derivative 'N_r_point':
Yaw_Added_Mass_Derivative = -0.01*Inertia_Moment; % Must try values in between -0.01*Inertia_Moment to -0.1*Inertia_Moment.

% Hydrodynamics derivative 'Y_delta':
Sway_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Length_Ship*Draft_Depth*Velocity_Ship_Mean^2*...
                                        (Density_Water_Salted*(pi/4)*(Longeron_Area/(Length_Ship*Draft_Depth)));
% Alternative more realistic 'Y_delta' for small ship:
%Sway_Control_Hydrodynamics_Derivative = Sway_Control_Hydrodynamics_Derivative*0.5;

% Hydrodynamics derivative 'N_delta':
Yaw_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Length_Ship^2*Draft_Depth*Velocity_Ship_Mean^2*(-0.5*...
                                       (Density_Water_Salted*(pi/4)*(Longeron_Area/(Length_Ship*Draft_Depth))));
% Alternative more realistic 'N_delta' for small ship:
%Yaw_Control_Hydrodynamics_Derivative = Density_Water_Salted*Velocity_Ship_Mean^2*Longeron_Area*Distance_Center_Of_Gravity_To_Center_Of_Pressure;

% ******************************************************************
% Transfer function between control angle and heading & parameters:
% ******************************************************************

% Inertia matrix terms:
Inertia_Matrix_11 = Mass_Ship - Sway_Acceleration_Added_Mass_Derivative;
Inertia_Matrix_12 = Mass_Ship*Position_Center_Of_Gravity - Sway_Yaw_Added_Mass_Derivative;
Inertia_Matrix_21 = Mass_Ship*Position_Center_Of_Gravity - Yaw_Velocity_Added_Mass_Derivative;
Inertia_Matrix_22 = Inertia_Moment - Yaw_Added_Mass_Derivative;
% The associated matrix:
Inertia_Matrix = [Inertia_Matrix_11 Inertia_Matrix_12;Inertia_Matrix_21 Inertia_Matrix_22];
% The determinent of the associated matrix:
Determinant_Inertia_Matrix = det(Inertia_Matrix);
% Cross-check:
Delta_Check = 1e-3;
Determinant_Inertia_Matrix_Crosscheck = Inertia_Matrix_11*Inertia_Matrix_22 - Inertia_Matrix_21*Inertia_Matrix_12;
if abs(Determinant_Inertia_Matrix_Crosscheck - Determinant_Inertia_Matrix) > Delta_Check
    warning('The determinant of the matrices are different');
end

% Inertia coefficient term:
Inertia_Coefficient = Inertia_Moment - Yaw_Added_Mass_Derivative;

% The coriolis centripetal perturbations terms matrix:
Coriolis_Centripetal_11 = -Sway_Velocity_Hydrodynamics_Derivative;
Coriolis_Centripetal_12 = Mass_Ship*Velocity_Ship_Mean - Sway_Yaw_Hydrodynamics_Derivative;
Coriolis_Centripetal_21 = -Yaw_Velocity_Hydrodynamics_Derivative;
Coriolis_Centripetak_22 = Mass_Ship*Position_Center_Of_Gravity*Velocity_Ship_Mean - Yaw_Hydrodynamics_Derivative;
% The associated matrix:
Coriolis_Centripetal = [Coriolis_Centripetal_11 Coriolis_Centripetal_12;Coriolis_Centripetal_21 Coriolis_Centripetak_22];
% The determinant of the associated matrix:
Determinant_Coriolis_Centripetal_Matrix = det(Coriolis_Centripetal);
% Cross-check:
Determinant_Coriolis_Centripetal_Matrix_Crosscheck = Coriolis_Centripetal_11*Coriolis_Centripetak_22 - Coriolis_Centripetal_21*Coriolis_Centripetal_12;

% The input control matrix terms:
Control_Inputs_Rudder_11 = ((Inertia_Moment - Yaw_Added_Mass_Derivative)*Sway_Control_Hydrodynamics_Derivative - ...
                           (Mass_Ship*Position_Center_Of_Gravity - Sway_Yaw_Added_Mass_Derivative)*Yaw_Control_Hydrodynamics_Derivative)/(Determinant_Inertia_Matrix);
Control_Inputs_Rudder_21 = ((Mass_Ship - Sway_Acceleration_Added_Mass_Derivative)*Yaw_Control_Hydrodynamics_Derivative -...
                           (Mass_Ship*Position_Center_Of_Gravity - Yaw_Velocity_Added_Mass_Derivative)*Sway_Control_Hydrodynamics_Derivative)/(Determinant_Inertia_Matrix);

% Determination of the time constant:
Time_Constant_Equation_1 = Determinant_Inertia_Matrix/Determinant_Coriolis_Centripetal_Matrix;
Time_Constant_Equation_2 = (Coriolis_Centripetal_11*Inertia_Matrix_22 + Coriolis_Centripetak_22*Inertia_Matrix_11 - ...
                            Coriolis_Centripetal_12*Inertia_Matrix_21 - Coriolis_Centripetal_21*Inertia_Matrix_12)/...
                            Determinant_Coriolis_Centripetal_Matrix;

% To obtain the time constant, one must solve a second order equation.
Delta_Equation_Time_Constant = Time_Constant_Equation_2^2 - 4*Time_Constant_Equation_1;

if Delta_Equation_Time_Constant > 0
    % Then 2 solutions:
    Time_Constant_11 = (Time_Constant_Equation_2 - sqrt(Delta_Equation_Time_Constant))/2;
    Time_Constant_12 = (Time_Constant_Equation_2 + sqrt(Delta_Equation_Time_Constant))/2;
    % If one is negative, keep the physical positive one:
    if Time_Constant_11 < 0 && Time_Constant_12 > 0
        Time_Constant_1 = Time_Constant_12;
    elseif Time_Constant_12 < 0 && Time_Constant_11 > 0
        Time_Constant_1 = Time_Constant_11;
    else
        % If none are negative or physical, must try them both:
        sprintf('No possibility to discretize the choice of the time constant.')
        Time_Constant_1 = Time_Constant_11;
        % Time_Constant_1 = Time_Constant_12;
    end
    % Then get the second time constant:
    Time_Constant_2 = Time_Constant_Equation_2 - Time_Constant_1;   
elseif Delta_Equation_Time_Constant == 0
    % Then only one unique solution:
    Time_Constant_1 = Time_Constant_Equation_2/2;
    % Then get the second time constant:
    Time_Constant_2 = Time_Constant_Equation_2 - Time_Constant_1;
else
    warning('Time cannot be imaginary.');
end

% Define the static gain 'K_r':
Static_Gain = (Coriolis_Centripetal_21*Control_Inputs_Rudder_11 - Coriolis_Centripetal_11*Control_Inputs_Rudder_21)/...
               Determinant_Coriolis_Centripetal_Matrix;

% Define the time constant number 3:
Time_Constant_3 = (Inertia_Matrix_21*Control_Inputs_Rudder_11 - Inertia_Matrix_11*Control_Inputs_Rudder_21)/...
                  (Static_Gain*Determinant_Coriolis_Centripetal_Matrix);

% Then define the transfer functions:
s = tf('s'); % Define the Laplace variable for easier comprehension.

% Static gain Nomoto:
Static_Gain_Nomoto = -Static_Gain;

% Second order Nomoto's model:
Heading_On_Rudder_Transfer_Second_Order = (Static_Gain_Nomoto*(1 + Time_Constant_3*s))/(s*(1 + Time_Constant_1*s)*(1 + Time_Constant_2*s));

% First order Nomoto's model:
Time_Constant_Total = Time_Constant_1 + Time_Constant_2 - Time_Constant_3;
Heading_On_Rudder_Transfer_First_Order = (Static_Gain_Nomoto)/(s*(1 + Time_Constant_Total*s));

% Conversion of the first order model into state-space form:
State_Space_Nomoto_First_Order = ss(Heading_On_Rudder_Transfer_First_Order);
A_Nomoto_First_Order = State_Space_Nomoto_First_Order.A;
B_Nomoto_First_Order = State_Space_Nomoto_First_Order.B;
C_Nomoto_First_Order = State_Space_Nomoto_First_Order.C;
D_Nomoto_First_Order = State_Space_Nomoto_First_Order.D;

















