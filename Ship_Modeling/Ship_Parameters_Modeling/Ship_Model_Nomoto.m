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
Length_Ship_Minimal = 4.9;
Length_Ship_Maximal = 6;

% Salted water density:
Density_Water_Salted = 1000;
Density_Water_Salted_Minimal = 990;
Density_Water_Salted_Maximal = 1225;

% Drag coefficient at zero angle of attack:
Drag_Coefficient = 0.05; % Typical values in between 0.5 to 0.1 SI.
Drag_Coefficient_Minimal = 0.4;
Drag_Coefficient_Maximal = 1.2;

% Depth of the draft:
Draft_Depth = 0.35;
Draft_Depth_Minimal = 0.3;
Draft_Depth_Maximal = 0.5;

% Velocity of the ship:
Velocity_Ship_Mean = 5/3.6;
Velocity_Ship_Mean_Minimal = 2/3.6;
Velocity_Ship_Mean_Maximal = 15/3.6;

% Rudder area:
% No rudder but a longeron:
Longeron_Area = 0.2; % Typical values from 0.2 to 0.3 m²: TBC.
Longeron_Area_Minimal = 0.16;
Longeron_Area_Maximal = 0.4;

% Mass of the ship:
Mass_Ship = 550;
Mass_Ship_Minimal = 530;
Mass_Ship_Maximal = 600;

% Position of the COG:
Position_Center_Of_Gravity = 0.5*Length_Ship; % Estimated to be at the center of the ship.
Position_Center_Of_Gravity_Minimal = 0.3*Length_Ship;
Position_Center_Of_Gravity_Maximal = 0.6*Length_Ship;

% Distance between COG and COP refered as 'x_p':
Distance_Center_Of_Gravity_To_Center_Of_Pressure = Position_Center_Of_Gravity - 0.1*Length_Ship; % Need to make this value vary +-0.1*L_SHIP.
Distance_Center_Of_Gravity_To_Center_Of_Pressure_Minimal = Position_Center_Of_Gravity_Minimal - (-0.1)*Length_Ship_Minimal;
Distance_Center_Of_Gravity_To_Center_Of_Pressure_Maximal = Position_Center_Of_Gravity_Maximal - (0.1)*Length_Ship_Maximal;

% The radius of giration:
Radius_Of_Giration = 0.15*Length_Ship; % Must vary in between 0.15*Length_Ship to 0.3*Length_Ship.
Radius_Of_Giration_Minimal = 0.15*Length_Ship_Minimal;
Radius_Of_Giration_Maximal = 0.30*Length_Ship_Maximal;

% The inertia caused by the radius of giration & mass:
Inertia_Radius_Of_Giration = Mass_Ship*Radius_Of_Giration^2;
Inertia_Radius_Of_Giration_Minimal = Mass_Ship_Minimal*Radius_Of_Giration_Minimal^2;
Inertia_Radius_Of_Giration_Maximal = Mass_Ship_Maximal*Radius_Of_Giration_Maximal^2;

% Moment of inertia along Z-axis:
Inertia_Moment = Mass_Ship*Position_Center_Of_Gravity^2 + Inertia_Radius_Of_Giration;
Inertia_Moment_Minimal = Mass_Ship_Minimal*Position_Center_Of_Gravity_Minimal^2 + Inertia_Radius_Of_Giration_Minimal;
Inertia_Moment_Maximal = Mass_Ship_Maximal*Position_Center_Of_Gravity_Maximal^2 + Inertia_Radius_Of_Giration_Maximal;

% ******************************************************************
% 2) Hydrodynamics & added mass derivatives:
% ******************************************************************

% Hydrodynamics derivative 'Y_v':
Sway_Velocity_Hydrodynamics_Derivative = -0.5*Density_Water_Salted*Length_Ship*Draft_Depth*Velocity_Ship_Mean*...
                                         ((pi*Draft_Depth/Length_Ship) - Drag_Coefficient);
Sway_Velocity_Hydrodynamics_Derivative_Minimal = -0.5*Density_Water_Salted_Minimal*Length_Ship_Minimal*Draft_Depth_Minimal*Velocity_Ship_Mean_Minimal*...
                                                 ((pi*Draft_Depth_Minimal/Length_Ship_Minimal) - Drag_Coefficient_Minimal);
Sway_Velocity_Hydrodynamics_Derivative_Maximal = -0.5*Density_Water_Salted_Maximal*Length_Ship_Maximal*Draft_Depth_Maximal*Velocity_Ship_Mean_Maximal*...
                                                 ((pi*Draft_Depth_Maximal/Length_Ship_Maximal) - Drag_Coefficient_Maximal);

% Added mass derivative 'Y_v_point':
Sway_Acceleration_Added_Mass_Derivative = -0.6*Mass_Ship; % Must vary between -0.7*Mass_Ship to -1*Mass_Ship.
Sway_Acceleration_Added_Mass_Derivative_Minimal = -0.7*Mass_Ship_Minimal; 
Sway_Acceleration_Added_Mass_Derivative_Maximal = -Mass_Ship_Maximal; 

% Added mass derivative prime II system 'Y_v_point_prime':
Sway_Acceleration_Added_Mass_Derivative_Prime = Sway_Acceleration_Added_Mass_Derivative/(0.5*Density_Water_Salted*Draft_Depth*Length_Ship^2);
Sway_Acceleration_Added_Mass_Derivative_Prime_Minimal = Sway_Acceleration_Added_Mass_Derivative_Minimal/(0.5*Density_Water_Salted_Minimal*...
                                                        Draft_Depth_Minimal*Length_Ship_Minimal^2);
Sway_Acceleration_Added_Mass_Derivative_Prime_Maximal = Sway_Acceleration_Added_Mass_Derivative_Maximal/(0.5*Density_Water_Salted_Maximal*...
                                                        Draft_Depth_Maximal*Length_Ship_Maximal^2);

% Hydrodynamics derivative 'Y_v_prime':
Sway_Velocity_Hydrodynamics_Derivative_Prime = -((pi*Draft_Depth/Length_Ship) - Drag_Coefficient);
Sway_Velocity_Hydrodynamics_Derivative_Prime_Minimal = -((pi*Draft_Depth_Minimal/Length_Ship_Minimal) - Drag_Coefficient_Minimal);
Sway_Velocity_Hydrodynamics_Derivative_Prime_Maximal = -((pi*Draft_Depth_Maximal/Length_Ship_Maximal) - Drag_Coefficient_Maximal);

% Added mass derivative 'X_u_point':
Surge_Acceleration_Added_Mass_Derivative = -0.05*Mass_Ship; % Must vary in between -0.05*Mass_Ship to -0.10*Mass_Ship.
Surge_Acceleration_Added_Mass_Derivative_Minimal = -0.05*Mass_Ship_Minimal;
Surge_Acceleration_Added_Mass_Derivative_Maximal = -0.05*Mass_Ship_Maximal;

% Hydrodynamic derivative in prime system II 'X_u_point_prime':
Surge_Acceleration_Added_Mass_Derivative_Prime = Surge_Acceleration_Added_Mass_Derivative/(0.5*Density_Water_Salted*...
                                                 Draft_Depth*Length_Ship^2);
Surge_Acceleration_Added_Mass_Derivative_Prime_Minimal = Surge_Acceleration_Added_Mass_Derivative_Minimal/(0.5*Density_Water_Salted_Minimal*...
                                                         Draft_Depth_Minimal*Length_Ship_Minimal^2);
Surge_Acceleration_Added_Mass_Derivative_Prime_Maximal = Surge_Acceleration_Added_Mass_Derivative_Maximal/(0.5*Density_Water_Salted_Maximal*...
                                                         Draft_Depth_Maximal*Length_Ship_Maximal^2);

% Hydrodynamics derivative 'Y_r':
Sway_Yaw_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Draft_Depth*Velocity_Ship_Mean*Length_Ship^2*...
                                    (Surge_Acceleration_Added_Mass_Derivative_Prime + (Distance_Center_Of_Gravity_To_Center_Of_Pressure/Length_Ship)*...
                                    Sway_Velocity_Hydrodynamics_Derivative_Prime);
Sway_Yaw_Hydrodynamics_Derivative_Minimal = 0.5*Density_Water_Salted_Minimal*Draft_Depth_Minimal*Velocity_Ship_Mean_Minimal*Length_Ship_Minimal^2*...
                                            (Surge_Acceleration_Added_Mass_Derivative_Prime_Minimal + (Distance_Center_Of_Gravity_To_Center_Of_Pressure_Minimal/Length_Ship_Minimal)*...
                                            Sway_Velocity_Hydrodynamics_Derivative_Prime_Minimal);
Sway_Yaw_Hydrodynamics_Derivative_Maximal = 0.5*Density_Water_Salted_Maximal*Draft_Depth_Maximal*Velocity_Ship_Mean_Maximal*Length_Ship_Maximal^2*...
                                            (Surge_Acceleration_Added_Mass_Derivative_Prime_Maximal + (Distance_Center_Of_Gravity_To_Center_Of_Pressure_Maximal/Length_Ship_Maximal)*...
                                            Sway_Velocity_Hydrodynamics_Derivative_Prime_Maximal);

% Added mass derivative 'Y_r_point':
Sway_Yaw_Added_Mass_Derivative = 0;
Sway_Yaw_Added_Mass_Derivative_Minimal = 0;
Sway_Yaw_Added_Mass_Derivative_Maximal = 0;

% Hydrodynamics derivative 'N_v':
Yaw_Velocity_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Draft_Depth*Velocity_Ship_Mean*Length_Ship^2*((Distance_Center_Of_Gravity_To_Center_Of_Pressure/Length_Ship)*Sway_Velocity_Hydrodynamics_Derivative_Prime - ...
                                        (Surge_Acceleration_Added_Mass_Derivative_Prime - Sway_Acceleration_Added_Mass_Derivative_Prime));
Yaw_Velocity_Hydrodynamics_Derivative_Minimal = 0.5*Density_Water_Salted_Minimal*Draft_Depth_Minimal*Velocity_Ship_Mean_Minimal*Length_Ship_Minimal^2*((Distance_Center_Of_Gravity_To_Center_Of_Pressure_Minimal/Length_Ship_Minimal)*Sway_Velocity_Hydrodynamics_Derivative_Prime_Minimal - ...
                                                (Surge_Acceleration_Added_Mass_Derivative_Prime_Minimal - Sway_Acceleration_Added_Mass_Derivative_Prime_Minimal));
Yaw_Velocity_Hydrodynamics_Derivative_Maximal = 0.5*Density_Water_Salted_Maximal*Draft_Depth_Maximal*Velocity_Ship_Mean_Maximal*Length_Ship_Maximal^2*((Distance_Center_Of_Gravity_To_Center_Of_Pressure_Maximal/Length_Ship_Maximal)*Sway_Velocity_Hydrodynamics_Derivative_Prime_Maximal - ...
                                                (Surge_Acceleration_Added_Mass_Derivative_Prime_Maximal - Sway_Acceleration_Added_Mass_Derivative_Prime_Maximal));

% Added mass derivative 'N_v_point':
Yaw_Velocity_Added_Mass_Derivative = 0;
Yaw_Velocity_Added_Mass_Derivative_Minimal = 0;
Yaw_Velocity_Added_Mass_Derivative_Maximal = 0;

% Hydrodynamics derivative 'N_r':
Yaw_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Draft_Depth*Velocity_Ship_Mean*Length_Ship^3*(1/4)*Sway_Velocity_Hydrodynamics_Derivative_Prime;
Yaw_Hydrodynamics_Derivative_Minimal = 0.5*Density_Water_Salted_Minimal*Draft_Depth_Minimal*Velocity_Ship_Mean_Minimal*Length_Ship_Minimal^3*(1/4)*Sway_Velocity_Hydrodynamics_Derivative_Prime_Minimal;
Yaw_Hydrodynamics_Derivative_Maximal = 0.5*Density_Water_Salted_Maximal*Draft_Depth_Maximal*Velocity_Ship_Mean_Maximal*Length_Ship_Maximal^3*(1/4)*Sway_Velocity_Hydrodynamics_Derivative_Prime_Maximal;

% Added mass derivative 'N_r_point':
Yaw_Added_Mass_Derivative = -0.01*Inertia_Moment; % Must try values in between -0.01*Inertia_Moment to -0.1*Inertia_Moment.
Yaw_Added_Mass_Derivative_Minimal = -0.01*Inertia_Moment_Minimal;
Yaw_Added_Mass_Derivative_Maximal = -0.1*Inertia_Moment_Maximal;

% Hydrodynamics derivative 'Y_delta':
Sway_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Length_Ship*Draft_Depth*Velocity_Ship_Mean^2*...
                                        ((pi/4)*(Longeron_Area/(Length_Ship*Draft_Depth)));
Sway_Control_Hydrodynamics_Derivative_Minimal = 0.5*Density_Water_Salted_Minimal*Length_Ship_Minimal*Draft_Depth_Minimal*Velocity_Ship_Mean_Minimal^2*...
                                                ((pi/4)*(Longeron_Area_Minimal/(Length_Ship_Minimal*Draft_Depth_Minimal)));
Sway_Control_Hydrodynamics_Derivative_Maximal = 0.5*Density_Water_Salted_Maximal*Length_Ship_Maximal*Draft_Depth_Maximal*Velocity_Ship_Mean_Maximal^2*...
                                                ((pi/4)*(Longeron_Area_Maximal/(Length_Ship_Maximal*Draft_Depth_Maximal)));

% Hydrodynamics derivative 'N_delta':
Yaw_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Salted*Length_Ship^2*Draft_Depth*Velocity_Ship_Mean^2*(-0.5*...
                                       ((pi/4)*(Longeron_Area/(Length_Ship*Draft_Depth))));
Yaw_Control_Hydrodynamics_Derivative_Minimal = 0.5*Density_Water_Salted_Minimal*Length_Ship_Minimal^2*Draft_Depth_Minimal*Velocity_Ship_Mean_Minimal^2*(-0.5*...
                                               ((pi/4)*(Longeron_Area_Minimal/(Length_Ship_Minimal*Draft_Depth_Minimal))));
Yaw_Control_Hydrodynamics_Derivative_Maximal = 0.5*Density_Water_Salted_Maximal*Length_Ship_Maximal^2*Draft_Depth_Maximal*Velocity_Ship_Mean_Maximal^2*(-0.5*...
                                               ((pi/4)*(Longeron_Area_Maximal/(Length_Ship_Maximal*Draft_Depth_Maximal))));

% ********************************************************************
% 3) Transfer function between control angle and heading & parameters:
% ********************************************************************

% Inertia matrix terms:
Inertia_Matrix_11 = Mass_Ship - Sway_Acceleration_Added_Mass_Derivative;
Inertia_Matrix_12 = Mass_Ship*Position_Center_Of_Gravity - Sway_Yaw_Added_Mass_Derivative;
Inertia_Matrix_21 = Mass_Ship*Position_Center_Of_Gravity - Yaw_Velocity_Added_Mass_Derivative;
Inertia_Matrix_22 = Inertia_Moment - Yaw_Added_Mass_Derivative;
% Inertia matrix terms minimal:
Inertia_Matrix_11_Minimal = Mass_Ship_Minimal - Sway_Acceleration_Added_Mass_Derivative_Minimal;
Inertia_Matrix_12_Minimal = Mass_Ship_Minimal*Position_Center_Of_Gravity_Minimal - Sway_Yaw_Added_Mass_Derivative_Minimal;
Inertia_Matrix_21_Minimal = Mass_Ship_Minimal*Position_Center_Of_Gravity_Minimal - Yaw_Velocity_Added_Mass_Derivative_Minimal;
Inertia_Matrix_22_Minimal = Inertia_Moment_Minimal - Yaw_Added_Mass_Derivative_Minimal;
% Inertia matrix terms maximal:
Inertia_Matrix_11_Maximal = Mass_Ship_Maximal - Sway_Acceleration_Added_Mass_Derivative_Maximal;
Inertia_Matrix_12_Maximal = Mass_Ship_Maximal*Position_Center_Of_Gravity_Maximal - Sway_Yaw_Added_Mass_Derivative_Maximal;
Inertia_Matrix_21_Maximal = Mass_Ship_Maximal*Position_Center_Of_Gravity_Maximal - Yaw_Velocity_Added_Mass_Derivative_Maximal;
Inertia_Matrix_22_Maximal = Inertia_Moment_Maximal - Yaw_Added_Mass_Derivative_Maximal;
% The associated matrix:
Inertia_Matrix = [Inertia_Matrix_11 Inertia_Matrix_12;Inertia_Matrix_21 Inertia_Matrix_22];
Inertia_Matrix_Minimal = [Inertia_Matrix_11_Minimal Inertia_Matrix_12_Minimal;Inertia_Matrix_21_Minimal Inertia_Matrix_22_Minimal];
Inertia_Matrix_Maximal = [Inertia_Matrix_11_Maximal Inertia_Matrix_12_Maximal;Inertia_Matrix_21_Maximal Inertia_Matrix_22_Maximal];
% The determinent of the associated matrix:
Determinant_Inertia_Matrix = det(Inertia_Matrix);
Determinant_Inertia_Matrix_Minimal = det(Inertia_Matrix_Minimal);
Determinant_Inertia_Matrix_Maximal = det(Inertia_Matrix_Maximal);
% Cross-check:
Delta_Check = 1e-3;
Determinant_Inertia_Matrix_Crosscheck = Inertia_Matrix_11*Inertia_Matrix_22 - Inertia_Matrix_21*Inertia_Matrix_12;
if abs(Determinant_Inertia_Matrix_Crosscheck - Determinant_Inertia_Matrix) > Delta_Check
    error('The determinant of the matrices are different');
end

% Inertia coefficient term:
Inertia_Coefficient = Inertia_Moment - Yaw_Added_Mass_Derivative;

% The coriolis centripetal perturbations terms matrix:
Coriolis_Centripetal_11 = -Sway_Velocity_Hydrodynamics_Derivative;
Coriolis_Centripetal_12 = Mass_Ship*Velocity_Ship_Mean - Sway_Yaw_Hydrodynamics_Derivative;
Coriolis_Centripetal_21 = -Yaw_Velocity_Hydrodynamics_Derivative;
Coriolis_Centripetal_22 = Mass_Ship*Position_Center_Of_Gravity*Velocity_Ship_Mean - Yaw_Hydrodynamics_Derivative;
% Minimal terms:
Coriolis_Centripetal_11_Minimal = -Sway_Velocity_Hydrodynamics_Derivative_Minimal;
Coriolis_Centripetal_12_Minimal = Mass_Ship_Minimal*Velocity_Ship_Mean_Minimal - Sway_Yaw_Hydrodynamics_Derivative_Minimal;
Coriolis_Centripetal_21_Minimal = -Yaw_Velocity_Hydrodynamics_Derivative_Minimal;
Coriolis_Centripetal_22_Minimal = Mass_Ship_Minimal*Position_Center_Of_Gravity_Minimal*Velocity_Ship_Mean_Minimal - Yaw_Hydrodynamics_Derivative_Minimal;
% Maximal terms:
Coriolis_Centripetal_11_Maximal = -Sway_Velocity_Hydrodynamics_Derivative_Maximal;
Coriolis_Centripetal_12_Maximal = Mass_Ship_Maximal*Velocity_Ship_Mean_Maximal - Sway_Yaw_Hydrodynamics_Derivative_Maximal;
Coriolis_Centripetal_21_Maximal = -Yaw_Velocity_Hydrodynamics_Derivative_Maximal;
Coriolis_Centripetal_22_Maximal = Mass_Ship_Maximal*Position_Center_Of_Gravity_Maximal*Velocity_Ship_Mean_Maximal - Yaw_Hydrodynamics_Derivative_Maximal;
% The associated matrix:
Coriolis_Centripetal = [Coriolis_Centripetal_11 Coriolis_Centripetal_12;Coriolis_Centripetal_21 Coriolis_Centripetal_22];
Coriolis_Centripetal_Minimal = [Coriolis_Centripetal_11_Minimal Coriolis_Centripetal_12_Minimal;Coriolis_Centripetal_21_Minimal Coriolis_Centripetal_22_Minimal];
Coriolis_Centripetal_Maximal = [Coriolis_Centripetal_11_Maximal Coriolis_Centripetal_12_Maximal;Coriolis_Centripetal_21_Maximal Coriolis_Centripetal_22_Maximal];
% The determinant of the associated matrix:
Determinant_Coriolis_Centripetal_Matrix = det(Coriolis_Centripetal);
Determinant_Coriolis_Centripetal_Matrix_Minimal = det(Coriolis_Centripetal_Minimal);
Determinant_Coriolis_Centripetal_Matrix_Maximal = det(Coriolis_Centripetal_Maximal);
% Cross-check:
Determinant_Coriolis_Centripetal_Matrix_Crosscheck = Coriolis_Centripetal_11*Coriolis_Centripetal_22 - Coriolis_Centripetal_21*Coriolis_Centripetal_12;

% The input control matrix terms:
Control_Inputs_Rudder_11 = Sway_Control_Hydrodynamics_Derivative; % 'Y_delta'.
Control_Inputs_Rudder_21 = Yaw_Control_Hydrodynamics_Derivative; % 'N_delta'.
% Minimal terms:
Control_Inputs_Rudder_11_Minimal = Sway_Control_Hydrodynamics_Derivative_Minimal; 
Control_Inputs_Rudder_21_Minimal = Yaw_Control_Hydrodynamics_Derivative_Minimal;
% Maximal terms:
Control_Inputs_Rudder_11_Maximal = Sway_Control_Hydrodynamics_Derivative_Maximal; 
Control_Inputs_Rudder_21_Maximal = Yaw_Control_Hydrodynamics_Derivative_Maximal;

% Laplace's variable for transfer function definition:
s = tf('s'); 

% Designated nominal model determination:
% ---------------------------------------
% Determination of the time constant:
Time_Constant_Equation_1 = Determinant_Inertia_Matrix/Determinant_Coriolis_Centripetal_Matrix;
Time_Constant_Equation_2 = (Coriolis_Centripetal_11*Inertia_Matrix_22 + Coriolis_Centripetal_22*Inertia_Matrix_11 - ...
                            Coriolis_Centripetal_12*Inertia_Matrix_21 - Coriolis_Centripetal_21*Inertia_Matrix_12)/...
                            Determinant_Coriolis_Centripetal_Matrix;
% To obtain the time constant, one must solve a second order equation:
Delta_Equation_Time_Constant = Time_Constant_Equation_2^2 - 4*Time_Constant_Equation_1;
if Delta_Equation_Time_Constant > 0
    % Then 2 solutions:
    Time_Constant_1 = (Time_Constant_Equation_2 + sqrt(Delta_Equation_Time_Constant))/2;
    Time_Constant_2 = (Time_Constant_Equation_2 - sqrt(Delta_Equation_Time_Constant))/2;
elseif Delta_Equation_Time_Constant == 0
    % Then only one unique solution:
    Time_Constant_1 = Time_Constant_Equation_2/2;
    Time_Constant_2 = Time_Constant_1;
else
    warning('Delta of equation is negative.');
    Time_Constant_1 = (Time_Constant_Equation_2 + 1i*sqrt(-Delta_Equation_Time_Constant))/2;
    Time_Constant_2 = (Time_Constant_Equation_2 - 1i*sqrt(-Delta_Equation_Time_Constant))/2;
end
% Define the static gain 'K_r':
Static_Gain = (Coriolis_Centripetal_21*Control_Inputs_Rudder_11 - Coriolis_Centripetal_11*Control_Inputs_Rudder_21)/...
               Determinant_Coriolis_Centripetal_Matrix;
% Define the time constant number 3:
Time_Constant_3 = (Inertia_Matrix_21*Control_Inputs_Rudder_11 - Inertia_Matrix_11*Control_Inputs_Rudder_21)/...
                  (Static_Gain*Determinant_Coriolis_Centripetal_Matrix);
% Static gain Nomoto:
Static_Gain_Nomoto = -Static_Gain;
% Second order Nomoto's model:
Heading_On_Rudder_Transfer_Second_Order = (Static_Gain_Nomoto*(1 + Time_Constant_3*s))/(s*(1 + Time_Constant_1*s)*(1 + Time_Constant_2*s));
% First order Nomoto's model:
Time_Constant_Total = Time_Constant_1 + Time_Constant_2 - Time_Constant_3;
if Time_Constant_Total < 0
    warning('Equivalent time constant is negative');
end
Heading_On_Rudder_Transfer_First_Order = (Static_Gain_Nomoto)/(s*(1 + Time_Constant_Total*s));
% Conversion of the first order model into state-space form:
State_Space_Nomoto_First_Order = ss(Heading_On_Rudder_Transfer_First_Order);
A_Nomoto_First_Order = State_Space_Nomoto_First_Order.A;
B_Nomoto_First_Order = State_Space_Nomoto_First_Order.B;
C_Nomoto_First_Order = State_Space_Nomoto_First_Order.C;
D_Nomoto_First_Order = State_Space_Nomoto_First_Order.D;

% Minimal model determination:
% ----------------------------
% Determination of the time constant:
Time_Constant_Equation_1_Minimal = Determinant_Inertia_Matrix_Minimal/Determinant_Coriolis_Centripetal_Matrix_Minimal;
Time_Constant_Equation_2_Minimal = (Coriolis_Centripetal_11_Minimal*Inertia_Matrix_22_Minimal + Coriolis_Centripetal_22_Minimal*Inertia_Matrix_11_Minimal - ...
                                    Coriolis_Centripetal_12_Minimal*Inertia_Matrix_21_Minimal - Coriolis_Centripetal_21_Minimal*Inertia_Matrix_12_Minimal)/...
                                    Determinant_Coriolis_Centripetal_Matrix_Minimal;
% To obtain the time constant, one must solve a second order equation:
Delta_Equation_Time_Constant_Minimal = Time_Constant_Equation_2_Minimal^2 - 4*Time_Constant_Equation_1_Minimal;
if Delta_Equation_Time_Constant_Minimal > 0
    % Then 2 solutions:
    Time_Constant_1_Minimal = (Time_Constant_Equation_2_Minimal + sqrt(Delta_Equation_Time_Constant_Minimal))/2;
    Time_Constant_2_Minimal = (Time_Constant_Equation_2_Minimal - sqrt(Delta_Equation_Time_Constant_Minimal))/2;
elseif Delta_Equation_Time_Constant_Minimal == 0
    % Then only one unique solution:
    Time_Constant_1_Minimal = Time_Constant_Equation_2_Minimal/2;
    Time_Constant_2_Minimal = Time_Constant_1_Minimal;
else
    error('Time cannot be imaginary.');
end
% Define the static gain 'K_r':
Static_Gain_Minimal = (Coriolis_Centripetal_21_Minimal*Control_Inputs_Rudder_11_Minimal - Coriolis_Centripetal_11_Minimal*Control_Inputs_Rudder_21_Minimal)/...
                       Determinant_Coriolis_Centripetal_Matrix_Minimal;
% Define the time constant number 3:
Time_Constant_3_Minimal = (Inertia_Matrix_21_Minimal*Control_Inputs_Rudder_11_Minimal - Inertia_Matrix_11_Minimal*Control_Inputs_Rudder_21_Minimal)/...
                          (Static_Gain_Minimal*Determinant_Coriolis_Centripetal_Matrix_Minimal);
% Static gain Nomoto:
Static_Gain_Nomoto_Minimal = -Static_Gain_Minimal;
% Second order Nomoto's model:
Heading_On_Rudder_Transfer_Second_Order_Minimal = (Static_Gain_Nomoto_Minimal*(1 + Time_Constant_3_Minimal*s))/(s*(1 + Time_Constant_1_Minimal*s)*(1 + Time_Constant_2_Minimal*s));
% First order Nomoto's model:
Time_Constant_Total_Minimal = Time_Constant_1_Minimal + Time_Constant_2_Minimal - Time_Constant_3_Minimal;
if Time_Constant_Total_Minimal < 0
    warning('Equivalent time constant is negative');
end
Heading_On_Rudder_Transfer_First_Order_Minimal = (Static_Gain_Nomoto_Minimal)/(s*(1 + Time_Constant_Total_Minimal*s));

% Maximal model determination:
% ----------------------------
% Determination of the time constant:
Time_Constant_Equation_1_Maximal = Determinant_Inertia_Matrix_Maximal/Determinant_Coriolis_Centripetal_Matrix_Maximal;
Time_Constant_Equation_2_Maximal = (Coriolis_Centripetal_11_Maximal*Inertia_Matrix_22_Maximal + Coriolis_Centripetal_22_Maximal*Inertia_Matrix_11_Maximal - ...
                                    Coriolis_Centripetal_12_Maximal*Inertia_Matrix_21_Maximal - Coriolis_Centripetal_21_Maximal*Inertia_Matrix_12_Maximal)/...
                                    Determinant_Coriolis_Centripetal_Matrix_Maximal;
% To obtain the time constant, one must solve a second order equation:
Delta_Equation_Time_Constant_Maximal = Time_Constant_Equation_2_Maximal^2 - 4*Time_Constant_Equation_1_Maximal;
if Delta_Equation_Time_Constant_Maximal > 0
    % Then 2 solutions:
    Time_Constant_1_Maximal = (Time_Constant_Equation_2_Maximal + sqrt(Delta_Equation_Time_Constant_Maximal))/2;
    Time_Constant_2_Maximal = (Time_Constant_Equation_2_Maximal - sqrt(Delta_Equation_Time_Constant_Maximal))/2;
elseif Delta_Equation_Time_Constant_Maximal == 0
    % Then only one unique solution:
    Time_Constant_1_Maximal = Time_Constant_Equation_2_Maximal/2;
    Time_Constant_2_Maximal = Time_Constant_1_Maximal;
else
    error('Time cannot be imaginary.');
end
% Define the static gain 'K_r':
Static_Gain_Maximal = (Coriolis_Centripetal_21_Maximal*Control_Inputs_Rudder_11_Maximal - Coriolis_Centripetal_11_Maximal*Control_Inputs_Rudder_21_Maximal)/...
                       Determinant_Coriolis_Centripetal_Matrix_Maximal;
% Define the time constant number 3:
Time_Constant_3_Maximal = (Inertia_Matrix_21_Maximal*Control_Inputs_Rudder_11_Maximal - Inertia_Matrix_11_Maximal*Control_Inputs_Rudder_21_Maximal)/...
                          (Static_Gain_Maximal*Determinant_Coriolis_Centripetal_Matrix_Maximal);
% Static gain Nomoto:
Static_Gain_Nomoto_Maximal = -Static_Gain_Maximal;
% Second order Nomoto's model:
Heading_On_Rudder_Transfer_Second_Order_Maximal = (Static_Gain_Nomoto_Maximal*(1 + Time_Constant_3_Maximal*s))/(s*(1 + Time_Constant_1_Maximal*s)*(1 + Time_Constant_2_Maximal*s));
% First order Nomoto's model:
Time_Constant_Total_Maximal = Time_Constant_1_Maximal + Time_Constant_2_Maximal - Time_Constant_3_Maximal;
if Time_Constant_Total_Maximal < 0
    warning('Equivalent time constant is negative');
end
Heading_On_Rudder_Transfer_First_Order_Maximal = (Static_Gain_Nomoto_Maximal)/(s*(1 + Time_Constant_Total_Maximal*s));













