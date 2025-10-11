function Heading_Ship_Generated = Ship_First_Order_Velocity_Varying(Rudder_Commanded_Current,...
                                                                    Velocity_Current,...
                                                                    Length_Ship_Current,...
                                                                    Density_Water_Current,...
                                                                    Drag_Coefficient_Current,...
                                                                    Draft_Depth_Current,...
                                                                    Longeron_Area_Current,...
                                                                    Mass_Of_The_Ship_Current)

% This function serves as the modelling system for the heading control
% simulations. Since the Nomoto's transfer function modelling the relation
% between the rudder angle and the heading depends on the velocity of the
% ship, it has been decided to model the ship as an LPV and not LTI.

% Only the velocity will be taken into account at first, but all the other terms
% and parameters are considered in inputs for future potential dispersions
% considerations.

% Parameters definition:
% ----------------------
% Assumption on the posistion of the COG:
Position_Center_Of_Gravity = 0.5*Length_Ship_Current; % Estimated to be at the center of the ship.

% Distance between COG and COP refered as 'x_p':
Distance_Center_Of_Gravity_To_Center_Of_Pressure = Position_Center_Of_Gravity - 0.1*Length_Ship_Current; 

% The radius of giration:
Radius_Of_Giration = 0.15*Length_Ship_Current;

% The inertia caused by the radius of giration & mass:
Inertia_Radius_Of_Giration = Mass_Of_The_Ship_Current*Radius_Of_Giration^2;

% Moment of inertia along Z-axis:
Inertia_Moment = Mass_Of_The_Ship_Current*Position_Center_Of_Gravity^2 + Inertia_Radius_Of_Giration;

% 2) Hydrodynamics & added mass derivatives:
% ------------------------------------------
% Hydrodynamics derivative 'Y_v':
Sway_Velocity_Hydrodynamics_Derivative = -0.5*Density_Water_Current*Length_Ship_Current*Draft_Depth_Current*Velocity_Current*...
                                         ((pi*Draft_Depth_Current/Length_Ship_Current) - Drag_Coefficient_Current);

% Added mass derivative 'Y_v_point':
Sway_Acceleration_Added_Mass_Derivative = -0.6*Mass_Of_The_Ship_Current; % Must vary between -0.7*Mass_Ship to -1*Mass_Ship.

% Added mass derivative prime II system 'Y_v_point_prime':
Sway_Acceleration_Added_Mass_Derivative_Prime = Sway_Acceleration_Added_Mass_Derivative/(0.5*Density_Water_Current*Draft_Depth_Current*Length_Ship_Current^2);

% Hydrodynamics derivative 'Y_v_prime':
Sway_Velocity_Hydrodynamics_Derivative_Prime = -((pi*Draft_Depth_Current/Length_Ship_Current) - Drag_Coefficient_Current);

% Added mass derivative 'X_u_point':
Surge_Acceleration_Added_Mass_Derivative = -0.05*Mass_Of_The_Ship_Current; % Must vary in between -0.05*Mass_Ship to -0.10*Mass_Ship.

% Hydrodynamic derivative in prime system II 'X_u_point_prime':
Surge_Acceleration_Added_Mass_Derivative_Prime = Surge_Acceleration_Added_Mass_Derivative/(0.5*Density_Water_Current*...
                                                 Draft_Depth_Current*Length_Ship_Current^2);

% Hydrodynamics derivative 'Y_r':
Sway_Yaw_Hydrodynamics_Derivative = 0.5*Density_Water_Current*Draft_Depth_Current*Velocity_Current*Length_Ship_Current^2*...
                                    (Surge_Acceleration_Added_Mass_Derivative_Prime + (Distance_Center_Of_Gravity_To_Center_Of_Pressure/Length_Ship_Current)*...
                                    Sway_Velocity_Hydrodynamics_Derivative_Prime);

% Added mass derivative 'Y_r_point':
Sway_Yaw_Added_Mass_Derivative = 0;

% Hydrodynamics derivative 'N_v':
Yaw_Velocity_Hydrodynamics_Derivative = 0.5*Density_Water_Current*Draft_Depth_Current*Velocity_Current*Length_Ship_Current^2*((Distance_Center_Of_Gravity_To_Center_Of_Pressure/Length_Ship_Current)*Sway_Velocity_Hydrodynamics_Derivative_Prime - ...
                                        (Surge_Acceleration_Added_Mass_Derivative_Prime - Sway_Acceleration_Added_Mass_Derivative_Prime));

% Added mass derivative 'N_v_point':
Yaw_Velocity_Added_Mass_Derivative = 0;

% Hydrodynamics derivative 'N_r':
Yaw_Hydrodynamics_Derivative = 0.5*Density_Water_Current*Draft_Depth_Current*Velocity_Current*Length_Ship_Current^3*(1/4)*Sway_Velocity_Hydrodynamics_Derivative_Prime;

% Added mass derivative 'N_r_point':
Yaw_Added_Mass_Derivative = -0.01*Inertia_Moment; 

% Hydrodynamics derivative 'Y_delta':
Sway_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Current*Length_Ship_Current*Draft_Depth_Current*Velocity_Current^2*...
                                        ((pi/4)*(Longeron_Area_Current/(Length_Ship_Current*Draft_Depth_Current)));

% Hydrodynamics derivative 'N_delta':
Yaw_Control_Hydrodynamics_Derivative = 0.5*Density_Water_Current*Length_Ship_Current^2*Draft_Depth_Current*Velocity_Current^2*(-0.5*...
                                       ((pi/4)*(Longeron_Area_Current/(Length_Ship_Current*Draft_Depth_Current))));

% 3) Transfer function between control angle and heading & parameters:
% --------------------------------------------------------------------
% Inertia matrix terms:
Inertia_Matrix_11 = Mass_Of_The_Ship_Current - Sway_Acceleration_Added_Mass_Derivative;
Inertia_Matrix_12 = Mass_Of_The_Ship_Current*Position_Center_Of_Gravity - Sway_Yaw_Added_Mass_Derivative;
Inertia_Matrix_21 = Mass_Of_The_Ship_Current*Position_Center_Of_Gravity - Yaw_Velocity_Added_Mass_Derivative;
Inertia_Matrix_22 = Inertia_Moment - Yaw_Added_Mass_Derivative;

% The associated matrix:
Inertia_Matrix = [Inertia_Matrix_11 Inertia_Matrix_12;Inertia_Matrix_21 Inertia_Matrix_22];

% The determinent of the associated matrix:
Determinant_Inertia_Matrix = det(Inertia_Matrix);

% Cross-check:
Delta_Check = 1e-3;
Determinant_Inertia_Matrix_Crosscheck = Inertia_Matrix_11*Inertia_Matrix_22 - Inertia_Matrix_21*Inertia_Matrix_12;
if abs(Determinant_Inertia_Matrix_Crosscheck - Determinant_Inertia_Matrix) > Delta_Check
    error('The determinant of the matrices are different');
end

% The coriolis centripetal perturbations terms matrix:
Coriolis_Centripetal_11 = -Sway_Velocity_Hydrodynamics_Derivative;
Coriolis_Centripetal_12 = Mass_Of_The_Ship_Current*Velocity_Current - Sway_Yaw_Hydrodynamics_Derivative;
Coriolis_Centripetal_21 = -Yaw_Velocity_Hydrodynamics_Derivative;
Coriolis_Centripetal_22 = Mass_Of_The_Ship_Current*Position_Center_Of_Gravity*Velocity_Current - Yaw_Hydrodynamics_Derivative;

% The associated matrix:
Coriolis_Centripetal = [Coriolis_Centripetal_11 Coriolis_Centripetal_12;Coriolis_Centripetal_21 Coriolis_Centripetal_22];

% The determinant of the associated matrix:
Determinant_Coriolis_Centripetal_Matrix = det(Coriolis_Centripetal);

% The input control matrix terms:
Control_Inputs_Rudder_11 = Sway_Control_Hydrodynamics_Derivative; % 'Y_delta'.
Control_Inputs_Rudder_21 = Yaw_Control_Hydrodynamics_Derivative; % 'N_delta'.

% Laplace's variable for transfer function definition:
s = tf('s'); 

% Designated nominal model determination:
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
    error('Time cannot be imaginary.');
end
% Define the static gain 'K_r':
Static_Gain = (Coriolis_Centripetal_21*Control_Inputs_Rudder_11 - Coriolis_Centripetal_11*Control_Inputs_Rudder_21)/...
               Determinant_Coriolis_Centripetal_Matrix;
% Define the time constant number 3:
Time_Constant_3 = (Inertia_Matrix_21*Control_Inputs_Rudder_11 - Inertia_Matrix_11*Control_Inputs_Rudder_21)/...
                  (Static_Gain*Determinant_Coriolis_Centripetal_Matrix);
% Static gain Nomoto:
Static_Gain_Nomoto = -Static_Gain;
% First order Nomoto's model:
Time_Constant_Total = Time_Constant_1 + Time_Constant_2 - Time_Constant_3;
if Time_Constant_Total < 0
    warning('Equivalent time constant is negative');
end
Heading_On_Rudder_Transfer_First_Order = (Static_Gain_Nomoto)/(s*(1 + Time_Constant_Total*s));

% Compute the final heading angle:
Heading_Ship_Generated = Heading_On_Rudder_Transfer_First_Order*Rudder_Commanded_Current;

end

