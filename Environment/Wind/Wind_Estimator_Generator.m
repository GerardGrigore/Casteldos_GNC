function [Wind_Angle_Generated,...
          Wind_Velocity_Generated,...
          Wind_Speed,...
          Wind_Direction,...
          Yaw_Wind_Coefficient_Current,...
          Heading_Induced_Wind,...
          Torque_Wind] = Wind_Estimator_Generator(Velocity_Ship,...
                                                  Table_Wind_Coefficients,...
                                                  Lenght_Ship_Overall,...
                                                  Density_Air,...
                                                  Area_Lateral_Projected_Wind,...
                                                  Inertia_Ship_Vertical_Axis,...
                                                  Time_Sampling)


% Persistent definition:
persistent Heading_Wind_Previous;
persistent Temporar_Heading_Wind_Previous;

% Generate random numbers corresponding to the wind direction.
% Aim for a specific value with a higher chance to be obtained:
Aimed_Wind_Angle = 50*(pi/180);
Sigma_Wind_Angle = 5*(pi/180);   
Wind_Angle_Generated = Aimed_Wind_Angle + Sigma_Wind_Angle*randn();

% Then generate typical velocity wind.
% Based on Beaufort Wind Force scale:
Aimed_Velocity_Wind = 5;
Sigma_Velocity_Wind = 0.2;
Wind_Velocity_Generated = Aimed_Velocity_Wind + Sigma_Velocity_Wind*randn();

% Then calculate the relative velocity of the ship with respect to the
% wind:
East_Relative_Velocity_Wind_Ship = Velocity_Ship - Wind_Velocity_Generated*cos(Wind_Angle_Generated);
Noth_Relative_Velocity_Wind_Ship = -Wind_Velocity_Generated*sin(Wind_Angle_Generated);

% Compute the Wind Speed and Direction:
Wind_Speed = sqrt(East_Relative_Velocity_Wind_Ship^2 + Noth_Relative_Velocity_Wind_Ship^2);
Wind_Direction = -atan2(Noth_Relative_Velocity_Wind_Ship,East_Relative_Velocity_Wind_Ship);

% Identification of the Yaw (or Heading) wind coefficient (reffered to as
% 'CN' usually).
Relative_Wind_Angle_Reference = Table_Wind_Coefficients(1:15,1)*(pi/180);
Yaw_Wind_Coefficients_Reference = Table_Wind_Coefficients(1:15,2);
% Then interpolate linearly to obtain the Wind coefficient for the current
% Wind_Direction:
Yaw_Wind_Coefficient_Current = interp1(Relative_Wind_Angle_Reference,Yaw_Wind_Coefficients_Reference,Wind_Direction);

% Then calculate the current resulting torque due to the wind:
Torque_Wind = (1/2)*Lenght_Ship_Overall*Area_Lateral_Projected_Wind*Yaw_Wind_Coefficient_Current*Density_Air*Wind_Speed^2;

% Then convert it to be a heading perturbation:
if isempty(Temporar_Heading_Wind_Previous)
    Temporar_Heading_Wind_Previous = 0;
end
if isempty(Heading_Wind_Previous)
    Heading_Wind_Previous = 90*(pi/180);
end
Temporar_Heading_Wind_Current = Temporar_Heading_Wind_Previous + Time_Sampling*(Torque_Wind/Inertia_Ship_Vertical_Axis);
Heading_Wind_Current = Heading_Wind_Previous + Time_Sampling*(Temporar_Heading_Wind_Previous);

% Update of the persistents:
Temporar_Heading_Wind_Previous = Temporar_Heading_Wind_Current;
Heading_Wind_Previous = Heading_Wind_Current;
Heading_Induced_Wind = Heading_Wind_Current;

end

