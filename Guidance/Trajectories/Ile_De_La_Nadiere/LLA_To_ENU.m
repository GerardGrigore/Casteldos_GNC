function ENU_Vector = LLA_To_ENU(Latitude_Current,Longitude_Current,Altitude_Current,...
                                 Latitude_Reference,Longitude_Reference,Altitude_Reference,...
                                 Semi_Major_Axis,Flattening)

% LLA components definition for the current point:
Phi_Current = Latitude_Current*pi/180;
Lambda_Current = Longitude_Current*pi/180;

% LLA components definition for the reference point:
Phi_Reference = Latitude_Reference*pi/180;
Lambda_Reference = Longitude_Reference*pi/180;

% Orbital planetary parameters:
Eccentricity_Squared = Flattening*(2 - Flattening);

% Determine the ECEF coordinates of the reference point:
Ellipsoid_Curvature_Reference = Semi_Major_Axis/sqrt(1 - Eccentricity_Squared*sin(Phi_Reference)^2);
X_ECEF_REF = (Ellipsoid_Curvature_Reference + Altitude_Reference)*cos(Phi_Reference)*cos(Lambda_Reference);
Y_ECEF_REF = (Ellipsoid_Curvature_Reference + Altitude_Reference)*cos(Phi_Reference)*sin(Lambda_Reference);
Z_ECEF_REF = ((1 - Eccentricity_Squared)*Ellipsoid_Curvature_Reference + Altitude_Reference)*sin(Phi_Reference);

% Determine the ECEF coordinates of the current point:
Ellipsoid_Curvature_Current = Semi_Major_Axis/sqrt(1 - Eccentricity_Squared*sin(Phi_Current)^2);
X_ECEF_CURR = (Ellipsoid_Curvature_Current + Altitude_Current)*cos(Phi_Current)*cos(Lambda_Current);
Y_ECEF_CURR = (Ellipsoid_Curvature_Current + Altitude_Current)*cos(Phi_Current)*sin(Lambda_Current);
Z_ECEF_CURR = ((1 - Eccentricity_Squared)*Ellipsoid_Curvature_Current + Altitude_Current)*sin(Phi_Current);

% Determine the DELTA between the two vectors in ECEF:
X_DELTA_ECEF = X_ECEF_CURR - X_ECEF_REF;
Y_DELTA_ECEF = Y_ECEF_CURR - Y_ECEF_REF;
Z_DELTA_ECEF = Z_ECEF_CURR - Z_ECEF_REF;

% Compute the transition matrix from ECEF to ENU:
Matrix_ECEF_To_ENU = [-sin(Lambda_Reference) cos(Lambda_Reference) 0;
                      -sin(Phi_Reference)*cos(Lambda_Reference) -sin(Phi_Reference)*sin(Lambda_Reference) cos(Phi_Reference);
                      cos(Phi_Reference)*cos(Lambda_Reference) cos(Phi_Reference)*sin(Lambda_Reference) sin(Phi_Reference)];

% Compute the coordinates in the ENU frame:
ENU_Vector = Matrix_ECEF_To_ENU*[X_DELTA_ECEF;Y_DELTA_ECEF;Z_DELTA_ECEF];

end