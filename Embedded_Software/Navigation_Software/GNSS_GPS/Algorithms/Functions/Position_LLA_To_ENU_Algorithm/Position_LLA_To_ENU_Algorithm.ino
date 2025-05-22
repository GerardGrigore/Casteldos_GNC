// This function serves as a mean to convert the positions provided
// by the GPS in the LLA frame into the ENU frame.

// Creation of the structure output aimed:
// ---------------------------------------
struct COORDINATES_IN_ENU_FRAME{
  float EAST_ENU_FRAME;
  float NORTH_ENU_FRAME;
  float UP_ENU_FRAME;
};

// Creation of the function:
// -------------------------
COORDINATES_IN_ENU_FRAME FUNCTION_LLA_TO_ENU(float LATITUDE_CURRENT, float LONGITUDE_CURRENT, float ALTITUDE_CURRENT,
float LATITUDE_REF, float LONGITUDE_REF, float ALTITUDE_REF,
float SEMI_MAJOR_AXIS, float FLATTENING){
  // Declare all the used variables in the function:
  float PHI_CURRENT;
  float LAMBDA_CURRENT; 
  float PHI_REFERENCE;
  float LAMBDA_REFERENCE;
  float ECCENTRICITY_SQUARED;
  float ELLIPSOID_CURVATURE_REFERENCE;
  float X_ECEF_REFERENCE;
  float Y_ECEF_REFERENCE;
  float Z_ECEF_REFERENCE;
  float ELLIPSOID_CURVATURE_CURRENT;
  float X_ECEF_CURRENT;
  float Y_ECEF_CURRENT;
  float Z_ECEF_CURRENT;
  float X_DELTA_ECEF;
  float Y_DELTA_ECEF;
  float Z_DELTA_ECEF;
  float EAST_ENU_FRAME_TEMP;
  float NORTH_ENU_FRAME_TEMP;
  float UP_ENU_FRAME_TEMP;
  // Then create the object:
  COORDINATES_IN_ENU_FRAME COORDINATES_IN_ENU;
  // Conversion of the input angles in radians:
  PHI_CURRENT = LATITUDE_CURRENT*(PI/180);
  LAMBDA_CURRENT = LONGITUDE_CURRENT*(PI/180);
  // Same conversion for the reference points:
  PHI_REFERENCE = LATITUDE_REF*(PI/180);
  LAMBDA_REFERENCE = LONGITUDE_REF*(PI/180);
  // Orbital parameter definition:
  ECCENTRICITY_SQUARED = FLATTENING*(2 - FLATTENING);
  // Determine the ECEF coordinates of the reference frame:
  ELLIPSOID_CURVATURE_REFERENCE = SEMI_MAJOR_AXIS/sqrt(1 - ECCENTRICITY_SQUARED*sq(sin(PHI_REFERENCE)));
  X_ECEF_REFERENCE = (ELLIPSOID_CURVATURE_REFERENCE + ALTITUDE_REF)*cos(PHI_REFERENCE)*cos(LAMBDA_REFERENCE);
  Y_ECEF_REFERENCE = (ELLIPSOID_CURVATURE_REFERENCE + ALTITUDE_REF)*cos(PHI_REFERENCE)*sin(LAMBDA_REFERENCE);
  Z_ECEF_REFERENCE = ((1 - ECCENTRICITY_SQUARED)*ELLIPSOID_CURVATURE_REFERENCE + ALTITUDE_REF)*sin(PHI_REFERENCE);
  // Determine the ECEF coordinates of the current point:
  ELLIPSOID_CURVATURE_CURRENT = SEMI_MAJOR_AXIS/sqrt(1 - ECCENTRICITY_SQUARED*sq(sin(PHI_CURRENT)));
  X_ECEF_CURRENT = (ELLIPSOID_CURVATURE_CURRENT + ALTITUDE_CURRENT)*cos(PHI_CURRENT)*cos(LAMBDA_CURRENT);
  Y_ECEF_CURRENT = (ELLIPSOID_CURVATURE_CURRENT + ALTITUDE_CURRENT)*cos(PHI_CURRENT)*sin(LAMBDA_CURRENT);
  Z_ECEF_CURRENT = ((1 - ECCENTRICITY_SQUARED)*ELLIPSOID_CURVATURE_CURRENT + ALTITUDE_CURRENT)*sin(PHI_CURRENT);
  // Determine the DELTA between the two vectors in ECEF:
  X_DELTA_ECEF = X_ECEF_CURRENT - X_ECEF_REFERENCE;
  Y_DELTA_ECEF = Y_ECEF_CURRENT - Y_ECEF_REFERENCE;
  Z_DELTA_ECEF = Z_ECEF_CURRENT - Z_ECEF_REFERENCE;
  // Compute the output elements and store them into temporary variables:
  EAST_ENU_FRAME_TEMP = -sin(LAMBDA_REFERENCE)*X_DELTA_ECEF + cos(LAMBDA_REFERENCE)*Y_DELTA_ECEF;
  NORTH_ENU_FRAME_TEMP = -sin(PHI_REFERENCE)*cos(LAMBDA_REFERENCE)*X_DELTA_ECEF - sin(PHI_REFERENCE)*sin(LAMBDA_REFERENCE)*Y_DELTA_ECEF + cos(PHI_REFERENCE)*Z_DELTA_ECEF;
  UP_ENU_FRAME_TEMP = cos(PHI_REFERENCE)*cos(LAMBDA_REFERENCE)*X_DELTA_ECEF + cos(PHI_REFERENCE)*sin(LAMBDA_REFERENCE)*Y_DELTA_ECEF + sin(PHI_REFERENCE)*Z_DELTA_ECEF;
  // Finally feed the vector-like output structure:
  COORDINATES_IN_ENU.EAST_ENU_FRAME = EAST_ENU_FRAME_TEMP;
  COORDINATES_IN_ENU.NORTH_ENU_FRAME = NORTH_ENU_FRAME_TEMP;
  COORDINATES_IN_ENU.UP_ENU_FRAME = UP_ENU_FRAME_TEMP;
  return COORDINATES_IN_ENU;
}

void setup() {
  // The function allows to compute the coordinates in the ENU frame
  // based on measurements in the LLA frame.
  // Begin the serial monitor that will be used to print the data:
  Serial.begin(115200);  

}

void loop() {
  // put your main code here, to run repeatedly:

}
