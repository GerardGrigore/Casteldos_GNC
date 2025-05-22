// --------------------------------------------------------------------------------------------
// I) Include all the used libraries:
// --------------------------------------------------------------------------------------------
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// --------------------------------------------------------------------------------------------
// II) Fixe the Hardware physical connections & features:
// --------------------------------------------------------------------------------------------
// Define the used pins of the Microprocessor and the GNSS connection:
static const int RXPin = 4, TXPin = 3;
// NOTA - Rx will be the pin on which the data is Received.
//      - Tx will be the pin on which the data is Transmitted.

// A) GPS/GNSS object:
// -------------------
// Define the GPS baud rate:
static const uint32_t GPSBaud = 9600;
// Define the TinyGPSPlus object:
TinyGPSPlus gps;
// Define the serial connection to the GPS device:
SoftwareSerial Serial_Connection(RXPin, TXPin);

// ***********************CODE FUNCTIONALITIES*************************************************
// --------------------------------------------------------------------------------------------
// A) Determination of the LLA to ENU coordinates:
// --------------------------------------------------------------------------------------------
// ********************************************************************************************

// Definition of the reference point coordinate as a global variable:
// ------------------------------------------------------------------
// The latitude, longitude and altitude at a near current location:
float LATITUDE_REF_BLR = 43.022316;
float LONGITUDE_REF_BLR = 2.982227;
float ALTITUDE_REF_BLR = 0;
// Definition of other global variables:
float SEMI_MAJOR_AXIS = 6378137.0;
float FLATTENING = 1/298.257223563;

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

// --------------------------------------------------------------------------------------------
// III) Body of the algorithm:
// --------------------------------------------------------------------------------------------
void setup() {
  // Begin the serial monitor that will be used to print the data:
  Serial.begin(115200);
  Serial_Connection.begin(GPSBaud);
  // Indicate that the measurements will be available:
  Serial.println(F("The GPS measurements will be computed."));
}

void loop() {
  // This code displays information every time a new sentence is correctly encoded.
  // If the measurements are available, then display the coordinates with respect
  // to a reference point given. In order to determine the current latitude, longitude
  // and altitude, declare:
	while (Serial_Connection.available() > 0) {
		if (gps.encode(Serial_Connection.read())) {
			// GNSS_POSITION_MEASUREMENTS();
      // Insert the content of the old GNSS_POSITION_MEASUREMENTS here to ease the computation:
      Serial.print(F("Location: "));
      if (gps.location.isValid()) {
        // Then store the current latitude, longitude and altitude parameters:
        float LATITUDE_CURRENT;
        float LONGITUDE_CURRENT;
        float ALTITUDE_CURRENT;
        LATITUDE_CURRENT = gps.location.lat();
        LONGITUDE_CURRENT = gps.location.lng();
        ALTITUDE_CURRENT = gps.altitude.meters();
        // Then call the function:
        COORDINATES_IN_ENU_FRAME COORDINATES_IN_ENU = FUNCTION_LLA_TO_ENU(LATITUDE_CURRENT,LONGITUDE_CURRENT,ALTITUDE_CURRENT,LATITUDE_REF_BLR,
        LONGITUDE_REF_BLR,ALTITUDE_REF_BLR,SEMI_MAJOR_AXIS,FLATTENING);
        // Then print the results:
        // East with respect to the refence:
        Serial.print(COORDINATES_IN_ENU.EAST_ENU_FRAME, 6);
        Serial.print(F(","));
        // North with respect to the reference:
        Serial.print(COORDINATES_IN_ENU.NORTH_ENU_FRAME, 6);
        Serial.print(F(","));
        // Up with respect to the reference:
        Serial.print(COORDINATES_IN_ENU.UP_ENU_FRAME, 6);
        Serial.print(F(","));
      // If no measurements, then indicate:
      } else {
        Serial.print(F("INVALID"));
      }
      Serial.println();
		}
	}
  // If after 5 seconds no detection of data, then it means that the wiring might not be done
  // in the good way.
	if (millis() > 5000 && gps.charsProcessed() < 10) {
		Serial.println(F("No GPS detected: check wiring."));
		while(true);
	}
}
