// Algorithm description:
// -----------------------
// This algorithm allows to determine the true North for navigation purpose
// based on the current Declination of the current position and the measured 
// Magnetic Noth.

// I) Include the several used libraries:
// --------------------------------------
#include <Wire.h>
#include <QMC5883LCompass.h>

// II) Body:
// ---------
QMC5883LCompass compass; // Declaration of the QMC5883L object as a compass object.

void setup() {
  // Initialization of the I2C interface:
  Wire.begin();
  // Initialization of the serial monitor:
  Serial.begin(9600);
  // Initialize the magnetometer:
  // compass.calibrate(); // OPENWORK: Try to see of the function allows the direct transcription of the obtained calibration.
  // For the previous OPENWORK, maybe just import the line inside the algorithm of calibration.
  compass.setCalibrationOffsets(-436.00, -1054.00, 956.00);
  compass.setCalibrationScales(0.84, 0.99, 1.25);
  compass.init();
}

void loop() {
  
  // Int variables declaration for each axis:
  int x, y, z;
  // Later, conversion of heading in degrees to cardinal:
  // Then, declaration of a string named Cardinal:
  String Cardinal;
  // Store the readings in x, y & z variables:
  compass.read();
  // Then get the values:
  x = compass.getX();
  y = -compass.getY(); // Because soldered in the opposite orientation.
  z = -compass.getZ(); // Because soldered in the opposite orientation.

  // Local heading:
  float HEADING_IN_RAD = atan2(y, x);
  // In degrees:
  float HEADING_IN_DEG = HEADING_IN_RAD*(180/PI); 
  // Declination angle around Sigean:
  float DECLINATION_IN_DEG = 2.116; // In degrees.
  float DECLINATION_IN_RAD = DECLINATION_IN_DEG * (PI/180); // In radians.
  // Compute the true Heading in degrees:
  HEADING_IN_DEG = HEADING_IN_DEG + DECLINATION_IN_DEG;

  if (HEADING_IN_DEG < 0) {
    HEADING_IN_DEG = HEADING_IN_DEG + 360;
  }

  // For cardinal directions:
  if (HEADING_IN_DEG > 348.75 || HEADING_IN_DEG < 11.25) {
    Cardinal = " N";
  }

  else if (HEADING_IN_DEG > 11.25 && HEADING_IN_DEG < 33.75) {
    Cardinal = " NNE";
  }

  else if (HEADING_IN_DEG > 33.75 && HEADING_IN_DEG < 56.25) {
    Cardinal = " NE";
  }

  else if (HEADING_IN_DEG > 56.25 && HEADING_IN_DEG < 78.75) {
    Cardinal = " ENE";
  }

  else if (HEADING_IN_DEG > 78.75 && HEADING_IN_DEG < 101.25) {
    Cardinal = " E";
  }

  else if (HEADING_IN_DEG > 101.25 && HEADING_IN_DEG < 123.75) {
    Cardinal = " ESE";
  }

  else if (HEADING_IN_DEG > 123.75 && HEADING_IN_DEG < 146.25) {
    Cardinal = " SE";
  }

  else if (HEADING_IN_DEG > 146.25 && HEADING_IN_DEG < 168.75) {
    Cardinal = " SSE";
  }

  else if (HEADING_IN_DEG > 168.75 && HEADING_IN_DEG < 191.25) {
    Cardinal = " S";
  }

  else if (HEADING_IN_DEG > 191.25 && HEADING_IN_DEG < 213.75) {
    Cardinal = " SSW";
  }

  else if (HEADING_IN_DEG > 213.75 && HEADING_IN_DEG < 236.25) {
    Cardinal = " SW";
  }

  else if (HEADING_IN_DEG > 236.25 && HEADING_IN_DEG < 258.75) {
    Cardinal = " WSW";
  }

  else if (HEADING_IN_DEG > 258.75 && HEADING_IN_DEG < 281.25) {
    Cardinal = " W";
  }

  else if (HEADING_IN_DEG > 281.25 && HEADING_IN_DEG < 303.75) {
    Cardinal = " WNW";
  }

  else if (HEADING_IN_DEG > 303.75 && HEADING_IN_DEG < 326.25) {
    Cardinal = " NW";
  }

  else if (HEADING_IN_DEG > 326.25 && HEADING_IN_DEG < 348.75) {
    Cardinal = " NNW";
  }

// Then print the results on the Serial monitor:
Serial.print("Heading: ");
Serial.print(HEADING_IN_DEG);
Serial.println(Cardinal);

delay(250);

}
