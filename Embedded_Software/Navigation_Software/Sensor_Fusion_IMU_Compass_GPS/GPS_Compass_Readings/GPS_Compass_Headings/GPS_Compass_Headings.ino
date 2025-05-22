// -----------------------------------------------------------------------------------------------
// I) Algorithm description:
// -----------------------------------------------------------------------------------------------
// This algorithm serves as a mean to measure the heading by using the
// magnetometer only and the magnetometer embedded on the GPS.

// -----------------------------------------------------------------------------------------------
// II) Import the libraries:
// -----------------------------------------------------------------------------------------------
// Common libraries:
#include <Wire.h>
// Magnetometer only libraries:
#include <QMC5883LCompass.h>
// GPS-Magnetometer libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// -----------------------------------------------------------------------------------------------
// III) Create the objects:
// -----------------------------------------------------------------------------------------------
// GPS-Magnetometer only:
Adafruit_HMC5883_Unified GPS_MAGNETOMETER = Adafruit_HMC5883_Unified(123456);
// Magnetometer:
QMC5883LCompass MAGNETOMETER;

void setup() {
  // Initialization of the I2C interface:
  Wire.begin();
  // Initialization of the serial monitor:
  Serial.begin(9600);
  // Initialize the magnetometer:
  // MAGNETOMETER.calibrate(); 
  MAGNETOMETER.init();
  // Then test to see if the connection with the GPS-Magnetometer is working:
  if(!GPS_MAGNETOMETER.begin())
  {
    // A connection problem is spotted:
    Serial.println("No connection has been established with the GPS-Magnetometer.");
    while(1);
  }

}

void loop() {
  // Then store the results into two separated variables.
  // Define the current local magnetic declination used for both means of measurements:
  float DECLINATION_MAGNETIC_LOCAL = 2.116*PI/180; // In rad.
  // 1) The GPS-Magnetometer sensor:
  // -------------------------------
  // Based on tracking and saving an event:
  sensors_event_t EVENT_MAGNETIC; 
  GPS_MAGNETOMETER.getEvent(&EVENT_MAGNETIC);
  // If needed, print the results (in micro-Tesla):
  // Serial.print("X: "); Serial.print(EVENT_MAGNETIC.magnetic.x); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(EVENT_MAGNETIC.magnetic.y); Serial.print("  ");
  // Serial.print("Z: "); Serial.print(EVENT_MAGNETIC.magnetic.z); Serial.print("  ");Serial.println("uT");
  // Hold the module so that Z-axis is pointing upward, allowing to measure the heading with x and y measurements:
  float HEADING_GPS_MAGNETOMETER = atan2(EVENT_MAGNETIC.magnetic.y, EVENT_MAGNETIC.magnetic.x);
  HEADING_GPS_MAGNETOMETER += DECLINATION_MAGNETIC_LOCAL;
  // Correct the heading when signs are reversed:
  if(HEADING_GPS_MAGNETOMETER < 0)
    HEADING_GPS_MAGNETOMETER += 2*PI;
  // Check for wrap due to addition of declination.
  if(HEADING_GPS_MAGNETOMETER > 2*PI)
    HEADING_GPS_MAGNETOMETER -= 2*PI;
  // Convert radians to degrees for readability.
  float HEADING_GPS_MAGNETOMETER_DEGREES = HEADING_GPS_MAGNETOMETER * 180/PI; 
  Serial.print("Heading from GPS-Magnetometer (degrees): "); Serial.println(HEADING_GPS_MAGNETOMETER_DEGREES);

  // 2) The Magnetometer:
  // --------------------
  // Int variables declaration for each axis:
  int x, y, z;
  // Store the readings in x, y & z variables:
  MAGNETOMETER.read();
  // Then get the values:
  x = MAGNETOMETER.getX();
  y = MAGNETOMETER.getY();
  z = MAGNETOMETER.getZ();
  // Local heading:
  float HEADING_MAGNETOMETER_IN_RAD = atan2(y, x);
  // In degrees:
  float HEADING_MAGNETOMETER_IN_DEG = HEADING_MAGNETOMETER_IN_RAD*(180/PI); 
  // Compute the true Heading in degrees:
  HEADING_MAGNETOMETER_IN_DEG = HEADING_MAGNETOMETER_IN_DEG + DECLINATION_MAGNETIC_LOCAL*180/PI;
  if (HEADING_MAGNETOMETER_IN_DEG < 0) {
    HEADING_MAGNETOMETER_IN_DEG = HEADING_MAGNETOMETER_IN_DEG + 360;
  }
  // In order to be sure to get a measurement, use the azimuth that can be always retreived:
  float AZIMUTH_MAGNETOMETER = MAGNETOMETER.getAzimuth(); // Values from -180° to 180°.
  // AZIMUTH_MAGNETOMETER = AZIMUTH_MAGNETOMETER + 180; // To get the values from 0 to 360°.
  // NOTA - Bad idea to add 180°, because the switch must then be adapted OPENWORK.
  // Then print the results on the Serial monitor:
  // Serial.print("Heading: ");
  // Serial.print(HEADING_MAGNETOMETER_IN_DEG);
  // Serial.println(Cardinal);
  Serial.print("Azimuth from Magnetometer (degrees): "); Serial.println(AZIMUTH_MAGNETOMETER);
  Serial.print("Heading from Magnetometer (degrees): "); Serial.println(HEADING_MAGNETOMETER_IN_DEG);
  delay(1000);

}
