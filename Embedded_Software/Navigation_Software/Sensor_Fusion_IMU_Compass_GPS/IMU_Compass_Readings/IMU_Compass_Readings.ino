// This algorithm serves as a way to merge the measurements
// provided by the magnetometer and the IMU. It shall be used
// to see that it is possible to observe the measurements
// from the magnetometer and from the IMU at the same time.

// I) Include the several used libraries:
// --------------------------------------
// For the magnetometer:
#include <Wire.h>
#include <QMC5883LCompass.h>
// For the IMU:
#include <L3G.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// II) Create the objects:
// -----------------------
QMC5883LCompass MAGNETOMETER_COMPASS; // To retreive the heading.
L3G IMU_GYROSCOPE; // To retreive the angular rates.
Adafruit_LSM303_Accel_Unified LINEAR_ACCELERATION = Adafruit_LSM303_Accel_Unified(654321); // To retreive the acceleration.

void setup() {
  // Initialization of the I2C interface:
  Wire.begin();
  // Initialization of the serial monitor:
  Serial.begin(9600);
  // Calibrate and initialize the compass:
  MAGNETOMETER_COMPASS.calibrate();
  MAGNETOMETER_COMPASS.init();
  // Handle the abscence of detection of the sensor data:
  if(!IMU_GYROSCOPE.init() || !LINEAR_ACCELERATION.begin())
  {
    // Then the wirings must be checked:
    Serial.println("No IMU nor compass detected, check the wirings.");
    while(1);
  }
  // Then initialize the gyroscope value:
  IMU_GYROSCOPE.enableDefault();
}

void loop() {
  // put your main code here, to run repeatedly:

}
