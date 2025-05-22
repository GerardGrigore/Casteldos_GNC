// This algorithm serves at gathering the used information provided
// by the 9-DOF IMU in order to proceed to the sensor fusion of the data.
// The considered data for the sensor fusion provided by the IMU is:
// * ANGULAR_RATES: provided by the 3-axis gyroscopes.
// * ACCELERATION: provided by the 3-axis accelerometers.
// * HEADING: provided by the 3-axis magnetometers (not mandatory).

// 1) Import the used libraries:
// -----------------------------
#include <Wire.h>
#include <L3G.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// Create the gyroscope object:
L3G IMU_GYROSCOPE;

// Create the unique ID associated to the IMU sensor:
Adafruit_LSM303_Accel_Unified LINEAR_ACCELERATION = Adafruit_LSM303_Accel_Unified(654321);

void setup() {
  // Define the baud rate:
  Serial.begin(9600);
  Wire.begin();
  // Handle the no detection:
  if(!IMU_GYROSCOPE.init())
  {
    // Then the wirings must be checked:
    Serial.println("No IMU detected, check the wirings.");
    while(1);
  }
  // Then initialize the gyroscope value:
  IMU_GYROSCOPE.enableDefault();
}

void loop() {
  // Retreive all the used data in this loop:
  // ------------------------------------------------------------------------------------------------------------------
  // 1) Accelerations along the 3-axis of the accelerometers of the IMU
  // ------------------------------------------------------------------------------------------------------------------
  // Compute the acceleration:
  // Get a new sensor event:
  sensors_event_t event;
  LINEAR_ACCELERATION.getEvent(&event);
  // Store the results:
  float LINEAR_ACCELERATION_ALONG_X = event.acceleration.x;
  float LINEAR_ACCELERATION_ALONG_Y = event.acceleration.y;
  float LINEAR_ACCELERATION_ALONG_Z = event.acceleration.z;
  //Serial.print("LINEAR_ACCELERATION_X: "); Serial.print(LINEAR_ACCELERATION_ALONG_X); Serial.print("  ");
  //Serial.print("LINEAR_ACCELERATION_Y: "); Serial.print(LINEAR_ACCELERATION_ALONG_Y); Serial.print("  ");
  //Serial.print("LINEAR_ACCELERATION_Z: "); Serial.print(LINEAR_ACCELERATION_ALONG_Z); Serial.print("  ");
  //Serial.println("m/s² ");

  // ------------------------------------------------------------------------------------------------------------------
  // 2) Angular ranges along the 3-axis of the gyroscopes of the IMU
  // ------------------------------------------------------------------------------------------------------------------
  // Compute the angular ranges:
  IMU_GYROSCOPE.read();
  // Angular rates in degrees per seconds:
  int ANGULAR_RATES_ALONG_X_DPS = IMU_GYROSCOPE.g.x;
  int ANGULAR_RATES_ALONG_Y_DPS = IMU_GYROSCOPE.g.y;
  int ANGULAR_RATES_ALONG_Z_DPS = IMU_GYROSCOPE.g.z;
  // Serial.print("ANGULAR_RATES_X_DPS: "); Serial.print(ANGULAR_RATES_ALONG_X_DPS); Serial.print("  ");
  // Serial.print("ANGULAR_RATES_Y_DPS: "); Serial.print(ANGULAR_RATES_ALONG_Y_DPS); Serial.print("  ");
  // Serial.print("ANGULAR_RATES_Z_DPS: "); Serial.print(ANGULAR_RATES_ALONG_Z_DPS); Serial.print("  ");
  // Serial.println("deg/s ");
  // Angular rates in radians per seconds:
  float ANGULAR_RATES_ALONG_X_RPS = IMU_GYROSCOPE.g.x*(PI/180);
  float ANGULAR_RATES_ALONG_Y_RPS = IMU_GYROSCOPE.g.y*(PI/180);
  float ANGULAR_RATES_ALONG_Z_RPS = IMU_GYROSCOPE.g.z*(PI/180);
  Serial.print("ANGULAR_RATES_X_RPS: "); Serial.print(ANGULAR_RATES_ALONG_X_RPS); Serial.print("  ");
  Serial.print("ANGULAR_RATES_Y_RPS: "); Serial.print(ANGULAR_RATES_ALONG_Y_RPS); Serial.print("  ");
  Serial.print("ANGULAR_RATES_Z_RPS: "); Serial.print(ANGULAR_RATES_ALONG_Z_RPS); Serial.print("  ");
  Serial.println("rad/s ");

  // ------------------------------------------------------------------------------------------------------------------
  // 3) Heading along the 3-axis of the magnetometers of the IMU
  // ------------------------------------------------------------------------------------------------------------------
  // Not used

  delay(500);

}
