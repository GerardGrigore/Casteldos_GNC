#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // As for the magnetometer gyrocompass, calculate here the heading:
  int x, y, z;
  // x = event.magnetic.x;
  x = mag.raw.x;
  // y = event.magnetic.y;
  y = mag.raw.y;
  // z = event.magnetic.z;
  z = mag.raw.z;
  // Local heading:
  float HEADING_IN_RAD = atan2(y, x);
  // In degrees:
  float HEADING_IN_DEG = HEADING_IN_RAD*(180/PI); // OPENWORK: find a generic PI notation variable.
  // Declination angle around Sigean:
  float DECLINATION_IN_DEG = 2.116; // In degrees.
  float DECLINATION_IN_RAD = DECLINATION_IN_DEG * (PI/180); // In radians.
  // Compute the true Heading in degrees:
  HEADING_IN_DEG = HEADING_IN_DEG + DECLINATION_IN_DEG;
  // Protection for overlapping:
  if (HEADING_IN_DEG < 0) {
    HEADING_IN_DEG = HEADING_IN_DEG + 360;
  }

  // Then print the results on the Serial monitor:
  Serial.print("Heading: ");
  Serial.print(HEADING_IN_DEG);
  Serial.print(" ");

  /* Note: You can also get the raw (non unified values) for */
  /* the last data sample as follows. The .getEvent call populates */
  /* the raw values used below. */
  // Serial.print("X Raw: "); Serial.print(mag.raw.x); Serial.print("  ");
  // Serial.print("Y Raw: "); Serial.print(mag.raw.y); Serial.print("  ");
  // Serial.print("Z Raw: "); Serial.print(mag.raw.z); Serial.println("");

  /* Delay before the next sample */
  delay(500);
}
