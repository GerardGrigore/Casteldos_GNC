#define RXpin 3              //this is the pin that the Arduino will use to receive data from the GPS
#define TXpin 2              //this is the pin that the Arduino can use to send data (commands) to the GPS - not used

#include <SoftwareSerial.h>

SoftwareSerial GPS(RXpin, TXpin);

void loop()
{
  while (GPS.available())
  {
    Serial.write(GPS.read());
  }
}


void setup()
{
  GPS.begin(9600);
  Serial.begin(115200);
  Serial.println("26A_GPS_Echo_SoftwareSerial Starting");
}