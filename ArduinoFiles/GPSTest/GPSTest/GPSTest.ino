#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample code demonstrates just about every built-in operation of TinyGPSPlus (TinyGPSPlus).
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
double START_LAT = 0.0;
double START_LON = 0.0;
double lat = 0.0;
double lon = 0.0;
bool firstTime = true;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// For stats that happen every 5 seconds
unsigned long last = 0UL;

void setup()
{
  Serial.begin(115200);
  Serial3.begin(GPSBaud);

  Serial.println(F("KitchenSink.ino"));
  Serial.println(F("Demonstrating nearly every feature of TinyGPSPlus"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double earthRadius = 6371000; // in meters

  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = earthRadius * c;

  return distance;
}


void loop()
{
  // Dispatch incoming characters
  while (Serial3.available() > 0)
    gps.encode(Serial3.read());

  // if (gps.speed.isUpdated())
  // {
  //   Serial.print("SPEED: ");
  //   Serial.print(gps.speed.kmph());
  //   Serial.println(" km/h");
  //   Serial.flush();
  //   Serial.print(gps.speed.kmph());
  //   Serial.print(",");
  //   Serial.println();
  // }
  
  if (gps.location.isValid())
  {
    if(firstTime){
      START_LAT = gps.location.lat();
      START_LON = gps.location.lng();
      firstTime = false;
    }
    double distance = calculateDistance(
      START_LAT, START_LON,
      gps.location.lat(), gps.location.lng()
    );
    Serial.println(distance);
  }

  else if (millis() - last > 5000)
  {
    // Serial.println();
    if (gps.location.isValid())
    {
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      double distanceToLondon =
        TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
      double courseToLondon =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);

      // Serial.print(F("LONDON     Distance="));
      // Serial.print(distanceToLondon/1000, 6);
      // Serial.print(F(" km Course-to="));
      // Serial.print(courseToLondon, 6);
      // Serial.print(F(" degrees ["));
      // Serial.print(TinyGPSPlus::cardinal(courseToLondon));
      // Serial.println(F("]"));
    }

    // Serial.print(F("DIAGS      Chars="));
    // Serial.print(gps.charsProcessed());
    // Serial.print(F(" Sentences-with-Fix="));
    // Serial.print(gps.sentencesWithFix());
    // Serial.print(F(" Failed-checksum="));
    // Serial.print(gps.failedChecksum());
    // Serial.print(F(" Passed-checksum="));
    // Serial.println(gps.passedChecksum());

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

    last = millis();
  }
  
}