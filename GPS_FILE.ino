#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>


SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

char c;

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
}

void loop() {

  clearGPS();

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  Serial.print("Time: ");
  Serial.print(GPS.hour, DEC);
  Serial.print(':');
  Serial.print(GPS.minute, DEC);
  Serial.print(':');
  Serial.print(GPS.seconds, DEC);
  Serial.print('.');
  Serial.println(GPS.milliseconds);

  Serial.print("Date: ");
  Serial.print(GPS.day, DEC);
  Serial.print('/');
  Serial.print(GPS.month, DEC);
  Serial.print("/20");
  Serial.println(GPS.year, DEC);

  Serial.print("Fix: ");
  Serial.print(GPS.fix);
  Serial.print(" quality: ");
  Serial.println(GPS.fixquality);
  Serial.print("Satellites: ");
  Serial.println(GPS.satellites);

if (1) {
    Serial.print(F("Location: "));
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(F(", "));
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon);
    Serial.print(F("Google Maps location: "));
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(F(", "));
    Serial.println(GPS.longitudeDegrees, 4);

    Serial.print(F("Speed (knots): "));
    Serial.println(GPS.speed);
    Serial.print(F("Heading: "));
    Serial.println(GPS.angle);
    Serial.print(F("Altitude: "));
    Serial.println(GPS.altitude);
  }
  Serial.println(F("-------------------------------------"));
}

void clearGPS() {
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}
