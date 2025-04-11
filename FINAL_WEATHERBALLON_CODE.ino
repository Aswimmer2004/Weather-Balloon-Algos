//GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
char c;

//LSM9DS1
#include <Arduino_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

//HTS221
#include <Arduino_HTS221.h>

#include <Wire.h>


// Create an LSM9DS1 instance
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Define constants for the sensor settings
#define LSM9DS1_XGCS 0x6A  // I2C address for the gyro & accel
#define LSM9DS1_MCS 0x1C   // I2C address for the mag

//BMP280
#include  <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C Interface

//MAX31865
#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0



void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for the Serial Monitor

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);

  //LSM9DS1
  // Initialize the sensor
  if (!lsm.begin()) {
    Serial.println(F("Failed to find LSM9DS1 chip"));
    while (1) delay(10);
  }
 
  // Set ranges for accelerometer, gyroscope, and magnetometer
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  Serial.println(F("LSM9DS1 Found!"));

  //HTS221
  while (!Serial);

  if (!HTS.begin()) {
    Serial.println(F("Failed to initialize humidity temperature sensor!"));
    while (1);
  }

  //BMP280
  Serial.println(F("BMP280 test"));

  if  (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor,  check wiring!"));
    while (1);
  }

  /* Default settings from datasheet.  */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);  /* Standby time. */

  

  //MAX31865
  Serial.println(F("Adafruit MAX31865 PT100 Sensor Test!"));

  thermo.begin(MAX31865_3WIRE);
}


void loop() {

  /*GPS
    clearGPS();

    while (!GPS.newNMEAreceived()) {
      c = GPS.read();
    }

  */
  //GPS.parse(GPS.lastNMEA());

  Serial.print(F("Fix: "));
  Serial.print(GPS.fix);
  Serial.print(F(" quality: "));
  Serial.println(GPS.fixquality);
  Serial.print(F("Satellites: "));
  Serial.println(GPS.satellites);

  if (true) {
    Serial.print(F("Location: "));
    Serial.print(GPS.latitude);
    Serial.print(GPS.lat);
    Serial.print(F(", "));
    Serial.print(GPS.longitude);
    Serial.println(GPS.lon);
    Serial.print(F("Google Maps location: "));
    Serial.print(GPS.latitudeDegrees);
    Serial.print(F(", "));
    Serial.println(GPS.longitudeDegrees);

    Serial.print(F("Speed (knots): "));
    Serial.println(GPS.speed);
    Serial.print(F("Heading: "));
    Serial.println(GPS.angle);
    Serial.print(F("Altitude: "));
    Serial.println(GPS.altitude);
  }
  Serial.println(F("-------------------------------------"));

  //LSM9DS1

  // Get new sensor events
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);

  // Print accelerometer data
  Serial.println(F(" "));
  Serial.println("LSM9DS1");
  Serial.print(F("Accel X: ")); Serial.print(accel.acceleration.x); Serial.print(F(" m/s^2 "));
  Serial.print(F("Y: ")); Serial.print(accel.acceleration.y); Serial.print(F(" m/s^2 "));
  Serial.print(F("Z: ")); Serial.print(accel.acceleration.z); Serial.println(F(" m/s^2"));

  // Print gyroscope data
  Serial.print(F("Gyro X: ")); Serial.print(gyro.gyro.x); Serial.print(F(" rad/s "));
  Serial.print(F("Y: ")); Serial.print(gyro.gyro.y); Serial.print(F(" rad/s "));
  Serial.print(F("Z: ")); Serial.print(gyro.gyro.z); Serial.println(F(" rad/s"));

  // Print magnetometer data
  Serial.print(F("Mag X: ")); Serial.print(mag.magnetic.x); Serial.print(F(" uT "));
  Serial.print(F("Y: ")); Serial.print(mag.magnetic.y); Serial.print(F(" uT "));
  Serial.print(F("Z: ")); Serial.print(mag.magnetic.z); Serial.println(F(" uT"));

  // Print temperature data
  //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");

  Serial.println();  // Add a blank line between readings


  //HTS221
  float temperature = HTS.readTemperature();
  float humidity    = HTS.readHumidity();

  // print each of the sensor values
  Serial.println(F(" "));
  Serial.println(F("HTS221"));
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(F(" °C"));

  Serial.print(F("Humidity    = "));
  Serial.print(humidity);
  Serial.println(F(" %"));

  // print an empty line
  Serial.println();

  


  //BMP280
  Serial.println(F("BMP280"));
  Serial.print(F("Temperature  = "));
  Serial.print(bmp.readTemperature());
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure()/100);  //displaying the Pressure in hPa, you can change the unit
  Serial.println(F("  hPa"));

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1019.66));  //The "1019.66" is the pressure(hPa) at sea level in day in your region
  Serial.println(F("  m"));                    //If you don't know it, modify it until you get your current  altitude

  Serial.println();


  //MAX31865
  uint16_t rtd = thermo.readRTD();

  Serial.print(F("RTD value: ")); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.println(F(" "));
  Serial.println(F("MAX31865"));
  Serial.print(F("Ratio = ")); Serial.println(ratio,8);
  Serial.print(F("Resistance = ")); Serial.println(RREF*ratio,8);
  Serial.print(F("Temperature = ")); Serial.println(thermo.temperature(RNOMINAL, RREF));
  Serial.print(F(" "));
  Serial.print(F(" "));
  Serial.print(F(" "));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print(F("Fault 0x")); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println(F("RTD High Threshold")); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println(F("RTD Low Threshold")); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println(F("REFIN- > 0.85 x Bias")); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println(F("REFIN- < 0.85 x Bias - FORCE- open")); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println(F("RTDIN- < 0.85 x Bias - FORCE- open")); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println(F("Under/Over voltage")); 
    }
    thermo.clearFault();
  }
  Serial.println();
  
  
  float HTS_temp = HTS.readTemperature();
  float BMP_temp = bmp.readTemperature();
  float MAX_temp = thermo.temperature(RNOMINAL, RREF);

  float HTS_BMP_Dif = abs(HTS_temp - BMP_temp)/HTS_temp;
  float HTS_MAX_Dif = abs(HTS_temp - MAX_temp)/MAX_temp;
  float MAX_BMP_Dif = abs(MAX_temp - BMP_temp)/MAX_temp;
  

  if ((HTS_BMP_Dif <= 0.1) && (HTS_MAX_Dif <= 0.1) && (MAX_BMP_Dif <= 0.1) || (accel.acceleration.x != 0))
  {
    Serial.println(F("All temperature sensors are working!"));
  }
  else
  {
    if (HTS_BMP_Dif <= 0.1)
    {
      if ((HTS_MAX_Dif > 0.1) && (MAX_BMP_Dif > 0.1))
      {
        Serial.println(F("Likely Issue with MAX31865 Temperature Reading"));
      }
      else if (HTS_MAX_Dif > 0.1)
      {
        Serial.println(F("Discrepency between MAX31865 and HTS221 Temperature Readings"));
      }
      else if (MAX_BMP_Dif > 0.1)
      {
        Serial.println(F("Discrepency between MAX31865 and BMP280 Temperature Readings"));
      }
    }

    if (HTS_MAX_Dif <= 0.1)
    {
      if ((HTS_BMP_Dif > 0.1) && (MAX_BMP_Dif > 0.1))
      {
        Serial.println(F("Likely Issue with BMP280 Temperature Reading"));
      }
      else if (HTS_BMP_Dif > 0.1)
      {
        Serial.println(F("Discrepency between BMP280 and HTS221 Temperature Readings"));
      }
      else if (MAX_BMP_Dif > 0.1)
      {
        Serial.println(F("Discrepency between MAX31865 and BMP280 Temperature Readings"));
      }
    }

    if (MAX_BMP_Dif <= 0.1)
    {
      if ((HTS_MAX_Dif > 0.1) && (HTS_BMP_Dif > 0.1))
      {
        Serial.println(F("Likely Issue with HTS221 Temperature Reading"));
      }
      else if (HTS_MAX_Dif > 0.1)
      {
        Serial.println(F("Discrepency between MAX31865 and HTS221 Temperature Readings"));
      }
      else if (HTS_BMP_Dif > 0.1)
      {
        Serial.println(F("Discrepency between HTS221 and BMP280 Temperature Readings"));
      }
    }
  }
  
  delay(3000);
  Serial.println("///////////////////////////////////////////////////////////////////////////////////////////////");

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
