//LSM9DS1



#include <Arduino_LSM9DS1.h>

#include <Arduino_HTS221.h>

#include <Arduino_HTS221.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// Create an LSM9DS1 instance
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Define constants for the sensor settings
#define LSM9DS1_XGCS 0x6A  // I2C address for the gyro & accel
#define LSM9DS1_MCS 0x1C   // I2C address for the mag



//HTS221



#include <Arduino_HTS221.h>



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
  //LSM9DS1



  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for the Serial Monitor

  // Initialize the sensor
  if (!lsm.begin()) {
    Serial.println("Failed to find LSM9DS1 chip");
    while (1) delay(10);
  }
 
  // Set ranges for accelerometer, gyroscope, and magnetometer
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  Serial.println("LSM9DS1 Found!");


  //HTS221


    while (!Serial);

    if (!HTS.begin()) {
      Serial.println("Failed to initialize humidity temperature sensor!");
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


  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  thermo.begin(MAX31865_3WIRE);
}


void loop() {

  //LSM9DS1



  // Get new sensor events
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);

  // Print accelerometer data
  Serial.println(" ");
  Serial.println("LSM9DS1");
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2 ");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2 ");
  Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

  // Print gyroscope data
  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" rad/s ");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" rad/s ");
  Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

  // Print magnetometer data
  Serial.print("Mag X: "); Serial.print(mag.magnetic.x); Serial.print(" uT ");
  Serial.print("Y: "); Serial.print(mag.magnetic.y); Serial.print(" uT ");
  Serial.print("Z: "); Serial.print(mag.magnetic.z); Serial.println(" uT");

  // Print temperature data
  //Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");

  Serial.println();  // Add a blank line between readings


  //HTS221


  float temperature = HTS.readTemperature();
  float humidity    = HTS.readHumidity();

  // print each of the sensor values
  Serial.println(" ");
  Serial.println("HTS221");
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity    = ");
  Serial.print(humidity);
  Serial.println(" %");

  // print an empty line
  Serial.println();

  


  //BMP280



  
  Serial.println("BMP280");
  Serial.print(F("Temperature  = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure()/100);  //displaying the Pressure in hPa, you can change the unit
  Serial.println("  hPa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1019.66));  //The "1019.66" is the pressure(hPa) at sea level in day in your region
  Serial.println("  m");                    //If you don't know it, modify it until you get your current  altitude

  Serial.println();


  //MAX31865



  uint16_t rtd = thermo.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.println(" ");
  Serial.println("MAX31865");
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));
  Serial.print(" ");
  Serial.print(" ");
  Serial.print(" ");

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
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

  if ((HTS_BMP_Dif <= 0.1) && (HTS_MAX_Dif <= 0.1) && (MAX_BMP_Dif <= 0.1))
  {
    Serial.println("All temperature sensors are working!");
  }
  else
  {
    if (HTS_BMP_Dif <= 0.1)
    {
      if ((HTS_MAX_Dif > 0.1) && (MAX_BMP_Dif > 0.1))
      {
        Serial.println("Likely Issue with MAX31865 Temperature Reading");
      }
      else if (HTS_MAX_Dif > 0.1)
      {
        Serial.println("Discrepency between MAX31865 and HTS221 Temperature Readings");
      }
      else if (MAX_BMP_Dif > 0.1)
      {
        Serial.println("Discrepency between MAX31865 and BMP280 Temperature Readings");
      }
    }

    if (HTS_MAX_Dif <= 0.1)
    {
      if ((HTS_BMP_Dif > 0.1) && (MAX_BMP_Dif > 0.1))
      {
        Serial.println("Likely Issue with BMP280 Temperature Reading");
      }
      else if (HTS_BMP_Dif > 0.1)
      {
        Serial.println("Discrepency between BMP280 and HTS221 Temperature Readings");
      }
      else if (MAX_BMP_Dif > 0.1)
      {
        Serial.println("Discrepency between MAX31865 and BMP280 Temperature Readings");
      }
    }

    if (MAX_BMP_Dif <= 0.1)
    {
      if ((HTS_MAX_Dif > 0.1) && (HTS_BMP_Dif > 0.1))
      {
        Serial.println("Likely Issue with HTS221 Temperature Reading");
      }
      else if (HTS_MAX_Dif > 0.1)
      {
        Serial.println("Discrepency between MAX31865 and HTS221 Temperature Readings");
      }
      else if (HTS_BMP_Dif > 0.1)
      {
        Serial.println("Discrepency between HTS221 and BMP280 Temperature Readings");
      }
    }
  }
  delay(3000);
  Serial.println("///////////////////////////////////////////////////////////////////////////////////////////////");

}
