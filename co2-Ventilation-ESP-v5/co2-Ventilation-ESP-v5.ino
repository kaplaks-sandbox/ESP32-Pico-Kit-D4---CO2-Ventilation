// Version 5.0 - using ADdafruit sdc30 lib
// added storage of ssid param to nvram
// to add way to edit via serial menu at startup
//
// To do
// web setting
// alt sets from 4 ?

#include <Wire.h>
//#define I2C_SDA 21
//#define I2C_SCL 22

//https://github.com/finitespace/BME280
#include <BME280I2C.h>

// For MKR1010
// #include <WiFiNINA.h>

#include <Adafruit_SCD30.h>
//#include "SCD30.h"
#include <EnvironmentCalculations.h>

#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
//#include <Arduino_JSON.h>

//initial wifi creds
#include "arduino_secrets.h" 
#include <Preferences.h>
Preferences preferences;

String my_version = "5.1 Beta";
// version 5.1 - added core debuging 

// Assumed environmental values:
float referencePressure = 1013.25;  // hPa local QFF (official meteor-station reading)
float outdoorTemp = 4.7;           // °C  measured local outdoor temp.
float barometerAltitude = SECRET_BAROMETERAltitude_m ; // meters ... map readings + barometer position
#define SEALEVELPRESSURE_HPA (1013.25)

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_16,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76
);

BME280I2C bme(settings);
bool bme_status;
float alt_s;
bool bme_sensor_found = false;

//OLED Settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     33   // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D   ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };


//Shelly RGB Led controller
String my_led_ip; // = SECRETLED_IP;
//String dst_shelly = "http://" + ip_led + "/color/0/set";
String dst_shelly;
//char* serverName;
String serverName;

//for send the same to OLED and serial
String tmp_msg1 = "";
String tmp_msg2 = "";

//Colour vars
String red_s, green_s, blue_s;
float red, blue, green;

//Time Vars
float scd30_T_s, scd30_H_s, scd30_CO2_s;

// the following variables are unsigned longs because the time, measured in milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
// unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;
unsigned long delayTime;

unsigned long lastTimeCalibrated = 0;
unsigned long calibrationDelay = 20000; //20 sec after boot we do a calibration

unsigned long lastCO2Time;
unsigned long timerCO2Delay = 2000;


//char my_ssid[64];// = SECRET_SSID;        // your network SSID (name)
//char my_ssid_pwd[64];// = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
String my_ssid;
String my_ssid_pwd;
int keyIndex = 0;                 // your network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;
String my_ip ;
String my_hostname ; //= SECRET_HOSTNAME ; 
String led_update_status;
int my_altitude;
bool my_verbose = false;
String my_report;
String my_report_txt;

String e_ssid;
String e_ssid_pwd;
String e_led_ip;
String e_hostname;
int e_altitude;
String e_report;
String e_report_txt;

String menu_ssid;
String menu_ssid_pwd;







Adafruit_SCD30  scd30;


WiFiServer server(80);




void printWifiStatus() {
  // print the SSID of the network you're attached to:
  display.setTextSize(1); // Draw 2X-scale text
  display.clearDisplay();
  display.setCursor(0, 0);
  
  tmp_msg1 = "SSID: ";
  tmp_msg2 = WiFi.SSID();
  display.print(tmp_msg1);
  display.println(tmp_msg2);
  Serial.print(tmp_msg1);
  Serial.println(tmp_msg2);

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  display.print("IP: ");
  display.println(ip);


  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println("");
  display.print("RSSI:");
  display.print(rssi);
  display.println(" dBm");
  display.println("");
  
  display.display();      // Show initial text
  
}

void printRGBStatus(float wavelength){
  
    Serial.println("");
    Serial.print(wavelength);
    Serial.print(" -> Setting Lights...");
    Serial.print(red);
    Serial.print(":");
    Serial.print(green);
    Serial.print(":");
    Serial.print(blue);
    Serial.println("");
}
  
void wave2RGB( float wavelength){
    float Gamma = 0.80, IntensityMax = 255, factor; 
    wavelength = map(wavelength, 300, 1000, 380, 780); //we start at 300 to 1000 to pull it more into the blue spectrum for low (+- 400) values
    
    if (my_verbose == true){
      Serial.print("Mapped-");
      Serial.print(wavelength);
    }
  
    if((wavelength >= 380) && (wavelength<440)){
      red = -(wavelength - 440) / (440 - 380);
      //red = red / 3; //aajmod
      green = 0.0;
      blue = 1.0;
    }
    else if((wavelength >= 440) && (wavelength<490)){
      red = 0.0;
      green = (wavelength - 440) / (490 - 440);
      blue = 1.0;
    }
    else if((wavelength >= 490) && (wavelength<510)){
      red = 0.0;
      green = 1.0;
      blue = -(wavelength - 510) / (510 - 490);
    }
    else if((wavelength >= 510) && (wavelength<580)){
      red = (wavelength - 510) / (580 - 510);
      green = 1.0;
      blue = 0.0;
    }
    else if((wavelength >= 580) && (wavelength<645)){
      red = 1.0;
      green = -(wavelength - 645) / (645 - 580);
      blue = 0.0;
    }
    else if((wavelength >= 645)){
      // else if((wavelength >= 645) && (wavelength<781)){
      red = 1.0;
      green = 0.0;
      blue = 0.0;
    }
    else{
      red = 0.0;
      green = 0.0;
      blue = 0.0;
    }

  
    // Let the intensity fall off near the vision limits
    ///*
    //Serial.print("Mid-");
    //printRGBStatus();
    if((wavelength >= 380) && (wavelength<420)){
      factor = 0.3 + 0.7*(wavelength - 380) / (420 - 380);
    }else if((wavelength >= 420) && (wavelength<701)){
      factor = 1.0;
    }else if((wavelength >= 701) && (wavelength<781)){
    //   }else if((wavelength >= 701) && (wavelength<781)){
      factor = 0.3 + 0.7*(780 - wavelength) / (780 - 700);
    }
    else if(wavelength >= 781){
      factor = 1;
    }
    else{
      factor = 0.0;
    }
  //  */
    



  //  Serial.print("End-");
  //  printRGBStatus();
    if (red != 0){
      //red = ((IntensityMax * red));// * pow(red * factor, Gamma));
      red = (IntensityMax * pow(red * factor, Gamma));
    };
    if (green != 0){
      //green = ((IntensityMax * green));// * pow(green * factor, Gamma));
      green = (IntensityMax * pow(green * factor, Gamma));
    };
    if (blue != 0){
      //blue = ((IntensityMax * blue)) ;//* pow(blue * factor, Gamma));
      blue = (IntensityMax * pow(blue * factor, Gamma));
    };
    
    red_s = (String)red;
    blue_s = (String)blue;
    green_s = (String)green;
  // return [red,green,blue];
  
  if (my_verbose==true){
    printRGBStatus( wavelength);
    }
    
}


void wave2RGB2(float wavelength) {
    
    Serial.print("Mapping -");
    Serial.println(wavelength);
    
    float  gamma = 0.8;
    int  intensity_max = 255;
    int factor;
 
    if (wavelength < 380) {
        red = 0; 
        green = 0; 
        blue = 1;
      }
      else if (wavelength < 440) {
            red = -(wavelength - 440) / (440 - 380);
            green = 0;
            blue = 1;
        }
        else if (wavelength < 490) {
            red = 0;
            green = (wavelength - 440) / (490 - 440);
            blue = 1;
          }
          else if (wavelength < 510) {
              red = 0; 
              green =  1;
              blue = -(wavelength - 510) / (510 - 490);
            }
            else if (wavelength < 580) {
                red = (wavelength - 510) / (580 - 510);
                green = 1; 
                blue = 0;
              }
              else if (wavelength < 645) {
                  red = 1;
                  green = -(wavelength - 645) / (645 - 580);
                  blue = 0;
                }
                else if (wavelength <= 780) {
                    red = 1; 
                    green =0; 
                    blue = 0;
                  }
                  else {
                      red = 1; 
                      green =1; 
                      blue = 1;
    }
    
    
    /** 
    // let the intensity fall of near the vision limits
    if (wavelength < 380){
          factor = 0;
        }
        else if (wavelength < 420){
            factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
          }
          else if (wavelength < 700){
              factor = 1;
            }
            else if (wavelength <= 780){
                factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
               }
               else {
                  factor = 0;
   }
 **/
 
    

  if (red != 0){
    red = ( intensity_max * pow (red * factor, gamma));
  }
  if (green != 0){
    green = (intensity_max * pow (green * factor, gamma));
  }
  if (blue != 0){
    blue = (intensity_max * pow (blue * factor, gamma));
  }
            
    red_s = (String)red;
    blue_s = (String)blue;
    green_s = (String)green;
  
  if (my_verbose==true){
    printRGBStatus( wavelength);
    }
    

}






  
void displayOLED(float C, float H, float CO2){
    display.clearWriteError();
    
      display.clearDisplay();
      display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(16, 0);
     
      display.print((C));
      display.println(" C");
       
      display.setCursor(64, 0);
      display.print((H));
      display.println("%");
      
      display.setCursor(120, 0);
      display.println((my_report));


      //display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(0, 8);
      display.print("CO2:");
      
      display.setTextSize(2); // Draw 2X-scale text
      display.setCursor(0, 16);
      display.print(("  "));
      display.print((CO2));
      display.setTextSize(1); // Draw 2X-scale text
      display.print(("ppm"));

  if (bme_sensor_found){
      display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(0, 32);
      display.print((" Alt: "));
      display.print((alt_s));
      display.setTextSize(1); // Draw 2X-scale text
      display.print(("m"));
    }
        
        IPAddress ip = WiFi.localIP();

        display.setTextSize(1); // Draw 2X-scale text
        display.setCursor(0, 40);
        display.print("  SSID:");
        display.println(WiFi.SSID());
        display.setCursor(0, 48);
        display.print(" my IP:");
        display.println((ip));
    
      display.setCursor(0, 56);
      //  long rssi = WiFi.RSSI();
      //display.print("RSSI:");
      //display.print(rssi);
      //display.println(" dBm");
      
      display.print("LED IP:");
        display.print(my_led_ip);
        display.print(led_update_status);
        display.println("");

      display.display();      // Show initial text

        //display.setTextSize(1); // Draw 2X-scale text
}


void getSetupSCD30(){
  // if ((millis() - lastTimeCalibrated) > calibrationDelay) {
  //    Serial.println("");
  Serial.println("");
  Serial.println("-----------------------------------------------");
  Serial.println("Getting SCD30 config:");
  Serial.println("---------------------");
    /*** Adjust the rate at which measurements are taken, from 2-1800 seconds */
    //  if (!scd30.setMeasurementInterval(5)) {
    //    Serial.println("Failed to set measurement interval");
    //   while(1){ delay(10);}
    // }
    Serial.print("Measurement interval: ");
    Serial.print(scd30.getMeasurementInterval());
    Serial.println(" seconds");


    /*** Restart continuous measurement with a pressure offset from 700 to 1400 millibar.
   * Giving no argument or setting the offset to 0 will disable offset correction
   */
  // if (!scd30.startContinuousMeasurement(15)){
  //   Serial.println("Failed to set ambient pressure offset");
  //   while(1){ delay(10);}
  // }
  Serial.print("Ambient pressure offset: ");
  Serial.print(scd30.getAmbientPressureOffset());
  Serial.println(" mBar");


  /*** Set an altitude offset in meters above sea level.
   * Offset value stored in non-volatile memory of SCD30.
   * Setting an altitude offset will override any pressure offset.
   */
 // if (!scd30.setAltitudeOffset(my_alt)){
 //    Serial.println("Failed to set altitude offset");
 //    while(1){ delay(10);}
 //  }
  Serial.print("Altitude offset: ");
  Serial.print(scd30.getAltitudeOffset());
  Serial.println(" meters");


    /*** Set a temperature offset in hundredths of a degree celcius.
    * Offset value stored in non-volatile memory of SCD30.
   */

    
  //if (!scd30.setTemperatureOffset((my_temp - scd30.temperature) * 100.0)){ // 19.84 degrees celcius
  //   Serial.println("Failed to set temperature offset");
  //   while(1){ delay(10);}
  // }
  Serial.print("Temperature offset: ");
  Serial.print((float)scd30.getTemperatureOffset()/100.0);
  Serial.println(" degrees C");


  /*** Force the sensor to recalibrate with the given reference value
   * from 400-2000 ppm. Writing a recalibration reference will overwrite
   * any previous self calibration values.
   * Reference value stored in non-volatile memory of SCD30.
   */
  // if (!scd30.forceRecalibrationWithReference(400)){
  //   Serial.println("Failed to force recalibration with reference");
  //   while(1) { delay(10); }
  // }
    Serial.print("Forced Recalibration reference: ");
    Serial.print(scd30.getForcedCalibrationReference());
    Serial.println(" ppm");


  /*** Enable or disable automatic self calibration (ASC).
   * Parameter stored in non-volatile memory of SCD30.
   * Enabling self calibration will override any previously set
   * forced calibration value.
   * ASC needs continuous operation with at least 1 hour
   * 400ppm CO2 concentration daily.
   */
   //if (!scd30.selfCalibrationEnabled(true)){
   //  Serial.println("Failed to enable or disable self calibration");
   //  while(1) { delay(10); }
  // }
    if (scd30.selfCalibrationEnabled()) {
      Serial.println("Self calibration enabled");
    } else {
      Serial.println("Self calibration disabled");
    }
    //lastTimeCalibrated = millis();
    // }
    Serial.println("-----------------------------------------------");
    Serial.println("");
}
  
void setupSCD30(float my_alt, float my_temp){
  
  if ((millis() - lastTimeCalibrated) > calibrationDelay) {
      Serial.println("");
      Serial.println("-----------------------------------------------");
      Serial.println("Setting SCD30 config:");
      Serial.println("---------------------");
  
  /*** Adjust the rate at which measurements are taken, from 2-1800 seconds */
      if (!scd30.setMeasurementInterval(2)) {
        Serial.println("Failed to set measurement interval");
        while(1){ delay(10);}
      }
      Serial.print("Measurement interval: ");
      Serial.print(scd30.getMeasurementInterval());
      Serial.println(" seconds");


   /*** Restart continuous measurement with a pressure offset from 700 to 1400 millibar.
   * Giving no argument or setting the offset to 0 will disable offset correction
   */
      if (!scd30.startContinuousMeasurement(15)){
        Serial.println("Failed to set ambient pressure offset");
        while(1){ delay(10);}
      }
      Serial.print("Ambient pressure offset: ");
      Serial.print(scd30.getAmbientPressureOffset());
      Serial.println(" mBar");


  /*** Set an altitude offset in meters above sea level.
   * Offset value stored in non-volatile memory of SCD30.
   * Setting an altitude offset will override any pressure offset.
   */
      if (!scd30.setAltitudeOffset(my_alt)){
        Serial.println("Failed to set altitude offset");
        while(1){ delay(10);}
      }
      Serial.print("Altitude offset: ");
      Serial.print(scd30.getAltitudeOffset());
      Serial.println(" meters");


  /*** Set a temperature offset in hundredths of a degree celcius.
  * Offset value stored in non-volatile memory of SCD30.
  */
      if (!scd30.setTemperatureOffset((my_temp + scd30.temperature) * 100.0)){ // 19.84 degrees celcius
        Serial.println("Failed to set temperature offset");
        while(1){ delay(10);}
      }
      Serial.print("Temperature offset: ");
      Serial.print((float)scd30.getTemperatureOffset()/100.0);
      Serial.println(" degrees C");


    /*** Force the sensor to recalibrate with the given reference value
    * from 400-2000 ppm. Writing a recalibration reference will overwrite
    * any previous self calibration values.
    * Reference value stored in non-volatile memory of SCD30.
    */
      if (!scd30.forceRecalibrationWithReference(400)){
        Serial.println("Failed to force recalibration with reference");
        while(1) { delay(10); }
      }
      Serial.print("Forced Recalibration reference: ");
      Serial.print(scd30.getForcedCalibrationReference());
      Serial.println(" ppm");


    /*** Enable or disable automatic self calibration (ASC).
    * Parameter stored in non-volatile memory of SCD30.
    * Enabling self calibration will override any previously set
    * forced calibration value.
    * ASC needs continuous operation with at least 1 hour
    * 400ppm CO2 concentration daily.
    */
      if (!scd30.selfCalibrationEnabled(true)){
        Serial.println("Failed to enable or disable self calibration");
        while(1) { delay(10); }
      }
      if (scd30.selfCalibrationEnabled()) {
        Serial.println("Self calibration enabled");
      } else {
        Serial.println("Self calibration disabled");
      }
      lastTimeCalibrated = millis();
  }

 Serial.println("-----------------------------------------------");
  Serial.println("");

}
  
void printBME280Data ( Stream* client) {
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_hPa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? "C" :"F"));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->print(String(presUnit == BME280::PresUnit_hPa ? "hPa" : "Pa")); // expected hPa and Pa only

   EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
   EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

   /// To get correct local altitude/height (QNE) the reference Pressure should be taken from meteorologic messages (QNH or QFF)
   float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);
   float dewPoint = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);
  alt_s = altitude;
   /// To get correct seaLevel pressure (QNH, QFF) the altitude value should be independent on measured pressure.
   /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
   float seaLevel = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, temp, pres, envAltUnit, envTempUnit);
   float absHum = EnvironmentCalculations::AbsoluteHumidity(temp, hum, envTempUnit);

   client->print("\t\tAltitude: ");
   client->print(altitude);
   client->print((envAltUnit == EnvironmentCalculations::AltitudeUnit_Meters ? "m" : "ft"));
   client->print("\t\tDew point: ");
   client->print(dewPoint);
   client->print("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));
   client->print("\t\tEquivalent Sea Level Pressure: ");
   client->print(seaLevel);
   client->print(String( presUnit == BME280::PresUnit_hPa ? "hPa" :"Pa")); // expected hPa and Pa only

   client->print("\t\tHeat Index: ");
   float heatIndex = EnvironmentCalculations::HeatIndex(temp, hum, envTempUnit);
   client->print(heatIndex);
   client->print("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));

   client->print("\t\tAbsolute Humidity: ");
   client->println(absHum);
    alt_s = altitude;
    temp = 0.0;
   delay(1);

    // we need some values first from the bme ...
   setupSCD30(altitude, temp);
   
}

void printSensorData(){
     
      if ((my_report == "A") || (my_report == "T") ){
        if ((my_report_txt == "Y")){
          Serial.print("Temperature: ");
        }
        Serial.print(scd30_T_s);
        
        if ((my_report_txt == "Y")){
          Serial.print(" °C");
          Serial.print("     ");
        }
        // Serial.print(bme_T);
        // Serial.println(" °C");
      }
      
       if ((my_report == "A") || (my_report == "H") ){
          if ((my_report_txt == "Y")){
          Serial.print("Relative Humidity: ");
         }
        Serial.print(scd30_H_s);
         if ((my_report_txt == "Y")){
          Serial.print(" %");
          Serial.print("     ");
        }
        // Serial.print(bme_H);
        // Serial.println(" %");
      }

        if ((my_report == "A") || (my_report == "C") ){
          if ((my_report_txt == "Y")){
            Serial.print("CO2: ");
          }
        Serial.print(scd30_CO2_s, 3);
        if ((my_report_txt == "Y")){
         Serial.print(" ppm");
        }
       }

       if ((my_report == "A") || (my_report == "P") ){
          // Serial.print("Pressure: ");
          // Serial.print(bme_hpa);
          // Serial.print(" hPa");
          // Serial.println("");
       }
      
      if ((my_report == "A") || (my_report == "M") ){
          // Serial.print("Altitude: ");
          // Serial.print(bme_alt);
          // Serial.println(" m");
          // Serial.println("");
        }
        Serial.println("");
}

void setLights(int wavelength){ 
  
  if ((millis() - lastTime) > timerDelay) {


 if (my_verbose == true){
        Serial.println("");
        Serial.println("Verbose : ");
        Serial.print("Set Lights received:");
        Serial.print(wavelength); 
        Serial.println("");
     }


      wave2RGB (wavelength);
      //wave2RGB2 (wavelength);
      
    
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      WiFiClient client;
      HTTPClient http;
    
      // Your Domain name with URL path or IP address with path
      http.begin(client, serverName);

      // Specify content-type header
      // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      
      // Data to send with HTTP POST
      //String httpRequestData = "api_key=tPmAT5Ab3j7F9&sensor=BME280&value1=24.25&value2=49.54&value3=1005.14";           
      
      // Send HTTP POST request
      //int httpResponseCode = http.POST(httpRequestData);

      
      // If you need an HTTP request with a content type: text/plain
      http.addHeader("Content-Type", "text/plain");
      //int httpResponseCode = http.POST("Hello, World!");
      
      // If you need an HTTP request with a content type: application/json, use the following:
     
      //http.addHeader("Content-Type", "application/json");
      // int httpResponseCode = http.POST("{\"api_key\":\"tPmAT5Ab3j7F9\",\"sensor\":\"BME280\",\"value1\":\"24.25\",\"value2\":\"49.54\",\"value3\":\"1005.14\"}");

    //  int httpResponseCode = http.POST("{\"mode\":\"color\",\"red\":\"red_s\",\"green\":\"255\",\"blue\":\"255\",\"gain\":\"100\",\"white\":\"0\",\"effect\":\"0\",\"turn\":\"on\",\"transition\":\"500\"}");
      
      //http://192.168.x.x/light/0?turn=off

    // http://192.168.x.x/light/0?\mode\:\color\red\255,"green\":\" + green_s\",\"blue\":\"255\",\"gain\":\"100\",\"white\":\"0\",\"effect\":\"0\",\"turn\":\"on\",\"transition\":\"500\"}");
     
      
      //using JSON - Issues?
      
      DynamicJsonDocument doc(2048);
    // Add values in the document
    //
    doc["mode"] = "color";
    doc["red"] = (red_s);
    doc["green"] = (green_s);
    doc["blue"] = (blue_s);
    doc["gain"] = 100;
    doc["white"] = 0;
    doc["effect"] = 0; 
    doc["turn"] = "on"; 
    doc["transition"] = 500;
     
    String requestBody;
      serializeJson(doc, requestBody);

      if (my_verbose == true){
        Serial.println("");
        Serial.println("Verbose : ");
          Serial.print("JSON Value: ");
        // serializeJson(doc, Serial); 
        Serial.println("");
     }



     
    // int httpResponseCode = http.POST(requestBody);
    // or
    //red = 255;
    //green = 0;
    //blue = 0;
    
    red_s = (String(red,0));
    green_s = (String(green,0));
    blue_s = (String(blue,0));

    red_s.trim();
    green_s.trim();
    blue_s.trim();
    
   String  shelly_cmd = "/color/0?turn=on&red=" + (red_s) + "&green=" + (green_s) + "&blue=" + (blue_s) + "&white=0";
   //String shelly_cmd = "/color/0?turn=on&red=255&green=255&blue=0&white=0";
    

     if (my_verbose == true){
    Serial.println("");
    Serial.println("Verbose : ");
      Serial.print("Shelly Command: ");
      Serial.print( shelly_cmd);
    Serial.println("");
  }



    auto httpResponseCode = http.POST(shelly_cmd);  //was type int
  if (my_verbose == true){
    Serial.println("");
    Serial.println("Verbose : ");
    Serial.println(httpResponseCode); //Print HTTP return code 
    Serial.println("");
  }
    
   
 if (my_verbose == true){
    Serial.println("");
    Serial.println("Verbose : ");
    String payload = http.getString(); 
    Serial.println(payload);
    Serial.println("");
  }

     client.stop();

    /**
    shellies/shellyrgbw2-<deviceid>/color/0/
        shellies/shellyrgbw2-<deviceid>/color/0/set accepts a JSON payload described below:
    {
        "mode": "color",    /* "color" 
        "red": 0,           /* red brightness, 0..255 
        "green": 0,         /* green brightness, 0..255 
        "blue": 255,        /* blue brightness, 0..255 
        "gain": 100,        /* gain for all channels, 0..100 
        "white": 0,         /* white brightness, 0..255 
        "effect": 0,        /* applies an effect when set 
        "turn": "on",       /* "on", "off" or "toggle" 
      "transition": 500   /* One-shot transition, `0..5000` [ms] 
    }
    */

 if (my_verbose == true){
    Serial.println("");
    Serial.println("Verbose : ");
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    Serial.println("");
  }

     
        if (httpResponseCode == 200){
          led_update_status = "<";
        }
          else {
          led_update_status = "x";
          }
      // Free resources
      http.end();
      Serial.println("");

      //printBME280Data(&Serial);
   
    }
    else {
      Serial.println("WiFi setLights Disconnected");
    }
    lastTime = millis();
  }
}

    




void get_preferences(){
    //we load the secrets.h file here into the tmp vars for eprom  defaults
    Serial.println("Retrieving Variable...");
    preferences.begin("my-app", false); 
    e_ssid = preferences.getString("ssid", SECRET_SSID);
    Serial.print(" SSID:"); Serial.println(e_ssid);
    e_ssid_pwd = preferences.getString("ssid_pwd", SECRET_PASS);
      Serial.print(" PWD :"); Serial.println(e_ssid_pwd);
    e_led_ip = preferences.getString("led", SECRET_LED_IP);
      Serial.print(" LED IP:"); Serial.println(e_led_ip);
    e_hostname = preferences.getString("hostname", SECRET_HOSTNAME);
     Serial.print(" Hostname:"); Serial.println(e_hostname);
    e_altitude = preferences.getInt("altitude", SECRET_BAROMETERAltitude_m);
     Serial.print(" Altitude:"); Serial.println(e_altitude);
    e_report = preferences.getString("report", SECRET_REPORT);
     Serial.print(" Report A=All | T=temp | H=Hum | C=CO2 :"); Serial.println(e_report);
      e_report_txt = preferences.getString("report_txt", SECRET_REPORT_TXT);
     Serial.print(" Report Header and Units Y=Yes | N=No :"); Serial.println(e_report_txt);
    preferences.end();

    my_ssid = e_ssid;
    my_ssid_pwd = e_ssid_pwd;
    my_led_ip = e_led_ip;
    my_hostname = e_hostname;
    my_altitude = e_altitude;
    my_report = e_report;
    my_report_txt = e_report_txt;

    

}



void put_preferences(){

    //Serial.println("Storing Variable...");
    preferences.begin("my-app", false); 
    
      e_ssid = preferences.getString("ssid", SECRET_SSID);
      e_ssid_pwd = preferences.getString("ssid_pwd", SECRET_PASS);
      e_led_ip = preferences.getString("led", SECRET_LED_IP);
      e_hostname = preferences.getString("hostname", SECRET_HOSTNAME);
      e_altitude = preferences.getInt("altitude", SECRET_BAROMETERAltitude_m);
      e_report = preferences.getString("report", SECRET_REPORT);
      e_report_txt = preferences.getString("report_txt", SECRET_REPORT_TXT);

    if (my_ssid != e_ssid ){
      preferences.putString("ssid", my_ssid);
      Serial.println(" Updating SSID...");
      }
    
    if (my_ssid_pwd != e_ssid_pwd){
      preferences.putString("ssid_pwd", my_ssid_pwd);
        Serial.println(" Updating SSID password...");
      }
    
    if (my_led_ip != e_led_ip){
      preferences.putString("led", my_led_ip);
        Serial.println(" Updating LED IP...");
      }

    if (my_hostname != e_hostname){
      preferences.putString("hostname", my_hostname);
        Serial.println(" Updating Hostname...");
    }

    if (my_altitude != e_altitude){
      preferences.putInt("altitude", my_altitude);
      Serial.print(" Updating Altitude...");Serial.print(e_altitude);Serial.print("->");Serial.println(my_altitude);
    }

    if (my_report != e_report){
      preferences.putString("report", my_report);
      Serial.print(" Updating Reporting...");Serial.print(e_report);Serial.print("->");Serial.println(my_report);
    }

    if (my_report_txt != e_report_txt){
      preferences.putString("report_txt", my_report_txt);
      Serial.print(" Updating Reporting Txt and Units...");Serial.print(e_report_txt);Serial.print("->");Serial.println(my_report_txt);
    }

    preferences.end();

}


void Menu() { 
  int menu_timeout = 10000;
  unsigned long startTime, currentTime;
  //readStorage();
  //Serial.print("Time: ");
  startTime = millis();
  //Serial.println(startTime);
  Serial.println("");
  Serial.println("");
  Serial.println("|***************************************|");
  Serial.println("|**|           CO2 Sensor            |**|");
  Serial.println("|**|             Setup               |**|");
  Serial.println("|***************************************|");
  Serial.println("");
  Serial.println("   Select one of the following options:");
  Serial.print("   0 - Set WiFi SSID [");Serial.print(my_ssid);Serial.println("]");
  Serial.print("   1 - Set Wifi Password [");Serial.print(my_ssid_pwd);Serial.println("]");
  Serial.print("   2 - Set LED IP [");Serial.print(my_led_ip);Serial.println("]");
  Serial.print("   3 - Set Sensor Hostname [");Serial.print(my_hostname);Serial.println("]");
  Serial.print("   4 - Set Altitude [");Serial.print(my_altitude);Serial.println("]");
  Serial.print("   5 - Set Reporting (on Serial) [");Serial.print(my_report);Serial.println("]");
  Serial.print("   6 - Set Reporting Headers and Units (on Serial) [");Serial.print(my_report_txt);Serial.println("]");
  Serial.println();
  Serial.print("   8 - Enable verbose debug on serial [");Serial.print(my_verbose);Serial.println("]");
  Serial.println("   9 - EXIT menu (default after 10 seconds)");
  Serial.println("|**|                                 |**|");
  Serial.println("|***************************************|");
  Serial.println("");
  Serial.println("");
    
    for (;;) {
       currentTime = millis();
       if (( currentTime - startTime) > menu_timeout ) {
        return;
       }
        switch (Serial.read()) {
            case '0': setWifiSSID(); break;
            case '1': setWifiSSIDPwd(); break;
            case '2': setLedIP(); break;
            case '3': setHostname(); break;
            case '4': setAltitude(); break;
            case '5': setReport(); break;
            case '6': setReportTxt(); break;
            case '8': enableVerbose(); break;
            case '9': return;
            default: continue;  // includes the case 'no input'
        }
    }
}


void setWifiSSID(){

  Serial.println("");
  Serial.println("Menu 0:");
  Serial.print("Currently set to :");Serial.println(my_ssid);
  
  bool  finSetup = false;
  Serial.println("Enter Wifi SSID to join:"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting SSID to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_ssid = value_entered;
  put_preferences();
  Menu();
}

void setWifiSSIDPwd(){
  
  Serial.println("");
  Serial.println("Menu 1:");
  Serial.print("Currently set to :");Serial.println(my_ssid_pwd);
  
  bool  finSetup = false;
  Serial.println("Enter Wifi SSID Password:"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting SSID Password to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_ssid_pwd = value_entered;
  put_preferences();
  Menu();
}


void setLedIP(){
  
  Serial.println("");
  Serial.println("Menu 2:");
  Serial.print("Currently set to :");Serial.println(my_led_ip);
  
  bool  finSetup = false;
  Serial.println("Enter LED IP:"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting LED IP to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_led_ip = value_entered;
  put_preferences();
  Menu();
}

void setHostname(){
  
  Serial.println("");
  Serial.println("Menu 3:");
  Serial.print("Currently set to :");Serial.println(my_hostname);
  
  bool  finSetup = false;
  Serial.println("Enter Hostname:"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting Hostname to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_hostname= value_entered;
  put_preferences();
  Menu();
}


void setAltitude(){
  
  Serial.println("");
  Serial.println("Menu 4:");
  Serial.print("Currently set to :");Serial.println(my_altitude);
  
  bool  finSetup = false;
  Serial.println("Enter Altitude (m):"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting Altitude to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_altitude = value_entered.toInt();
  put_preferences();
  Menu();
}


void setReport(){
  
  Serial.println("");
  Serial.println("Menu 5:");
  Serial.print("Currently set to :");Serial.println(my_report);
  
  bool  finSetup = false;
  Serial.println("Enter Serial Reporting parameter (A=All | T=Temp | H=Hum | C=CO2):"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting Reporting to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_report = value_entered; //.toString();
  put_preferences();
  Menu();
}



void setReportTxt(){
  
  Serial.println("");
  Serial.println("Menu 6:");
  Serial.print("Currently set to :");Serial.println(my_report_txt);
  
  bool  finSetup = false;
  Serial.println("Display Serial Reporting Header and Units (Y=Yes | N=No):"); 
  String value_entered = "";
  char character;

  while(!finSetup){
    if (value_entered != "") {
      Serial.println(value_entered);
      Serial.print("Setting Reporting Headers and Units to:["); Serial.print(value_entered); Serial.println("]");
      finSetup = true;
    }
    else
    {
      while (Serial.available() > 0)
      {
       value_entered = Serial.readStringUntil('\n');
      }
    }
  }  //end fin calibration
  my_report_txt = value_entered; //.toString();
  put_preferences();
  Menu();
}


void enableVerbose(){
  my_verbose = true;
  Menu();
}

void setup(void) {
  pinMode(OLED_RESET, INPUT);           // set pin to input
  digitalWrite(OLED_RESET, HIGH); 
  
  Wire.begin(21,22);
  Serial.begin(115200);
  delay(2000);

  digitalWrite(OLED_RESET, LOW); 
  

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Cannot find OLED SSD1306... No display... But continuing!");
  }

  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.display();
  
  //while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println("");
  Serial.println("CO2 Ventilation Sensor");
  display.print("Version:");// 5.0 Beta");
  display.println(my_version);
  Serial.println("");
  Serial.println("Serial Setup Menu available now for 10 seconds...");
  Serial.println("");

  display.clearDisplay();
  
    display.setCursor(0, 0);
    display.println("CO2 Sensor");
    display.print("Version:");// 5.0 Beta");
     display.println(my_version);
    display.setCursor(0, 32);
    display.println("Serial Menu starting...");  
    display.setCursor(0, 40);
    display.println("Baud 115200");    
  delay(3000);

  get_preferences();
  Menu();
    // check for the WiFi module:
    //  if (WiFi.status() == WL_NO_MODULE) {
    //    Serial.println("Communication with WiFi module failed!");
    //    // don't continue
    //    while (true);
    //  }

  //String fv = WiFi.firmwareVersion();
  //if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
  //  Serial.println("Please upgrade the firmware");
  // }
  


  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    
    
    Serial.println("");
    display.setCursor(0, 48);
    Serial.println("Connecting to SSID: ");
    Serial.println(my_ssid);
    
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    
    display.println("Connecting to SSID:");
    display.println(my_ssid);
    display.display();  
    
    status = WiFi.begin(my_ssid.c_str(), my_ssid_pwd.c_str());
    // wait 10 seconds for connection:
    WiFi.setHostname(my_hostname.c_str()); //
    delay(10000);
    get_preferences();
  }
  // you're connected now, so print out the status:
  
  server.begin();
  printWifiStatus();
  
 // delay(3000);

    
  // if (!scd30.begin(I2C_SDA, I2C_SCL, 100000)) {
  // scd30.initialize();
   
  
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 Sensor!!! - Fatal! Stop.");
    display.println("NO SCD30 CO2 Sensor!");
    display.println("Fatal! Stop.");
    display.display();     
    while (1) { delay(10); }
  }
  
  Serial.println("SCD30 Found!");
  display.println("Found SCD30 Sensor");
  display.display();  

  setupSCD30(1350, 0.0);
  getSetupSCD30();

  //bme.begin(0x76, &Wire2
  if (!bme.begin()) {
    Serial.println("Failed to find BME280 sensor");
    display.println("NO BME280 Alt Sensor!");
    display.display();     
    bme_sensor_found = false;
    }
    else {
      Serial.println("BME280 Found!");
      display.println("Found BME280 Sensor");
      display.display();
      bme_sensor_found = true;  
    }

  // if (!scd30.setMeasurementInterval(10)){
  //   Serial.println("Failed to set measurement interval");
  //   while(1){ delay(10);}
  // }
  // Serial.print("Measurement Interval: "); 
  //Serial.print(scd30.getMeasurementInterval()); 
  // Serial.println(" seconds");
 
  Serial.println("Starting HTTP client...");
  display.println("Starting HTTP client...");
  display.display();  
  HTTPClient http;
  delay(2000);    

  dst_shelly = "http://" + my_led_ip + "/color/0";
  //shellies/shellyrgbw2-<deviceid>/color/0/set
  serverName = dst_shelly.c_str();

  if (my_verbose == true){
    Serial.println("");
    Serial.println("Verbose : ");
    Serial.println(serverName);
    Serial.println("");
  }

}



void loop() {
   
   WiFiClient client = server.available();  
    
    
    
    if ((millis() - lastCO2Time) > timerCO2Delay) {
      lastCO2Time = millis();
      if (scd30.dataReady()){
        // Serial.println("Data available!");
          if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }
          //float result[3] = {0}; 
          //  scd30.CO2(result);
          float scd30_T = scd30.temperature;        //result[1];  //scd30.temperature;
          float scd30_H =  scd30.relative_humidity ;//result[2];  //scd30.relative_humidity;
          float scd30_CO2 =  scd30.CO2;             //result[0];  //scd30.CO2;
          scd30_T_s = scd30_T;
          scd30_H_s = scd30_H;
          scd30_CO2_s = scd30_CO2;
          printSensorData();
        }
      lastCO2Time = millis();
    }
   displayOLED(scd30_T_s,scd30_H_s, scd30_CO2_s);
    setLights(scd30_CO2_s);
 
    if (client) {
      //Serial.println("New client connected");
      // an HTTP request ends with a blank line
      
      boolean currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          
          // Serial.write(c);
          // if you've gotten to the end of the line (received a newline
          // character) and the line is blank, the HTTP request has ended,
          // so you can send a reply
          
          if (c == '\n' && currentLineIsBlank) {
            // send a standard HTTP response header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");  // the connection will be closed after completion of the response
            client.println("Refresh: 5");  // refresh the page automatically every 5 sec
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            
            client.print("<p>");
            client.print("Temperature: ");
            client.print(scd30_T_s);
            client.println("<br />");
            client.println("</p>");

            client.print("<p>");
            client.print("Rel Humidity: ");
            client.print(scd30_H_s);
            client.println("<br />");
            client.println("</p>");

            client.print("<p>");
            client.print("CO2: ");
            client.print(scd30_CO2_s);
            client.println("<br />");
            client.println("</p>");
            /*
            client.print("<p>");
            client.print("bme T: ");
            client.print(bme_T);
            client.println("<br />");
            client.println("</p>");
            */
            //client.print("<p>");
            //client.print("Alt: ");
            //client.print(alt_s);
            //client.println("<br />");
            //client.println("</p>");
            /*
            client.print("<p>");
            client.print("bme H: ");
            client.print(bme_H);
            client.println("<br />");
            client.println("</p>");
            */
            client.println("</html>");
            break;
          }
          
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
          } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
      
      // give the web browser time to receive the data
      delay(10);

      // close the connection:
      client.stop();
     // Serial.println("client disconnected");
    }
  delay(1000);
}
