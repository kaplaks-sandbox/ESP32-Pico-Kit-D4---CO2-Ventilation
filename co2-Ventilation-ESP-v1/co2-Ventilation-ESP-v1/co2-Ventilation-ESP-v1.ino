// version 1 using scd30 from seeed studio
#include <Wire.h>
#define I2C_SDA 21
#define I2C_SCL 22

//https://github.com/finitespace/BME280
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
// Assumed environmental values:
float referencePressure = 1013.25;  // hPa local QFF (official meteor-station reading)
float outdoorTemp = 4.7;           // °C  measured local outdoor temp.
float barometerAltitude = 1650.3;  // meters ... map readings + barometer position
#define SEALEVELPRESSURE_HPA (1013.25)


#define SERIAL_BAUD   115200

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
float pres_s;
float bme_t_s;
float bme_h_s;


#include <math.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
//#include <Arduino_JSON.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     32     // Reset pin # (or -1 if sharing Arduino reset pin)
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
String ip_led = "192.168.1.117";
String dst_shelly = "http://" + ip_led + "/color/0?";
const char* serverName = dst_shelly.c_str();
int red_s, green_s, blue_s;


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
// unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;
unsigned long delayTime;

unsigned long lastTimeCalibrated = 0;
unsigned long calibrationDelay = 20000; //20 sec after boot we do a calibration

//#include <Adafruit_SCD30.h>
#include "SCD30.h"



// For MKR1010
// #include <WiFiNINA.h>

#include "arduino_secrets.h" 
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;
String my_ip ; 
WiFiServer server(80);

void test_wavelength()
{
  timerDelay = 800;
  for (int i = 0; i <= 3000; i=i+100)
  {
    setLights(i);
    delay(1000);
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  display.setTextSize(1); // Draw 2X-scale text
  display.clearDisplay();
  display.setCursor(0, 0);
  
  display.print("SSID: ");
  display.println(WiFi.SSID());
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

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
    Serial.print(red_s);
    Serial.print(":");
    Serial.print(green_s);
    Serial.print(":");
    Serial.print(blue_s);
    Serial.println("");
}
  
void wave2RGB( float wavelength){
   float wavelength_min = 300.0;
  float wavelength_max = 2500.0;
  float scaleing = wavelength_max / 781.0;
  //wavelength = wavelength / scaleing;
//map(value, fromLow, fromHigh, toLow, toHigh)
  wavelength = map(wavelength, wavelength_min, wavelength_max, 380, 781);
  
  float Gamma = 0.80, IntensityMax = 255, factor, red, blue, green;
  
 // Serial.print("Pre-");
 // Serial.print(wavelength);
 // printRGBStatus();
  
  if((wavelength >= 380) && (wavelength<440)){
    red = -(wavelength - 440) / (440 - 380);
    green = 0.0;
    blue = 1.0;
  }else if((wavelength >= 440) && (wavelength<490)){
    red = 0.0;
    green = (wavelength - 440) / (490 - 440);
    blue = 1.0;
  }else if((wavelength >= 490) && (wavelength<510)){
    red = 0.0;
    green = 1.0;
    blue = -(wavelength - 510) / (510 - 490);
  }else if((wavelength >= 510) && (wavelength<580)){
    red = (wavelength - 510) / (580 - 510);
    green = 1.0;
    blue = 0.0;
  }else if((wavelength >= 580) && (wavelength<645)){
    red = 1.0;
    green = -(wavelength - 645) / (645 - 580);
    blue = 0.0;
  }else if((wavelength >= 645) && (wavelength<781)){
    red = 1.0;
    green = 0.0;
    blue = 0.0;
  }
  //else{
  //  red = 0.0;
  //  green = 0.0;
  //  blue = 0.0;
 // }

 
  // Let the intensity fall off near the vision limits
  /*
   *  Serial.print("Mid-");
  printRGBStatus();
  if((wavelength >= 380) && (wavelength<420)){
    factor = 0.3 + 0.7*(wavelength - 380) / (420 - 380);
  }else if((wavelength >= 420) && (wavelength<701)){
    factor = 1.0;
  }else if((wavelength >= 701) && (wavelength<781)){
    factor = 0.3 + 0.7*(780 - wavelength) / (780 - 700);
  }else{
    factor = 0.0;
  }
  */
  



//  Serial.print("End-");
//  printRGBStatus();
  if (red != 0){
    red = round(IntensityMax * red);// * pow(red * factor, Gamma));
  };
  if (green != 0){
    green = round(IntensityMax * green);// * pow(green * factor, Gamma));
  };
  if (blue != 0){
    blue = round(IntensityMax * blue);// * pow(blue * factor, Gamma));
  }
  
  red_s = red;
  blue_s = blue;
  green_s = green;
 // return [red,green,blue];
 
  printRGBStatus( wavelength);
  
  }

void wave3RGB( float wavelength){
 
  float Gamma = 0.80, IntensityMax = 255, factor, red, blue, green;
  
 // Serial.print("Pre-");
 // Serial.print(wavelength);
 // printRGBStatus();
  float wavelength_min = 0.0;
  float wavelength_max = 3000.0;
  
  float bar1 = 0.2 * wavelength_max;
  float bar2 = 0.4 * wavelength_max;
  float bar3 = 0.6 * wavelength_max;
  float bar4 = 0.8 * wavelength_max; 
  float bar5 = 1 * wavelength_max;

 
  if((wavelength >= wavelength_min) && (wavelength < bar1)){
    red = -(wavelength - bar1) / (bar1 - (bar1 / 2.0));
    green = 0.0;
    blue = 1.0;
  }else if((wavelength >= bar1) && (wavelength < bar2)){
    red = 0.0;
    green =- (wavelength - bar2) / (bar2 - bar1);
    blue = 1.0;
  }else if((wavelength >= bar2) && (wavelength < bar3)){
    red = 0.0;
    green = 1.0;
    blue = -(wavelength - bar3) / (bar3 - bar2);
  }else if((wavelength >= bar3) && (wavelength < bar4)){
    red = -(wavelength - bar3) / (bar4 - bar3);
    green = 1.0;
    blue = 0.0;
  }else if((wavelength >= bar4) && (wavelength < bar5)){
    red = 1.0;
    green = -(wavelength - bar5) / (bar5 - bar4);
    blue = 0.0;
  }else if((wavelength >= bar5) ){
    red = 1.0;
    green = 0.0;
    blue = 0.0;
  }

 
  // Let the intensity fall off near the vision limits
  /*
   *  Serial.print("Mid-");
  printRGBStatus();
  if((wavelength >= 380) && (wavelength<420)){
    factor = 0.3 + 0.7*(wavelength - 380) / (420 - 380);
  }else if((wavelength >= 420) && (wavelength<701)){
    factor = 1.0;
  }else if((wavelength >= 701) && (wavelength<781)){
    factor = 0.3 + 0.7*(780 - wavelength) / (780 - 700);
  }else{
    factor = 0.0;
  }
  */
  



//  Serial.print("End-");
//  printRGBStatus();
  if (red != 0){
    red = round(IntensityMax * red);// * pow(red * factor, Gamma));
  };
  if (green != 0){
    green = round(IntensityMax * green);// * pow(green * factor, Gamma));
  };
  if (blue != 0){
    blue = round(IntensityMax * blue);// * pow(blue * factor, Gamma));
  }
  
  red_s = red;
  blue_s = blue;
  green_s = green;
 // return [red,green,blue];
 
  printRGBStatus( wavelength);
  
  }
  
void displayOLED(float C, float H, float CO2){
      display.clearDisplay();
      display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(0, 0);
     display.print("T:");
      display.print((C));
      display.println(" C");
       
      display.setCursor(62, 0);
       display.print(" RH:");
      display.print((H));
      display.println("%");
      
      //display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(0, 8);
      display.print("CO2:");
      
      display.setTextSize(2); // Draw 2X-scale text
      //display.setCursor(0, 16);
       // display.print(("  "));
      display.print((CO2));
      display.setTextSize(1); // Draw 2X-scale text
      display.print(("ppm"));

      //display.setTextSize(1); // Draw 2X-scale text
      //display.setCursor(0, 32);
      //display.print((" Alt: "));
      //display.print((alt_s));
      //display.setTextSize(1); // Draw 2X-scale text
      //display.print(("m"));

            display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(0, 32);
      display.print(("Pressure: "));
      display.print((pres_s));
      display.setTextSize(1); // Draw 2X-scale text
      display.print((" hPa"));
      
      
      IPAddress ip = WiFi.localIP();

      display.setTextSize(1); // Draw 2X-scale text
      display.setCursor(0, 40);
      display.print("SSID: ");
      display.println(WiFi.SSID());
      display.setCursor(0, 48);
      display.print("  IP: ");
      display.println((ip));
        
      
      display.setCursor(0, 56);
    //  long rssi = WiFi.RSSI();
  //display.print("RSSI:");
  //display.print(rssi);
  //display.println(" dBm");
  display.print(" Dst: ");
  display.print(ip_led);
  display.println("");

  
  
display.display();      // Show initial text

      //display.setTextSize(1); // Draw 2X-scale text
      

  }
  
void setLights(int wavelength){
 
  if ((millis() - lastTime) > timerDelay) {
      Serial.println("");
      Serial.print("Set Lights received:");
      Serial.print(wavelength);
      Serial.println("");
      
      wave2RGB (wavelength);
      
    
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
//String dst_shelly = "http://" + ip_led + "/color/0?";
//const char* serverName = dst_shelly.c_str();
    String cmd_shelly_s = "mode=color&turn=on&red=" + String(red_s) + "&green=" + String(green_s) + "&blue=" + String(blue_s) + "&gain=100&white=0&effect=0&turn=on&transition=500";
    const char* cmd_shelly_c = cmd_shelly_s.c_str();

      int httpResponseCode = http.POST(cmd_shelly_c);
    //  Examples:
    //  http://192.168.0.50/color/0?turn=on&red=255&green=86&blue=112&white=0
   /**
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
    
    Serial.print(dst_shelly);
      //Serial.print("Value: ");
      serializeJson(doc, Serial); 
    
    int httpResponseCode = http.POST(requestBody);
*/

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


      Serial.println("");
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      http.end();
      Serial.println("");

    //  printBME280Data(&Serial);
   
    }
    else {
      Serial.println("WiFi setLights Disconnected");
    }
    lastTime = millis();
  }
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
    pres_s = pres;
    bme_t_s = temp;
    bme_h_s = hum;
   
   delay(1);

  
}


void setup(void) {
 Serial.begin(SERIAL_BAUD);
 // I2C_SDA 21
//#define I2C_SCL 22

  Wire.begin(I2C_SDA,I2C_SCL);
  
  int timeout = 0;
 
 while (!Serial) {
    delay(10);
     timeout = timeout + 1;
     if (timeout > 500) 
     break;   
  }
  
  delay(5000);
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Cannot find OLED SSD1306... No display... But continuing!");
   // for(;;); // Don't proceed, loop forever
  }

  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  //display.display();
  //delay(2000); // Pause for 2 seconds
  // Clear the buffer
  display.clearDisplay();
  
  //while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println("");
  Serial.println("CO2 Ventilation Sensor");
  Serial.println("Version 1.0 Beta");


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
    display.clearDisplay();
  
    display.setCursor(0, 0);
    display.println("CO2 Sensor");
    //display.setCursor(0, 16);
    display.println("Version 1.0 Beta");
         display.setCursor(0, 24);
    display.print("Serial baud:");
    display.println(SERIAL_BAUD);
     display.setCursor(0, 40);
    display.println("Connecting to SSID:");
    display.println(ssid);
    display.display();  
    
    Serial.println(""); 
    Serial.println("Connecting to SSID: ");
    Serial.println(ssid);
    
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
   
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the status:
  
  server.begin();
  printWifiStatus();
  delay(3000);

    
  // if (!scd30.begin(I2C_SDA, I2C_SCL, 100000)) {
  scd30.initialize();
   
  
  if (!scd30.isAvailable()) {
    Serial.println("Failed to find SCD30 Sensor!!! - Fatal! Stop.");
    display.println("NO SCD30 CO2 Sensor!");
    display.println("Fatal! Stop.");
    display.display();     
    while (1) { delay(10); }
  }
  
  Serial.println("SCD30 Found!");
  display.println("Found SCD30 Sensor");
  display.display();  

 

//bme.begin(0x76, &Wire2
if (!bme.begin()) {
    Serial.println("Failed to find BME280 sensor");
    display.println("NO BME280 Alt Sensor!");
    display.display();     
    }
    else {
    Serial.println("BME280 Found!");
    display.println("Found BME280 Sensor");
    display.display();  
    }

  // if (!scd30.setMeasurementInterval(10)){
  //   Serial.println("Failed to set measurement interval");
  //   while(1){ delay(10);}
  // }
  // Serial.print("Measurement Interval: "); 
  //Serial.print(scd30.getMeasurementInterval()); 
  // Serial.println(" seconds");
 
  Serial.println("Starting HTTP client...");
  display.println("Start HTTP client...");
  display.display();  
  HTTPClient http;
  delay(2000);
 
}



void loop() {

 // test_wavelength();

  
  WiFiClient client = server.available();
  
  if (scd30.isAvailable()){
    
    // Serial.println("Data available!");

//    if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }
      float result[3] = {0};
      
      scd30.getCarbonDioxideConcentration(result);
      float scd30_T = result[1];  //scd30.temperature;
      float scd30_H = result[2];  //scd30.relative_humidity;
      float scd30_CO2 = result[0];  //scd30.CO2;

      Serial.print("Temperature: ");
      Serial.print(scd30_T);
      Serial.print(" C");
      Serial.print("     ");
      // Serial.print(bme_T);
      // Serial.println(" °C");
      
      Serial.print("Relative Humidity: ");
      Serial.print(scd30_H);
      Serial.print(" %");
      Serial.print("     ");
      // Serial.print(bme_H);
      // Serial.println(" %");
      
      Serial.print("CO2: ");
      Serial.print(scd30_CO2, 3);
      Serial.println(" ppm");
      
      // Serial.print("Pressure: ");
      // Serial.print(bme_hpa);
      // Serial.print(" hPa");
      // Serial.println("");
      
      // Serial.print("Altitude: ");
      // Serial.print(bme_alt);
      // Serial.println(" m");
      // Serial.println("");

      displayOLED(scd30_T,scd30_H, scd30_CO2);
      setLights(scd30_CO2);
      
      
  
    if (client) {
      Serial.println("New client connected");
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
            // output the value of each analog input pin
            
            /**for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            //  int sensorReading = analogRead(analogChannel);
            //  client.print("analog input ");
              client.print(analogChannel);
              client.print(" is ");
              client.print(sensorReading);
            
              client.println("<br />");
            }
            client.println("</html>");
            break;
            **/
            client.print("<p>");
            client.print("Temperature: ");
            client.print(scd30_T);
            client.println("<br />");
            client.println("</p>");

            client.print("<p>");
            client.print("Rel Humidity: ");
            client.print(scd30_H);
            client.println("<br />");
            client.println("</p>");

            client.print("<p>");
            client.print("CO2: ");
            client.print(scd30_CO2);
            client.println("<br />");
            client.println("</p>");
            
            client.print("<p>");
            client.print("bme T: ");
            client.print(bme_t_s);
            client.println("<br />");
            client.println("</p>");
            
            client.print("<p>");
            client.print("Alt: ");
            client.print(alt_s);
            client.println("<br />");
            client.println("</p>");
            
             client.print("<p>");
            client.print("Pres: ");
            client.print(pres_s);
            client.println("<br />");
            client.println("</p>");
            
            
            client.print("<p>");
            client.print("bme H: ");
            client.print(bme_h_s);
            client.println("<br />");
            client.println("</p>");
            
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
      Serial.println("client disconnected");
    }

  }

  delay(100);
}
