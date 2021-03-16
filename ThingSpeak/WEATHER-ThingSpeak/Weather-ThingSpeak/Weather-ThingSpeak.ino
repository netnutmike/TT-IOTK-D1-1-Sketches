/******************************************************************************************************************************************************** 
 *  Weather-ThingSpeak.ino
 *  
 *  This is a Sketch that reads data from a weather system using ambient air temperature, humidity and barometric pressure as well as light brightness.  
 *  This data is sent via the Internet to ThingSpeak, a data collector and can also be displayed on a OLED screen.  
 *  
 *  Be sure to set the serial monitor to 115,200 to view the debug output
 *  
 *  Change Log:
 *    03/08/2021 - Initial Release
 *    
 *  Copyright:
 *    Copyright 2021 Techtactile, LLC
 *    
 *    Permission is hereby granted, for no charge, to anyone who has a copy of this software and any associated documentation file
 *    from here forward refferences as "The Software", to use or modify the software without limitation to use, copy, modifym merge, or
 *    publish subject to the following conditions:
 *    
 *    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the software.
 *    
 *  Warranty:
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, INCLUDING EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *    WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRIDGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *    
 *******************************************************************************************************************************************************/

//*************************************************************************
//******************  LIBRARY SECTION *************************************
//*************************************************************************

#include "ThingSpeak.h"
#include <ESP8266WiFi.h>          //if you get an error here you need to install the ESP8266 board manager 
#include <Wire.h>                 //Wire Driver
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//****************************************************************************
//V V V V V V V V V V V V V V   REQUIRED SETTINGS   V V V V V V V V V V V V V 
//****************************************************************************

// *** WI-FI SETTINGS ***

#define USER_SSID                 "YOUR_WIFI_SSID"
#define USER_PASSWORD             "YOUR_WIFI_PASSWORD"

// *** THINGSPEAK SETTINGS ***

#define USER_THINGSPEAK_API_KEY    "YOUR_THINGSPEAK_API_KEY"
#define CHANNEL_ID                  YOUR_THINGSPEAK_CHANNEL_ID

// *** SEND UPDATE INTERVAL ***
#define SEND_DELAY 60000            // Default Number of Milliseconds between update messages

// *** DEBUG MODE ON ***
#define DEBUG true

// DEFAULTS FOR CALCULATING BATTERY VOLTAGE
const float input_voltage = 5;
const float ADCVoltage = 3.61;
const float CellVoltage = 4.25;

// *** OLED SCREEN SETTINGS ***

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREEN_DEFAULT_TEXT_SIZE 2
#define SCREEN_DEFAULT_TEXT_COLOR WHITE

//****************************************************************************
//^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^  END REQUIRED SETTINGS   ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ 
//****************************************************************************

//************************************************************************************************
//*************************  SETTINGS THAT SHOULD NOT NEED CHANGED  ******************************
//
// The settings below should not need changed to get an operational IoT system up and running.
//
// If you are going to make changes to the settings below, please be aware that it will affect
// other parts of this program.  If you are going to be modifying the program, then have fun...
//
//************************************************************************************************


// *** CONSTANTS
const char* ssid = USER_SSID ; 
const char* password = USER_PASSWORD ;
const char* thingspeak_server = USER_THINGSPEAK_SERVER ;
const char* thingspeak_API_KEY = USER_THINGSPEAK_API_KEY ;
unsigned long myChannelNumber = CHANNEL_ID;
 

// *****************  DEFINITIONS AND DECLARATIONS  *************************

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WIFI
WiFiClient espClient;

// SENSORS
Adafruit_BME280 bme;
BH1750 lightMeter;


//**************************  GLOBAL VARIABLES  *******************************

bool boot = true;
bool missingBME = false;
bool missingDisplay = false;

int wakeDelay = 1000;
int sendDelay = SEND_DELAY;
int lastSendMilli = 0;

// SENSOR STRUCTURE
struct {
  float pressure;             // Pressure (hPa) (mb)
  float pressureinHG;         // Pressure (inHG) Inches of Mercury
  float temp;                 // temperature (°C)
  float tempf;                // temperature (°F)
  float humidity;             // relative humidity (%)
  float A0raw;                // A0 raw reading
  float battery;              // Calculated Battery Voltage
  long  lightLevel;           // light intensity (lux)
  long  wifiRSSI;             // WiFi signal strength (dBm)
  String myIPAddress;
  bool  bme280Presence = false; 
  bool  bh1750Presence = false; 
  bool  lowRSSI = false;         // low WiFi signal alarm flag
} sensorData; 

//****************************************************************************
//*********************  WIFI FUNCTIONS  *************************************
//****************************************************************************

void setup_wifi() 
{
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}



//****************************************************************************
//*********************  INDICATOR AND DISPLAY FUNCTIONS  ********************
//****************************************************************************

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: displayText
//
// PARAMETERS:
//   String -- textToDisplay: The text to be displayed on the screen
//   int -- Text Size: The size of the text to display.  Defaults to SCREEN_DEFAULT_TEXT_SIZE
//   int -- Text Color: The color code for the text to display.  Defaults to SCREEN_DEFAULT_TEXT_COLOR
//
// RETURN:
//   void: No Value Returned
//
// CALLED FROM:
//   updateDisplay, Any other
//
// DESCRIPTION:
//   This subroutine displays the text that is passed in on the OLED display.  There are two
//   optional parameters that let you set the text size and text colors.  If not provided, 
//   the text size defaults to the SCREEN_DEFAULT_TEXT_SIZE and the color defaults to
//   SCREEN_DEFAULT_TEXT_COLOR.  Both of these default values are set at the top of the program.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void displayText(String textToDisplay1, int textSize = SCREEN_DEFAULT_TEXT_SIZE, int textColor = SCREEN_DEFAULT_TEXT_COLOR) {
  display.clearDisplay();

  display.setTextSize(textSize);      // Normal 1:1 pixel scale
  display.setTextColor(textColor); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.println(textToDisplay1);

  display.display();
  if (DEBUG) {
    Serial.println("Setting Display to: "  + textToDisplay1);
  }
}

//****************************************************************************
//*********************  SENSOR AND UPDATE FUNCTIONS  ************************
//****************************************************************************

//****************************************************************************

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: updateDisplay
//
// PARAMETERS:
//   -- NONE
//
// RETURN:
//   void: No Value Returned
//
// CALLED FROM:
//   main
//
// DESCRIPTION:
//   This subroutine builds the display output based upon the data in the sensorData structure
//   and then calls the displayText routine to put the new output on the display.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void updateDisplay() {

  String displayString;
  
//  displayString += "\n" + String(sensorData.waterLevel) + " % \n";
//  displayString += "W T:" + String(sensorData.waterTemp) + "  TDS:" + String(sensorData.waterTDS) + "\n";
  displayString = "T:" + String(sensorData.temp) + " c \n";
  displayString += "H:" + String(sensorData.humidity) + " % \n";
  displayString += "P:" + String(sensorData.pressureinHG) + " in\n";
  displayText(displayString);
  
 
}


// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: readSensors
//
// PARAMETERS:
//   -- NONE
//
// RETURN:
//   void: No Value Returned
//
// CALLED FROM:
//   main
//
// DESCRIPTION:
//   This subroutine reads the sensors and puts all of the data it receives into the sensorData
//   structure.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void readSensors()
{

  // Check to make sure a BME280 was detected before attempting to read values
  if (sensorData.bme280Presence) {
    if (DEBUG) {
      Serial.println("Reading Sensors...");
    }

    sensorData.temp = bme.readTemperature();
    sensorData.tempf = ((sensorData.temp * 1.8) + 32);
    sensorData.humidity = bme.readHumidity();
    sensorData.pressure = bme.readPressure()/100.0F;
    sensorData.pressureinHG = (sensorData.pressure * .03);
  } else {
    sensorData.temp = 0;
    sensorData.tempf = 0;
    sensorData.humidity = 0;
    sensorData.pressure = 0;
    sensorData.pressureinHG = 0;
  }

  // Read analog sensor
  sensorData.A0raw = analogRead(A0);

  float adjustment = CellVoltage / ADCVoltage;
  sensorData.battery = input_voltage * analogRead(A0) * adjustment / 1023.0;

  // Read light level sensor
  sensorData.lightLevel = lightMeter.readLightLevel();

  sensorData.myIPAddress = WiFi.localIP().toString();
  sensorData.wifiRSSI = WiFi.RSSI();

}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: sendStats
//
// PARAMETERS:
//   -- NONE
//
// RETURN:
//   void: No Value Returned
//
// CALLED FROM:
//   main
//
// DESCRIPTION:
//   This subroutine reads sends the stats that were retrieved during the readSensors routine.
//   It send the Stats from the sensorData structure.  If DEBUG is true, it will output the 
//   stats to the serial output before sending the stats.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void sendStats()
{

  String msg;
  
  if (DEBUG) {
    Serial.print("Air Temp in Celsius: ");
    Serial.println(sensorData.temp);
    Serial.print("Air Temp in Farenheight: ");
    Serial.println(sensorData.tempf);
    Serial.print("Air Humidity: ");
    Serial.println(sensorData.humidity);
    Serial.print("Pressure in mb: ");
    Serial.println(sensorData.pressure);
    Serial.print("Pressure in inHG: ");
    Serial.println(sensorData.pressureinHG);
    Serial.print("Raw A0: ");
    Serial.println(sensorData.A0raw);
    Serial.print("Battery: ");
    Serial.println(sensorData.battery);
    Serial.print("Light Level: ");
    Serial.println(sensorData.lightLevel);
    Serial.print("RSSI: ");
    Serial.println(sensorData.wifiRSSI);
    Serial.print("IP Address: ");
    Serial.println(sensorData.myIPAddress);
    Serial.print("BME280 Presence: ");
    Serial.println(sensorData.bme280Presence);
    Serial.print("BH1750 Presence: ");
    Serial.println(sensorData.bh1750Presence);
    Serial.println(" ");
    Serial.println(" ");
  }

  ThingSpeak.setField(1, sensorData.pressure);    //Pressure
  ThingSpeak.setField(2, sensorData.temp);        //Temp
  ThingSpeak.setField(3, sensorData.humidity);    //Humidity
  ThingSpeak.setField(4, sensorData.lightLevel);  //LightLevel
  ThingSpeak.setField(5, sensorData.battery);     //Battery
  ThingSpeak.setField(6, sensorData.wifiRSSI);    //RSSI
  
  // write to the ThingSpeak channel 
  int x = ThingSpeak.writeFields(myChannelNumber, thingspeak_API_KEY);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  delay(2000);
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: SetupSensors
//
// PARAMETERS:
//   -- NONE
//
// RETURN:
//   void - No Return Value
//
// CALLED FROM:
//   setup
//
// DESCRIPTION:
//   This subroutine sets up the BME temp/hum/pres sensor and the bh1750 light sensor.  If 
//   either sensor is not found a flag is set indicating that it is not detected.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setupSensors() 
{
  if ( !bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 Sensor!");
    sensorData.bme280Presence = false;
  } else {
    sensorData.bme280Presence = true;
  }

  if ( lightMeter.begin() == false ) {
    sensorData.bh1750Presence = false;
    Serial.println( "BH1750 sensor failure." );
  } else {
    sensorData.bh1750Presence = true;
  }
}


//****************************************************************************
//*********************  SETUP FUNCTION  *************************************
//****************************************************************************

void setup() 
{
  // Setup Serial Port
  Serial.begin(115200);

  // Setup WiFi
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  setup_wifi();

  ThingSpeak.begin(espClient); 

  //Setup Sensors
  setupSensors();

  //Setup Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    missingDisplay = true;
  } else {
    // Clear the buffer
    display.clearDisplay();
    display.display();
  }
  
  //Setup Pins
   pinMode(A0, INPUT);

  Serial.println("Setup Complete");
}


//*****************************************************************************
//******************  MAIN LOOP FUNCTION  *************************************
//*****************************************************************************
void loop() 
{
  // Reconnect if not connected
  if (WiFi.status() != WL_CONNECTED) 
  {
    setup_wifi();
  }

  // If we did not just boot, do the work....
  if(boot == false)
  {
    //if (millis() >= (lastSendMilli + sendDelay)) {
      readSensors();
      sendStats();
      updateDisplay();
      lastSendMilli = millis();
      Serial.println("Going To Sleep");
      ESP.deepSleep(60e6);
      Serial.println("Awake");
      setup_wifi();
    //}
  }
  boot=false;
}
