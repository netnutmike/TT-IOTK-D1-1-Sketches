/******************************************************************************************************************************************************** 
 *  Hydroponics_IoT_MQTT.ino
 *  
 *  This is a Sketch that reads data from a hydroponics system using water level sensors, water temperature, TDS (total Dissolved Solids),
 *  PH Sensor, ambient air temperature, humidity and barometric pressure as well as light brightness.  This data is sent via MQTT to a data collector 
 *  and can also be displayed on a OLED screen.  There is also an indicator LED that blinks based on what is needed.
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

#include <PubSubClient.h>         //https://github.com/knolleary/pubsubclient
#include <ESP8266WiFi.h>          //if you get an error here you need to install the ESP8266 board manager 
#include <ESP8266mDNS.h>          //if you get an error here you need to install the ESP8266 board manager 
#include <ArduinoOTA.h>           //ArduinoOTA is now included with the ArduinoIDE
#include <ArduinoJson.h>          //ArduinoJson
#include <Wire.h>                 //Wire Driver
#include <OneWire.h>
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

// *** MQTT SETTINGS ***

#define USER_MQTT_SERVER          "YOUR_MQTT_SERVER_IP"
#define USER_MQTT_PORT            1883
#define USER_MQTT_USERNAME        "YOUR_MQTT_USERNAME"
#define USER_MQTT_PASSWORD        "YOUR_MQTT_PASSWORD"
#define USER_MQTT_CLIENT_NAME     "THIS_CLIENT_DEVICE_NAME"           //used to define MQTT topics, MQTT Client ID, and ArduinoOTA

// *** SEND UPDATE INTERVAL ***
#define SEND_DELAY 60000            // Default Number of Milliseconds between update messages

// *** DEBUG MODE ON ***

#define DEBUG     true

// *** OLED SCREEN SETTINGS ***

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREEN_DEFAULT_TEXT_SIZE 1
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


// *** MQTT TOPICS
#define USER_MQTT_TEMPC            USER_MQTT_CLIENT_NAME"/tempc"
#define USER_MQTT_TEMPF            USER_MQTT_CLIENT_NAME"/tempf"
#define USER_MQTT_HUMIDITY         USER_MQTT_CLIENT_NAME"/humidity"
#define USER_MQTT_PRESSURE_HPA     USER_MQTT_CLIENT_NAME"/pressure_HPA"
#define USER_MQTT_PRESSURE_inHG    USER_MQTT_CLIENT_NAME"/pressure_inHG"
#define USER_MQTT_BME_STATUS       USER_MQTT_CLIENT_NAME"/bmestatus"
#define USER_MQTT_BH1750_STATUS    USER_MQTT_CLIENT_NAME"/bh1750status"
#define USER_MQTT_DISPLAY_STATUS   USER_MQTT_CLIENT_NAME"/displaystatus"
#define USER_MQTT_SENSOR_1         USER_MQTT_CLIENT_NAME"/sensor1"
#define USER_MQTT_SENSOR_2         USER_MQTT_CLIENT_NAME"/sensor2"
#define USER_MQTT_SENSOR_3         USER_MQTT_CLIENT_NAME"/sensor3"
#define USER_MQTT_SENSOR_4         USER_MQTT_CLIENT_NAME"/sensor4"
#define USER_MQTT_A0_RAW           USER_MQTT_CLIENT_NAME"/A0_raw"
#define USER_MQTT_WATER_TEMP       USER_MQTT_CLIENT_NAME"/water_temp"
#define USER_MQTT_WATER_TEMPF      USER_MQTT_CLIENT_NAME"/water_tempf"
#define USER_MQTT_TDS              USER_MQTT_CLIENT_NAME"/tds"
#define USER_MQTT_WATER_LEVEL      USER_MQTT_CLIENT_NAME"/water_level"
#define USER_MQTT_LIGHT_LEVEL      USER_MQTT_CLIENT_NAME"/light_level"
#define USER_MQTT_WIFI_RSSI        USER_MQTT_CLIENT_NAME"/wifi_rssi"
#define USER_MQTT_WIFI_IP          USER_MQTT_CLIENT_NAME"/wifi_ip"
#define USER_MQTT_REFRESH_RATE     USER_MQTT_CLIENT_NAME"/refresh_rate"

// *** PIN DEFINITIONS
#define SENSOR_1_PIN  D6
#define SENSOR_2_PIN  D7
#define SENSOR_3_PIN  D0
#define ONE_WIRE      D5
#define ATTENTION_LED D4

// MISC Settings
#define BLINKEMPTY 200
#define BLINK25    500
#define BLINK50    1000
#define BLINK75    0

// *** CONSTANTS
const char* ssid = USER_SSID ; 
const char* password = USER_PASSWORD ;
const char* mqtt_server = USER_MQTT_SERVER ;
const int mqtt_port = USER_MQTT_PORT ;
const char *mqtt_user = USER_MQTT_USERNAME ;
const char *mqtt_pass = USER_MQTT_PASSWORD ;
const char *mqtt_client_name = USER_MQTT_CLIENT_NAME ; 

// *****************  DEFINITIONS AND DECLARATIONS  *************************

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DEFINE JSON SIZE

const int capacity = JSON_OBJECT_SIZE(3) + 6 * JSON_OBJECT_SIZE(1);
StaticJsonDocument<capacity> doc;

// WIFI
WiFiClient espClient;
PubSubClient client(espClient);

// SENSORS
Adafruit_BME280 bme;
BH1750 lightMeter;

//Temperature chip i/o
OneWire ds(ONE_WIRE); 


//**************************  GLOBAL VARIABLES  *******************************

bool boot = true;
bool missingDisplay = false;

char charPayload[50];
int wakeDelay = 1000;
int lastSendMilli = 0;
int sendDelay = SEND_DELAY;

int blinkRate;
int lastBlinkTime;
int lastBlinkMode;

int waterLevel;

// MQTT MESSAGING
String currentMessage;
String currentCommand; 

// SENSOR STRUCTURE
struct {
  float pressure;             // Pressure (hPa) (mb)
  float pressureinHG;         // Pressure (inHG) Inches of Mercury
  float temp;                 // temperature (°C)
  float tempf;                // temperature (°F)
  float humidity;             // relative humidity (%)
  float waterTemp;            // water temperature in Celcius
  float waterTempF;           // water temperature in Farenheight
  float waterLevel;           // water level
  float waterTDS;             // calculated water TDS value
  float A0raw;                // A0 raw reading
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

  WiFi.hostname(USER_MQTT_CLIENT_NAME);
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


// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: reconnect
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
//   This subroutine will attempt to connect to wifi and then MQTT.  It also resubscribes to 
//   the MQTT topics it need to look for.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void reconnect() 
{
  // Loop until we're reconnected
  int retries = 0;
  while (!client.connected()) {
    if(retries < 150)
    {
      if (DEBUG) { Serial.print("Attempting MQTT connection..."); }
      // Attempt to connect
      if (client.connect(mqtt_client_name, mqtt_user, mqtt_pass)) 
      {
        if (DEBUG) { Serial.println("connected"); }
        // Once connected, publish an announcement...
        if(boot == true)
        {
          client.publish(USER_MQTT_CLIENT_NAME"/checkIn","Rebooted");
          boot = false;
        }
        if(boot == false)
        {
          client.publish(USER_MQTT_CLIENT_NAME"/checkIn","Reconnected"); 
        }
        client.subscribe(USER_MQTT_CLIENT_NAME"/settings");
      } 
      else 
      {
        if (DEBUG) { 
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
        }
        retries++;
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
    else
    {
      ESP.restart();
    }
  }
}

//****************************************************************************
//*****************************  MQTT FUNCTIONS  *****************************
//****************************************************************************

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: callback
//
// PARAMETERS:
//   char *-- topic: topic that was recevied
//   byte *-- payload: text that was recevied in the payload in byte array format
//   Uint -- length: the length of the byte array
//
// RETURN:
//   void: No Value Returned
//
// CALLED FROM:
//   main
//
// DESCRIPTION:
//   This subroutine is called when a new MQTT message is received.  It is set as the callback
//   method in the setup function.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void callback(char* topic, byte* payload, unsigned int length) 
{
  if (DEBUG) { 
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
  }
  String newTopic = topic;
  
  payload[length] = '\0';
  String newPayload = String((char *)payload);
  int intPayload = newPayload.toInt();

  newPayload.toCharArray(charPayload, newPayload.length() + 1); 
  String stateMessage;

  if (newTopic == USER_MQTT_CLIENT_NAME"/settings")
  {
    //settings:
    // refresh

    DeserializationError err = deserializeJson(doc, payload);
    
    if (err)
    {
      stateMessage = "Deserialization Error: " + String((char *)err.c_str());
      stateMessage.toCharArray(charPayload, stateMessage.length() + 1);
      client.publish(USER_MQTT_CLIENT_NAME"/status", charPayload);
    } else {
      if (doc["option"] == "refresh") {
        if (doc["value"] == "DEFAULT") {
          sendDelay = SEND_DELAY;
        } else {
          String newDelay = doc[String("value")];
          sendDelay = newDelay.toInt();
        }

        client.publish(USER_MQTT_CLIENT_NAME"/status", "option updated");
      }
    }
  }
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


// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: setBlinkRate
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
//   This subroutine sets the blink rate of the LED based upon the water level that is being
//   detected by the water level sensors.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setBlinkRate() {

  // blink rate set by level
    switch (waterLevel) {
      case 0:   //empty
        blinkRate = BLINKEMPTY;
        break;

      case 25:   //25%
        blinkRate = BLINK25;
        break;

      case 50:   //50%
        blinkRate = BLINK50;
        break;

      case 75:   //75%
        blinkRate = BLINK75;
        break;
    }
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: blinkNow
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
//   This subroutine determines if it is time to blink the LED based upon the current blinkRate
//   that is set by the setBlinkRate function.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void blinkNow()
{
  if (millis() >= (lastBlinkTime + blinkRate)) {
    lastBlinkTime = millis();
    if (lastBlinkMode == true) {
      digitalWrite(ATTENTION_LED, false);
      lastBlinkMode = false;
    } else {
      digitalWrite(ATTENTION_LED, true);
      lastBlinkMode = true;
    }
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
  
  if (sensorData.waterLevel <= 24) {
    displayString = "FILL\nWATER\nFILL\nWATER";
    displayText(displayString, 2);
  } else {
    switch (waterLevel) {
      case 25:   //25%
        displayString = "*******";
        break;

      case 50:   //50%
        displayString = "**************";
        break;

      case 75:   //75%
        displayString = "*********************";
        break;
    }
    displayString += "\n" + String(sensorData.waterLevel) + " % \n";
    displayString += "W T:" + String(sensorData.waterTemp) + "  TDS:" + String(sensorData.waterTDS) + "\n";
    displayString += "T:" + String(sensorData.temp) + " c \n";
    displayString += "H:" + String(sensorData.humidity) + " % \n";
    displayString += "P:" + String(sensorData.pressureinHG) + " in\n";
    displayText(displayString);
  }
 
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

  // Read water sensors
  sensorData.A0raw = analogRead(A0);
  sensorData.waterTemp = getTemp();
  sensorData.waterTempF = ((sensorData.waterTemp * 1.8) + 32);
  sensorData.waterTDS = getTDSValue();

  // Determine Water Level
  waterLevel = 0;
  
  if (digitalRead(SENSOR_3_PIN)) {
    waterLevel = 25;
  }

  if (digitalRead(SENSOR_2_PIN)) {
    waterLevel = 50;
  }

  if (digitalRead(SENSOR_1_PIN)) {
    waterLevel = 75;
  }
  
  sensorData.waterLevel = waterLevel;

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
    Serial.print("Water TDS: ");
    Serial.println(sensorData.waterTDS);
    Serial.print("Water Level: ");
    Serial.println(sensorData.waterLevel);
    Serial.print("Water Temp Celsius: ");
    Serial.println(sensorData.waterTemp);
    Serial.print("Water Temp Farenheight: ");
    Serial.println(sensorData.waterTempF);
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

  msg = String(sensorData.temp);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_TEMPC, charPayload);

  msg = String(sensorData.tempf);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_TEMPF, charPayload);

  msg = String(sensorData.humidity);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_HUMIDITY, charPayload);

  msg = String(sensorData.pressure);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_PRESSURE_HPA, charPayload);

  msg = String(sensorData.pressureinHG);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_PRESSURE_inHG, charPayload);

  if (sensorData.bme280Presence == 1) {
    msg = String("Present");
  } else {
    msg = String("Not Present");
  }
  
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_BME_STATUS, charPayload);

  if (sensorData.bh1750Presence == 1) {
    msg = String("Present");
  } else {
    msg = String("Not Present");
  }
  
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_BH1750_STATUS, charPayload);

  msg = String(sensorData.A0raw);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_A0_RAW, charPayload);
    
  msg = String(sensorData.waterTemp);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_WATER_TEMP, charPayload);

  msg = String(sensorData.waterTempF);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_WATER_TEMPF, charPayload);
  
  msg = String(sensorData.waterTDS);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_TDS, charPayload);

  msg = String(sensorData.waterLevel);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_WATER_LEVEL, charPayload);

  msg = String(sensorData.lightLevel);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_LIGHT_LEVEL, charPayload);

  msg = String(sensorData.wifiRSSI);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_WIFI_RSSI, charPayload);

  msg = sensorData.myIPAddress;
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_WIFI_IP, charPayload);

  msg = String((sendDelay / 1000));
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_REFRESH_RATE, charPayload);
  
  delay(2000);
}


// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: getDTSValue
//
// PARAMETERS:
//   -- NONE
//
// RETURN:
//   float: TDS Sensor Value
//
// CALLED FROM:
//   readSensors
//
// DESCRIPTION:
//   This subroutine reads the TDS value from the TDS sensor connected to A0.  That value is
//   then calculated based on the sensor manufactures provided algorythm and returned as a
//   float.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

float getTDSValue() {
  float VREF = 3.6;
  float temp = 13;
  float voltage, compensation, compensationVoltage, tdsValue;

  voltage = analogRead(A0) * VREF / 1024;
  compensation = 1.0 + 0.02 * (temp - 25);
  compensationVoltage = voltage / compensation;
  tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.30 * compensationVoltage * compensationVoltage) * 0.5;

  return tdsValue;
  
}


// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// NAME: getTemp
//
// PARAMETERS:
//   -- NONE
//
// RETURN:
//   float: Temerature read from sensor
//
// CALLED FROM:
//   readSensors
//
// DESCRIPTION:
//   This subroutine reads the temperature from the OneWire temperature sensor and returns 
//   the temperature in Celcius.
//
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1001;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1002;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
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
    client.publish(USER_MQTT_CLIENT_NAME"/error", "Could not find a valid BME280 Sensor!");
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

  //Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  ArduinoOTA.setHostname(USER_MQTT_CLIENT_NAME);
  ArduinoOTA.begin(); 

  //Setup Sensors
  setupSensors();
  
  //Setup Dispaly
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
   pinMode(SENSOR_1_PIN, INPUT_PULLUP);
   pinMode(SENSOR_2_PIN, INPUT_PULLUP);
   pinMode(SENSOR_3_PIN, INPUT);
   pinMode(ATTENTION_LED, OUTPUT);
  
}

//*****************************************************************************
//******************  MAIN LOOP FUNCTION  *************************************
//*****************************************************************************
void loop() 
{
  // Reconnect if not connected
  if (!client.connected()) 
  {
    reconnect();
  }

  // Handle any MQTT messages
  client.loop();
  ArduinoOTA.handle();

  // If we did not just boot, do the work....
  if(boot == false)
  {
    // Set the blink rate based on the water level and blink 
    setBlinkRate();
    blinkNow();    

    // If it is time to send a new update, read the sensors, send the new stats, update the display and set the next send time
    if (millis() >= (lastSendMilli + sendDelay) && sendDelay > 0) {
      readSensors();
      sendStats();
      updateDisplay();
      lastSendMilli = millis();     // SET DELAY TO NEXT SENSOR READ
    }
  }
}
