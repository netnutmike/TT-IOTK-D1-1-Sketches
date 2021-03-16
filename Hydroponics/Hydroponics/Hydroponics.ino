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
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//****************************************************************************
//******************  REQUIRED SETTINGS  *************************************
//****************************************************************************

// *** WI-FI SETTINGS ***

#define USER_SSID                 "hahn-2g"
#define USER_PASSWORD             "11ab3f4ef3"

// *** MQTT SETTINGS ***

#define USER_MQTT_SERVER          "10.232.0.204"
#define USER_MQTT_PORT            1883
#define USER_MQTT_USERNAME        "mqtt"
#define USER_MQTT_PASSWORD        "mqtt"
#define USER_MQTT_CLIENT_NAME     "HydroPonics-1"           //used to define MQTT topics, MQTT Client ID, and ArduinoOTA

// *** DEBUG MODE ON ***

#define DEBUG     true

// *** OLED SCREEN SETTINGS ***

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREEN_DEFAULT_TEXT_SIZE 1
#define SCREEN_DEFAULT_TEXT_COLOR WHITE



/***********************  WIFI AND MQTT SETUP *****************************/
/***********************  DON'T CHANGE THIS INFO *****************************/

#define SENSOR_1_PIN  3
#define SENSOR_2_PIN  4
#define SENSOR_3_PIN  5
#define SENSOR_4_PIN  6
#define ONE_WIRE      7
#define ATTENTION_LED 8

const char* ssid = USER_SSID ; 
const char* password = USER_PASSWORD ;
const char* mqtt_server = USER_MQTT_SERVER ;
const int mqtt_port = USER_MQTT_PORT ;
const char *mqtt_user = USER_MQTT_USERNAME ;
const char *mqtt_pass = USER_MQTT_PASSWORD ;
const char *mqtt_client_name = USER_MQTT_CLIENT_NAME ; 

#define USER_MQTT_TEMPC            USER_MQTT_CLIENT_NAME"/tempc"
#define USER_MQTT_TEMPF            USER_MQTT_CLIENT_NAME"/tempf"
#define USER_MQTT_HUMIDITY         USER_MQTT_CLIENT_NAME"/humidity"
#define USER_MQTT_PRESSURE_HPA     USER_MQTT_CLIENT_NAME"/pressure_HPA"
#define USER_MQTT_PRESSURE_inHG    USER_MQTT_CLIENT_NAME"/pressure_inHG"
#define USER_MQTT_BME_STATUS       USER_MQTT_CLIENT_NAME"/bmestatus"
#define USER_MQTT_DISPLAY_STATUS   USER_MQTT_CLIENT_NAME"/displaystatus"
#define USER_MQTT_SENSOR_1         USER_MQTT_CLIENT_NAME"/sensor1"
#define USER_MQTT_SENSOR_2         USER_MQTT_CLIENT_NAME"/sensor2"
#define USER_MQTT_SENSOR_3         USER_MQTT_CLIENT_NAME"/sensor3"
#define USER_MQTT_SENSOR_4         USER_MQTT_CLIENT_NAME"/sensor4"
#define USER_MQTT_A0_RAW           USER_MQTT_CLIENT_NAME"/A0_raw"

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// *****************  DEFINE JSON SIZE *************************************

const int capacity = JSON_OBJECT_SIZE(3) + 6 * JSON_OBJECT_SIZE(1);
StaticJsonDocument<capacity> doc;

/*****************  DECLARATIONS  ****************************************/
WiFiClient espClient;
PubSubClient client(espClient);
//SimpleTimer timer;
Adafruit_BME280 bme;


/*****************  GENERAL VARIABLES  *************************************/

bool boot = true;
bool missingBME = false;
bool missingDisplay = false;

char charPayload[50];
int wakeDelay = 1000;
int sendDelay = 10000;
int lastSendMilli = 0;

float temp;
float hum;
float pres;

String currentMessage;
String currentCommand;

// CONSTANTS

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

void reconnect() 
{
  // Loop until we're reconnected
  int retries = 0;
  while (!client.connected()) {
    if(retries < 150)
    {
        Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect(mqtt_client_name, mqtt_user, mqtt_pass)) 
      {
        Serial.println("connected");
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
        client.subscribe(USER_MQTT_CLIENT_NAME"/command");
        client.subscribe(USER_MQTT_CLIENT_NAME"/test");
        client.subscribe(USER_MQTT_CLIENT_NAME"/settings");
      } 
      else 
      {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
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

/************************** MQTT CALLBACK ***********************/

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  String newTopic = topic;
  Serial.print(topic);
  Serial.print("] ");
  payload[length] = '\0';
  String newPayload = String((char *)payload);
  int intPayload = newPayload.toInt();
  Serial.println(newPayload);
  Serial.println();
  newPayload.toCharArray(charPayload, newPayload.length() + 1); 
  String stateMessage;

  // Command
  // Test
  if (newTopic == USER_MQTT_CLIENT_NAME"/command") 
  {
    //commands:
    // set wakeup timer
    // read

    DeserializationError err = deserializeJson(doc, payload);
    
    if (err)
    {
      stateMessage = "Deserialization Error: " + String((char *)err.c_str());
      stateMessage.toCharArray(charPayload, stateMessage.length() + 1);
      client.publish(USER_MQTT_CLIENT_NAME"/status", charPayload);
    } else {
      if (doc["action"] == "off") {
        //off();
        client.publish(USER_MQTT_CLIENT_NAME"/state", "OFF");
      }
      
    }
  }
  
  if (newTopic == USER_MQTT_CLIENT_NAME"/setings")
  {
    //settings:
    // set wakeup timer
    
    //timer.setTimeout(wakeDelay, increaseSunPhase);
    
  }
  


  if (newTopic == USER_MQTT_CLIENT_NAME"/test")
  {
    
    client.publish(USER_MQTT_CLIENT_NAME"/state", "Running Test");
    runTest();
  }
}


void runTest()
{
  client.publish(USER_MQTT_CLIENT_NAME"/test", "Test Complete");
}

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
  
  //textToDisplay1.toCharArray(charPayload, textToDisplay1.length() + 1);
  //client.publish(USER_MQTT_DISPLAY_STATUS, charPayload);
}

void sendStats()
{
  String msg;
  String displayString;
  unsigned int raw=0;
  float volt=0.0;
  float adjustment;

  // Check to make sure a BME280 was detected before attempting to read values
  if (!missingBME) {
    if (DEBUG) {
      Serial.println("Reading Temp, Humidity and Pressure...");
    }
    
    temp = bme.readTemperature();
    hum = bme.readHumidity();
    pres = bme.readPressure()/100.0F;

    if (DEBUG) {
      Serial.println(temp);
      Serial.println(hum);
      Serial.println(pres);
      Serial.println((pres * .03));
      Serial.println(raw);
      Serial.println(volt);
    }
        
    msg = String(temp);
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_TEMPC, charPayload);
  
    msg = String(((temp * 1.8) + 32));
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_TEMPF, charPayload);
  
    msg = String(hum);
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_HUMIDITY, charPayload);
  
    msg = String(pres);
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_PRESSURE_HPA, charPayload);

    msg = String((pres * .03));
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_PRESSURE_inHG, charPayload);

    msg = String("BME Detected");
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_BME_STATUS, charPayload);
  } else {
    msg = String("No BME Detected!");
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_BME_STATUS, charPayload);
  }

  // Read sensors
  raw = analogRead(A0);

  msg = String(raw);
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_A0_RAW, charPayload);
    
  msg = String(digitalRead(SENSOR_1_PIN));
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_SENSOR_1, charPayload);

  msg = String(digitalRead(SENSOR_2_PIN));
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_SENSOR_2, charPayload);

  msg = String(digitalRead(SENSOR_3_PIN));
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_SENSOR_3, charPayload);

  msg = String(digitalRead(SENSOR_4_PIN));
  msg.toCharArray(charPayload, msg.length() + 1);
  client.publish(USER_MQTT_SENSOR_4, charPayload);

  if (!missingDisplay) {
    displayString = "T:" + String(temp) + " c \nH:" + String(hum) + " % \nP:" + String((pres * .03)) + " in\nD2:" + String(digitalRead(D2)) + " A0:" + String(raw);
    displayText(displayString);
  } else {
    msg = String("No Display Detected!");
    msg.toCharArray(charPayload, msg.length() + 1);
    client.publish(USER_MQTT_DISPLAY_STATUS, charPayload);
  }

  delay(2000);
}

void setup() 
{
  Serial.begin(115200);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  ArduinoOTA.setHostname(USER_MQTT_CLIENT_NAME);
  ArduinoOTA.begin(); 

  if ( !bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 Sensor!");
    client.publish(USER_MQTT_CLIENT_NAME"/error", "Could not find a valid BME280 Sensor!");
    missingBME = true;
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    missingDisplay = true;
  } else {
    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();
    display.display();
  }

   pinMode(A0, INPUT);
   pinMode(D2, INPUT_PULLUP);
  
}

void loop() 
{
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle();
  if(boot == false)
  {
    if (millis() >= (lastSendMilli + sendDelay)) {
      sendStats();
      lastSendMilli = millis();
    //  Serial.println("Going To Sleep");
    //  ESP.deepSleep(60e6);
    //  Serial.println("Awake");
    //  setup_wifi();
    }
  }
}
