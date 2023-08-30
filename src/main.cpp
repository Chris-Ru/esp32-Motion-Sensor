// std library for Arduinos
#include <Arduino.h>

#include <AsyncMqttClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

// external library dependency
// Network Time Protocal client
#include <NTPClient.h>

// c++ std library for wifi
#include <WiFi.h>
// c++ std library for wifi User Datagram Protocol
#include <WiFiUdp.h>

// c++ std library for time
#include "time.h"

// PIN + PIN STATE
const int red_light_pin = 25;         // GPIO 25 connected for pin for red light
const int green_light_pin = 33;       // GPIO 33 connected for pin for green light
const int blue_light_pin = 32;        // GPIO 32 connected for pin for blue light
const int onboardLedPin = 2;          // GPIO 2 connected for blue light on ESP32 board
const int PIN_TO_SENSOR = 18;         // GIOP19 pin connected to OUTPUT pin of PIR sensor
const int LDR = A0;                   // GIOP36 pin connected for photoresistor

// Pin State for motion sensor
int pinStateCurrent   = LOW;          // current state of pin
int pinStatePrevious  = LOW;          // previous state of pin

// TIME + WIFI
// Replace with your network credentials
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

#define MQTT_USER "Username"
#define MQTT_PASSWORD "Password"
#define MQTT_PUB_TOPIC "Topic"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
AsyncWebServer server(80);

// Function PROTOTYPES
void initTime(String timezone);

void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void connectToWifi();
void initWifi();

void initMQTT();

void setTimezone(String timezone);
void printLocalTime();
void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst);

void RGB_color(int red_light_value, int green_light_value, int blue_light_value);

void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);

void initWebServer();

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(9600);

  //initialize pins for output or input for each device 
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
  pinMode(onboardLedPin, OUTPUT);
  pinMode(PIN_TO_SENSOR, INPUT);      // set ESP32 pin to input mode to read value from OUTPUT pin of sensor
  pinMode(LDR, INPUT);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  initMQTT();

  connectToWifi();

  initWebServer();
}

// the loop function runs over and over again forever
void loop() {
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }

  String currTime = timeClient.getFormattedTime();
  int splitT = formattedDate.indexOf("T");
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1); // get hour

  pinStatePrevious = pinStateCurrent; // store old state
  pinStateCurrent = digitalRead(PIN_TO_SENSOR);   // read new state

  int ldrStatus = analogRead(LDR);        // read value from A0 analog pin
  int pir = 0;

  if (pinStatePrevious == LOW) {         // pin state change: LOW -> HIGH
    if(pinStateCurrent == HIGH) {
      Serial.println("Motion detected!");
      pir = 1;

      if(timeStamp > 21 && timeStamp < 6) {
        digitalWrite(onboardLedPin, HIGH);
        Serial.println("LED is on\n");
      }
        
      // Print the raw photocell value and the converted led value (e,g., for Serial Console and Serial Plotter)
      Serial.print("LDR resistance: ");
      Serial.println(ldrStatus);
      Serial.println("\n");

      // publish a new MQTT message
      char data[140];

      sprintf(data, "{\"sensor\": {\"PIR\": %d, \"LDR\": %d}}", pir, ldrStatus);

      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TOPIC, 0, true, data);

      Serial.printf("Publishing on topic %s at QoS 0, packetId: %i - ", MQTT_PUB_TOPIC, packetIdPub1);
      Serial.printf("Message: %s\n", data);
    }
  }

  else if (pinStatePrevious == HIGH) {   // pin state change: HIGH -> LOW
    if(pinStateCurrent == LOW) {
      Serial.println("Motion stopped!");

      digitalWrite(onboardLedPin, LOW);
      Serial.println("LED is off\n");

      // Print the raw photocell value and the converted led value (e,g., for Serial Console and Serial Plotter)
      Serial.print("LDR resistance: ");
      Serial.println(ldrStatus);
      Serial.println("");

      // publish a new MQTT message
      char data[140];

      sprintf(data, "{\"sensor\": {\"PIR\": %d, \"LDR\": %d}}", pir, ldrStatus);

      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TOPIC, 0, true, data);

      Serial.printf("Publishing on topic %s at QoS 0, packetId: %i - ", MQTT_PUB_TOPIC, packetIdPub1);
      Serial.printf("Message: %s\n", data);
    }
  }
}

int version = 2;

void initWebServer()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
      char message[80];
      sprintf(message, "Hi. This is the Motion and LDR Sensor - version %d.", version);

      request->send(200, "text/plain", message);
  });

  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request)
  {
      request->send(200, "text/plain", "Rebooting...");
      ESP.restart();
  });


  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

void initMQTT()
{
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer("IP Address", 1883);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.print("Disconnected from MQTT, Reason = ");
  Serial.println((int)reason);

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void initWifi()
{
  WiFi.onEvent(WiFiEvent);

  connectToWifi();
}

void connectToWifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to WiFi...");

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }

  Serial.println("Connected to WiFi!");
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");

  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);

  if (event == SYSTEM_EVENT_STA_GOT_IP)
  {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
  }
  else if (event == SYSTEM_EVENT_STA_DISCONNECTED)
  {
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
  }
}



void RGB_color(int red_light_value, int green_light_value, int blue_light_value) {
  digitalWrite(red_light_pin, red_light_value);
  digitalWrite(green_light_pin, green_light_value);
  digitalWrite(blue_light_pin, blue_light_value);
}

void setTimezone(String timezone){
  Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void initTime(String timezone){
  struct tm timeinfo;

  Serial.println("Setting up time");
  configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
  while(!getLocalTime(&timeinfo)){
    Serial.println("  Failed to obtain time");
    return;
  }
  Serial.println("  Got the time from NTP");
  // Now we can set the real timezone
  setTimezone(timezone);
}

void printLocalTime(){
  struct tm timeinfo;
  while(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time 1");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S "); // zone %Z %z
}

void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst){
  struct tm tm;

  tm.tm_year = yr - 1900;   // Set date
  tm.tm_mon = month-1;
  tm.tm_mday = mday;
  tm.tm_hour = hr;      // Set time
  tm.tm_min = minute;
  tm.tm_sec = sec;
  tm.tm_isdst = isDst;  // 1 or 0
  time_t t = mktime(&tm);
  Serial.printf("Setting time: %s", asctime(&tm));
  struct timeval now = { .tv_sec = t };
}