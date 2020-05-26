#include <ArduinoJson.h>
#include <WiFi.h>
#include <EasyButton.h>
#include "mqtt.h"
#include "opt3001.h"
#include "BQ27441.h"
#include "SHT3X.h"

//@WIFI
#define WIFI_SSID ""
#define WIFI_PASSWD ""

//@MQTT
#ifndef MQTT_KEEPALIVE
#define MQTT_KEEPALIVE 30
#endif
unsigned long lastMqttConnectMs = 0;
uint8_t report_ms = 0;
uint8_t report_period = 30;
unsigned int postMsgId = 0;
#define PRODUCT_KEY "XXXXXXXX"
#define DEVICE_NAME "XXXXXXXX"
#define DEVICE_SECRET "XXXXXXXX"
#define ALINK_BODY_FORMAT "{\"id\":\"%u\",\"version\":\"1.0\",\"method\":\"%s\",\"params\":%s}"
#define ALINK_TOPIC_PROP_POST "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/event/property/post"
#define ALINK_TOPIC_PROP_SET "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/service/property/set"
#define ALINK_METHOD_PROP_POST "thing.event.property.post"
WiFiClient objClient;
PubSubClient objMQTTClient(objClient);

//@Buttons
#define USR_BTN1_PIN 25
#define USR_BTN2_PIN 26
#define USR_BTN3_PIN 27
EasyButton objUserButton1(USR_BTN1_PIN);
EasyButton objUserButton2(USR_BTN2_PIN);
EasyButton objUserButton3(USR_BTN3_PIN);

void initButtons(void)
{
  objUserButton1.begin();
  //objUserButton1.onPressed(onButtonPressed);
  //objUserButton1.onPressedFor(5000, onPressedForDuration);
  //objUserButton1.onSequence(3 /* number of presses */, 2000 /* timeout */, onSequenceMatched /* callback */);
  
  objUserButton2.begin();
  //objUserButton2.onPressed(onButtonPressed);
  //objUserButton2.onPressedFor(5000, onPressedForDuration);
  //objUserButton2.onSequence(3 /* number of presses */, 2000 /* timeout */, onSequenceMatched /* callback */);

  objUserButton3.begin();
  //objUserButton3.onPressed(onButtonPressed);
  //objUserButton3.onPressedFor(5000, onPressedForDuration);
  //objUserButton3.onSequence(3 /* number of presses */, 2000 /* timeout */, onSequenceMatched /* callback */);
}

void report_buttons()
{
  char param[100];
  char jsonBuf[256];
  sprintf(param, "{\"button1\":%d,\"button2\":%d, \"button3\":%d}", digitalRead(USR_BTN1_PIN), digitalRead(USR_BTN2_PIN), digitalRead(USR_BTN3_PIN));
  postMsgId += 1;
  sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
  if (objMQTTClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
  {
    Serial.print("++++++ POST BUTTONS: ");
    Serial.println(jsonBuf);
  }
}

//@RELAY
#define RELAY1_PIN 33
#define RELAY2_PIN 32

void initRelays()
{
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
}

void report_relays()
{
  char param[32];
  char jsonBuf[100];
  sprintf(param, "{\"relay1\":%d,\"relay2\":%d}", digitalRead(RELAY1_PIN), digitalRead(RELAY2_PIN));
  postMsgId += 1;
  sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
  if (objMQTTClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
  {
    Serial.print("++++++ POST RELAYS: ");
    Serial.println(jsonBuf);
  }
}

//@ECM

//@Faul Gauge
BQ27441 objFaulGauge;
#define CHARGER_ENABLE_PIN 13

//@Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 6800; // e.g. 850mAh battery

void initBQ27441(void)
{
  pinMode(CHARGER_ENABLE_PIN, OUTPUT);
  digitalWrite(CHARGER_ENABLE_PIN, HIGH);
  
  // Use objCharger.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!objFaulGauge.begin()) // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println(" Check wiring and try again.");
    Serial.println(" (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("====== BQ27441 CONNECTED!");
  objFaulGauge.setCapacity(BATTERY_CAPACITY);
}

void report_battery()
{
  //@Read battery stats from the BQ27441-G1A
  unsigned int soc = objFaulGauge.soc();  // Read state-of-charge (%)
  unsigned mvolts = objFaulGauge.voltage(); //// Read battery voltage (mV)
  double volts = objFaulGauge.voltage() / 1000; // Read battery voltage (V)
  int current = objFaulGauge.current(AVG); // Read average current (mA)
  unsigned int fullCapacity = objFaulGauge.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = objFaulGauge.capacity(REMAIN); // Read remaining capacity (mAh)
  int power = objFaulGauge.power(); // Read average power draw (mW)
  int health = objFaulGauge.soh(); // Read state-of-health (%)

  // Now print out those values:
  String toPrint = String(soc) + "% | ";
  toPrint += String(mvolts) + " mV | ";
  toPrint += String(volts) + " V | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "%";

  Serial.println(toPrint);
  
  char param[100];
  char jsonBuf[256];
  sprintf(param, "{\"battery_voltage\":%f,\"battery_percentage\":%d, \"battery_current\":%d, \"battery_health\":%d}", volts, soc, current, health);
  postMsgId += 1;
  sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
  if (objMQTTClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
  {
    Serial.print("++++++ POST BATTERY: ");
    Serial.println(jsonBuf);
  }
}

//@OPT3001 Light Sensor
#define USE_USCI_B1
#define SCL_PIN      22
#define SDA_PIN      21
Opt3001 objOPT3001;
void initOPT3001()
{
  Wire.begin();
  objOPT3001.begin(DEFAULT_CONFIG_100);
}

void report_lux()
{
  uint32_t lux;
  lux = objOPT3001.readResult();
  Serial.print("====== LUX: ");
  Serial.println(lux, DEC);
  
  char param[100];
  char jsonBuf[256];
  sprintf(param, "{\"lux\":%d}", lux);
  postMsgId += 1;
  sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
  if (objMQTTClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
  {
    Serial.print("++++++ POST LUX: ");
    Serial.println(jsonBuf);
  }
}

//@SHT30 Temperature and Humidity Sensor
SHT3X objSHT30(0x44);
#define USE_USCI_B1
#define SCL_PIN      22
#define SDA_PIN      21

void report_sht30()
{
  if(objSHT30.get()==0)
  {
    Serial.print("====== Temperature in Celsius : ");
    Serial.println(objSHT30.cTemp);
    Serial.print("====== Temperature in Fahrenheit : ");
    Serial.println(objSHT30.fTemp);
    Serial.print("====== Relative Humidity : ");
    Serial.println(objSHT30.humidity);
    Serial.println();

    char param[100];
    char jsonBuf[256];
    sprintf(param, "{\"temperature\":%2f,\"humidity\":%2f}", objSHT30.cTemp, objSHT30.humidity);
    postMsgId += 1;
    sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
    if (objMQTTClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
    {
      Serial.print("++++++ POST TEMPERATURE: ");
      Serial.println(jsonBuf);
    }
  }
}

#define LED LED_BUILTIN
#define LED_OFF 0
#define LED_ON 1

int ledState = LED_OFF;
bool boolReportNeeded = true;
//int ledStateMapOutput[2] = {HIGH, LOW};
int ledStateMapOutput[2] = {LOW, HIGH};

String translateEncryptionType(wifi_auth_mode_t encryptionType)
{
  switch (encryptionType)
  {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
  }
}

void scanNetworks()
{
  int numberOfNetworks = WiFi.scanNetworks();
  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);
  for (int i = 0; i < numberOfNetworks; i++)
  {
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));
    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");
  }
}

void connect_wifi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("====== CONNECTING TO WIFI");
  }
  Serial.println("++++++ WIFI CONNECTED");
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
}

void connect_mqtt()
{
  bool boolConnected = connectAliyunMQTT(objMQTTClient, PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET);
  if (boolConnected)
  {
    Serial.println("====== MQTT CONNECTED");
    if (objMQTTClient.subscribe(ALINK_TOPIC_PROP_SET))
    {
      Serial.println("++++++ TOPIC SUBSCRIBED.");
    }
    else
    {
      Serial.println("------ TOPIC SUBSCRIBE FAILED.");
    }
  }
}

void mqttPublish()
{
  char param[32];
  char jsonBuf[128];
  sprintf(param, "{\"LightSwitch\":%d}", ledState);
  postMsgId += 1;
  sprintf(jsonBuf, ALINK_BODY_FORMAT, postMsgId, ALINK_METHOD_PROP_POST, param);
  if (objMQTTClient.publish(ALINK_TOPIC_PROP_POST, jsonBuf))
  {
    Serial.print("Post message to cloud: ");
    Serial.println(jsonBuf);
  }
  else
  {
    Serial.println("Publish message to cloud failed!");
  }
}

// https://pubsubclient.knolleary.net/api.html#callback
void callback(char* topic, byte* payload, unsigned int length)
{
  if (strstr(topic, ALINK_TOPIC_PROP_SET))
  {
    Serial.print("Set message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    payload[length] = '\0';
    Serial.println((char *)payload);

    // Deserialization break change from 5.x to 6.x of ArduinoJson
    DynamicJsonDocument doc(100);
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
      Serial.println("parse json failed");
      return;
    }

    // {"method":"thing.service.property.set","id":"282860794","params":{"LightSwitch":1},"version":"1.0.0"}
    JsonObject setAlinkMsgObj = doc.as<JsonObject>();
    // LightSwitch
    int desiredLedState = setAlinkMsgObj["params"]["LightSwitch"];

    if (desiredLedState == LED_ON || desiredLedState == LED_OFF) {
      boolReportNeeded = true;
      ledState = desiredLedState;

      const char* cmdStr = desiredLedState == LED_ON ? "on" : "off";
      Serial.print("Cloud command: Turn ");
      Serial.print(cmdStr);
      Serial.println(" the light.");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //scanNetworks();
  connect_wifi();

  initButtons();

  initOPT3001();
  
  initBQ27441();
  
  // WiFi.disconnect(true);
  // Serial.println(WiFi.localIP());
  
  objMQTTClient.setCallback(callback);
  
  lastMqttConnectMs = millis();
  connect_mqtt();

}

void loop()
{

  objUserButton1.read();
  objUserButton2.read();
  objUserButton3.read();
  
  if (millis() - lastMqttConnectMs >= 5000)
  {
    lastMqttConnectMs = millis();
    connect_mqtt();
  }

  //@Report
  if(report_ms > report_period)
  {
    report_ms = 0;
    report_sht30();
    report_relays();
    report_battery();
    report_lux();
  }
  else
  {
    report_ms++;
    delay(100);
  }
  
  // https://pubsubclient.knolleary.net/api.html#loop
  if (!objMQTTClient.loop())
  {
    Serial.println("------ MQTT DISCONNECTED");
  }

  /*
  digitalWrite(LED, ledStateMapOutput[ledState]);
  
  if (boolReportNeeded)
  {
    //mqttPublish();
    boolReportNeeded = false;
  }
  */
}
