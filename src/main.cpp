#include <Arduino.h>


// Demonstrate the use of WiFi.shutdown() and WiFi.resumeFromShutdown()
// Released to public domain

// Current on WEMOS D1 mini (including: LDO, usbserial chip):
// ~85mA during normal operations
// ~30mA during wifi shutdown
//  ~5mA during deepsleep


#ifndef RTC_USER_DATA_SLOT_WIFI_STATE
#define RTC_USER_DATA_SLOT_WIFI_STATE 33u
#endif

#include <ESP8266WiFi.h>
#include <include/WiFiState.h> // WiFiState structure details
#include "DHT.h"
#include <ArduinoJson.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <PubSubClient_JSON.h>

#include <secrets.h>


#define DEVICENAME "esp01dht22"


#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define WIFI_RETRY_TIME 60e6 //TODO: store in rtc number of retries and make this progresively longer
#define MEASUREMENT_INTERVAL 5*60e6 // time in us, 5 min

const char* ssid = SSID;
const char* password = PASSWORD;

// If true, failing to connect to the first wifi 
bool wifi2 = true; 
const char* ssid2 = SSID2;
const char* password2 = PASSWORD2;

// MQTT server info
const char *mqtt_server = MQTT_SERVER;
const char *status_topic = STATUS_TOPIC;
const char *data_topic = DATA_TOPIC;
const char *debug_topic = DEBUG_TOPIC;


ADC_MODE(ADC_VCC) // Needed to measure vcc with the internal ADC

WiFiState state;
WiFiClient espClient;
PubSubClient client(espClient);

DHT dht(DHTPIN, DHTTYPE);


float h,t,hic,vcc;
StaticJsonDocument<300> doc;
String debug_msg;

bool prewifi()
{
  dht.begin();

  vcc=ESP.getVcc()/1024.0;

  if(vcc<2.5){
    //Wait and double-check
    delay(1000);
    vcc=ESP.getVcc()/1024.0;
    if(vcc<2.5)
    {
      Serial.println("Battery is dying, going to sleep for as long as possible");
      Serial.flush();
      // Go back to sleep and dont bother with wifi, the previous vcc values should give us a hint that the battery is low
      ESP.deepSleep(3*60*60e6, RF_DISABLED); // 3 hours, close to the max of the esp8266
      return false;
    }
  }
  doc["vcc"]=vcc;

  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
    // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));   
    debug_msg = "Failed to read from DHT sensor!";
    return false;
  }
  hic = dht.computeHeatIndex(t, h, false);

  doc["temp"]=t;
  doc["hum"]=h;
  doc["hic"]=hic;

  debug_msg = "Sensor working correctly.";
  Serial.println("Measurements taken, connecting");
  return true;
}

void callback(const MQTT::Publish& pub){}

bool wifistuff(){
  client.set_server(mqtt_server, 1883);
  client.set_callback(callback);
  
  if (!client.connected()) {
    int tries=0;
    while (!client.connected()&&tries<3) {
      Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      MQTT::Connect con(DEVICENAME);
      con.set_will(status_topic, "Device down.",true).set_keepalive(MEASUREMENT_INTERVAL/1e6*1.2);
      if (client.connect(con)) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish(MQTT::Publish(status_topic, "Device online.").set_retain());
        // ... and resubscribe
        //client.subscribe("nodemcu8266/led");
      } else {
        tries++;
        Serial.println("Failed try again in 2 seconds");
        // Wait 2 seconds before retrying
        delay(2000);
      }
    }  
    if(tries ==3){
      Serial.println("Cant connect to MQTT.");
      return false;
    }
  }
  client.loop();

  client.publish(MQTT::PublishJSON(data_topic, doc).set_qos(1));
  client.publish(MQTT::Publish(debug_topic,debug_msg).set_retain());
  return true;
}

void setup() {
  Serial.begin(115200);
  //Serial.setDebugOutput(true);  // If you need debug output
  Serial.println("Trying to resume WiFi connection...");

  // May be necessary after deepSleep. Otherwise you may get "error: pll_cal exceeds 2ms!!!" when trying to connect
  delay(1);

  // ---
  // Here you can do whatever you need to do that doesn't need a WiFi connection.
  // ---
  if(!prewifi())
  {
    Serial.println("Sensor unavailable, going back to sleep."); // we wont do this to allow debugging
//    Serial.flush();
//    ESP.deepSleep(10e6, RF_DISABLED);
  }

  // NOW WE CONNECT TO WIFI
  ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));
  unsigned long start = millis();

  if (!WiFi.resumeFromShutdown(state)
      || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
    Serial.println("Cannot resume WiFi connection, connecting via begin...");
    WiFi.persistent(false);

    if (!WiFi.mode(WIFI_STA)
        || !(WiFi.begin(ssid, password) || (wifi2 && WiFi.begin(ssid2, password2)))
        || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
      WiFi.mode(WIFI_OFF);
      Serial.println("Cannot connect!");
      Serial.flush();
      ESP.deepSleep(WIFI_RETRY_TIME, RF_DISABLED);
      return;
    }
  }else{
    Serial.println("Resumed wifi connection.");
  }

  unsigned long duration = millis() - start;
  Serial.printf("Duration: %f", duration * 0.001);
  Serial.println();

  // ---
  // Here you can do whatever you need to do that needs a WiFi connection.
  // ---
  wifistuff();

  //Shut down wifi
  WiFi.shutdown(state);
  ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));

  // ---
  // Here you can do whatever you need to do that doesn't need a WiFi connection anymore.
  // ---

  Serial.println("Done. Sleeping...");
  Serial.flush();
  ESP.deepSleep(MEASUREMENT_INTERVAL, RF_DISABLED);
}

void loop() {
  // Nothing to do here.
}

