#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoJson.h>
#include "wiring_private.h"
#include <TinyGPS++.h>

void initialize_sensor();
void initialize_wifi();
boolean reconnect();
void send_data();

struct sensorData{

  float temperature = 0.0F;
  float humidity = 0.0F;
  float pressure = 0.0F;

  float acc_x = 0.0F;
  float acc_y = 0.0F;
  float acc_z = 0.0F;

  float gy_x = 0.0F;
  float gy_y = 0.0F;
  float gy_z = 0.0F;

  double lat = 0.0F;
  double lng = 0.0F;
  double alt = 0.0F;

};


static sensorData sensor_data;
static char payload[256];
StaticJsonDocument<256>doc;

const char ssid[] = "LANTAI BAWAH 2";
const char password[] = "ibudini17";

WiFiClient net;
PubSubClient mqttClient(net);

const char mqtt_broker[] = "w7b0b774.en.emqx.cloud";
const char publish_topic[] = "v1/devices/me/telemetry";

#define CLIENT_ID "mqttx_c2035c2a"
#define USERNAME "ceri_12345"
#define PASSWORD "CeriTech12345"

Adafruit_BME680 bme;
const long interval = 1000;
unsigned long lastMillis = 0;
long lastReconnectAttempt = 0;

TinyGPSPlus gps;
//5 = Rx & 6 = Tx
Uart gpsSerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0); 
//Uart gpsSerial (&sercom1, 13, 8, SERCOM_RX_PAD_1, UART_TX_PAD_2);

void SERCOM0_Handler(){

  gpsSerial.IrqHandler();

}

void setup() {

  Serial.begin(9600);
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  gpsSerial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  initialize_sensor();
  initialize_wifi();
  mqttClient.setServer(mqtt_broker, 12143); //12143
  lastReconnectAttempt = 0;

}

void loop() {

  if (WiFi.status() != WL_CONNECTED)
  {
    initialize_wifi();
  }

  if (!mqttClient.connected())
  {
    long now = millis();

    if (now - lastReconnectAttempt > 5000)
    {
      lastReconnectAttempt = now;

      if (reconnect())
      {
        lastReconnectAttempt = 0;
      }
      
    }
    
  } else
  {
    mqttClient.loop();
  }
  
  if (millis() - lastMillis > interval) {

    lastMillis = millis();
    send_data();
    
  }

}

void initialize_sensor(){

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME680");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }

}

void initialize_wifi(){

  delay(100);
  WiFi.disconnect();
  Serial.println();
  Serial.print("Firmware version: ");
  Serial.println(WiFi.firmwareVersion());
  Serial.print("Connecting WiFi to: ");
  Serial.println(ssid);

  while(WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    Serial.print("Attempting WiFi connection .....");
    
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.println("\nWiFi RSSI: ");
      Serial.println(WiFi.RSSI());
      digitalWrite(LED_BUILTIN, HIGH);
    }

    else
    {
      Serial.print("Failed to connect to WiFi");
      Serial.println(", Try again in 5 seconds");
      digitalWrite(LED_BUILTIN, LOW);
      delay(5000);
    }
    
  }
  
}

boolean reconnect(){

  Serial.println("Attempting to connect MQTT");

  if (mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD))
  {
    Serial.println("Connected to MQTT broker");
  }

  return mqttClient.connected();
  
}

void send_data(){

  if (!bme.performReading())
  {
    Serial.println("Failed to perform reading");
    return;
  }

  float t = bme.temperature;
  float h = bme.humidity;
  float p = bme.pressure / 100.0;

  sensor_data.temperature = t;
  sensor_data.humidity = h;
  sensor_data.pressure = p;

  float acc_x, acc_y, acc_z;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    sensor_data.acc_x = acc_x;
    sensor_data.acc_y = acc_y;
    sensor_data.acc_z = acc_z;
  }

  float gy_x, gy_y, gy_z;

  if (IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(gy_x, gy_y, gy_z);
    sensor_data.gy_x = gy_x;
    sensor_data.gy_y = gy_y;
    sensor_data.gy_z = gy_z;
  }

 
  while (gpsSerial.available() > 0)
    (gps.encode(gpsSerial.read()));

  double lat, lng, alt;

  lat = gps.location.lat();
  lng = gps.location.lng();
  alt = gps.altitude.meters();

  sensor_data.lat = lat;
  sensor_data.lng = lng;
  sensor_data.alt = alt;      
 
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring:");
    while(true);
  }
  //For environment data
  doc["Temp"] = sensor_data.temperature;
  doc["Hum"] = sensor_data.humidity;
  doc["Press"] = sensor_data.pressure;
  
  //For landslide and soil movement data (Accelerometer & gyroscope)
  doc["ACC_X"] = sensor_data.acc_x;
  doc["ACC_Y"] = sensor_data.acc_y;
  doc["ACC_Z"] = sensor_data.acc_z;

  doc["GY_X"] = sensor_data.gy_x;
  doc["GY_Y"] = sensor_data.gy_y;
  doc["GY_Z"] = sensor_data.gy_z;
  
  //For tracking location data
  doc["LAT"] = sensor_data.lat;
  doc["LNG"] = sensor_data.lng;
  doc["ALT"] = sensor_data.alt;
  
  serializeJsonPretty(doc, payload);
  mqttClient.publish(publish_topic, payload);
  Serial.println(payload);
  
}

