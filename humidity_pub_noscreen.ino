#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> // BMP280 library
//#include "SSD1306.h"  // Library for SSD1306 OLED display

//it's got a display, so I'm using "ESP32S2 Dev Module"    hold boot>press rst> release boot
//libraries via sketch>include:   

// Wi-Fi credentials
const char* ssid = "";
const char* password = "";

// MQTT broker details
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = ""; // Anonymous access
const char* mqtt_password = "";



// OLED display setup (I2C address: 0x3C, pins: GPIO 5 (SCL), GPIO 4 (SDA))
//SSD1306 display(0x3C, 5, 4);

// BMP280 setup
Adafruit_BMP280 bmp; // I2C interface (default address: 0x76 or 0x77)

// Wi-Fi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to Wi-Fi
void setupWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
}

// Function to connect to MQTT broker
void connectToMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32_Humidity_Sensor", mqtt_user, mqtt_password)) {
      Serial.println("Connected to MQTT!");
    } else {
      Serial.print("Failed MQTT connection. State: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void setup_spoof() {
  Serial.begin(115200);
  pinMode(15, OUTPUT);

  for (int i = 0; i < 3; i++){
    //digitalWrite(15, HIGH);delay(1000);digitalWrite(15, LOW);delay(1000); 
  }

  // Initialize Wi-Fi
  setupWiFi();

  // Initialize MQTT
  client.setServer(mqtt_server, mqtt_port);
  connectToMQTT();
}

void setup() {
  Serial.begin(115200);
  pinMode(15, OUTPUT);

    // Initialize I2C with explicit pins: SDA = GPIO33, SCL = GPIO35
  Wire.begin(33, 35);

  for (int i = 0; i < 8; i++){
    //digitalWrite(15, HIGH);delay(1000);digitalWrite(15, LOW);delay(1000); 
  }

  // Initialize OLED display
  // display.init();
  // display.setFont(ArialMT_Plain_24);
  // display.setTextAlignment(TEXT_ALIGN_CENTER);
  // display.clear();
  // display.drawStringMaxWidth(64, 20, 128, "Initializing...");
  // display.display();

  // delay(2000);
  // display.clear();
  // display.drawStringMaxWidth(64, 20, 128, "Initialized.");
  // display.display();

  // Initialize BMP280 sensor
  if (!bmp.begin(0x76)) { // Default I2C address
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    //display.clear();
    //display.drawStringMaxWidth(64, 20, 128, "BMP280 Error!");
    //display.display();

  for (int i = 0; i < 10; i++){
    digitalWrite(15, HIGH);delay(3000);digitalWrite(15, LOW);delay(3000); 
  }

    while (1); // Stop execution
  }

  // Set BMP280 settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // Pressure oversampling
                  Adafruit_BMP280::SAMPLING_X16,  // Temperature oversampling
                  Adafruit_BMP280::FILTER_X16,    // Filter
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time

  // Initialize Wi-Fi
  setupWiFi();
  // display.clear();
  // display.drawStringMaxWidth(64, 20, 128, "WiFi connected.");
  // display.display();

  // Initialize MQTT
  client.setServer(mqtt_server, mqtt_port);
  connectToMQTT();
  // display.clear();
  // display.drawStringMaxWidth(64, 20, 128, "MQTT connected.");
  // display.display();

  // Clear display after initialization
  // display.clear();
  // display.drawStringMaxWidth(64, 20, 128, "Ready!");
  // display.display();
}

// MQTT topic for humidity data
const char* mqtt_topic = "home/sensors999/s22/humidity";

void loop() {
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  // Read data from BMP280
  float temp_c = bmp.readTemperature(); // Temperature in °C //random(30, 70) + random(0, 99) / 100.0;//
  float pressure_h = bmp.readPressure() / 100.0F; // Pressure in hPa

  // Publish the data to MQTT
  char payload[100]; // Adjust buffer size if needed
  snprintf(payload, sizeof(payload), "{\"temperature\": %.2f, \"pressure\": %.2f}", temp_c, pressure_h);
  client.publish(mqtt_topic, payload);

  // Display data on the OLED
  // display.clear();
  // display.drawStringMaxWidth(64, 10, 128, "Humidity:");
  // display.drawStringMaxWidth(64, 40, 128, String(temp_c) + " %");
  // display.display();

  // Debug output to Serial Monitor
  Serial.print("Published fake humidity data: ");
  Serial.println(payload);



  for (int i = 0; i < 20; i++){
    digitalWrite(15, HIGH);delay(500);digitalWrite(15, LOW);delay(500); 
  }

  // Send data every 10 seconds
}
