#include <WiFi.h>
#include <ThingsBoard.h>
#include "esp_wpa2.h"
#include <Wire.h>

// Nucleo address
#define SLAVE_ADDR 9

// Pins for I2C data transfer and clock.
#define I2C_SDA 21
#define I2C_SCL 22

// WiFi
#define WIFI_AP_NAME        "zoltan"
#define WIFI_PASSWORD       "zoltan123"


// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define TOKEN               "ePT1c7vn6CDhCNmqta0m"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

WiFiClient espClient;           // Initialize ThingsBoard client
PubSubClient client(espClient);
ThingsBoard tb(espClient);      // Initialize ThingsBoard instance
int status = WL_IDLE_STATUS;    // the Wifi radio's status

int quant = 1;                  // 50
int updateDelay = 10000;        // Initial update delay.
int lastUpdate  = 0;            // Time of last update.
bool subscribed = false;        // Set to true if application is subscribed for the RPC messages.

// Initialise current variables and target defaults.
double current_temp, current_ph, target_temp = 30, target_ph = 5;
int current_rpm, target_rpm = 1250;

// Size for I2C byte array.
const int size = sizeof(double)*2 + sizeof(int);


RPC_Response processTemperatureChange(const RPC_Data &data)
{
  Serial.println("--- TEMP CHANGE ---");

  // Process data
  target_temp = data;
  Serial.print("Temperature: ");
  Serial.println(target_temp);

  return RPC_Response();
}

RPC_Response processPhChange(const RPC_Data &data)
{
  Serial.println("--- PH CHANGE ---");

  // Process data
  target_ph = data;
  Serial.print("Ph: ");
  Serial.println(target_ph);

  return RPC_Response();
}

RPC_Response processRpmChange(const RPC_Data &data)
{
  Serial.println("--- RPM CHANGE ---");
  // Process data
  target_rpm = data;
  Serial.print("RPM : ");
  Serial.println(target_rpm);

  return RPC_Response();
}
// RPC handlers
const size_t callbacks_size = 3;
RPC_Callback callbacks[callbacks_size] = {
  { "setTemperature",    processTemperatureChange },
  { "setpH",             processPhChange },
  { "setRPM",            processRpmChange }
};

// ---------------------------------------------------

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

// ---------------------------------------------------

void packData(byte* data, double t, double p, int r) {
  memcpy(data, &t, sizeof(double));
  memcpy(data + sizeof(double), &p, sizeof(double));
  memcpy(data + sizeof(double)*2, &r, sizeof(int));
}

void unpackData(byte* data, double* t, double* p, int* r) {
  memcpy(t, data, sizeof(double));
  memcpy(p, data + sizeof(double), sizeof(double));
  memcpy(r, data + sizeof(double)*2, sizeof(int));
}

// ---------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
  Serial.println("Setup complete.");
}


void loop() {
  long now = millis();

  delay(quant);

  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return;
  }

  // Reconnect to ThingsBoard, if needed
  if (!tb.connected()) {
    subscribed = false;

    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  }

  // Subscribe for RPC, if needed
  if (!subscribed) {
    Serial.println("Subscribing for RPC...");

    // Perform a subscription. All consequent data processing will happen in
    // callbacks as denoted by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;
  }

  // Send the target values from ThingsBoard to the Nucleo.
  byte data[size];
  packData(data, target_temp, target_ph, target_rpm);
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(data, size);
  Wire.endTransmission();

  // Wait before requesting the current values from the Nucleo.
  // This ensure that the wire buffer is emptied before we request.
  delay(10);

  Wire.requestFrom(SLAVE_ADDR, size);
  if (Wire.available()) {
    byte data[size];
    int i=0;
    while (Wire.available() && i<size) {
      data[i] = Wire.read();
      i++;
    }
    unpackData(data, &current_temp, &current_ph, &current_rpm);

    // Log the data.
    Serial.print("Received payload: "); Serial.print(current_temp); Serial.print(" "); Serial.print(current_ph); Serial.print(" "); Serial.println(current_rpm);
    
    // Send data to Thingsboard.
    tb.sendTelemetryFloat("Temperature", current_temp);
    tb.sendTelemetryFloat("pH", current_ph);
    tb.sendTelemetryFloat("RPM", current_rpm);
  }

  // Process messages
  tb.loop();   
}