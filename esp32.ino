#include <Wire.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include <ThingsBoard.h>

// Pins for I2C data transfer and clock.
#define NUCLEO 9
#define I2C_SDA 21
#define I2C_SCL 22

// WiFi credentials
#define SSID ""
#define PASSWORD ""

// ThingsBoard credentials
#define SERVER "demo.thingsboard.io"
#define TOKEN ""

// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// Initialise WiFi, PubSub (MQTT) and ThingsBoard clients.
WiFiClient wifi;
PubSubClient client(wifi);
ThingsBoard tb(wifi);

int loop_delay = 1; // ms of delay for loop function.

// Initialise current variables and target defaults.
double current_temp, current_ph, target_temp = 30, target_ph = 5;
int current_rpm, target_rpm = 1250;

// Size for I2C byte array.
const int size = sizeof(double)*2 + sizeof(int);

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

void connect() {
  Serial.print("Connecting to network: "); Serial.println(SSID);

  // Connext to the WiFi network.
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // WiFi is now connected.
  Serial.println("WiFi connected.");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
}

// Responses to ThingsBoard RPC requests.
RPC_Response processTemperatureChange(const RPC_Data &data) {
  target_temp = data;

  Serial.println("--- TEMP CHANGE ---");
  Serial.print("Temperature: ");
  Serial.println(target_temp);
  Serial.println("-------------------");

  return RPC_Response();
}

RPC_Response processPhChange(const RPC_Data &data) {
  target_ph = data;

  Serial.println("--- PH CHANGE ---");
  Serial.print("pH: ");
  Serial.println(target_ph);
  Serial.println("-----------------");

  return RPC_Response();
}

RPC_Response processRpmChange(const RPC_Data &data) {
  target_rpm = data;
  
  Serial.println("--- RPM CHANGE ---");
  Serial.print("RPM : ");
  Serial.println(target_rpm);
  Serial.println("------------------");

  return RPC_Response();
}

// RPC handlers
const size_t callbacks_size = 3;
RPC_Callback callbacks[callbacks_size] = {
  { "setTemperature", processTemperatureChange },
  { "setpH", processPhChange },
  { "setRPM", processRpmChange }
};

void setup() {
  // Initialise Wire transmission.
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialise WiFi.
  connect();

  // Initialise Serial transmission.
  Serial.begin(115200);
  Serial.println("Setup complete.");
}


void loop() {
  long now = millis();

  // Reconnect to WiFi, if needed.
  if (WiFi.status() != WL_CONNECTED) {
    connect();
  }

  // Reconnect to ThingsBoard, if needed.
  if (!tb.connected()) {
    bool connected = false, subscribed = false;

    // Connect to ThingsBoard.
    Serial.print("Connecting to: "); Serial.print(SERVER); Serial.print(" with token: "); Serial.println(TOKEN);

    while (!connected) {
      connected = tb.connect(SERVER, TOKEN);
      if (!connected) {
        Serial.println("Failed to connect to ThingsBoard.");
        delay(500);
      }
    }

    // Resubscribe to RPC events.
    Serial.println("Subscribing to RPC events ...");

    while(!subscribed) {
      subscribed = tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks));
      if (!subscribed) {
        Serial.println("Failed to subscribe to RPC events.");
        delay(500);
      }
    }

    Serial.println("Subscribed to RPC events.");
  }

  // Send the target values from ThingsBoard to the Nucleo.
  byte data[size];
  packData(data, target_temp, target_ph, target_rpm);
  Wire.beginTransmission(NUCLEO);
  Wire.write(data, size);
  Wire.endTransmission();

  // Wait before requesting the current values from the Nucleo.
  // This ensure that the wire buffer is emptied before we request.
  delay(10);

  // Request the current values from the Nucleo.
  Wire.requestFrom(NUCLEO, size);
  if (Wire.available()) {
    byte data[size];
    int i=0;
    while (Wire.available() && i<size) {
      data[i] = Wire.read();
      i++;
    }

    unpackData(data, &current_temp, &current_ph, &current_rpm);

    // Log the data.
    Serial.print("Received payload: "); Serial.println("{"); Serial.print("  \"Temperature\": "); Serial.print(current_temp); Serial.println(","); Serial.print("  \"pH\": "); Serial.print(current_ph); Serial.println(","); Serial.print("  \"RPM\": "); Serial.println(current_rpm); Serial.println("}");
    
    // Send data to Thingsboard.
    tb.sendTelemetryFloat("Temperature", current_temp);
    tb.sendTelemetryFloat("pH", current_ph);
    tb.sendTelemetryFloat("RPM", current_rpm);
  }

  // Process messages
  tb.loop();

  delay(loop_delay);
}