#include <WiFi.h>             // WiFi control for ESP32
#include <ThingsBoard.h>      // ThingsBoard SDK
#include "esp_wpa2.h"   //wpa2 library for connections to Enterprise networks
#include <Wire.h>

// Define Slave I2C Address. All slaves must be allocated a
// unique number
#define SLAVE_ADDR 9

// Define the pins used for SDA and SCL. This is important because
// there is a problem with the TTGO and I2C will not work properly
// unless you do.
#define I2C_SDA 21
#define I2C_SCL 22

//#define EAP_IDENTITY "insert your user name@ucl.ac.uk here"                
//#define EAP_PASSWORD "insert your password here"
//const char* ssid = "eduroam";

// WiFi
//
#define WIFI_AP_NAME        "zoltan"
#define WIFI_PASSWORD       "zoltan123"


// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
//


#define TOKEN               "ePT1c7vn6CDhCNmqta0m"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

WiFiClient espClient;           // Initialize ThingsBoard client
PubSubClient client(espClient);
ThingsBoard tb(espClient);      // Initialize ThingsBoard instance
int status = WL_IDLE_STATUS;    // the Wifi radio's status

int quant = 1;                  // Main application loop delay
int updateDelay = 10000;        // Initial update delay.
int lastUpdate  = 0;            // Time of last update.
bool subscribed = false;        // Set to true if application is subscribed for the RPC messages.
int temp = 0; // ? change to double
int pH = 0; // ? change to double
int rpm = 0;
int r_temp, r_ph, r_rpm;

// Processes function for RPC call "setValue"
// RPC_Data is a JSON variant, that can be queried using operator[]
// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
RPC_Response processTemperatureChange(const RPC_Data &data)
{
  Serial.println("Received the set temperature RPC method");

  // Process data
  r_temp = data;
  Serial.print("Temperature: ");
  Serial.println(r_temp);

  return RPC_Response();
}

RPC_Response processPhChange(const RPC_Data &data)
{
  Serial.println("Received the set ph RPC method");

  // Process data
  r_ph = data;
  Serial.print("Ph: ");
  Serial.println(r_ph);

  return RPC_Response();
}

RPC_Response processRpmChange(const RPC_Data &data)
{
  Serial.println("Received the set rpm RPC method");

  // Process data
  r_rpm = data;
  Serial.print("RPM : ");
  Serial.println(r_rpm);

  return RPC_Response();
}
// RPC handlers
const size_t callbacks_size = 3;
RPC_Callback callbacks[callbacks_size] = {
  { "setTemperature",    processTemperatureChange },
  { "setpH",             processPhChange },
  { "setRPM",            processRpmChange }
};


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


void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); // Configure the pins
  // tft.init();
  // tft.setRotation(1);

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
}


void loop() {
  long now = millis();
 // device SLAVE_ADDR

  delay(300);

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

  Wire.requestFrom(SLAVE_ADDR, 3); // request 3 bytes from slave
  //delay(10);
  while (1<Wire.available()) { // slave may send less than requested
    temp = Wire.read();
    pH = Wire.read();
    rpm = Wire.read();
  }

  //Serial.print(temp); Serial.print(" "); Serial.print(pH); Serial.print(" "); Serial.println(rpm);

  // Uploads new telemetry to ThingsBoard using MQTT. 
  // See https://thingsboard.io/docs/reference/mqtt-api/#telemetry-upload-api 
  // for more details
  tb.sendTelemetryFloat("Temperature", temp);
  tb.sendTelemetryFloat("pH", pH);
  tb.sendTelemetryFloat("RPM", rpm);

  Serial.println("sending to arduino");
  // wire.write for pH temp RPM
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(r_temp);
  Wire.write(r_ph);
  Wire.write(r_rpm);
  Wire.endTransmission();

  // Process messages
  tb.loop();   
}
