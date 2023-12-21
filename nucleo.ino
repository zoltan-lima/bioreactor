#include <Wire.h>

// Pin for I2C communication.
#define ESP32 9

// Temperature pins.
#define THERMISTOR A0
#define HEATING_ELEMENT 5

// pH pins.
#define PH_PROBE A1
#define ACID_PUMP 10 // red
#define ALKALI_PUMP 11 // black

// Stirring pins.
#define INTERRUPT 2
#define MOTOR 6

// Initialise current variables and target defaults.
double current_temp, current_ph, target_temp = 30, target_ph = 5;
int current_rpm, target_rpm = 1250;

// Struct for PI controllers.
struct controller {
  float Kp, Ki;
  float error, interror;
  float output;
};

// Required values for PI calculations.
long currtime, prevtime, deltaT;

// Required values for the temp subsystem.
controller t;
t.Ki = 0.351; t.Kp = 100; t.interror = 0; t.output = 0;

// Required values for the pH subsystem.
const double ph_tolerance = 0.5;
double prev_target_ph = 5;
long last_change;
int ph_pwm_value = 100;

// Required values for the stirring subsystem.
const float Kv=800;
const float T=0.25;

const float wn=4;
const float zeta=1;

const float wo=1/T;

controller s;
s.Ki = wn*wn/Kv/wo; s.Kp = (2*zeta*wn/wo-1)/Kv; s.interror = 0; s.output = 0;

long pulseT, prevpulseT, prevprevpulseT;

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

void reach_ph(double target_pH) {
  int current_bits = analogRead(PH_PROBE);

  // Straight line constants.
  double m = 3.0456;
  double c = 3.939;

  int target_bits = (((target_ph - c) / m) / 3.3) * 1023;
  int tolerance = (((ph_tolerance - c) / m) / 3.3) * 1023;

  int bit_error = abs((int)(current_bits - target_bits));

  // Ramp up the speed instead of setting to max straight away (so gears don't shear).
  long currtime = micros();
  long delta_time = currtime - last_change;
  if (delta_time >= 500000) {ph_pwm_value = ph_pwm_value + 20; last_change = currtime;} // If 0.5s have passed since last speed bump, increase speed.
  ph_pwm_value = constrain(ph_pwm_value, 0, 180); // Cap at 180 for safety.

  if (bit_error < tolerance) {
    // Turn off both of the pumps.
    digitalWrite(ALKALI_PUMP, LOW);
    digitalWrite(ACID_PUMP, LOW);
  }
  else if (current_bits < target_bits) {
    // Turn on the alkali pump.
    digitalWrite(ACID_PUMP, LOW);
    analogWrite(ALKALI_PUMP, ph_pwm_value);
  }
  else if (current_bits > target_bits) {
    // Turn on the acid pump.
    analogWrite(ACID_PUMP, ph_pwm_value);
    digitalWrite(ALKALI_PUMP, LOW);
  }
}

void requestEvent() {
  // ESP32 is requesting current data.

  // Wait before sending the current values to the ESP32.
  // This ensure that the wire buffer is emptied before we request.
  delay(10);

  byte data[size];
  packData(data, current_temp, current_ph, current_rpm);

  // Send the data to the ESP32.
  Wire.write(data, size);
}

void receiveEvent(int bits) {
  // Received new target values from ThingsBoard.
  byte data[size];
  int i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  unpackData(data, &target_temp, &target_ph, &target_rpm);

  // Log the data
  Serial.print("Received payload: "); Serial.println("{"); Serial.print("  \"Temperature\": "); Serial.print(target_temp); Serial.println(","); Serial.print("  \"pH\": "); Serial.print(target_ph); Serial.println(","); Serial.print("  \"RPM\": "); Serial.println(target_rpm); Serial.println("}");
}


void freqcount() {
  pulseT = micros();
  if(pulseT-prevpulseT>6000){
    // Mitigate sensor false triggers (due to PWM current spikes).
    freq = 1e6/float(pulseT-prevprevpulseT);
  }

  // Calculate frequency
  prevprevpulseT = prevpulseT;
  prevpulseT = pulseT;
}
void setup() {
  // Connectivity subsystem.
  Wire.begin(ESP32);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  // Temperature subsystem.
  pinMode(THERMISTOR, INPUT);
  pinMode(HEATING_ELEMENT, OUTPUT);
  
  // pH subsystem.
  pinMode(PH_PROBE, INPUT);
  pinMode(ALKALI_PUMP, OUTPUT);
  pinMode(ACID_PUMP, OUTPUT);
  digitalWrite(ALKALI_PUMP, LOW);
  digitalWrite(ACID_PUMP, LOW);

  // Stirrer subsystem.
  pinMode(INTERRUPT, INPUT);
  pinMode(MOTOR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT), freqcount, CHANGE); // IR sensor will interrupt program when 1/2 revolutions detected.
  analogWriteResolution(10);
  analogWriteFrequency(8000);
  analogWrite(MOTOR, 0);

 // Data logging.
 Serial.begin(9600);
 Serial.println("Setup complete.");
}

void loop() {
  currtime = micros();
  deltaT = (currtime-prevtime)*1e-6;
  prevtime = currtime;

  // Heating subsystem
  int bits = analogRead(THERMISTOR);

  // Straight line constants
  double m = 0.10053;
  int c = 27;
  current_temp = (m * (double)bits) + (double)c;
  t.error = target_temp-current_temp;

  t.interror = t.output + (t.error * deltaT);
  t.output = round((t.Kp * t.interror) + (t.Ki * t.interror));
  t.output = constrain(t.output, 0, 180);
  analogWrite(HEATING_ELEMENT, t.output);

  // pH subsystem.
  if (prev_target_ph != target_ph) {
    prev_target_ph = target_ph;
    ph_pwm_value = 100;
    last_change = micros();
  }
  reach_ph(target_ph);

  // Stirring subsystem.
  int measspeed = freq * (60 / 2);
  s.error = target_rpm - measspeed;
  s.interror = s.interror + (s.Ki * s.error * deltaT);
  s.interror = constrain(s.interror, 0, 3);
  s.output = round(204*((s.Kp * s.error) + s.interror));
  s.output = constrain(s.output, 0, 180);
  analogWrite(MOTOR, s.output);
  current_rpm = (0.1 * measspeed) + (0.9 * current_rpm);
}