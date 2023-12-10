#include <Wire.h>

// Address for I2C communication.
#define SLAVE_ADDR 9

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

// Size for I2C byte array.
const int size = sizeof(double)*2 + sizeof(int);

// Required values for the temp subsystem.
const float t_Ki = 0.351, t_Kp = 100; // assumes PIoutpin (0-1023 PWM) scales to 0 - 1023 W
const float Tcal = 0.1; // assumes 20.5 K/V temp sensor (Tcal = 20.5*5/1023)
long t_currtime, t_prevtime;
float t_deltaT, Te, TeInt;
int Pheater;
int control = 1;

// Required values for the pH subsystem.
const double ph_tolerance = 0.5;

// Required values for the stirring subsystem.
const float Kv=800;
const float T=0.25;

const float wn=4;
const float zeta=1;

const float wo=1/T;
// corresponding motor response frequency in rad/s
const float Kp=(2*zeta*wn/wo-1)/Kv;
const float KI=wn*wn/Kv/wo;
//And finally we need to declare the remaining constants and variables:
long currtime, prevtime, pulseT, prevpulseT, prevprevpulseT, T1, T2;
float measspeed, meanmeasspeed, freq, error, KIinterror, deltaT;
bool ctrl;
int Vmotor, onoff;

void reach_ph(double target_pH) {
  int current_bits = analogRead(PH_PROBE);

  // Straight line constants.
  double m = 3.0456;
  double c = 3.939;

  int target_bits = (((target_ph - c) / m) / 3.3) * 1023;
  int tolerance = (((ph_tolerance - c) / m) / 3.3) * 1023;

  int bit_error = abs((int)(current_bits - target_bits));

  int pwm_value = 120; // Constant PWM so we don't kill the motors.

  if (bit_error < tolerance) {
    // Turn off both of the pumps.
    digitalWrite(ALKALI_PUMP, LOW);
    digitalWrite(ACID_PUMP, LOW);
  }
  else if (current_bits < target_bits) {
    // Turn the alkali pump.
    digitalWrite(ACID_PUMP, LOW);
    analogWrite(ALKALI_PUMP, pwm_value);
  }
  else if (current_bits > target_bits) {
    // Turn on the acid pump.
    analogWrite(ACID_PUMP, pwm_value);
    digitalWrite(ALKALI_PUMP, LOW);
  }
}

void freqcount() {
  pulseT= micros();
  if(pulseT-prevpulseT>6000){
    // Attempt to mitigate sensor false triggers due to PWM current spikes, apparent speeds > 2500 RPM
    freq= 1e6/float(pulseT-prevprevpulseT);
  }
  // Calculate speed sensor frequency
  prevprevpulseT= prevpulseT;
  prevpulseT= pulseT;
}

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

void requestEvent() {
  // ESP32 is requesting current data.
  delay(10);
  byte data[size];
  packData(data, current_temp, current_ph, current_rpm);
  // Send the data to the ESP32.
  Wire.write(data, size);
}

void receiveEvent(int bits) {
  // ESP32 has sent over the new target values from ThingsBoard.
  byte data[size];
  int i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  unpackData(data, &target_temp, &target_ph, &target_rpm);
  Serial.print("Received payload: "); Serial.print(target_temp); Serial.print(" "); Serial.print(target_ph); Serial.print(" "); Serial.println(target_rpm);
}

void setup() {
  // Connectivity subsystem.
  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestEvent); // register event so whenever a request is received, the requestEvent function is called.
  Wire.onReceive(receiveEvent);

  // Temperature subsystem.
  pinMode(THERMISTOR, INPUT);
  pinMode(HEATING_ELEMENT, OUTPUT);
  
  // pH subsystem.
  // pinMode(PH_PROBE, INPUT);
  pinMode(ALKALI_PUMP, OUTPUT);
  pinMode(ACID_PUMP, OUTPUT);
  digitalWrite(ALKALI_PUMP, LOW);
  digitalWrite(ACID_PUMP, LOW);

  // Stirrer subsystem.
  pinMode(INTERRUPT, INPUT);
  pinMode(MOTOR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT), freqcount, CHANGE);
  analogWriteResolution(10); // 10-bit PWM -TCCR1A = 0b00000011; for Uno/Nano
  analogWriteFrequency(8000); //8 kHz PWM -TCCR1B = 0b00000001; for Uno/Nano
  analogWrite(MOTOR, 0);

 // Data logging.
 Serial.begin(9600);
 Serial.println("Setup complete.");
}

void loop() {
  // Heating subsystem
  t_currtime = micros();
  t_deltaT = (t_currtime-t_prevtime)*1e-6;
  t_prevtime = t_currtime;
  int bits = analogRead(THERMISTOR);
  current_temp = -0.9545*bits+72.66;
  Te = target_temp-current_temp; // Temperature error.

  TeInt = TeInt + Te*t_deltaT*control;
  Pheater = round(t_Kp*Te+t_Ki*TeInt);
  Pheater = constrain(Pheater,0,100);
  analogWrite(HEATING_ELEMENT, Pheater);

  // pH subsystem.
  reach_ph(target_ph);

  // Stirring subsystem.
  /*
  if(ctrl==1 && currtime-T2>1000000) {ctrl=0;}
  if (ctrl==0 && digitalRead(A2)==0) {onoff++; ctrl=1; T2=currtime; target_rpm=400;} // ! Based off ESP input1!!!
  */
  if (currtime-T1 > 0) {
    prevtime = currtime;
    T1 = T1+10000;
    measspeed = freq*30;
    if (currtime-pulseT>5e5) {measspeed=0; meanmeasspeed=0;}
    error = target_rpm-measspeed;
    KIinterror = KIinterror+KI*error*deltaT;
    KIinterror = constrain(KIinterror,0,3);
    Vmotor = round(204*(Kp*error+KIinterror));
    Vmotor = constrain(Vmotor, 0, 150);
    analogWrite(MOTOR, Vmotor);
    meanmeasspeed = 0.1*measspeed+0.9*meanmeasspeed;
    current_rpm = meanmeasspeed;
  }
}