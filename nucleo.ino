#include <Wire.h>

#define SLAVE_ADDR 9
#define PH_PROBE A0
#define THERMISTOR A1
#define ALKALI_PUMP 11 // black
#define ACID_PUMP 10 // red

int target_temp = 30, target_ph = 5, target_rpm = 400; // Initial values. TODO: Change to double.
int current_temp = 0, current_ph = 0, current_rpm = 0; // Change to double.

// Initial values for the stirring subsystem.
const float Kv=800;
const float T=0.25;

const float wn=4;
const float zeta=1;

// Calculate the required PI controller parameters
const float wo=1/T;
// corresponding motor response frequency in rad/s
const float Kp=(2*zeta*wn/wo-1)/Kv;
const float KI=wn*wn/Kv/wo;
//And finally we need to declare the remaining constants and variables:
const byte encoderpin=2, motorpin=6;
long currtime, prevtime, pulseT, prevpulseT, prevprevpulseT, T1, T2;
float measspeed, meanmeasspeed, freq, error, KIinterror, deltaT;
bool ctrl;
int Vmotor, onoff;

void reach_ph(double target_pH) {
  int current_bits = analogRead(PH_PROBE);
  Serial.print("current_bits: "); Serial.println(current_bits);

  // Straight line constants.
  double m = 3.0456;
  double c = 3.939;

  int target_bits = (((target_ph - c) / m) / 3.3) * 1023;
  Serial.print("target bits: "); Serial.println(target_bits);
  int tolerance = (((ph_tolerance - c) / m) / 3.3) * 1023;

  int bit_error = abs((int)(current_bits - target_bits));

  int pwm_value = 120;

  if (bit_error < tolerance) {
    // Turn off both of the pumps.
    digitalWrite(ALKALI_PUMP, LOW);
    digitalWrite(ACID_PUMP, LOW);
    Serial.println("turning both off.");
  }
  else if (current_bits < target_bits) {
    // Turn the alkali pump.
    digitalWrite(ACID_PUMP, LOW);
    analogWrite(ALKALI_PUMP, pwm_value);
    Serial.println("Turn on alkali pump.");
  }
  else if (current_bits > target_bits) {
    // Turn on the acid pump.
    analogWrite(ACID_PUMP, pwm_value);
    digitalWrite(ALKALI_PUMP, LOW);
    Serial.println("Turn on acid pump");
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

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  delay(10);
  // Serial.println("Request event called.");
  Serial.print("Nucleo data: "); Serial.print(temp); Serial.print(" "); Serial.print(pH); Serial.print(" "); Serial.println(RPM);
  Wire.write(current_temp);
  Wire.write(current_ph);
  Wire.write(current_rpm);
}

void receiveEvent(int bits) {
  while (Wire.available()) {
    target_temp = Wire.read();
    target_ph = Wire.read();
    target_rpm = Wire.read();
    }
    Serial.print("ESP data: "); Serial.print(target_temp); Serial.print(" "); Serial.print(target_ph); Serial.print(" "); Serial.println(target_rpm);
    // ! Add reach functions.
    reach_ph(target_ph);
}

void setup() {
  // Connectivity subsystem
  Wire.begin(SLAVE_ADDR); // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent); // register event
  
  // pH subsystem.
  pinMode(ALKALI_PUMP, OUTPUT);
  pinMode(ACID_PUMP, OUTPUT);
  digitalWrite(ALKALI_PUMP, LOW);
  digitalWrite(ACID_PUMP, LOW);

  // Stirrer subsystem.
  pinMode(encoderpin, INPUT);
  pinMode(motorpin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderpin), freqcount, CHANGE);
  analogWriteResolution(10); // 10-bit PWM -TCCR1A = 0b00000011; for Uno/Nano
  analogWriteFrequency(8000); //8 kHz PWM -TCCR1B = 0b00000001; for Uno/Nano
  analogWrite(motorpin, 0);

 // Data logging.
 Serial.begin(9600);
 Serial.println("Setup complete.");
}

void loop() {
  currtime = micros();
  deltaT = (currtime-prevtime)*1e-6;
  if(ctrl==1 && currtime-T2>1000000) {ctrl=0;}
  if (ctrl==0 && digitalRead(A2)==0) {onoff++; ctrl=1; T2=currtime; target_rpm=400;} // ! Based off ESP input1!!!
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
    analogWrite(motorpin, Vmotor);
    meanmeasspeed = 0.1*measspeed+0.9*meanmeasspeed;
    current_rpm = meanmeasspeed;
  }
}