## Engineering Challenges (ENGF0001): Bioreactor
#### Project Description
Model the control system for a Bioreactor consisting of:
- Temperature subsystem
- pH subsystem
- Stirring subsystem
- Connectivity and Data Logging subsystem

The system must connect to the IoT (Internet of Things) for data logging and to set the parameters of the system (temperature, pH and RPM).

#### Requirements
- The temperature subsystem must be able to maintain the temperature at a set point in the range 25-35&deg;C to within &plusmn;0.5&deg;C of the set point.
- The stirring subsystem must be able to maintain the stirring speed at a set point in the range 500-1500 RPM within &plusmn;20 RPM of the set point.
- The pH subsystem must be able to maintain the pH value at a set point in the sensing range 3-7.
- A user interface must be provided for the connectivity and data logging subsystem.

#### Equipment
**Temperature subsystem:**
-   10kΩ NTC thermistor (ND06P00103K).
-   A 3Ω, 30W heater element, that can be immersed.  

**pH subsystem:**
- A pH probe (PH-100ATC Probe).
- Two 6V, 1A peristaltic pumps.

**Stirring subsystem:**
- CP2S700HCP photo-interrupter.
- A 3V DC motor (maximum current 1A).

**Connectivity and Data Logging Subsystem:**
- STM32F401RE Nucleo-64.
- ESP32-WROVER-E.