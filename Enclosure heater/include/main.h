/* 
    Header file for printer enclosure heater project
*/

/**
 * Thermal Protection provides additional protection to your printer from damage
 * and fire. Marlin always includes safe min and max temperature ranges which
 * protect against a broken or disconnected thermistor wire.
 *
 * The issue: If a thermistor falls out, it will report the much lower
 * temperature of the air in the room, and the the firmware will keep
 * the heater on.
 *
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too
 * long (period), the firmware will halt the machine as a safety precaution.
 *
 * If you get false positives for "Thermal Runaway", increase
 * THERMAL_PROTECTION_HYSTERESIS and/or THERMAL_PROTECTION_PERIOD
 */
#define THERMAL_PROTECTION_PERIOD     40000 // Millieconds
#define THERMAL_PROTECTION_HYSTERESIS 4     // Degrees Celsius
//#define TEMP_RISING_RATE              3     // Degrees Celsius difference within "thermal protection period"
//#define TEMP_FALLING_RATE             1     // Degrees Celsius difference within "thermal protection period"

// Thermal parameters
#define TEMP_AIR_HYSTERESIS      3  // (°C) Temperature proximity considered "close enough" to the target
#define TEMP_BED_HYSTERESIS      2  // (°C) Temperature proximity considered "close enough" to the target

// Below this temperature the heater will be switched off
// because it probably indicates a broken thermistor wire.
#define BED_MINTEMP    5
#define AIR_MINTEMP    5

// Above this temperature the heater will be switched off.
// This can protect components from overheating, but NOT from shorts and failures.
// (Use MINTEMP for thermistor short/failure protection.)
#define BED_MAXTEMP  150
#define AIR_MAXTEMP  60

// Default starting temperature values and limits
//
#define BED_SP_DEFAULT    60
#define BED_SP_MIN        25
#define BED_SP_MAX        130

#define AIR_SP_DEFAULT    30
#define AIR_SP_MIN        20
#define AIR_SP_MAX        50


//===========================================================================
//====================== PID > Bed Temperature Control ======================
//===========================================================================

/**
 * PID Bed Heating
 *
 * The PID frequency will be the same as the extruder PWM.
 * If PID_dT is the default, and correct for the hardware/configuration, that means 7.689Hz,
 * which is fine for driving a square wave into a resistive load and does not significantly
 * impact FET heating. This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W
 * heater. If your configuration is significantly different than this and you don't understand
 * the issues involved, don't use bed PID until someone else verifies that your hardware works.
 */

#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current.
                          // Reduce this if the bed heats up too quickly
#define MIN_BED_POWER 0

// Too slow??
//#define BED_KP 1.00
//#define BED_KI 0.07
//#define BED_KD 0.50

// Tuning
#define BED_KP 200.0 // Close now!
#define BED_KI 10.0  // Close now!
#define BED_KD 0.00  // Needs work??

// Original values
// #define BED_KP 10.00
// #define BED_KI 0.23
// #define BED_KD 305.4

//===========================================================================
//==================== PID > Air Temperature Control ====================
//===========================================================================

/*   #define MAX_AIR_POWER 255 // limits duty cycle to air heater; 255=full current
  #define MIN_AIR_POWER 0

  // Lasko "MyHeat Personal Heater" (200w) modified with a Fotek SSR-10DA to control only the heating element
  // and placed inside the small Creality printer enclosure tent.
  //
  #define AIR_KP 37.04
  #define AIR_KI 1.40
  #define AIR_KD 655.17

  // Window for conversion of fast PWM to relay-style very long PWM
  // Time in milliseconds
  const unsigned int WindowSize = 5000; */


  // Settings for thermistors
  //
  #define SENSOR_PIN_BED             A1
  #define REFERENCE_RESISTANCE_BED   4700
  #define NOMINAL_RESISTANCE_BED     100000
//  #define NOMINAL_RESISTANCE_BED     10000
  #define NOMINAL_TEMPERATURE_BED    25
  #define B_VALUE_BED                3950

  #define SENSOR_PIN_AIR             A2
  #define REFERENCE_RESISTANCE_AIR   4700
  #define NOMINAL_RESISTANCE_AIR     10000
  #define NOMINAL_TEMPERATURE_AIR    25
  #define B_VALUE_AIR                3950
  
  //  How many readings are taken to determine a mean temperature.
  //  The more values, the longer a calibration is performed,
  //  but the readings will be more accurate.
  #define READINGS_NUMBER 10

  //  Delay time between a temperature readings
  //  from the temperature sensor (ms).
  #define DELAY_TIME 10
    

  // Settings for spin timers
  // Times in milliseconds
  //
  const unsigned long LCD_UPDATE_MILLIS =        1000;
  const unsigned long EXTEND_FAN_RUNTIME =       120000;


  // Main digital oputputs
  const unsigned int fanPin = 2;  // Heated bed fan output pin
  const unsigned int bedPin = 3;  // Heated bed power control output pin
