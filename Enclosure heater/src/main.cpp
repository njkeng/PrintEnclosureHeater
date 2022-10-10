/* 
    Temperature controller for 3d printer air temperature

    Control equipment needed:
    Arduino UNO
    16 x 2 LCD keypad shield DF Robot or clone
    Heater controller
    Heated bed
    Heaated bed temperature thermistor
    Air temperature thermistor

    Heater hardware:
    3D printer heated bed
    Fan to disperse the heat
    LCD controller and temperature display

    Short acting PID control of heated bed temperature

    Long acting PID control of air air temperature

*/

#include <main.h>
#include <Arduino.h>
#include <EEPROM.h>


#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <AverageThermistor.h>

// Setup thermistors for the air temperature and the heated bed temperature
// Air temperature thermistor pin   = A1
// Bed temperature thermistor pin   = A2
Thermistor* thermistor_air;
Thermistor* thermistor_bed;


// LcdKeypad, https://github.com/dniklaus/arduino-display-lcdkeypad
// Pins used by LCD keypad DFRobot
// Data 4 pin             = D4
// Data 5 pin             = D5
// Data 6 pin             = D6
// Data 7 pin             = D7
// Reset Pin              = D8
// Enable pin             = D9
// Backlight control pin  = D10
// ADC pin for keys       = A0

#include <LcdKeypad.h>

LcdKeypad* myLcdKeypad = 0;

// Define general system variables
bool heaterRunning = true;
bool heaterNeeded = false;
bool tempReached = false;


// Define variables for LCD menu system
bool updateLCD = false;
bool upKey = false;
bool downKey = false;
bool leftKey = false;
bool rightKey = false;
const unsigned int numberLCDpages = 3;
bool settingMode = false;
unsigned int settingValue = 0;
unsigned int settingMax = 0;
unsigned int settingMin = 0;
bool settingOn = true;
enum {
  HEATER_PAGE   = 0,
  AIR_PAGE      = 1,
  BED_PAGE      = 2
} LCDPage;
bool airThermistorError = false;
bool bedThermistorError = false;
bool bedWatchdogError = false;
bool checkBedTemp = false;

// Function to step forward through the pages in the LCD menu system
void forwardPage()
{
  if (LCDPage == HEATER_PAGE) LCDPage=AIR_PAGE;  
  else if (LCDPage == AIR_PAGE) LCDPage = BED_PAGE;
}
// Function to step back through the pages in the LCD menu system
void backPage()
{
  if (LCDPage == BED_PAGE) LCDPage=AIR_PAGE;  
  else if (LCDPage == AIR_PAGE) LCDPage = HEATER_PAGE;
}


// Implement specific LcdKeypadAdapter in order to allow receiving key press events
class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
private:
  LcdKeypad* m_lcdKeypad;
  unsigned char m_value;
public:
  MyLcdKeypadAdapter(LcdKeypad* lcdKeypad)
  : m_lcdKeypad(lcdKeypad)
  , m_value(7)
  { }

  // Specific handleKeyChanged() method implementation
  void handleKeyChanged(LcdKeypad::Key newKey)
  {
    if (0 != m_lcdKeypad)
    {
      if (LcdKeypad::UP_KEY == newKey)
      {
        upKey = true;
      }
      else if (LcdKeypad::DOWN_KEY == newKey)
      {
        downKey = true;
      }
      else if (LcdKeypad::LEFT_KEY == newKey)
      {
        leftKey = true;
      }
      else if (LcdKeypad::RIGHT_KEY == newKey)
      {
        rightKey = true;
      }
    }
  }
};


#include <PID_v1.h>

//Define PID Variables we'll be connecting to
// Variables for control of the heating element temperature
double BedTempSP, BedTempPV, BedTempOP;
double BedMainSP;
double BedOldPV = BED_MINTEMP;

// PID controller definition for heating element temperature
PID BedPID(&BedTempPV, &BedTempOP, &BedTempSP, BED_KP, BED_KI, BED_KD, P_ON_M, DIRECT);

// Variables for control of the air temperature
double AirTempSP, AirTempPV, AirTempOP; 


// SpinTimer library, https://github.com/dniklaus/spin-timer
#include <SpinTimer.h>

// Define spin timers
// Recurring timer for LCD update
class LCDUpdate : public SpinTimerAction
{
public:
  void timeExpired()
  {
    updateLCD = true;
  }
};
SpinTimer lcdUpdate(LCD_UPDATE_MILLIS, new LCDUpdate(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);

// One-shot timer for extending the operation of the heated bed fan
SpinTimer FanDelayOff(EXTEND_FAN_RUNTIME, 0, SpinTimer::IS_NON_RECURRING, SpinTimer::IS_NON_AUTOSTART);

// Recurring timer for heated bed watchdog
class ThermalProtectionTimer : public SpinTimerAction
{
public:
  void timeExpired()
  {
    checkBedTemp = true;
  }
};
SpinTimer thermalProtectionTimer(THERMAL_PROTECTION_PERIOD, new ThermalProtectionTimer(), SpinTimer::IS_RECURRING);


void setup() {
  // put your setup code here, to run once:

  // main output pins
  pinMode(bedPin, OUTPUT);
  pinMode(fanPin, OUTPUT);

  myLcdKeypad = new LcdKeypad();  // instantiate an object of the LcdKeypad class, using default parameters
  
  // Attach the specific LcdKeypadAdapter implementation (dependency injection)
  myLcdKeypad->attachAdapter(new MyLcdKeypadAdapter(myLcdKeypad));
  myLcdKeypad->setBacklight(LcdKeypad::LCDBL_WHITE); // Turn on LCD backlight white

  // Setup thermistors for the air temperature and the heated bed temperature
  thermistor_air = new AverageThermistor(
    new NTC_Thermistor(
    SENSOR_PIN_AIR,
    REFERENCE_RESISTANCE_AIR,
    NOMINAL_RESISTANCE_AIR,
    NOMINAL_TEMPERATURE_AIR,
    B_VALUE_AIR
    ),
    READINGS_NUMBER,
    DELAY_TIME
   );

  thermistor_bed = new NTC_Thermistor(
    SENSOR_PIN_BED,
    REFERENCE_RESISTANCE_BED,
    NOMINAL_RESISTANCE_BED,
    NOMINAL_TEMPERATURE_BED,
    B_VALUE_BED
  );

  // Read initial thermistor temperatures
  AirTempPV = thermistor_air->readCelsius();
  BedTempPV = thermistor_bed->readCelsius();


  // Retrieve previous setpoints from EEPROM
  // Air temperature SP
  EEPROM.get(0, settingValue);  
  if (settingValue < AIR_SP_MIN or settingValue > AIR_SP_MAX) { // Account for bad data or first initialisation
    settingValue = AIR_SP_DEFAULT;
  }
  AirTempSP = settingValue;
  EEPROM.put(0, settingValue);

  // Bed temperature SP
  EEPROM.get(2, settingValue);  
  if (settingValue < BED_SP_MIN or settingValue > BED_SP_MAX) { // Account for bad data or first initialisation
    settingValue = BED_SP_DEFAULT;
  }
  BedMainSP = settingValue;
  BedTempSP = BedMainSP;
  EEPROM.put(2, settingValue);


  // Start the heater.  
  // We assume if the heater unit gets powered up, then heating is required
  heaterRunning = true;

  // turn the PIDs on
  BedPID.SetMode(AUTOMATIC);

  // Start serial for debugging
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  // Run all of the defined timers in SpinTimer
  scheduleTimers();  // Get the timers ticked

  if (updateLCD) {
    // Actions to update LCD display
    //
    myLcdKeypad->clear();
    myLcdKeypad->noCursor();
    myLcdKeypad->noBlink();
    char s[16];  // Temporary storage of numerical values for display

    // Display error messages
    // Air temperature thermistor error
    if (airThermistorError) {
      myLcdKeypad->print("FAULT Air Temp.");
      myLcdKeypad->setCursor(0, 1);
      myLcdKeypad->print("Check Thermistor");
    } else 

    // Bed temperature thermistor error
    if (bedThermistorError) {
      myLcdKeypad->print("FAULT Bed Temp.");
      myLcdKeypad->setCursor(0, 1);
      myLcdKeypad->print("Check Thermistor");
    } else 

    // Bed heating watchdog error
    // Usually means that the bed thermistor has been mechanically
    // disconnected from the bed
    if (bedWatchdogError) {
      myLcdKeypad->print("FAULT Heating");
      myLcdKeypad->setCursor(0, 1);
      myLcdKeypad->print("Check heated bed");
    } else {

      switch (LCDPage) {
        // Heater on / off page
        case HEATER_PAGE:  // Update the first page of LCD display
          myLcdKeypad->print("Heater ");
          if (settingMode) {
            if (settingOn) {
              myLcdKeypad->print("Turn ON ");
              myLcdKeypad->print((char)127); // arrow left symbol
              myLcdKeypad->setCursor(7, 1);
              myLcdKeypad->print("Turn OFF");
            } else {
              myLcdKeypad->print("Turn ON");
              myLcdKeypad->setCursor(7, 1);
              myLcdKeypad->print("Turn OFF");
              myLcdKeypad->print((char)127); // arrow left symbol
            }
          } else {
            if (heaterRunning) {
              myLcdKeypad->print("RUNNING");
            } else {
              myLcdKeypad->print("OFF");
            }
            myLcdKeypad->setCursor(0, 1);
            myLcdKeypad->print("Air temp ");
            myLcdKeypad->print(dtostrf(AirTempPV, 3, 1, s));
            myLcdKeypad->print((char)223); // degree symbol
            myLcdKeypad->print("C");
          }
          break;

        // Air temperature page
        case AIR_PAGE:  // Update the second page of LCD display
          myLcdKeypad->print("Air temp ");
          myLcdKeypad->print(dtostrf(AirTempPV, 3, 1, s));
          myLcdKeypad->print((char)223); // degree symbol
          myLcdKeypad->print("C");

          myLcdKeypad->setCursor(0, 1);
          myLcdKeypad->print("Setpoint ");
          if (settingMode) {
            myLcdKeypad->print(settingValue);
            myLcdKeypad->cursor();
            myLcdKeypad->blink();
          } else {
            myLcdKeypad->print(dtostrf(AirTempSP, 3, 1, s));
            myLcdKeypad->print((char)223); // degree symbol
            myLcdKeypad->print("C");
          }
          break;

        // Bed temperature page
        case BED_PAGE:  // Update the third page of LCD display
          myLcdKeypad->print("Bed temp ");
          myLcdKeypad->print(dtostrf(BedTempPV, 3, 1, s));
          myLcdKeypad->print((char)223); // degree symbol
          myLcdKeypad->print("C");

          myLcdKeypad->setCursor(0, 1);
          myLcdKeypad->print("Setpoint ");
          if (settingMode) {
            myLcdKeypad->print(settingValue);
            myLcdKeypad->cursor();
            myLcdKeypad->blink();
          } else {
            myLcdKeypad->print(dtostrf(BedMainSP, 3, 1, s));
            myLcdKeypad->print((char)223); // degree symbol
            myLcdKeypad->print("C");
          }
          break;
      }
    }
    updateLCD = false;

    // Serial output of information
    Serial.print("Air SP: ");
    Serial.print(AirTempSP);
    Serial.print(" PV:");
    Serial.print(AirTempPV);
    Serial.print(" OP:");
    Serial.print(AirTempOP);

    Serial.print("      ");
    Serial.print("Bed SP: ");
    Serial.print(BedTempSP);
    Serial.print(" PV:");
    Serial.print(BedTempPV);
    Serial.print(" OP:");
    Serial.print(BedTempOP);
    Serial.print(" Mode:");
    Serial.print(BedPID.GetMode());
    
    Serial.print("      ");
    Serial.print("Run: ");
    Serial.print(heaterRunning);
    Serial.print(" TPT:");
    Serial.print(thermalProtectionTimer.isRunning());
    Serial.print(" FanT:");
    Serial.print(FanDelayOff.isRunning());

    Serial.println(" ");
    

  }

  // Only handle key presses if there are no faults
  if (!airThermistorError and !bedThermistorError and !bedWatchdogError) {

    // Actions if the UP key is pressed
    //
    if (upKey) {
      if (settingMode) {
        switch (LCDPage) {
          case HEATER_PAGE:
            settingOn = !settingOn;
            break;
          default:
            if (settingValue < settingMax) {
              settingValue++;
            }
            break;
        }

      } else {
        backPage();
      }
      updateLCD = true;
      upKey = false;
    }

    // Actions if the DOWN key is pressed
    //
    if (downKey) {
      if (settingMode) {
        switch (LCDPage) {
          case HEATER_PAGE:
            settingOn = !settingOn;
            break;
          default:
            if (settingValue > settingMin) {
              settingValue--;
            }
            break;
        }

      } else {
        forwardPage();
      }
      updateLCD = true;
      downKey = false;
    }


    // Actions if the LEFT key is pressed
    //
    if (leftKey) {
      settingMode = false;
      updateLCD = true;
      leftKey = false;
    }


    // Actions if the RIGHT key is pressed
    //
    if (rightKey) {
      if (settingMode) {
        // Copy temporary setpoint into the active variable
        // Save setpoint value in EEPROM
        switch (LCDPage)
        {
          case HEATER_PAGE: 
            heaterRunning = settingOn;
            break;
          case AIR_PAGE: 
            AirTempSP = settingValue;
            EEPROM.update(0,settingValue);
            break;
          case BED_PAGE: 
            BedMainSP = settingValue;
            BedTempSP = BedMainSP;
            EEPROM.update(2,settingValue);
            break;
        }
        settingMode = false;

      } else {

        // Copy active variable into the temporary setpoint
        switch (LCDPage)
        {
          case HEATER_PAGE: 
            settingOn = heaterRunning;
            break;
          case AIR_PAGE: 
            settingValue = AirTempSP;
            settingMax = AIR_MAXTEMP;
            settingMin = AIR_MINTEMP;
            break;
          case BED_PAGE: 
            settingValue = BedMainSP;
            settingMax = BED_MAXTEMP;
            settingMin = BED_MINTEMP;
            break;
        }
        settingMode = true;
      }
      updateLCD = true;
      rightKey = false;
    }

  }

  // Control the heated bed fan
  // Fan runs if the heater is ON or if the off delay timer is still running
  if (heaterRunning ) FanDelayOff.start();
  digitalWrite(fanPin, FanDelayOff.isRunning());


  // Control the heated bed
  // Read thermistor temperature values
  AirTempPV = thermistor_air->readCelsius();
  BedTempPV = thermistor_bed->readCelsius();

  // Check for bad thermistor values
  // Air temperature thermistor
  if (AirTempPV < AIR_MINTEMP) {
      airThermistorError = true;
      heaterRunning = false;
  } else {
      airThermistorError = false;
  }

  // Bed temperature thermistor
  if (BedTempPV < BED_MINTEMP) {
      bedThermistorError = true;
      heaterRunning = false;
  } else {
      bedThermistorError = false;
  }

  // Check for air temperature within hysteresis bounds
  // Air temp is increasing
  if (AirTempPV >= AirTempSP and !tempReached) {
    tempReached = true;
  }
  // Air temp is decreasing
  if ((AirTempPV < (AirTempSP - TEMP_AIR_HYSTERESIS)) and tempReached) {
    tempReached = false;
  }
  // Air temp is below SP
  if (AirTempPV < AirTempSP) {
    heaterNeeded = true;
  }

  // Control state of the bed PID
  if (heaterRunning and heaterNeeded and !tempReached) {
    // Compute the air temperature PID
    BedPID.SetMode(AUTOMATIC);
  } else { // If heater is not running
    BedPID.SetMode(MANUAL);
    BedTempOP = 0;
  }


  // Compute temperature PID loops
  BedPID.Compute();
  analogWrite(bedPin, BedTempOP);


  // Check for heater watchdog error
  // This is to safeguard against the bed thermistor being
  // mechanically disconnected from the bed
  // If the difference between the SP and the PV is within the
  // acceptable window, we assume everything is fine.
  if (BedPID.GetMode() != 0) {
    if (abs(BedTempPV - BedTempSP) > THERMAL_PROTECTION_HYSTERESIS) {
      // Run the thermal protection watchdog timer.  Do not restart if it is already running.
      if (!thermalProtectionTimer.isRunning()) {
        thermalProtectionTimer.start();
      }
    } 
  } else {
      thermalProtectionTimer.cancel();
  }

  // If the heated bed PV is a long way below the SP, check for increasing temperature
  // at least at the specified rate. 
  // Else if the heated bed PV is a long way above the SP, check for decreasing temperature
  // at least at the specified rate. 
  //
  if (checkBedTemp) {
    // Check if the temperature should be rising
    if (BedTempPV < BedTempSP) {
      // Check if the rate of change is too low
      if (BedTempPV - BedOldPV < TEMP_RISING_RATE) {
        bedWatchdogError = true;
        heaterRunning = false;
      }
    }

    // Check if the temperature should be falling
    if (BedTempPV > BedTempSP) {
      // Check if the rate of change is too low
      if (BedOldPV - BedTempPV < TEMP_FALLING_RATE) {
        bedWatchdogError = true;
        heaterRunning = false;
      }
    }

    // Store current PV for next check
    BedOldPV = BedTempPV;
    checkBedTemp = false;
  }

}