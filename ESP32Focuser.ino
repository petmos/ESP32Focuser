#include <Arduino.h>
#include <SPIFFS.h>
#include <AccelStepper.h>
//#include <ESP32Encoder.h>
#include <ArduinoJson.h>

//#include "LM335.h"
#include "src/Moonlite/Moonlite.h"

// Our configuration structure.
//
// Never use a JsonDocument to store the configuration!
// A JsonDocument is *not* a permanent storage; it's only a temporary storage
// used during the serialization phase. See:
// https://arduinojson.org/v6/faq/why-must-i-create-a-separate-config-object/
struct Config {
  long lastSavedPosition;
};

const char *filename = "/config.dat";  // <- SD library uses 8.3 filenames
//Config config;                         // <- global configuration object

/* You only need to format SPIFFS the first time */
#define FORMAT_SPIFFS_IF_FAILED true


const int directionPin = 32;
const int stepPin      = 33;
const int sleepPin     = 25;
const int resetPin     = 26;
const int stepMode3    = 27;
const int stepMode2    = 14;
const int stepMode1    = 12;
const int enablePin    = 13;

const int encoderPin1  = 2;
const int encoderPin2  = 15;

#define RXD2 16
#define TXD2 17

const int encoderMotorstepsRelation = 5;

//const int temperatureSensorPin = 3;

unsigned long timestamp;
unsigned long displayTimestamp;

//LM335 TemperatureSensor(temperatureSensorPin);
Moonlite SerialProtocol;
//ESP32Encoder encoder;

// multiplier of SPEEDMUX, currently max speed is 480.
int speedFactor = 16;
int speedFactorRaw = 4;
int speedMult = 30;

long currentPosition = 0;
long targetPosition = 0;
long lastSavedPosition;

long millisLastMove = 0;
const long millisDisableDelay = 15000;

int temperatureCompensationCoefficient = 0;

bool isRunning = false;
bool isEnabled = false;

// initialize the stepper library
const int motorPin1    = 27;  // IN1 on ULN2003 ==> Blue   on 28BYJ-48
const int motorPin2    = 25;  // IN2 on ULN2003 ==> Pink   on 28BYJ-48
const int motorPin3    = 32;  // IN3 on ULN2003 ==> Yellow on 28BYJ-48
const int motorPin4    = 12;  // IN4 on ULN2003 ==> Orange on 28BYJ-48

// NOTE: The sequence 1-3-2-4 is required for proper sequencing of 28BYJ-48
AccelStepper stepper(4, motorPin1, motorPin3, motorPin2, motorPin4);

float temp = 0;
long pos = 0;
bool pageIsRefreshing = false;

hw_timer_t * timer = NULL;

void processCommand()
{
  switch (SerialProtocol.getCommand().commandID)
  {
    case ML_C:
      // Initiate temperature convertion
      // Not implemented
      break;
    case ML_FG:
      // Goto target position
      stepper.enableOutputs();
      isEnabled = true;
      stepper.moveTo(targetPosition);
      break;
    case ML_FQ:
      // Motor stop movement
      stepper.stop();
      break;
    case ML_GB:
      // Set the Red Led backligth value
      // Dump value necessary to run the official moonlite software
      SerialProtocol.setAnswer(2, 0x00);
      break;
    case ML_GC:
      // Return the temperature coefficient
      SerialProtocol.setAnswer(2, (long)temperatureCompensationCoefficient);
      break;
    case ML_GD:
      // Return the current motor speed
      SerialProtocol.setAnswer(2, (long)speedFactorRaw);
      break;
    case ML_GH:
      // Return the current stepping mode (half or full step)
      // whether half-step is enabled or not, always return "00"
      SerialProtocol.setAnswer(2, (long)0x00);
      break;
    case ML_GI:
      // get if the motor is moving or not
//      SerialProtocol.setAnswer(2, (long)(Motor.isInMove() ? 0x01 : 0x00));
      SerialProtocol.setAnswer(2, (long)(abs(targetPosition - currentPosition) > 0 ? 0x01 : 0x00));
      break;
    case ML_GN:
      // Get the target position
      SerialProtocol.setAnswer(4, targetPosition / 4);  // Factor 4 for ULN2003
      break;
    case ML_GP:
      // Return the current position
      currentPosition = stepper.currentPosition();
      SerialProtocol.setAnswer(4, currentPosition / 4); // Factor 4 for ULN2003
      break;
    case ML_GT:
      // Return the temperature
      //SerialProtocol.setAnswer(4, (long)((TemperatureSensor.getTemperature() * 2)));
      SerialProtocol.setAnswer(4, (long)(20 * 2));
      break;
    case ML_GV:
      // Get the version of the firmware
      SerialProtocol.setAnswer(2, (long)(0x01));
      break;
    case ML_SC:
      // Set the temperature coefficient
      temperatureCompensationCoefficient = SerialProtocol.getCommand().parameter;
      break;
    case ML_SD:
      // Set the motor speed
/*      switch (SerialProtocol.getCommand().parameter)
      {
        case 0x02:
          Motor.setSpeed(7000);
          speedFactor = 32 / speedFactorRaw;
          stepper.setMaxSpeed(speedFactor * speedMult);
          break;
        case 0x04:
          Motor.setSpeed(5000);
          break;
        case 0x08:
          Motor.setSpeed(3000);
          break;
        case 0x10:
          Motor.setSpeed(1000);
          break;
        case 0x20:
          Motor.setSpeed(500);
          break;
        default:
          break;
      }*/
      speedFactorRaw = SerialProtocol.getCommand().parameter;
      // SpeedFactor: smaller value means faster
      speedFactor = 32 / speedFactorRaw;
      stepper.setMaxSpeed(speedFactor * speedMult);
      break;
    case ML_SF:
      // Set the stepping mode to full step
/*      Motor.setStepMode(SC_16TH_STEP);
      if (Motor.getSpeed() >= 6000)
      {
        Motor.setSpeed(6000);
      }*/
      break;
    case ML_SH:
      // Set the stepping mode to half step
//      Motor.setStepMode(SC_32TH_STEP);
      break;
    case ML_SN:
      // Set the target position
//      encoder.setCount(SerialProtocol.getCommand().parameter * encoderMotorstepsRelation);
      targetPosition = 4 * SerialProtocol.getCommand().parameter; // Factor 4 for ULN2003
      break;
    case ML_SP:
      // Set the current motor position
//      encoder.setCount(SerialProtocol.getCommand().parameter * encoderMotorstepsRelation);
      currentPosition = 4 * SerialProtocol.getCommand().parameter; // Factor 4 for ULN2003
      stepper.setCurrentPosition(currentPosition);
      break;
    case ML_PLUS:
      // Activate temperature compensation focusing
//      Motor.enableTemperatureCompensation();
      break;
    case ML_MINUS:
      // Disable temperature compensation focusing
//      Motor.disableTemperatureCompensation();
      break;
    case ML_PO:
      // Temperature calibration
      //TemperatureSensor.setCompensationValue(SerialProtocol.getCommand().parameter / 2.0);
      break;
    default:
      break;
  }
}

/*
void SetupEncoder()
{
  delay(1);
  // Enable the weak pull down resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;
  // set starting count value
	encoder.clearCount();
  // Attach pins for use as encoder pins
	encoder.attachSingleEdge(encoderPin1, encoderPin2);
}
*/

void loadConfiguration(const char *filename) {
  // Open file for reading
  File file = SPIFFS.open(filename);
  if (!file) {
    //Serial2.println(F("Failed to read file"));
    return;
  }

  DynamicJsonDocument doc(512);
  // Deserialize the JSON document
  DeserializationError err = deserializeJson(doc, file);
  if (err) {
    //Serial2.print(F("deserializeJson() failed: "));
    //Serial2.println(err.c_str());
  }

  lastSavedPosition = doc["lastSavedPosition"] | 60000;

  // Close the file
  file.close();
}

// Write content to a json file
void saveConfiguration(const char *filename) {
  // Open file for writing
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    //Serial2.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  // You can use DynamicJsonDocument as well
  DynamicJsonDocument doc(512);

  // Set the values in the document
  doc["lastSavedPosition"] = lastSavedPosition;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    //Serial2.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  SerialProtocol.init(9600);
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  // Serial2.println("Begin debugging");

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    return;
  }

  loadConfiguration(filename);
  currentPosition = lastSavedPosition;
  targetPosition = currentPosition;

  // Set the motor speed to a valid value for Moonlite
  stepper.setMaxSpeed(speedFactor * speedMult);
  stepper.setAcceleration(100);

  stepper.setCurrentPosition(currentPosition);

  timestamp = millis();
  //displayTimestamp = millis();
  millisLastMove = millis();

//  SetupEncoder();
}


void HandleHandController()
{
/*  long targetPosition = Motor.getTargetPosition();
  long encoderPosition = encoder.getCount() / encoderMotorstepsRelation;
//  Motor.setTargetPosition(encoderPosition);
  if(targetPosition != encoderPosition)
  {
//    Motor.goToTargetPosition();
    stepper.moveTo(targetPosition);
  }
  if (!Motor.isInMove())
  {
//    Motor.goToTargetPosition();
    stepper.moveTo(targetPosition);
  }
  while(Motor.isInMove())
  {
    Motor.Manage();
  }*/
}


void loop()
{
  // Move the motor one step
  stepper.run();


  if (millis() - timestamp > 2)
  {
    stepper.run();
    timestamp = millis();
  }

/*  if (!Motor.isInMove())
  {
    //TemperatureSensor.Manage();
    if (Motor.isTemperatureCompensationEnabled() && ((millis() - timestamp) > 30000))
    {
     // Motor.setCurrentTemperature(TemperatureSensor.getTemperature());
      Motor.setCurrentTemperature(20);
      Motor.compensateTemperature();
      timestamp = millis();
    }
  }*/

//  HandleHandController();


  SerialProtocol.Manage();

  if (SerialProtocol.isNewCommandAvailable())
  {
    processCommand();
  }

  if (stepper.distanceToGo() != 0)
  {
    isRunning = true;
    millisLastMove = millis();
    currentPosition = stepper.currentPosition();
  }

  // check if motor was'nt moved for several seconds and save position and disable motors
  if (millis() - millisLastMove > millisDisableDelay)
  {
    isRunning = false;
    // Save current location in EEPROM
    if (lastSavedPosition != currentPosition)
    {
      lastSavedPosition = currentPosition;
      saveConfiguration(filename);
    }
    if(isEnabled){
      // set motor to sleep state
      stepper.disableOutputs();
      isEnabled = false;
    }
  }
}
