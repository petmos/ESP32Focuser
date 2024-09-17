#include <Arduino.h>
//#include "LM335.h"
#include "Moonlite/Moonlite.h"
#include "StepperControl/StepperControl.h"
#include <ESP32Encoder.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

//#include <U8x8lib.h>
//#include <U8g2lib.h>

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

/* Breadboard
const int stepPin = 0;
const int directionPin = 2;
const int stepMode1 = 18;
const int stepMode2 = 19;
const int stepMode3 = 25;
const int enablePin = 27;
const int sleepPin = 4;
const int resetPin = 32;
*/

/* Platine */
const int stepPin = 0;
const int directionPin = 2;
const int sleepPin = 4;
const int resetPin = 17;
const int stepMode1 = 18; // Dummy
const int stepMode2 = 19; // Dummy
const int stepMode3 = 27;
const int enablePin = 22;

const int Mode1 = 32;
const int Mode2 = 25;

/*
const int directionPin = 27;
const int stepPin      = 25;
const int sleepPin     = 32;
const int resetPin     = 12;
const int stepMode3    = 4;
const int stepMode2    = 17;
const int stepMode1    = 16;
const int enablePin    = 15;
*/
const int encoderPin1  = 6;
const int encoderPin2  = 7;
/*
const int encoderPin1  = 2;
const int encoderPin2  = 0;
*/
#define RXD2 10
#define TXD2 09

const int encoderMotorstepsRelation = 5;

// moonlite allows only 65565 steps (16bit)
const int multiplierMotorsteps = 5;

//const int temperatureSensorPin = 3;

unsigned long timestamp;
unsigned long displayTimestamp;

//LM335 TemperatureSensor(temperatureSensorPin);
StepperControl Motor(stepPin,
                           directionPin,
                           stepMode1,
                           stepMode2,
                           stepMode3,
                           enablePin,
                           sleepPin,
                           resetPin);
Moonlite SerialProtocol;
ESP32Encoder encoder;

// Declaration of the display
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C Display(U8G2_R0);

float temp = 0;
long pos = 0;
long currentPosition = 0;
long targetPosition = 0;
long lastSavedPosition;

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
      Motor.goToTargetPosition();
      break;
    case ML_FQ:
      // Motor stop movement
      Motor.stopMovement();
      break;
    case ML_GB:
      // Set the Red Led backligth value
      // Dump value necessary to run the official moonlite software
      SerialProtocol.setAnswer(2, 0x00);
      break;
    case ML_GC:
      // Return the temperature coefficient
      SerialProtocol.setAnswer(2, (long)Motor.getTemperatureCompensationCoefficient());
      break;
    case ML_GD:
      // Return the current motor speed
      switch (Motor.getSpeed())
      {
        case 500:
          SerialProtocol.setAnswer(2, (long)0x20);
          break;
        case 1000:
          SerialProtocol.setAnswer(2, (long)0x10);
          break;
        case 3000:
          SerialProtocol.setAnswer(2, (long)0x8);
          break;
        case 5000:
          SerialProtocol.setAnswer(2, (long)0x4);
          break;
        case 7000:
          SerialProtocol.setAnswer(2, (long)0x2);
          break;
        default:
          SerialProtocol.setAnswer(2, (long)0x20);
          break;
      }
      break;
    case ML_GH:
      // Return the current stepping mode (half or full step)
      SerialProtocol.setAnswer(2, (long)(Motor.getStepMode() == SC_32TH_STEP ? 0xFF : 0x00));
      break;
    case ML_GI:
      // get if the motor is moving or not
      SerialProtocol.setAnswer(2, (long)(Motor.isInMove() ? 0x01 : 0x00));
      break;
    case ML_GN:
      // Get the target position
      SerialProtocol.setAnswer(4, (long)(Motor.getTargetPosition() / multiplierMotorsteps));
      break;
    case ML_GP:
      // Return the current position
      SerialProtocol.setAnswer(4, (long)(Motor.getCurrentPosition() / multiplierMotorsteps));
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
      Motor.setTemperatureCompensationCoefficient(SerialProtocol.getCommand().parameter);
      break;
    case ML_SD:
      // Set the motor speed
      switch (SerialProtocol.getCommand().parameter)
      {
        case 0x02:
          Motor.setSpeed(7000);
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
      }
      break;
    case ML_SF:
      // Set the stepping mode to full step
      Motor.setStepMode(SC_16TH_STEP);
      if (Motor.getSpeed() >= 6000)
      {
        Motor.setSpeed(6000);
      }
      break;
    case ML_SH:
      // Set the stepping mode to half step
      Motor.setStepMode(SC_32TH_STEP);
      break;
    case ML_SN:
      // Set the target position
//      encoder.setCount(SerialProtocol.getCommand().parameter * encoderMotorstepsRelation);
      Motor.setTargetPosition(SerialProtocol.getCommand().parameter * multiplierMotorsteps);
      break;
    case ML_SP:
      // Set the current motor position
//      encoder.setCount(SerialProtocol.getCommand().parameter * encoderMotorstepsRelation);
      Motor.setCurrentPosition(SerialProtocol.getCommand().parameter * multiplierMotorsteps);
      break;
    case ML_PLUS:
      // Activate temperature compensation focusing
      Motor.enableTemperatureCompensation();
      break;
    case ML_MINUS:
      // Disable temperature compensation focusing
      Motor.disableTemperatureCompensation();
      break;
    case ML_PO:
      // Temperature calibration
      //TemperatureSensor.setCompensationValue(SerialProtocol.getCommand().parameter / 2.0);
      break;
    default:
      break;
  }
}

void SetupEncoder()
{
  delay(1);
  // Enable the weak pull down resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // set starting count value
	encoder.clearCount();
  // Attach pins for use as encoder pins
	encoder.attachSingleEdge(encoderPin1, encoderPin2);
}

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
  SerialProtocol.init(9600);
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  // Serial2.println("Begin debugging");

  //Display.begin();
  //Display.setContrast(0);
  //Display.setFont(u8g2_font_crox4hb_tr);

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    return;
  }

  loadConfiguration(filename);
  currentPosition = lastSavedPosition;

  pinMode(Mode1, OUTPUT);
  pinMode(Mode2, OUTPUT);
  digitalWrite(Mode1, HIGH);
  digitalWrite(Mode2, HIGH);
  

  // Set the motor speed to a valid value for Moonlite
  Motor.setSpeed(7000);
  Motor.setStepMode(SC_32TH_STEP);
  Motor.setMoveMode(SC_MOVEMODE_SMOOTH);
  Motor.setCurrentPosition(currentPosition);


  timestamp = millis();
  //displayTimestamp = millis();

//  SetupEncoder();
}

void HandleHandController()
{
  long targetPosition = Motor.getTargetPosition();
  long encoderPosition = encoder.getCount() / encoderMotorstepsRelation;
  Motor.setTargetPosition(encoderPosition);  
  if(targetPosition != encoderPosition)
  {
    Motor.goToTargetPosition();
  }
  if (!Motor.isInMove())
  {
    Motor.goToTargetPosition();
  }
  while(Motor.isInMove())
  {
    Motor.Manage();
  }
}

void loop()
{
  if (!Motor.isInMove())
  {
    //TemperatureSensor.Manage();
    if (Motor.isTemperatureCompensationEnabled() && ((millis() - timestamp) > 30000))
    {
     // Motor.setCurrentTemperature(TemperatureSensor.getTemperature());
      Motor.setCurrentTemperature(20);
      Motor.compensateTemperature();
      timestamp = millis();
    }
    // Save current location in EEPROM
    currentPosition = Motor.getCurrentPosition();
    if (lastSavedPosition != currentPosition)
    {
      lastSavedPosition = currentPosition;
      saveConfiguration(filename);
    }
  }


//  HandleHandController();

  Motor.Manage();
  SerialProtocol.Manage();

  if (SerialProtocol.isNewCommandAvailable())
  {
    processCommand();
  }



//  if ((millis() - displayTimestamp) >= 1000 && !Motor.isInMove())
//  {
//    Display.firstPage();
//    temp = TemperatureSensor.getTemperature();
//    pos = Motor.getCurrentPosition();
//    do
//    {
//      Display.drawStr(0, 16, ((String("T: ") + String(temp, 1) + " C").c_str()));
//      Display.drawStr(0, 55, (String("Pos: ") + pos).c_str());
//    } while (Display.nextPage());
//    displayTimestamp = millis();
//  }
}
