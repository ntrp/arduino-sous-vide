//-------------------------------------------------------------------
//
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries
// by Brett Beauregard
//------------------------------------------------------------------

#include "main.h"

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the Adafruit RGB/LCD Shield
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

#include <SoftwareSerial.h>

#include <CmdMessenger.h>
#include <Base64.h>
#include <Streaming.h>

// So we can save and retrieve settings
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 10

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 4

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// RX message options
#define RX_MAX_LENGTH 8
#define RX_TERMINATOR 13

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000;
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember = 2;

double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

LiquidCrystal_I2C lcd(0x27, 20, 4);

byte degree[8] = // define the degree symbol
{
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000
};

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, RUN, AUTOTUNE};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

SoftwareSerial sSerial(12, 13);

char field_separator = ',';
char command_separator = ';';

CmdMessenger cmdMessenger = CmdMessenger(sSerial, field_separator, command_separator);

enum
{
  kCOMM_ERROR    = 000, // Lets Arduino report serial port comm error back to the PC (only works for some comm errors)
  kACK           = 001, // Arduino acknowledges cmd was received
  kARDUINO_READY = 002, // After opening the comm port, send this cmd 02 from PC to check arduino is ready
  kERR           = 003, // Arduino reports badly formatted cmd, or cmd not recognised

  // Now we can define many more 'send' commands, coming from the arduino -> the PC, eg
  // kICE_CREAM_READY,
  // kICE_CREAM_PRICE,
  // For the above commands, we just call cmdMessenger.sendCmd() anywhere we want in our Arduino program.

  kGET_TEMP      = 004,
  kRUN           = 005,

  kSEND_CMDS_END, // Mustnt delete this line
};

void get_temp()
{
  // In response to ping. We just send a throw-away Acknowledgement to say "im alive"
  sensors.requestTemperatures();
  cmdMessenger.sendCmd(kACK, String(sensors.getTempC(tempSensor)));
}

void system_run()
{
  lcd.print(F("Sp: "));
  lcd.print(Setpoint);
  lcd.write(1);
  lcd.print(F("C : "));

  SaveParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  opState = RUN;
}

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
  Serial.begin(9600);
  sSerial.begin(115200);

  // Initialize Relay Control:

  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start

  // Set up Ground & Power for the sensor from GPIO pins

  pinMode(ONE_WIRE_GND, OUTPUT);
  digitalWrite(ONE_WIRE_GND, LOW);

  pinMode(ONE_WIRE_PWR, OUTPUT);
  digitalWrite(ONE_WIRE_PWR, HIGH);

  cmdMessenger.printLfCr();
  cmdMessenger.attach(kGET_TEMP, get_temp);
  cmdMessenger.attach(kRUN, system_run);

  // Initialize LCD DiSplay

  lcd.begin();
  lcd.createChar(1, degree); // create degree symbol from the binary
  lcd.backlight();

  lcd.clear();
  lcd.print(F("    Qantic"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Sous Vide!"));

  // Start up the DS18B20 One Wire Temperature Sensor

  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0))
  {
    lcd.setCursor(0, 1);
    lcd.print(F("Sensor Error"));
  }
  sensors.setResolution(tempSensor, 12);
  //sensors.setWaitForConversion(false);

  delay(3000);  // Splash screen

  // Initialize the PID and related variables
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);

  // Run timer2 interrupt every 15 ms
  TCCR2A = 0;
  TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1 << TOIE2;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) {

  if (opState == OFF) {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  } else {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop() {

  if (opState == RUN)
  {
    DoControl();

    lcd.setCursor(0, 1);
    lcd.print(Input);
    lcd.write(1);
    lcd.print(F("C : "));

    float pct = map(Output, 0, WindowSize, 0, 1000);
    lcd.setCursor(10, 1);
    lcd.print(F("      "));
    lcd.setCursor(10, 1);
    lcd.print(pct / 10);
    //lcd.print(Output);
    lcd.print("%");

    // periodically log to serial port in csv format
    if (millis() - lastLogTime > logInterval)
    {
      Serial.print(Input);
      Serial.print(",");
      Serial.println(Output);
    }
  }

  delay(100);
  cmdMessenger.feedinSerialData();
}

/*
void test() {

  char rxString[RX_MAX_LENGTH + 1];
  char* rxParameter;
  byte rxByteItr = 0;

  while (Serial.available() && rxString[rxByteItr] != 13 && ++rxByteItr < RX_MAX_LENGTH) {
    rxString[rxByteItr] = Serial.read();
  }

  if (rxByteItr > 0) {
    rxString[rxByteItr] = '\0';
    rxParameter = rxString + 1;
    Serial.println(rxString[0]);
    switch (rxString[0]) {
      case 'OFF':
        Off();
        opState = OFF;
        break;
      case 'SETP':
        Setpoint = atof(rxParameter);
        txLine("Setpoint set to '" + String(rxParameter) + "'");
        break;
      case 'RUN':
        lcd.print(F("Sp: "));
        lcd.print(Setpoint);
        lcd.write(1);
        lcd.print(F("C : "));

        SaveParameters();
        myPID.SetTunings(Kp, Ki, Kd);

        opState = RUN;
        break;
      case 'TUNE_P':
        Kp = atof(rxParameter);
        txLine("Kp set to '" + String(rxParameter) + "'");
        break;
      case 'TUNE_I':
        Ki = atof(rxParameter);
        txLine("Ki set to '" + String(rxParameter) + "'");
        break;
      case 'TUNE_D':
        Kd = atof(rxParameter);
        txLine("Kd set to '" + String(rxParameter) + "'");
        break;
      case 'SAVE_PID':
        SaveParameters();
        txLine("PID parameters persisted");
        break;
      default:
        txLine("Usage:");
        txLine("OFF - Stop the controller");
        txLine("RUN - Start the controller");
        txLine("SETP # - Set target temperature in C°");
        txLine("TUNE_P # - Set proportional PID parameter");
        txLine("TUNE_I # - Set integral PID parameter");
        txLine("TUNE_D # - Set derivative PID parameter");
        txLine("SAVE_PID - Persist PID configuration parameters");
    }
  }
}

void txLine(String str) {
  Serial.println(str);
}
*/

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
  myPID.SetMode(MANUAL);
  lcd.noBacklight();
  digitalWrite(RelayPin, LOW);  // make sure it is off
  lcd.print(F("    Qantic"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Sous Vide!"));
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }

  if (tuning) // run the auto-tuner
  {
    if (aTune.Runtime()) // returns 'true' when done
    {
      FinishAutoTune();
    }
  }
  else // Execute control algorithm
  {
    myPID.Compute();
  }

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output;
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime)))
  {
    digitalWrite(RelayPin, HIGH);
  }
  else
  {
    digitalWrite(RelayPin, LOW);
  }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
  // REmember the mode we were in
  ATuneModeRemember = myPID.GetMode();

  // set up the auto-tune parameters
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
  tuning = false;

  // Extract the auto-tune calculated parameters
  Kp = aTune.GetKp();
  Ki = aTune.GetKi();
  Kd = aTune.GetKd();

  // Re-tune the PID and revert to normal control mode
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(ATuneModeRemember);

  // Persist any changed parameters to EEPROM
  SaveParameters();
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  if (Setpoint != EEPROM_readDouble(SpAddress))
  {
    EEPROM_writeDouble(SpAddress, Setpoint);
  }
  if (Kp != EEPROM_readDouble(KpAddress))
  {
    EEPROM_writeDouble(KpAddress, Kp);
  }
  if (Ki != EEPROM_readDouble(KiAddress))
  {
    EEPROM_writeDouble(KiAddress, Ki);
  }
  if (Kd != EEPROM_readDouble(KdAddress))
  {
    EEPROM_writeDouble(KdAddress, Kd);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(SpAddress);
  Kp = EEPROM_readDouble(KpAddress);
  Ki = EEPROM_readDouble(KiAddress);
  Kd = EEPROM_readDouble(KdAddress);

  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint))
  {
    Setpoint = 60;
  }
  if (isnan(Kp))
  {
    Kp = 850;
  }
  if (isnan(Ki))
  {
    Ki = 0.5;
  }
  if (isnan(Kd))
  {
    Kd = 0.1;
  }
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}
