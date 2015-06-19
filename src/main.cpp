//-------------------------------------------------------------------
//
// Sous Vide Controller
//
//------------------------------------------------------------------

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

#include "main.h"
#include "comunication.h"

// ************************************************
// PID Variables and constants
// ************************************************

// Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

volatile long onTime = 0;

// EEPROM addresses for persisted data
const int kSpAddress = 0;
const int kKpAddress = 8;
const int kKiAddress = 16;
const int kKdAddress = 24;

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int kWindowSize = 10000;
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember = 2;

double kATuneStep = 500;
double kATuneNoise = 1;
unsigned int kATuneLookBack = 20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

LiquidCrystal_I2C lcd(0x27, 20, 4);

byte degree[8] =  // define the degree symbol
    {B00110, B01001, B01001, B00110, B00000, B00000, B00000, B00000};

const int logInterval = 10000;  // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, RUN, AUTOTUNE };
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just
// Maxim/Dallas temperature ICs) and pass our oneWire reference to Dallas
// Temperature.
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensor;  // arrays to hold device address

// Software Serial interface
SoftwareSerial sSerial(SOFTWARE_SERIAL_RX, SOFTWARE_SERIAL_TX);

// Command Messenger for command interface
CmdMessenger cmdMessenger =
    CmdMessenger(sSerial, FIELD_SEPARATOR, COMMAND_SEPARATOR);

void ConfigureCmdMessenger() {
  cmdMessenger.printLfCr();
  cmdMessenger.attach(kARDUINO_READY, SystemReady);
  cmdMessenger.attach(kRUN, SystemRun);
  cmdMessenger.attach(kOFF, SystemOff);
  cmdMessenger.attach(kSET_KP, SystemSetKp);
  cmdMessenger.attach(kSET_KD, SystemSetKd);
  cmdMessenger.attach(kSET_KI, SystemSetKi);
  cmdMessenger.attach(kSET_SP, SystemSetSetpoint);
  cmdMessenger.attach(UnknownCmd);
}

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup() {
  // Setup serial interfaces
  Serial.begin(9600);
  sSerial.begin(115200);

  // Initialize Relay Control:
  pinMode(RELAY_PIN, OUTPUT);    // Output mode to drive relay
  digitalWrite(RELAY_PIN, LOW);  // make sure it is off to start

  // Set up Ground & Power for the sensor from GPIO pins
  pinMode(ONE_WIRE_GND, OUTPUT);
  digitalWrite(ONE_WIRE_GND, LOW);
  pinMode(ONE_WIRE_PWR, OUTPUT);
  digitalWrite(ONE_WIRE_PWR, HIGH);

  // Configure CmdMessenger callbacks
  ConfigureCmdMessenger();

  // Initialize LCD DiSplay
  lcd.begin();
  lcd.createChar(0, degree);  // create degree symbol from the binary
  lcd.backlight();

  lcd.clear();
  lcd.print(F("    Qantic"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Sous Vide!"));

  // Start up the DS18B20 One Wire Temperature Sensor
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0)) {
    lcd.setCursor(0, 1);
    lcd.print(F("Sensor Error"));
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);
  Input = sensors.getTempC(tempSensor);

  delay(3000);  // Splash screen
  lcd.clear();

  // Initialize the PID and related variables
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, kWindowSize);

  // Run timer2 interrupt every 15 ms
  TCCR2A = 0;
  TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

  // Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1 << TOIE2;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) {
  if (opState == OFF) {
    digitalWrite(RELAY_PIN, LOW);  // make sure relay is off
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
  // read sent commands
  cmdMessenger.feedinSerialData();

  // Read the input:
  if (sensors.isConversionAvailable(0)) {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures();  // prime the pump for the next one - but
                                    // don't wait
  }
  // state
  lcd.setCursor(0, 0);
  switch (opState) {
    case RUN:
      lcd.print("> Running! ");
      break;
    case OFF:
      lcd.print("> IDLE     ");
      break;
  }
  // Temp
  lcd.setCursor(0, 2);
  lcd.print("T: ");
  lcd.print(Input);
  lcd.print(" ");
  lcd.write(0);
  lcd.print(F("C"));
  // Setpoint
  lcd.setCursor(13, 2);
  lcd.print("[" + String(Setpoint) + "]");

  float pct = map(Output, 0, kWindowSize, 0, 1000);
  lcd.setCursor(0, 3);
  lcd.print("P: ");
  lcd.print(pct / 10);
  // lcd.print(Output);
  lcd.print("%");

  // periodically log to serial port in csv format
  if (millis() - lastLogTime > logInterval) {
    cmdMessenger.sendCmdStart(kUPDATE);
    cmdMessenger.sendCmdArg(Input);
    cmdMessenger.sendCmdArg(Output);
    cmdMessenger.sendCmdArg(Setpoint);
    cmdMessenger.sendCmdArg(Kp);
    cmdMessenger.sendCmdArg(Kd);
    cmdMessenger.sendCmdArg(Ki);
    cmdMessenger.sendCmdEnd();
    lastLogTime = millis();
  }

  if (opState == RUN) {
    DoControl();
  }

  delay(100);
}

// ************************************************
// Commands callbacks
// ************************************************
void SystemReady() {
  // In response to ping. We just send a throw-away Acknowledgement to say "im
  // alive"
  cmdMessenger.sendCmd(kACK, "System ready");
}

void UnknownCmd() {
  // Default response for unknown commands and corrupt messages
  cmdMessenger.sendCmd(kERR, "Unknown command");
}

void SystemSetSetpoint() {
  float _sp = cmdMessenger.readFloatArg();
  if (_sp >= 0) {
    Setpoint = _sp;
    cmdMessenger.sendCmd(kACK, "Target temperature set to " + String(Setpoint) + "° C");
  }
}

void SystemSetKp() {
  float _kp = cmdMessenger.readFloatArg();
  if (_kp >= 0) {
    Kp = _kp;
    myPID.SetTunings(Kp, Ki, Kd);
    cmdMessenger.sendCmd(kACK, "Kp parameter set to " + String(Kp));
  }
}

void SystemSetKd() {
  float _kd = cmdMessenger.readFloatArg();
  if (_kd >= 0) {
    Kd = _kd;
    myPID.SetTunings(Kp, Ki, Kd);
    cmdMessenger.sendCmd(kACK, "Kd parameter set to " + String(Kd));
  }
}

void SystemSetKi() {
  float _ki = cmdMessenger.readFloatArg();
  if (_ki >= 0) {
    Ki = _ki;
    myPID.SetTunings(Kp, Ki, Kd);
    cmdMessenger.sendCmd(kACK, "Ki parameter set to " + String(Ki));
  }
}

void SystemRun() {
  myPID.SetMode(AUTOMATIC);
  windowStartTime = millis();
  opState = RUN;
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void SystemOff() {
  myPID.SetMode(MANUAL);
  lcd.noBacklight();
  digitalWrite(RELAY_PIN, LOW);  // make sure it is off
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl() {
  if (tuning) {  // run the auto-tuner
    if (aTune.Runtime()) {  // returns 'true' when done
      FinishAutoTune();
    }
  } else {  // Execute control algorithm
    myPID.Compute();
  }

  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output;
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput() {
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > kWindowSize) {  // time to shift the Relay Window
    windowStartTime += kWindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime))) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune() {
  // REmember the mode we were in
  ATuneModeRemember = myPID.GetMode();

  // set up the auto-tune parameters
  aTune.SetNoiseBand(kATuneNoise);
  aTune.SetOutputStep(kATuneStep);
  aTune.SetLookbackSec((int)kATuneLookBack);
  tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune() {
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
void SaveParameters() {
  if (Setpoint != EEPROM_readDouble(kSpAddress)) {
    EEPROM_writeDouble(kSpAddress, Setpoint);
  }
  if (Kp != EEPROM_readDouble(kKpAddress)) {
    EEPROM_writeDouble(kKpAddress, Kp);
  }
  if (Ki != EEPROM_readDouble(kKiAddress)) {
    EEPROM_writeDouble(kKiAddress, Ki);
  }
  if (Kd != EEPROM_readDouble(kKdAddress)) {
    EEPROM_writeDouble(kKdAddress, Kd);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters() {
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(kSpAddress);
  Kp = EEPROM_readDouble(kKpAddress);
  Ki = EEPROM_readDouble(kKiAddress);
  Kd = EEPROM_readDouble(kKdAddress);

  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint)) {
    Setpoint = 60;
  }
  if (isnan(Kp)) {
    Kp = 850;
  }
  if (isnan(Ki)) {
    Ki = 0.5;
  }
  if (isnan(Kd)) {
    Kd = 0.1;
  }
}

// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address) {
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    *p++ = EEPROM.read(address++);
  }
  return value;
}
