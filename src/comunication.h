#ifndef H_COMUNICATION
#define H_COMUNICATION

#define FIELD_SEPARATOR ','
#define COMMAND_SEPARATOR ';'

enum {
  kCOMM_ERROR = 000,  // Lets Arduino report serial port comm error back to the
                      // PC (only works for some comm errors)
  kACK = 001,         // Arduino acknowledges cmd was received
  kARDUINO_READY = 002,  // After opening the comm port, send this cmd 02 from
                         // PC to check arduino is ready
  kERR = 003,  // Arduino reports badly formatted cmd, or cmd not recognised

  // Defined commands
  kRUN = 004,   // run the controller
  kIDLE = 005,  // stop controller
  kLCD_ON = 006,   // power up lcd 
  kLCD_OFF = 007,   // shutdown lcd

  kSET_SP = 012,  // 10, set target temperature
  kSET_KP = 013,  // 11, set Kp PID parameter
  kSET_KI = 014,  // 12, set Ki PID parameter
  kSET_KD = 015,  // 13, set Kd PID parameter
  kLOAD = 016,    // 14, load setting from memory
  kSAVE = 017,    // 15, save current settings to memory

  kATUNE = 020,   // 16, start autotune
  kATUNE_STOP = 021,   // 16, start autotune

  // Sending commands
  kUPDATE = 020,

  kSEND_CMDS_END,  // Mustnt delete this line
};

void SystemReady();
void SystemRun();
void SystemIdle();
void SystemStartAutoTune();
void SystemStopAutoTune();
void LcdOn();
void LcdOff();

void SystemSetSetpoint();
void SystemSetKp();
void SystemSetKd();
void SystemSetKi();

void SystemLoadParameters();
void SystemSaveParameters();

void UnknownCmd();

#endif  // H_COMUNICATION