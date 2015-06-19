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
  kSTOP = 005,  // stop controller
  kOFF = 006,   // shutdown all components

  kSET_SP = 012,  // 10, set target temperature
  kSET_KP = 013,  // 11, set Kp PID parameter
  kSET_KD = 014,  // 12, set Kd PID parameter
  kSET_KI = 015,  // 13, set Ki PID parameter
  kLOAD = 016,    // 14, load setting from memory
  kSAVE = 017,    // 15, save current settings to memory

  kUPDATE = 020,

  kSEND_CMDS_END,  // Mustnt delete this line
};

void SystemRun();
void SystemOff();
void SystemSetSetpoint();
void SystemSetKp();
void SystemSetKd();
void SystemSetKi();
void SystemReady();
void UnknownCmd();

#endif  // H_COMUNICATION