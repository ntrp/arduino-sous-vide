// ************************************************
// Pin definitions
// ************************************************

// Output Relayasd
#define RELAY_PIN 10

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 4

#define SOFTWARE_SERIAL_RX 12
#define SOFTWARE_SERIAL_TX 13

// Function definitions
void DoControl();
void DriveOutput();

void StartAutoTune();
void FinishAutoTune();

void SaveParameters();
void LoadParameters();

void EEPROM_writeDouble(int address, double value);
double EEPROM_readDouble(int address);
