void get_temp();
void system_run();

void Off();

void DoControl();
void DriveOutput();

void StartAutoTune();
void FinishAutoTune();

void SaveParameters();
void LoadParameters();

void EEPROM_writeDouble(int address, double value);
double EEPROM_readDouble(int address);