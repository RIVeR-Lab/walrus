uint8 SET_ENABLE = 0            # no accompanying data
uint8 SET_DISABLE = 1           # no accompanying data
uint8 ERROR = 2                 # msg = Error message
uint8 KEEP_ALIVE = 3            # no accompanying data
uint8 SET_LED = 4               # index = LED number - 1; value = intensity (0-255)
uint8 POWER_OFF = 5             # no accompanying data
uint8 POWER_ON = 6              # no accompanying data
uint8 REQ_BATT_INFO = 7         # index = Battery numbber - 1   
uint8 BATT_MFR = 8              # index = Battery numbber - 1; msg = Battery manufacturer 
uint8 BATT_NAME = 9             # index = Battery numbber - 1;  msg = Batter device name
uint8 BATT_CHEM = 10            # index = Battery numbber - 1;  msg = battry chemistry
uint8 STATUS = 11               # value = DiagnosticStatus level; msg = Message
uint8 REQ_STATUS = 12           # no accompanying data

uint8 type
uint8 index
int16 value
string msg
