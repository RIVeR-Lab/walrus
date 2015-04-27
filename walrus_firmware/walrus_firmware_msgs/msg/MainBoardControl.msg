uint8 ERROR = 0                 # msg = Error message
uint8 KEEP_ALIVE = 2            # no accompanyingdata
uint8 SET_LED = 3               # index = LED number - 1; value = intensity (0-1000)
uint8 POWER_OFF = 4             # no accompanying data
uint8 POWER_ON = 5              # no accompanying data
uint8 REQ_BATT_INFO = 6         # index = Battery numbber - 1   
uint8 BATT_MFR = 7              # index = Battery numbber - 1;  msg = Battery manufacturer 
uint8 BATT_NAME = 8             # index = Battery numbber - 1;  msg = Batter device name
uint8 BATT_CHEM = 9             # index = Battery numbber - 1;  msg = battry chemistry

uint8 type
uint8 index
int16 value
string msg
