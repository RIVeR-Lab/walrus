uint8 SET_ENABLE = 0            # no accompanying data
uint8 SET_DISABLE = 1           # no accompanying data
uint8 ERROR = 2                 # msg = Error message
uint8 KEEP_ALIVE = 3            # no accompanying data
uint8 SET_CAM_LED = 4           # value = intensity (0-255)
uint8 STATUS = 11               # value = DiagnosticStatus level; msg = Message
uint8 REQ_STATUS = 12           # no accompanying data

uint8 type
uint8 value
string msg
