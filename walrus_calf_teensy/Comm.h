#ifndef COMM_H
#define COMM_H


#define START_BYTE 0x5E

/*
COM PROTOCOL:

Incoming Packets:
  start-byte    length     command   data
|   0x5E     |  1 byte  |  1 byte  | remaining bytes  |

Outgoing Packets:
  start-byte    length     data
|   0x5E     |  1 byte  | remaining bytes  |

*length does not include the start-byte or length feilds

An exchange of packets must always be initated by the server
and will always be responded to by this client.

*/

#include "CommandReceiver.h"

class Comm
{
  private:
    CommandReceiver* receivers[100];
    int num_receivers;
  
  
  public:
    Comm();
    
    void RegisterCommandReceiver(CommandReceiver* module);
    void sustain();
};

#endif
