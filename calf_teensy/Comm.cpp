#include "Comm.h"

byte blockRead()
{
  while (Serial.available() <= 0);
  return Serial.read();
}

Comm::Comm()
{
	num_receivers = 0;
	Serial.begin(115200);
}


void Comm::RegisterCommandReceiver(CommandReceiver* module)
{
	receivers[num_receivers] = module;
        num_receivers++;
}   

void Comm::sustain()
{
	byte len;
	byte cmd;
	byte data[255];
	byte checksum;
	byte total;
	byte response[255];
	byte response_len;
	while (Serial.available() > 0)
	{
		if (Serial.read() == START_BYTE)
		{
                        total = START_BYTE;
			len = blockRead();
			total += len;
			cmd = blockRead();
			total += cmd;
			for (int l = 0; l < len-2; l++)
			{
				data[l] = blockRead();
				total += data[l];
			}
			checksum = blockRead();
                        total += checksum;
			if (total == 0xFF)
			{
				for (int l = 0; l < num_receivers; l++)
				{
					if (receivers[l]->processCommand(cmd, data, len-2, response, &response_len))
					{
                                                response[0] = 0x5E;
                                                response[1] = response_len + 1;
                                                total = 0;
                                                for (int l = 0; l < response_len + 2; l++)
                                                  total += response[l];
                                                response[response_len+2] = (0xFF-total); 
                                                Serial.write(response, response_len+3);
						break;
					}
				}
			}
		}
	}
}
