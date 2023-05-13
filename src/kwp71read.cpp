#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "kwp71.h"

int main(int argc, char** argv)
{
  int status = 0;
  const Kwp71Version ver = Kwp71::getLibraryVersion();

  std::cout << "kwp71read using libkwp71 v" <<
    (int)ver.major << "." << (int)ver.minor << "." << (int)ver.patch << std::endl;

  Kwp71 kwp;
  if (kwp.connect(0x0403, 0xfa20, 0x10, 4800))
  {
    for (uint8_t paramIndex = 0; paramIndex < 0x5; paramIndex++)
    {
      // Trying to determine the correct format of the ReadParamData command.
      // So far, simply sending a single byte payload isn't producing any result
      // other than a NACK from the ECU...
      Kwp71Command cmd;
      cmd.type = Kwp71PacketType::ReadParamData;
      cmd.payload = std::vector<uint8_t>({ paramIndex });
      std::vector<uint8_t> response;
      if (kwp.sendCommand(cmd, response))
      {
        printf("\nGot valid response for payload %02X (%d byte response payload).\n", paramIndex, response.size());
      }
      else
      {
        //printf("\nFailed to get valid response.\n");
      }
    }
    kwp.disconnect();
  }
  else
  {
    printf("connect() failed\n");
  }

  return status;
}

