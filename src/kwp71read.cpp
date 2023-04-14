#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "kwp71.h"

int main(int argc, char** argv)
{
  int status = 0;
  const Kwp71Version ver = Kwp71::getLibraryVersion();
  uint8_t addr = 0x10;

  std::cout << "kwp71read using libkwp71 v" <<
    (int)ver.major << "." << (int)ver.minor << "." << (int)ver.patch << std::endl;

  Kwp71 kwp;
  std::cout << "Calling connect(" << (int)addr << ")..." << std::endl;
  if (kwp.connect(0x0403, 0xfa20, addr))
  {
    Kwp71Command cmd;
    cmd.type = Kwp71PacketType::ReadParamData;
    std::vector<uint8_t> response;
    printf("Requesting command injection...\n");
    if (kwp.sendCommand(cmd, response))
    {
      printf("Got %u bytes!\n", response.size());
      for (int i = 0; i < response.size(); i++)
      {
        std::cout << " " << response[i];
        if (i && (i % 16 == 0))
        {
          std::cout << std::endl;
        }
      }
    }
    else
    {
      printf("Failed to get response.\n");
    }
    kwp.disconnect();
  }
  else
  {
    printf("connect() failed\n");
  }

  return status;
}

