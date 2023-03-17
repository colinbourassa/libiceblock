#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include "kwp71.h"

void usage(const Kwp71Version ver, const char* name)
{
  std::cout << "kwp71read using libkwp71 v" <<
    ver.major << "." << ver.minor << "." << ver.patch << std::endl <<
    "Usage: " << name << " <serial device> <ecu address>" << std::endl;
}

int main(int argc, char** argv)
{
  int status = 0;
  const Kwp71Version ver = Kwp71::getLibraryVersion();
  uint8_t addr;

  if (argc < 3)
  {
    usage(ver, argv[0]);
    return 0;
  }

  addr = strtoul(argv[2], NULL, 0);
  Kwp71 kwp;
  std::cout << "Calling connect(" << argv[1] << ", " << addr << ")..." << std::endl;
  if (kwp.connect(std::string(argv[1]), addr))
  {
    std::vector<std::string> idResponse;
    if (kwp.requestIDInfo(idResponse))
    {
      std::cout << "--- ID info ---" << std::endl;
      for (int i = 0; i < idResponse.size(); i++)
      {
        std::cout << idResponse[i] << std::endl;
      }
      std::cout << "---------------" << std::endl;
    }
    else
    {
      std::cout << "requestIDInfo() failed" << std::endl;
    }
  }
  else
  {
    std::cout << "connect() failed" << std::endl;
  }
  std::cout << "Calling shutdown()..." << std::endl;
  kwp.shutdown();

  return status;
}

