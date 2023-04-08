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
    char ch;
    do {
      if (scanf("%c", &ch) == 1)
      {
        printf("\nkeypress: %c\n", ch);
        if (ch == 'i')
        {
          std::vector<std::string> idResp;
          if (kwp.requestIDInfo(idResp))
          {
            printf("Got %u strings!\n", idResp.size());
            for (int i = 0; i < idResp.size(); i++)
            {
              std::cout << "    " << idResp[i] << std::endl;
            }
          }
          else
          {
            printf("Failed to get ID response.\n");
          }
        }
      }
    } while (ch != 'q');
  }
  else
  {
    std::cout << "connect() failed" << std::endl;
  }
  std::cout << "Calling shutdown()..." << std::endl;
  kwp.disconnect();

  return status;
}

