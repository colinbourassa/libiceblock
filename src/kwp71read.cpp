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
  if (kwp.connect(addr))
  {
    char ch;
    do {
      if (scanf("%c", &ch) == 1)
        printf("\nkeypress: %c\n", ch);
    } while (ch != 'q');
  }
  else
  {
    std::cout << "connect() failed" << std::endl;
  }
  std::cout << "Calling shutdown()..." << std::endl;
  kwp.shutdown();

  return status;
}

