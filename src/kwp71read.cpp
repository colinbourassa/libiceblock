#include <stdio.h>
#include <stdint.h>
#include "kwp71.h"

#define FTDI_VID 0x0403
#define FTDI_PID 0xfa20
#define ECU_ADDR 0x10
#define ECU_BAUD 4800

int main(int argc, char** argv)
{
  int status = 0;
  const Kwp71Version ver = Kwp71::getLibraryVersion();

  printf("kwp71read using libkwp71 v%d.%d.%d\n",
    ver.major, ver.minor, ver.patch);

  Kwp71 kwp;
  std::vector<FtdiDeviceInfo> devices = kwp.enumerateFtdiDevices();
  printf("Found %d device(s).\n", devices.size());

  for (int i = 0; i < devices.size(); i++)
  {
    printf("%03d:%03d - %s %s\n",
      devices[i].busNumber,
      devices[i].deviceAddress,
      devices[i].manufacturer.c_str(),
      devices[i].description.c_str());
  }

  /*
  int err = 0;
  printf("Attempting connection via FTDI %04x/%04x to ECU addr %02x with baud %d...\n",
    FTDI_VID, FTDI_PID, ECU_ADDR, ECU_BAUD);

  if (kwp.connect(FTDI_VID, FTDI_PID, ECU_ADDR, ECU_BAUD, err))
  {
    printf("Connected successfully.\n");
    std::vector<uint8_t> data;

    const uint8_t bytecount = 252;
    if (kwp.readROM(0x0000, bytecount, data))
    {
      printf("Successfully read %d bytes.\n", bytecount);
      for (int index = 0; index < bytecount; index++)
      {
        if (index && (index % 16 == 0))
        {
          printf("\n");
        }
        printf(" %02X", data[index]);
      }
      printf("\n");
    }
    else
    {
      printf("readRAM() failed.\n");
      status = -2;
    }

    kwp.disconnect();
  }
  else
  {
    printf("connect() failed.\n");
    status = -1;
  }
  */

  return status;
}

