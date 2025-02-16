#include <stdio.h>
#include <cstdint>
#include "Bilstein.h"
#include "ftdi_enumerator.h"
#include "library_info.h"

#define FTDI_VID 0x0403
#define FTDI_PID 0xfa20

int main(int argc, char** argv)
{
  int status = 0;
  const IceblockVersion ver = getIceblockVersion();

  printf("bilsteinread using libiceblock v%d.%d.%d\n",
    ver.major, ver.minor, ver.patch);

  const uint8_t ecuAddr = 0x00;
  Bilstein bilstein(9600, LineType::KLine, true);

  std::vector<FtdiDeviceInfo> devices = enumerateFtdiDevices();
  printf("Found %d device(s).\n", devices.size());

  for (int i = 0; i < devices.size(); i++)
  {
    printf("%03d:%03d - %s %s\n",
      devices[i].busNumber,
      devices[i].deviceAddress,
      devices[i].manufacturer.c_str(),
      devices[i].description.c_str());
  }

  printf("Attempting connection via FTDI %04x/%04x to ECU addr %02x...\n",
    FTDI_VID, FTDI_PID, ecuAddr);

  if (bilstein.connectByDeviceId(FTDI_VID, FTDI_PID, ecuAddr))
  {
    printf("Connected successfully.\n");

    std::vector<uint8_t> data;
    const uint16_t address = 0x0056;
    const uint8_t bytecount = 1;

    if (bilstein.readMemory(MemoryType::RAM, address, bytecount, data) &&
        (data.size() > 0))
    {
      printf("Read a byte from %04X : %02X\n", address, data.at(0));
    }
    else
    {
      printf("Read failed.\n");
      status = -2;
    }

    bilstein.disconnect();
  }
  else
  {
    printf("connect() failed.\n");
    status = -1;
  }

  return status;
}

