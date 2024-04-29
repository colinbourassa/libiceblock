#include <stdio.h>
#include <cstdint>
#include "Fiat9141.h"
#include "library_info.h"

#define FTDI_VID 0x0403
#define FTDI_PID 0xfa20

int main(int argc, char** argv)
{
  int status = 0;
  const IceblockVersion ver = getIceblockVersion();

  printf("fiat9141read using libiceblock v%d.%d.%d\n",
    ver.major, ver.minor, ver.patch);

  // This is for TRW airbag ECU (p/n 60631206, 46538798, and 60615633)
  const uint8_t ecuAddr = 0x80;
  Fiat9141 fiat(4800, LineType::KLine, true);

  std::vector<FtdiDeviceInfo> devices = fiat.enumerateFtdiDevices();
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

  if (fiat.connectByDeviceId(FTDI_VID, FTDI_PID, ecuAddr))
  {
    printf("Connected successfully.\n");
    std::vector<uint8_t> data;

    if (fiat.readFaultCodes(data))
    {
      printf("Read %d bytes.\n", data.size());
      for (int index = 0; index < data.size(); index++)
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
      printf("Read failed.\n");
      status = -2;
    }

    fiat.disconnect();
  }
  else
  {
    printf("connect() failed.\n");
    status = -1;
  }

  return status;
}

