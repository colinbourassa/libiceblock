// libkwp71 - a communications library for KP-71/KWP-71 capable ECUs
//
// protocol.c: This file contains routines specific to handling
//             the software protocol used by the ECU over its
//             serial link.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#if defined(WIN32)
#include <windows.h>
#endif

#include "kwp71.h"
#include "kwp71_internal.h"

void printBuffer(const uint8_t* buf, uint32_t len)
{
  uint32_t index = 0;
  while (index < len)
  {
    printf(" %02X", buf[index]);
    index++;
    if (index % 16 == 0)
    {
      printf("\n");
    }
  }
}

/**
 * Reads bytes from the serial device using an OS-specific call.
 * @param buffer Buffer into which data should be read
 * @param quantity Number of bytes to read
 * @return Number of bytes read from the device, or -1 if no bytes could be read
 */
int16_t kwp71_readSerial(kwp71_info* info, uint8_t* buffer, uint16_t quantity)
{
  int16_t bytesRead = -1;

  if (info->connected)
  {
#if defined(WIN32)
    DWORD w32BytesRead = 0;

    if ((ReadFile(info->sd, (UCHAR*) buffer, quantity, &w32BytesRead, NULL) == TRUE) &&
        (w32BytesRead > 0))
    {
      bytesRead = w32BytesRead;
    }
#else
    bytesRead = read(info->sd, buffer, quantity);
#endif
    printf("Read:\n");
    printBuffer(buffer, quantity);
  }
  else
  {
    dprintf_warn("kwp71(warning): Not connected.\n");
  }

  return bytesRead;
}

/**
 * Writes bytes to the serial device using an OS-specific call
 * @param buffer Buffer from which written data should be drawn
 * @param quantity Number of bytes to write
 * @return Number of bytes written to the device, or -1 if no bytes could be written
 */
int16_t kwp71_writeSerial(kwp71_info* info, const uint8_t* const buffer, uint16_t quantity)
{
  int16_t bytesWritten = -1;

  if (info->connected)
  {
    printf("Write:\n");
    printBuffer(buffer, quantity);
#if defined(WIN32)
    DWORD w32BytesWritten = 0;

    if ((WriteFile(info->sd, (UCHAR*) buffer, quantity, &w32BytesWritten, NULL) == TRUE) &&
        (w32BytesWritten == quantity))
    {
      bytesWritten = w32BytesWritten;
    }
#else
    bytesWritten = write(info->sd, buffer, quantity);
#endif
  }

  return bytesWritten;
}

