#ifndef KWP71_H
#define KWP71_H

#include <stdint.h>
#include <stdbool.h>

#if defined(WIN32)
#include <windows.h>
#else
#include <pthread.h>
#include <errno.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Major/minor/patch version numbers for this build of the library
 */
typedef struct
{
  //! Major version number
  uint8_t major;
  //! Minor version number
  uint8_t minor;
  //! Patch version number
  uint8_t patch;
} kwp71_version;

/**
 * Contains information about the state of the current connection to the ECU.
 */
typedef struct
{
  bool connected;
#if defined(WIN32)
  //! Descriptor for the serial port device
  HANDLE sd;
  //! Lock to prevent multiple simultaneous open/close/read/write operations
  HANDLE mutex;
#else
  //! Descriptor for the serial port device
  int sd;
  //! Lock to prevent multiple simultaneous open/close/read/write operations
  pthread_mutex_t mutex;
#endif
} kwp71_info;

void kwp71_init(kwp71_info* info);
void kwp71_cleanup(kwp71_info* info);
void kwp71_disconnect(kwp71_info* info);
bool kwp71_connect(kwp71_info* info, const char* dev, kwp71_protocol_variant variant, kwp71_ecuinfo* ecuinfo);

kwp71_version kwp71_getLibraryVersion();

bool kwp71_sendACK(kwp71_info* info);
bool kwp71_sendNAK(kwp71_info* info);

uint8_t* kwp71_requestID(kwp71_info* info);
uint8_t* kwp71_readRAM(kwp71_info* info, uint16_t addr, uint8_t numBytes);

/* Closing brace for 'extern "C"' */
#ifdef __cplusplus
}
#endif

#endif // KWP71_H

