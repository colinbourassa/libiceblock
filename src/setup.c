// libkwp71 - a communications library for FIAT 9141 capable ECUs
//
// setup.c: This file contains routines that perform the
//          setup/initialization of the library and the
//          serial port.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <libftdi1/ftdi.h>

#if defined(WIN32)
#include <windows.h>
#else
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <arpa/inet.h>
#endif

#if defined(linux)
#include <linux/serial.h>
#endif

#include "kwp71.h"
#include "kwp71_internal.h"
#include "kwp71_version.h"
#include <stdio.h>

/**
 * Swaps multibyte big-endian data (from the ECU) into the local endianness.
 */
#if defined(WIN32)
uint16_t swapshort(const uint16_t source)
{
  return ((source & 0xff00) >> 8) | ((source & 0xff) << 8);
}
#else
uint16_t swapshort(const uint16_t source)
{
  return ntohs(source);
}
#endif

/**
 * Sleeps for the commanded duration (in milliseconds)
 */
int msleep (long msec)
{
  struct timespec ts;
  int res;

  if (msec < 0)
  {
    errno = EINVAL;
    return -1;
  }

  ts.tv_sec = msec / 1000;
  ts.tv_nsec = (msec % 1000) * 1000000;

  do {
    res = nanosleep(&ts, &ts);
  } while (res && errno == EINTR);

  return res;
}

/**
 * Performs the 'slow init' sequence by bit-banging the FTDI's transmit line to
 * clock out the provided address (with optional parity) at the requisite 5 baud.
 */
bool kwp71_5baudinit(uint8_t address, int databits, int parity)
{
  bool status = false;
  struct ftdi_context ftdic;
  unsigned char c;
  int f = 0;
  int bitindex = 0;
  int parityCount = 0;

  // return immediately if basic init fails
  if ((ftdi_init(&ftdic) != 0) ||
      (ftdi_set_interface(&ftdic, INTERFACE_A) != 0))
  {
    fprintf(stderr, "kwp71(error): ftdi_init() or ftdi_set_interface() failed\n");
    return false;
  }

  // Open FTDI device based on FT232R vendor & product IDs
  // TODO: open device associated with the passed-in /dev/ttyUSBx instead?
  if ((f = ftdi_usb_open(&ftdic, 0x0403, 0x6001)) == 0)
  {
    // Enable bitbang mode with a single output line (TXD)
    if ((f = ftdi_set_bitmode(&ftdic, 0x01, BITMODE_BITBANG)) == 0)
    {
      // start bit
      c = 0;
      ftdi_write_data(&ftdic, &c, 1);
      msleep(200);

      // data bits
      for (bitindex = 0; bitindex < databits; bitindex++)
      {
        c = ((1 << bitindex) & address) ? 1 : 0;
        if (c)
        {
          parityCount++;
        }
        ftdi_write_data(&ftdic, &c, 1);
        msleep(200);
      }

      if (parity == 1)
      {
        // odd parity
        c = (parityCount % 2) ? 1 : 0;
        ftdi_write_data(&ftdic, &c, 1);
        msleep(200);
      }
      else if (parity == 2)
      {
        // even parity
        c = (parityCount % 2) ? 0 : 1;
        ftdi_write_data(&ftdic, &c, 1);
        msleep(200);
      }

      // stop bit
      c = 1;
      ftdi_write_data(&ftdic, &c, 1);
      msleep(200);

      if (ftdi_set_bitmode(&ftdic, 0x01, BITMODE_RESET) == 0)
      {
        status = true;
      }
      else
      {
        fprintf(stderr, "kwp71(error): Failed to reset bitmode to FIFO/serial\n");
      }
    }
    else
    {
      fprintf(stderr, "kwp71(error): Failed to set bitbang mode\n");
    }

    ftdi_usb_close(&ftdic);
  }
  else
  {
    fprintf(stderr, "kwp71(error): Failed to open FTDI device: %d, %s\n",
      f, ftdi_get_error_string(&ftdic));
  }

  ftdi_deinit(&ftdic);

  return status;
}

/**
 * Sets initial values in the state-info struct.
 * Note that this routine does not actually open the serial port or attempt
 * to connect to the ECU; that requires kwp71_connect().
 */
void kwp71_init(kwp71_info* info)
{
  info->connected = false;
#if defined(WIN32)
  info->sd = INVALID_HANDLE_VALUE;
  info->mutex = CreateMutex(NULL, TRUE, NULL);
#else
  info->sd = 0;
  pthread_mutex_init(&info->mutex, NULL);
#endif
}

/**
 * Disconnects (if necessary) and closes the mutex handle.
 * @param info State information for the current connection.
 */
void kwp71_cleanup(kwp71_info* info)
{
#if defined(WIN32)

  if (info->connected)
  {
    CloseHandle(info->sd);
    info->sd = INVALID_HANDLE_VALUE;
  }

  CloseHandle(info->mutex);
#else

  if (info->connected)
  {
    close(info->sd);
    info->sd = 0;
  }

  pthread_mutex_destroy(&info->mutex);
#endif
}

/**
 * Returns version information for this build of the library.
 * @return Version of this build of libkwp71
 */
kwp71_version kwp71_getLibraryVersion()
{
  kwp71_version ver;

  ver.major = KWP71_VER_MAJOR;
  ver.minor = KWP71_VER_MINOR;
  ver.patch = KWP71_VER_PATCH;

  return ver;
}

/**
 * Closes the serial device.
 * @param info State information for the current connection.
 */
void kwp71_disconnect(kwp71_info* info)
{
#if defined(WIN32)

  if (WaitForSingleObject(info->mutex, INFINITE) == WAIT_OBJECT_0)
  {
    if (info->connected)
    {
      CloseHandle(info->sd);
      info->sd = INVALID_HANDLE_VALUE;
      info->conected = false;
    }
    ReleaseMutex(info->mutex);
  }

#else
  pthread_mutex_lock(&info->mutex);
  if (info->connected)
  {
    close(info->sd);
    info->sd = 0;
    info->connected = false;
  }
  pthread_mutex_unlock(&info->mutex);
#endif
}

/**
 * Opens the serial port (or returns with success if it is already open.)
 * @param info State information for the current connection.
 * @param dev Full path to the serial device (e.g. "/dev/ttyUSB0" or "COM2")
 * @return True if the serial device was successfully opened and its
 *   baud rate was set; false otherwise.
 */
bool kwp71_connect(kwp71_info* info, const char* dev, kwp71_protocol_variant variant, kwp71_ecuinfo* ecuinfo)
{
  bool result = false;

#if defined(WIN32)
  if (WaitForSingleObject(info->mutex, INFINITE) == WAIT_OBJECT_0)
  {
#elif defined(linux)
    pthread_mutex_lock(&info->mutex);
#endif
    // TODO: see if we can do this in a different order for better timing:
    // it might be possible to first open/configure the serial device and
    // temporarily take over with bitbang mode for the 5-baud init
    result = info->connected ||
      (kwp71_5baudinit(0x10, 7, 1) &&
       kwp71_openSerial(info, dev) &&
       kwp71_readIntroduction(info, variant, ecuinfo));
#if defined(WIN32)
    ReleaseMutex(info->mutex);
  }
#else // Linux/Unix
    pthread_mutex_unlock(&info->mutex);
#endif

  return result;
}

/**
 * Opens the serial device associated with the FTDI USB serial converter and
 * sets the parameters for the link to match those specified by the KWP-71
 * protocol (8N1, 9600 baud). Note that this function should only be called
 * immediately after the 5-baud init sequence has completed, and it should
 * be immediately followed by a call to kwp71_readIntroduction().
 *
 * Note for FreeBSD users: Do not use the ttyX devices, as they block
 * on the open() call while waiting for a carrier detect line, which
 * will never be asserted. Instead, use the equivalent cuaX device.
 * Example: /dev/cuaU0 (instead of /dev/ttyU0)
 * @return True if the open/setup was successful, false otherwise
 */
bool kwp71_openSerial(kwp71_info* info, const char* dev)
{
  bool retVal = false;

#if defined(linux)
  struct termios newtio;
  bool success = true;

  dprintf_info("kwp71(info): Opening the serial device (%s)...\n", dev);
  info->sd = open(dev, O_RDWR | O_NOCTTY);

  if (info->sd > 0)
  {
    dprintf_info("kwp71(info): Opened device successfully.\n");

    if (tcgetattr(info->sd, &newtio) != 0)
    {
      dprintf_err("kwp71(error): Unable to read serial port parameters.\n");
      success = false;
    }

    if (success)
    {
      // set up the serial port:
      // * enable the receiver, set 8-bit fields, set local mode, disable hardware flow control
      // * set non-canonical mode, disable echos, disable signals
      // * disable all special handling of CR or LF, disable all software flow control
      // * disable all output post-processing
      newtio.c_cflag &= ((CREAD | CS8 | CLOCAL) & ~(CRTSCTS));
      newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      newtio.c_iflag &= ~(INLCR | ICRNL | IGNCR | IXON | IXOFF | IXANY);
      newtio.c_oflag &= ~OPOST;

      // when waiting for responses, wait until we haven't received any
      // characters for a period of time before returning with failure
      newtio.c_cc[VTIME] = 1;
      newtio.c_cc[VMIN] = 0;

      cfsetispeed(&newtio, B4800);
      cfsetospeed(&newtio, B4800);

      // attempt to set the termios parameters
      dprintf_info("kwp71(info): Setting serial port parameters...\n");

      // flush the serial buffers and set the new parameters
      if ((tcflush(info->sd, TCIFLUSH) != 0) ||
          (tcsetattr(info->sd, TCSANOW, &newtio) != 0))
      {
        dprintf_err("kwp71(error): Failure setting up port\n");
        close(info->sd);
        success = false;
      }
    }

    // close the device if it couldn't be configured
    if (retVal == false)
    {
      close(info->sd);
    }
  }
  else // open() returned failure
  {
    dprintf_err("kwp71(error): Error opening device (%s)\n", strerror(errno));
  }

#elif defined(WIN32)

  DCB dcb;
  COMMTIMEOUTS commTimeouts;

  // attempt to open the device
  dprintf_info("kwp71(info): Opening the serial device (Win32) '%s'...\n", devPath);

  // open and get a handle to the serial device
  info->sd = CreateFile(devPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

  // verify that the serial device was opened
  if (info->sd != INVALID_HANDLE_VALUE)
  {
    if (GetCommState(info->sd, &dcb) == TRUE)
    {
      // set the serial port parameters, including the custom baud rate
      dcb.BaudRate = 4800;
      dcb.fParity = FALSE;
      dcb.fOutxCtsFlow = FALSE;
      dcb.fOutxDsrFlow = FALSE;
      dcb.fDtrControl = FALSE;
      dcb.fRtsControl = FALSE;
      dcb.ByteSize = 8;
      dcb.Parity = 0;
      dcb.StopBits = 0;

      if ((SetCommState(info->sd, &dcb) == TRUE) &&
          (GetCommTimeouts(info->sd, &commTimeouts) == TRUE))
      {
        // modify the COM port parameters to wait 100 ms before timing out
        commTimeouts.ReadIntervalTimeout = 100;
        commTimeouts.ReadTotalTimeoutMultiplier = 0;
        commTimeouts.ReadTotalTimeoutConstant = 100;

        if (SetCommTimeouts(info->sd, &commTimeouts) == TRUE)
        {
          retVal = true;
        }
      }
    }

    // the serial device was opened, but couldn't be configured properly;
    // close it before returning with failure
    if (!retVal)
    {
      dprintf_err("kwp71(error): Failure setting up port; closing serial device...\n");
      CloseHandle(info->sd);
    }
  }
  else
  {
    dprintf_err("kwp71(error): Error opening device.\n");
  }

#endif

  return retVal;
}

