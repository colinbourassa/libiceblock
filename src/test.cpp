#include <libftdi1/ftdi.h>
#include <thread>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

struct ftdi_context m_ftdi;

bool readSerial(uint8_t* buf, int count)
{
  int readCount = 0;
  int status = 0;
  std::chrono::time_point start = std::chrono::steady_clock::now();

  do {
    status = ftdi_read_data(&m_ftdi, buf + readCount, count - readCount);
    if (status > 0)
    {
      readCount += status;
    }
  } while ((readCount < count) &&
           (status >= 0) &&
           ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start))
             < std::chrono::milliseconds(1000)));
  
  return (readCount == count);
}

bool writeSerial(uint8_t* buf, int count)
{
  int status = 0;
  std::chrono::time_point start = std::chrono::steady_clock::now();
  printf("writeSerial() called with count %d, buf[0] == %02X\n", count, buf[0]);
  struct ftdi_transfer_control* tc = ftdi_write_data_submit(&m_ftdi, buf, count);
  status = ftdi_transfer_data_done(tc);

  printf("ftdi_transfer_data_done returned %d\n", status);
  
  return (status == count);
}


/**
 * Performs the 'slow init' sequence by bit-banging the FTDI's transmit line to
 * clock out the provided address (with optional parity) at the requisite 5 baud.
 */
bool slowInit(uint8_t address, int databits, int parity)
{
  bool status = false;
  unsigned char c;
  int f = 0;
  int bitindex = 0;
  int parityCount = 0;

  printf("slowinit: ");
  // Enable bitbang mode with a single output line (TXD)
  if ((f = ftdi_set_bitmode(&m_ftdi, 0x01, BITMODE_BITBANG)) == 0)
  {
    // start bit
    c = 0;
    printf("start:%d ", c); fflush(stdout);
    ftdi_write_data(&m_ftdi, &c, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // data bits
    for (bitindex = 0; bitindex < databits; bitindex++)
    {
      c = ((1 << bitindex) & address) ? 1 : 0;
      if (c)
      {
        parityCount++;
      }
    printf("%d ", c); fflush(stdout);
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (parity == 1)
    {
      // odd parity
      c = (parityCount % 2) ? 1 : 0;
    printf("parity:%d ", c); fflush(stdout);
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else if (parity == 2)
    {
      // even parity
      c = (parityCount % 2) ? 0 : 1;
    printf("parity:%d ", c); fflush(stdout);
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // stop bit
    c = 1;
    printf("stop:%d ", c); fflush(stdout);
    ftdi_write_data(&m_ftdi, &c, 1);
    //std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (ftdi_set_bitmode(&m_ftdi, 0xff, BITMODE_RESET) == 0)
    {
      status = true;
    }
    else
    {
      std::cerr << "Failed to disable bitbang mode" << std::endl;
    }
  }
  else
  {
    std::cerr << "Failed to set bitbang mode" << std::endl;
  }
  printf("\n");

  //std::this_thread::sleep_for(std::chrono::milliseconds(3500));

  return status;
}

/**
 * Reads the three sync/keyword bytes that are transmitted by the ECU
 * immediately after the 5-baud slow init, and replies with the bitwise
 * inversion of the third byte as an acknowledgement.
 */
bool readAckKeywordBytes()
{
  bool status = false;
  uint8_t kwpBytes[3] = { 0, 0, 0 };
  int byteCount = 0;

  uint8_t tmpByte = 0;
  int bytesReceived = 0;

  while (tmpByte != 0x81)
  {
   // CMB: it looks like there is some problem with libftdi that causes it
   // to return immediately from ftdi_read_data() -- this happens 256 times
   // before starting to return bytes that were actually received over serial.
   // It doesn't matter if we wait a long time before calling ftdi_read_data();
   // it seems those 256 extra bytes are stuck in some sort of fifo.
   // If we call ftdi_usb_reset() at some point in here, the number of false bytes
   // is reduced, but they still exist.

    int rs = readSerial(&tmpByte, 1);
    if (rs == 0)
    {
      printf(" xx");
    }
    else if (rs == 1)
    {
      printf(" %02X", tmpByte);
      bytesReceived++;
    }
    else if (rs < 0)
    {
      printf(" ee");
    }
    fflush(stdout);
  }

  printf("\n%d bytes received.\n", bytesReceived);
  /*
  for (int byteCount = 0; byteCount < 3; byteCount++)
  {
    if (readSerial(&kwpBytes[byteCount], 1))
    {
      printf("read byte: %02X\n", kwpBytes[byteCount]);
    }
    else
    {
      printf("failed reading keyword byte\n");
    }
  }
  */

  if ((kwpBytes[0] == 0x55) &&
      (kwpBytes[1] == 0x00) && 
      (kwpBytes[2] == 0x81))
  {
    // write the bitwise inversion to acknowledge, and read it
    // back to to clear the rx buffer
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    kwpBytes[2] = ~(kwpBytes[2]);

    if (writeSerial(&kwpBytes[2], 1) &&
        readSerial(&kwpBytes[2], 1))
    {
      printf("loopback byte was %02X\n", kwpBytes[2]);
      status = true;
    }

    exit(0);
  }
  else
  {
    printf("\nKeyword bytes (%02X %02X %02X) did not match expected.\n",
      kwpBytes[0], kwpBytes[1], kwpBytes[2]);
  }

  return status;
}

/**
 * Reads a packet from the serial port, the length of which is determined by
 * the first byte. Each byte (except the last) is positively acknowledged with
 * its bitwise inversion. Returns true if all the expected bytes are received,
 * false otherwise.
 */
bool recvPacket()
{
  uint8_t m_recvPacketBuf[256];
  bool status = true;
  uint8_t index = 1;
  uint8_t loopback = 0;
  uint8_t ack = 0;

  if (readSerial(&m_recvPacketBuf[0], 1))
  {
    printf("Received pkt length byte: %02X\n", m_recvPacketBuf[0]);

    // ack the pkt length byte
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ack = ~(m_recvPacketBuf[0]);
    printf("send ack: %02X\n", ack);
    status = writeSerial(&ack, 1);
    if (readSerial(&loopback, 1))
    {
      printf("recv (loopback): %02X\n", loopback);
    }
    else
    {
      status = false;
      printf("read() timeout when attempting to read loopback byte\n");
    }

    while (status && (index < m_recvPacketBuf[0]))
    {
      if (readSerial(&m_recvPacketBuf[index], 1))
      {
        printf("recv: %02X\n", m_recvPacketBuf[index]);

        // every byte except the last in the packet is ack'd
        if (index < m_recvPacketBuf[0])
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          ack = ~(m_recvPacketBuf[index]);
          printf("send ack: %02X\n", ack);
          status = writeSerial(&ack, 1);
          if (readSerial(&loopback, 1))
          {
            printf("recv (loopback): %02X\n", loopback);
          }
          else
          {
            status = false;
            printf("read() timeout when attempting to read loopback byte\n");
          }
        }
      }
      else
      {
        std::cerr << std::endl << "Timed out while receiving packet data!" << std::endl;
        status = false;
      }
    }
  }

  return status;
}


int main()
{
  uint8_t addr = 0x10;

  ftdi_init(&m_ftdi);
  ftdi_set_interface(&m_ftdi, INTERFACE_A);
  if ((ftdi_usb_open(&m_ftdi, 0x0403, 0x6001) == 0))
  {
    printf("-- ftdi_usb_open() success\n");

    ftdi_tcioflush(&m_ftdi);

    slowInit(addr, 8, 0);
    /*
    if (ftdi_usb_reset(&m_ftdi) != 0)
    {
      printf("usb reset failed\n");
    }
    */
    ftdi_usb_reset(&m_ftdi);
    ftdi_usb_close(&m_ftdi);
    if (ftdi_usb_open(&m_ftdi, 0x0403, 0x6001) == 0)
    {
    ftdi_usb_reset(&m_ftdi);
      ftdi_set_baudrate(&m_ftdi, 4800);
      ftdi_set_line_property(&m_ftdi, BITS_8, STOP_BIT_1, NONE);
      ftdi_set_latency_timer(&m_ftdi, 1);
      ftdi_setflowctrl(&m_ftdi, SIO_DISABLE_FLOW_CTRL);
      ftdi_tcioflush(&m_ftdi);
      if (readAckKeywordBytes())
      {
        printf("-- readAckKeywordBytes() success\n");
        recvPacket();
      }
      ftdi_usb_close(&m_ftdi);
    }
  }
  ftdi_deinit(&m_ftdi);

  return 0;
}

