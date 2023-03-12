#include "kwp71interface.h"
#include <libftdi1/ftdi.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <chrono>

Kwp71Interface::Kwp71Interface(std::string device) :
  m_shutdown(false),
  m_deviceName(device),
  m_fd(-1),
  m_lastUsedSeqNum(0)
{
  m_ifThread = std::thread(commLoop, this, &m_shutdown);
}

/**
 * Opens and sets up the serial port device.
 */
bool Kwp71Interface::openSerialPort()
{
  return false;
}

/**
 * Closes the serial port device.
 */
void Kwp71Interface::closeDevice()
{
  close(m_fd);
}

/**
 * Shuts down the connection and waits for the connection thread to
 * finish.
 */
void Kwp71Interface::shutdown()
{
  m_shutdown = true;
  m_ifThread.join();
  closeDevice();
}

/**
 * Sends a KWP-71 packet by transmitting it one byte at a time and waiting
 * for positive acknowledgement of each byte by the receiving ECU.
 * Returns true if every byte (except the last) is met with a bitwise
 * inversion of that byte in response from the ECU; false otherwise.
 */
bool Kwp71Interface::sendPacket()
{
  bool status = true;
  uint8_t index = 0;
  uint8_t loopback = 0;
  uint8_t ack = 0;

  while (status && (index <= m_sendPacketBuf[0]))
  {
    // Transmit the packet one byte at a time, reading back the same byte as
    // it appears in the Rx buffer (due to the loopback) and then reading the
    // acknowledgement byte from the ECU (for every byte except the last).
    // Verify that the ack byte is the bitwise inversion of the sent byte.
    status =
      (write(m_fd, &m_sendPacketBuf[index], 1) == 1) &&
      (read(m_fd, &loopback, 1) == 1);

    if (index < m_sendPacketBuf[3])
    {
      status = status &&
        (read(m_fd, &ack, 1) == 1) &&
        (ack == ~(m_sendPacketBuf[index]));
    }
    index++;
  }
  return status;
}

bool Kwp71Interface::recvPacket(Kwp71PacketType& type)
{
  bool status = true;
  uint8_t index = 1;
  uint8_t loopback = 0;
  uint8_t ack = 0;

  if (read(m_fd, &m_recvPacketBuf[0], 1) == 1)
  {
    std::cout << "Received pkt length byte: " << std::hex << m_recvPacketBuf[0] << std::dec << std::endl;
    while (status && (index < m_recvPacketBuf[0]))
    {
      std::cout << "Receiving pkt content:" << std::setfill('0') <<
        std::setw(2) << std::right << std::hex;
      if (read(m_fd, &m_recvPacketBuf[index], 1) == 1)
      {
        std::cout << " " << m_recvPacketBuf[index];
        if (index < m_recvPacketBuf[0])
        {
          ack = ~(m_recvPacketBuf[index]);
          status =
            (write(m_fd, &ack, 1) == 1) &&
            (read(m_fd, &loopback, 1) == 1);
        }
      }
      else
      {
        std::cerr << std::endl << "Timed out while receiving packet data!" << std::endl;
        status = false;
      }
    }
  }

  std::cout << std::dec;

  if (status)
  {
    type = (Kwp71PacketType)(m_recvPacketBuf[2]);
    m_lastUsedSeqNum = m_recvPacketBuf[1];
  }

  return status;
}

bool Kwp71Interface::populatePacket(Kwp71PacketType type,
                                    const std::vector<uint8_t>& payload)
{
  bool status = false;

  switch (type)
  {
  case Kwp71PacketType::Empty:
  case Kwp71PacketType::ReadTroubleCodes:
  case Kwp71PacketType::RequestID:
  case Kwp71PacketType::RequestSnapshot:
  case Kwp71PacketType::Disconnect:
    status = true;
    m_sendPacketBuf[0] = 0x03;
    break;
  case Kwp71PacketType::ReadDACChannel:
    if (payload.size() == 1)
    {
      status = true;
      m_sendPacketBuf[0] = 0x04;
      m_sendPacketBuf[3] = payload[0];
    }
    break;
  case Kwp71PacketType::ReadRAM:
  case Kwp71PacketType::ReadROM:
  case Kwp71PacketType::ReadEEPROM:
    if (payload.size() == 3)
    {
      status = true;
      m_sendPacketBuf[0] = 0x06;
      memcpy(&m_sendPacketBuf[3], &payload[0], payload.size());
    }
    break;
  case Kwp71PacketType::WriteRAM:
  case Kwp71PacketType::WriteEEPROM:
    if (payload.size() == (payload[0] + 3))
    {
      status = true;
      m_sendPacketBuf[0] = payload.size() + 3;
      memcpy(&m_sendPacketBuf[3], &payload[0], payload.size());
    }
  default:
    break;
  }

  if (status)
  {
    m_sendPacketBuf[1] = ++m_lastUsedSeqNum;
    m_sendPacketBuf[2] = (uint8_t)type;
    m_sendPacketBuf[m_sendPacketBuf[0]] = s_endOfPacket;
  }

  return status;
}

/**
 * Performs the 'slow init' sequence by bit-banging the FTDI's transmit line to
 * clock out the provided address (with optional parity) at the requisite 5 baud.
 */
bool slowInit(uint8_t address, int databits, int parity)
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
    std::cerr << "ftdi_init() or ftdi_set_interface() failed" << std::endl;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      // data bits
      for (bitindex = 0; bitindex < databits; bitindex++)
      {
        c = ((1 << bitindex) & address) ? 1 : 0;
        if (c)
        {
          parityCount++;
        }
        ftdi_write_data(&ftdic, &c, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      if (parity == 1)
      {
        // odd parity
        c = (parityCount % 2) ? 1 : 0;
        ftdi_write_data(&ftdic, &c, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      else if (parity == 2)
      {
        // even parity
        c = (parityCount % 2) ? 0 : 1;
        ftdi_write_data(&ftdic, &c, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      // stop bit
      c = 1;
      ftdi_write_data(&ftdic, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));

      if (ftdi_set_bitmode(&ftdic, 0x01, BITMODE_RESET) == 0)
      {
        status = true;
      }
      else
      {
        std::cerr << "Failed to reset bitmode to FIFO/serial" << std::endl;
      }
    }
    else
    {
      std::cerr << "Failed to set bitbang mode" << std::endl;
    }

    ftdi_usb_close(&ftdic);
  }
  else
  {
    std::cerr << "Failed to open FTDI device: " << f << ", " <<
      ftdi_get_error_string(&ftdic) << std::endl;
  }

  ftdi_deinit(&ftdic);

  return status;
}

void Kwp71Interface::getNextPendingCommand(Kwp71Command& cmd)
{
  // TODO: read from a queue of pending commands
  if (0)
  {
  }
  else
  {
    cmd.type = Kwp71PacketType::Empty;
  }
}

bool Kwp71Interface::readAckKeywordBytes()
{
  bool status = false;
  uint8_t kwpBytes[3];

  if (read(m_fd, kwpBytes, 3) == 3)
  {
    if ((kwpBytes[0] == 0x55) &&
        (kwpBytes[1] == 0x00) && 
        (kwpBytes[2] == 0x81))
    {
      // write the bitwise inversion to acknowledge, and read it
      // back to to clear the rx buffer
      kwpBytes[2] = ~(kwpBytes[2]);
      if (write(m_fd, &kwpBytes[2], 1) &&
          read(m_fd, &kwpBytes[2], 1))
      {
        status = true;
      }
    }
    else
    {
      std::cerr << "Keyword bytes (" << std::setfill('0') <<
        std::setw(2) << std::right << std::hex <<
        kwpBytes[0] << " " << kwpBytes[1] << " " << kwpBytes[2] <<
        ") did not match expected" << std::dec << std::endl;
    }
  }
  else
  {
    std::cerr << "Failed to read 3 bytes of keyword protocol ID" << std::endl;
  }

  return status;
}

void Kwp71Interface::processReceivedPacket()
{
  //TODO
}

/**
 * Loop that maintains a connection with the ECU. When no particular
 * commands are queued, the empty/ACK (09) command is sent as a
 * keepalive.
 */
void Kwp71Interface::commLoop(Kwp71Interface* iface, bool* shutdown)
{
  bool connectionAlive = false;

  while (!shutdown)
  {
    // loop until the initialization sequence has completed,
    // re-attempting the 5-baud slow init if necessary
    while (!connectionAlive && !shutdown)
    {
      iface->slowInit(0x10, 8, 0);
      if (iface->openSerialPort())
      {
        connectionAlive = iface->readAckKeywordBytes();
      }
    }

    Kwp71Command cmd;
    Kwp71PacketType lastReceivedPacketType;

    // continue taking turns with the ECU, transmitting one
    // packet per turn
    while (connectionAlive && !shutdown)
    {
      if (lastReceivedPacketType == Kwp71PacketType::Empty)
      {
        // only look for the next command to send when we're not
        // in the middle of receiving packets from the ECU that
        // were sent in response to a previous command
        iface->getNextPendingCommand(cmd);
      }
      else
      {
        cmd.type = Kwp71PacketType::Empty;
      }

      iface->populatePacket(cmd.type, cmd.payload);
      iface->sendPacket();
      if (iface->recvPacket(lastReceivedPacketType))
      {
        iface->processReceivedPacket();
      }
      else
      {
        // the ECU didn't respond during its turn, so we need to consider
        // the connection dead
        std::cout << "ECU didn't respond during its turn; connection has died" << std::endl;
        connectionAlive = false;
      }
    }

    if (!connectionAlive)
    {
      iface->closeDevice();
    }
  }

  if (connectionAlive)
  {
    iface->populatePacket(Kwp71PacketType::Disconnect, std::vector<uint8_t>());
    iface->sendPacket();
  }
}

