#include "kwp71.h"
#include <libftdi1/ftdi.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

Kwp71Version Kwp71::getLibraryVersion()
{
  Kwp71Version ver;

  ver.major = KWP71_VER_MAJOR;
  ver.minor = KWP71_VER_MINOR;
  ver.patch = KWP71_VER_PATCH;

  return ver;
}

Kwp71::Kwp71(std::string device, uint8_t addr) :
  m_connectionActive(false),
  m_ecuAddr(addr),
  m_shutdown(false),
  m_deviceName(device),
  m_fd(-1),
  m_lastUsedSeqNum(0),
  m_readyForCommand(true),
  m_receivingData(false),
  m_responseReadSuccess(false)
{
}

bool Kwp71::connect()
{
  m_ifThread = std::thread(threadEntry, this);
  // TODO: block until the 5-baud init has been completed, the serial port
  // has been opened, and the connection to the ECU is active. Return true
  // if all of that was successful; false otherwise.
  return false;
}

/**
 * Opens and sets up the serial port device.
 */
bool Kwp71::openSerialPort()
{
#if defined(linux)
  struct termios newtio;
  bool success = true;

  std::cout << "Opening the serial device (" << m_deviceName << ")..." << std::endl;
  m_fd = open(m_deviceName.c_str(), O_RDWR | O_NOCTTY);

  if (m_fd > 0)
  {
    std::cout << "Opened device successfully." << std::endl;

    if (tcgetattr(m_fd, &newtio) != 0)
    {
      std::cerr << "Unable to read serial port parameters." << std::endl;
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

      cfsetispeed(&newtio, B9600);
      cfsetospeed(&newtio, B9600);

      // attempt to set the termios parameters
      std::cout << "Setting serial port parameters..." << std::endl;

      // flush the serial buffers and set the new parameters
      if ((tcflush(m_fd, TCIFLUSH) != 0) ||
          (tcsetattr(m_fd, TCSANOW, &newtio) != 0))
      {
        std::cerr << "Failure setting up port" << std::endl;
        close(m_fd);
        success = false;
      }
    }

    // close the device if it couldn't be configured
    if (!success)
    {
      close(m_fd);
    }
  }
  else // open() returned failure
  {
    std::cerr << "Error opening device (" << m_deviceName << ")" << std::endl;
  }

#elif defined(WIN32)

  DCB dcb;
  COMMTIMEOUTS commTimeouts;

  // attempt to open the device
  std::cout << "Opening the serial device (Win32) '" << m_deviceName << "'..." << std::endl;

  // open and get a handle to the serial device
  m_fd = CreateFile(devPath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

  // verify that the serial device was opened
  if (m_fd != INVALID_HANDLE_VALUE)
  {
    if (GetCommState(m_fd, &dcb) == TRUE)
    {
      // set the serial port parameters
      dcb.BaudRate = 9600;
      dcb.fParity = FALSE;
      dcb.fOutxCtsFlow = FALSE;
      dcb.fOutxDsrFlow = FALSE;
      dcb.fDtrControl = FALSE;
      dcb.fRtsControl = FALSE;
      dcb.ByteSize = 8;
      dcb.Parity = 0;
      dcb.StopBits = 0;

      if ((SetCommState(m_fd, &dcb) == TRUE) &&
          (GetCommTimeouts(m_fd, &commTimeouts) == TRUE))
      {
        // modify the COM port parameters to wait 100 ms before timing out
        commTimeouts.ReadIntervalTimeout = 100;
        commTimeouts.ReadTotalTimeoutMultiplier = 0;
        commTimeouts.ReadTotalTimeoutConstant = 100;

        if (SetCommTimeouts(m_fd, &commTimeouts) == TRUE)
        {
          success = true;
        }
      }
    }

    // the serial device was opened, but couldn't be configured properly;
    // close it before returning with failure
    if (!success)
    {
      std::cerr << "Failure setting up port; closing serial device..." << std::cerr;
      CloseHandle(m_fd);
    }
  }
  else
  {
    std::cerr << "Error opening device." << std::endl;
  }

#endif

  return success;
}

/**
 * Closes the serial port device.
 */
void Kwp71::closeDevice()
{
  close(m_fd);
}

/**
 * Shuts down the connection and waits for the connection thread to
 * finish.
 */
void Kwp71::shutdown()
{
  m_shutdown = true;
  m_ifThread.join();
  closeDevice();
}

bool Kwp71::isConnectionActive() const
{
  return m_connectionActive;
}

/**
 * Sends a KWP-71 packet by transmitting it one byte at a time and waiting
 * for positive acknowledgement of each byte by the receiving ECU.
 * Returns true if every byte (except the last) is met with a bitwise
 * inversion of that byte in response from the ECU; false otherwise.
 */
bool Kwp71::sendPacket()
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

/**
 * Reads a packet from the serial port, the length of which is determined by
 * the first byte. Each byte (except the last) is positively acknowledged with
 * its bitwise inversion. Returns true if all the expected bytes are received,
 * false otherwise.
 */
bool Kwp71::recvPacket(Kwp71PacketType& type)
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
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

/**
 * Populates a local buffer with the contents of a packet to be transmitted.
 */
bool Kwp71::populatePacket(bool usePendingCommand)
{
  bool status = false;
  Kwp71PacketType type;
  std::vector<uint8_t> payload;

  if (usePendingCommand)
  {
    type = m_pendingCmd.type;
    payload = m_pendingCmd.payload;
  }
  else
  {
    type = Kwp71PacketType::Empty;
    payload = {};
  }

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
bool Kwp71::slowInit(uint8_t address, int databits, int parity)
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

/**
 * Reads the three sync/keyword bytes that are transmitted by the ECU
 * immediately after the 5-baud slow init, and replies with the bitwise
 * inversion of the third byte as an acknowledgement.
 */
bool Kwp71::readAckKeywordBytes()
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
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
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

/**
 * Queues a command to read the ID information from the ECU, which is returned
 * as a collection of strings.
 */
bool Kwp71::requestIDInfo(std::vector<std::string>& idResponse)
{
  // wait until any previous command has been fully processed
  std::unique_lock<std::mutex> lock(m_responseMutex);

  m_pendingCmd.type = Kwp71PacketType::RequestID;
  m_pendingCmd.payload = std::vector<uint8_t>();

  // wait until the response from this command has been completely received
  std::cout << "Command sender waiting for response..." << std::endl;
  m_responseCondVar.wait(lock);
  if (m_responseReadSuccess)
  {
    idResponse = m_responseStringData;
    m_responseStringData.clear();
  }

  return m_responseReadSuccess;
}

/**
 * Queues a command to be sent to the ECU at the next opportunity, and waits
 * to receive the response.
 */
bool Kwp71::sendCommand(Kwp71Command cmd, std::vector<uint8_t>& response)
{
  // wait until any previous command has been fully processed
  std::unique_lock<std::mutex> lock(m_responseMutex);

  m_pendingCmd = cmd;

  // wait until the response from this command has been completely received
  std::cout << "Command sender waiting for response..." << std::endl;
  m_responseCondVar.wait(lock);
  if (m_responseReadSuccess)
  {
    response = m_responseBinaryData;
    m_responseBinaryData.clear();
  }

  return m_responseReadSuccess;
}

/**
 * Parses the data in the received packet buffer and takes the appropriate
 * action to package and return the data.
 */
void Kwp71::processReceivedPacket()
{
  const Kwp71PacketType type = (Kwp71PacketType)(m_recvPacketBuf[2]);
  const uint8_t payloadLen = m_recvPacketBuf[0] - 3;

  switch (type)
  {
  case Kwp71PacketType::ParamRecordConf:
  case Kwp71PacketType::Snapshot:
    break;
  case Kwp71PacketType::ASCIIString:
    m_responseStringData.push_back(std::string(m_recvPacketBuf[3], payloadLen));
    break;
  case Kwp71PacketType::DACValue:
  case Kwp71PacketType::BinaryData:
  case Kwp71PacketType::RAMContent:
  case Kwp71PacketType::ROMContent:
  case Kwp71PacketType::EEPROMContent:
  case Kwp71PacketType::ParametricData:
    m_responseBinaryData.insert(m_responseBinaryData.end(),
                                &m_recvPacketBuf[3],
                                &m_recvPacketBuf[3 + payloadLen]);
    break;
  }
}

/**
 * Thread entry point, which calls the main communication loop function.
 */
void Kwp71::threadEntry(Kwp71* iface)
{
  iface->commLoop();
}

/**
 * Loop that maintains a connection with the ECU. When no particular
 * command is queued, the empty/ACK (09) command is sent as a keepalive.
 */
void Kwp71::commLoop()
{
  while (!m_shutdown)
  {
    // loop until the initialization sequence has completed,
    // re-attempting the 5-baud slow init if necessary
    while (!m_connectionActive && !m_shutdown)
    {
      slowInit(m_ecuAddr, 7, 0);

      // TODO: can we attempt to open/configure the serial port before we
      // do the slow init sequence? That would allow us to exit early with
      // failure before attempting the 5-baud stuff.
      if (openSerialPort())
      {
        m_connectionActive = readAckKeywordBytes();
      }
      else
      {
        std::cerr << "Error opening serial device '" << m_deviceName << "'" << std::endl;
      }
    }

    Kwp71PacketType recvType;

    // TODO: the ECU might send its ID info unsolicited here;
    // be prepared to receive it (possibly by moving the send()
    // routine to the bottom of the main loop)

    // continue taking turns with the ECU, sending one packet per turn
    while (m_connectionActive && !m_shutdown)
    {
      // the readyForCommand flag is only set when the ECU is simply
      // sending empty ACK packets, waiting for us to send a command
      populatePacket(m_readyForCommand);
      sendPacket();
 
      if (recvPacket(recvType))
      {
        if (recvType == Kwp71PacketType::Empty)
        {
          m_readyForCommand = true;
          if (m_receivingData)
          {
            m_receivingData = false;
            m_responseReadSuccess = true;
            m_responseCondVar.notify_one();
          }
        }
        else
        {
          m_readyForCommand = false;
          m_receivingData = true;
        }
      }
      else
      {
        if (m_receivingData)
        {
          m_receivingData = false;
          m_responseReadSuccess = false;
          m_responseCondVar.notify_one();
        }
        std::cerr << "ECU didn't respond during its turn" << std::endl;
        m_connectionActive = false;
      }
    }

    if (!m_connectionActive)
    {
      closeDevice();
    }
  }
}

