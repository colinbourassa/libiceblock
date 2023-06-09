#include "kwp71.h"
#include <stdio.h>
#include <libusb.h>
#include <chrono>
#include <mutex>
#include <unistd.h>

Kwp71Version Kwp71::getLibraryVersion()
{
  Kwp71Version ver;

  ver.major = KWP71_VER_MAJOR;
  ver.minor = KWP71_VER_MINOR;
  ver.patch = KWP71_VER_PATCH;

  return ver;
}

Kwp71::Kwp71(bool verbose) :
  m_verbose(verbose),
  m_connectionActive(false),
  m_ecuAddr(0),
  m_baudRate(4800),
  m_initDataBits(8),
  m_initParity(0),
  m_shutdown(false),
  m_ifThreadPtr(nullptr),
  m_lastUsedSeqNum(0),
  m_lastReceivedBlockType(Kwp71BlockType::Empty),
  m_commandIsPending(false),
  m_waitingForReply(false),
  m_responseReadSuccess(false)
{
  ftdi_init(&m_ftdi);
}

/**
 * Performs the slow init sequence using the provided ECU address and then
 * opens the serial port device, using it to read the ECU's keyword bytes.
 * Returns true if all of this is successful; false otherwise.
 */
bool Kwp71::connect(uint16_t vid, uint16_t pid, uint8_t addr, int baud, int& err)
{
  bool status = false;
  std::unique_lock<std::mutex> lock(m_connectMutex);

  m_ecuAddr = addr;
  m_connectionActive = false;
  m_baudRate = baud;

  m_shutdown = false;
  if (m_ifThreadPtr == nullptr)
  {
    if (ftdi_set_interface(&m_ftdi, INTERFACE_A) == 0)
    {
      if ((ftdi_usb_open(&m_ftdi, vid, pid) == 0))
      {
        // TODO: parameterize the data bits and parity
        if (slowInit(m_ecuAddr, 8, 0))
        {
          if (setFtdiSerialProperties())
          {
            if (readAckKeywordBytes())
            {
              m_connectionActive = true;
              m_ifThreadPtr = std::make_unique<std::thread>(threadEntry, this);
              err = 0;
              status = true;
            }
            else
            {
              // readAckKeywordBytes() failed
              err = -1;
            }
          }
          else
          {
            // setFtdiSerialProperties() failed
            err = -2;
          }
        }
        else
        {
          // slowInit() failed
          err = -3;
        }
      }
      else
      {
        // ftdi_usb_open() failed
        err = -4;
      }
    }
    else
    {
      // ftdi_set_interface() failed
      err = -5;
    }
  }

  return status;
}

/**
 * Shuts down the connection and waits for the connection thread to
 * finish.
 */
void Kwp71::disconnect()
{
  std::unique_lock<std::mutex> lock(m_connectMutex);

  if (m_ifThreadPtr && m_ifThreadPtr->joinable())
  {
    m_shutdown = true;
    m_ifThreadPtr->join();
  }
  ftdi_usb_close(&m_ftdi);
  ftdi_deinit(&m_ftdi);
}

bool Kwp71::isConnectionActive() const
{
  return m_connectionActive;
}

/**
 * Sends a KWP-71 block by transmitting it one byte at a time and waiting
 * for positive acknowledgement of each byte by the receiving ECU.
 * Returns true if every byte (except the last) is met with a bitwise
 * inversion of that byte in response from the ECU; false otherwise.
 */
bool Kwp71::sendBlock()
{
  bool status = true;
  uint8_t index = 0;
  uint8_t loopback = 0;
  uint8_t ack = 0;
  uint8_t ackCompare = 0;
  bool usedPendingCommand = false;

  populateBlock(usedPendingCommand);

  if (m_verbose)
  {
    printf("send: ");
  }

  while (status && (index <= m_sendBlockBuf[0]))
  {
    // Transmit the block one byte at a time, reading back the same byte as
    // it appears in the Rx buffer (due to the loopback) and then reading the
    // acknowledgement byte from the ECU (for every byte except the last).
    // Verify that the ack byte is the bitwise inversion of the sent byte.
    if (writeSerial(&m_sendBlockBuf[index], 1) && readSerial(&loopback, 1))
    {
      if (m_verbose)
      {
        printf("%02X ", m_sendBlockBuf[index]);
      }
      if ((index < m_sendBlockBuf[0]))
      {
        ackCompare = ~(m_sendBlockBuf[index]);
        status = readSerial(&ack, 1) && (ack == ackCompare);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      index++;
    }
    else
    {
      status = false;
    }
  }
  if (m_verbose)
  {
    printf("\n");
  }

  // if we had been waiting to send a command block (i.e. not just an 09/ACK),
  // then this sendBlock() call would have transmitted it so the 'pending'
  // flag may be cleared
  if (status && usedPendingCommand)
  {
    m_commandIsPending = false;
    m_waitingForReply = true;
  }

  return status;
}

/**
 * Reads a block from the serial port, the length of which is determined by
 * the first byte. Each byte (except the last) is positively acknowledged with
 * its bitwise inversion. Returns true if all the expected bytes are received,
 * false otherwise.
 */
bool Kwp71::recvBlock()
{
  bool status = true;
  uint16_t index = 1;
  uint8_t loopback = 0;
  uint8_t ack = 0;

  if (m_verbose)
  {
    printf("recv: ");
  }

  if (readSerial(&m_recvBlockBuf[0], 1))
  {
    if (m_verbose)
    {
      printf("%02X ", m_recvBlockBuf[0]);
    }
    // ack the pkt length byte
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ack = ~(m_recvBlockBuf[0]);
    status = writeSerial(&ack, 1) && readSerial(&loopback, 1);

    while (status && (index <= m_recvBlockBuf[0]))
    {
      if (readSerial(&m_recvBlockBuf[index], 1))
      {
        if (m_verbose)
        {
          printf("%02X ", m_recvBlockBuf[index]);
        }

        // every byte except the last in the block is ack'd
        if (index < m_recvBlockBuf[0])
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          ack = ~(m_recvBlockBuf[index]);
          status = writeSerial(&ack, 1) && readSerial(&loopback, 1);
        }
        index++;
      }
      else
      {
        if (m_verbose)
        {
          fprintf(stderr, "Timed out while receiving block data!\n");
        }
        status = false;
      }
    }
    if (m_verbose)
    {
      printf("\n");
    }
  }

  if (status)
  {
    processReceivedBlock();
  }

  return status;
}

/**
 * Populates a local buffer with the contents of a block to be transmitted.
 */
bool Kwp71::populateBlock(bool& usedPendingCommand)
{
  bool status = false;
  Kwp71BlockType type;
  std::vector<uint8_t> payload;

  const bool ecuReadyForCmd = (m_lastReceivedBlockType == Kwp71BlockType::Empty);

  if (ecuReadyForCmd && m_commandIsPending)
  {
    type = m_pendingCmd.type;
    payload = m_pendingCmd.payload;
    usedPendingCommand = true;
  }
  else
  {
    type = Kwp71BlockType::Empty;
    payload = {};
    usedPendingCommand = false;
  }

  switch (type)
  {
  /** TODO: implement missing block types:
   *  ActivateActuators
   *  EraseTroubleCodes
   *  ReadDACChannel
   *  ReadParamData
   *  RecordParamData
   */
  case Kwp71BlockType::Empty:
  case Kwp71BlockType::ReadTroubleCodes:
  case Kwp71BlockType::RequestID:
  case Kwp71BlockType::RequestSnapshot:
  case Kwp71BlockType::Disconnect:
    status = true;
    m_sendBlockBuf[0] = 0x03;
    break;
  case Kwp71BlockType::ReadParamData:
    status = true;
    m_sendBlockBuf[0] = payload.size() + 0x03;
    memcpy(&m_sendBlockBuf[3], &payload[0], payload.size());
    break;
  case Kwp71BlockType::ReadDACChannel:
    if (payload.size() == 1)
    {
      status = true;
      m_sendBlockBuf[0] = 0x04;
      m_sendBlockBuf[3] = payload[0];
    }
    break;
  case Kwp71BlockType::ReadRAM:
  case Kwp71BlockType::ReadROM:
  case Kwp71BlockType::ReadEEPROM:
    if (payload.size() == 3)
    {
      status = true;
      m_sendBlockBuf[0] = 0x06;
      memcpy(&m_sendBlockBuf[3], &payload[0], payload.size());
    }
    break;
  case Kwp71BlockType::WriteRAM:
  case Kwp71BlockType::WriteEEPROM:
    if (payload.size() == (payload[0] + 3))
    {
      status = true;
      m_sendBlockBuf[0] = payload.size() + 3;
      memcpy(&m_sendBlockBuf[3], &payload[0], payload.size());
    }
  default:
    break;
  }

  if (status)
  {
    m_sendBlockBuf[1] = ++m_lastUsedSeqNum;
    m_sendBlockBuf[2] = (uint8_t)type;
    m_sendBlockBuf[m_sendBlockBuf[0]] = s_endOfBlock;
  }

  return status;
}

/**
 * Sets up the FTDI baud rate, line properties, and latency.
 */
bool Kwp71::setFtdiSerialProperties()
{
  int status = ftdi_set_baudrate(&m_ftdi, m_baudRate);
  if (status == 0)
  {
    status = ftdi_set_line_property(&m_ftdi, BITS_8, STOP_BIT_1, NONE);
    status = status && ftdi_setflowctrl(&m_ftdi, SIO_DISABLE_FLOW_CTRL);
    if (status == 0)
    {
      status = ftdi_set_latency_timer(&m_ftdi, 1);
      if ((status != 0) && m_verbose)
      {
        fprintf(stderr, "Failed to set FTDI latency timer (\"%s\")\n", ftdi_get_error_string(&m_ftdi));
      }
    }
    else if (m_verbose)
    {
      fprintf(stderr, "Failed to set FTDI line properties (\"%s\")\n", ftdi_get_error_string(&m_ftdi));
    }
  }
  else if (m_verbose)
  {
    fprintf(stderr, "Failed to set FTDI baud rate (\"%s\")\n", ftdi_get_error_string(&m_ftdi));
  }
  return (status == 0);
}

/**
 * Reads bytes from the FTDI's receive FIFO until the specified number have been
 * received or a timeout expires.
 */
bool Kwp71::readSerial(uint8_t* buf, int count)
{
  int readCount = 0;
  int status = 0;
  std::chrono::time_point start = std::chrono::steady_clock::now();

  do {
    status = ftdi_read_data(&m_ftdi, buf + readCount, count - readCount);
    readCount += (status > 0) ? status : 0;
    if (readCount < count)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  } while ((readCount < count) &&
           (status != -666) &&
           ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(1000)));
  
  return (readCount == count);
}

/**
 * Writes bytes to the FTDI's transmit FIFO.
 */
bool Kwp71::writeSerial(uint8_t* buf, int count)
{
  int status = 0;
  int numWritten = 0;
  std::chrono::time_point start = std::chrono::steady_clock::now();

  do {
    status = ftdi_write_data(&m_ftdi, buf + numWritten, count - numWritten);
    numWritten += (status > 0) ? status : 0;
  } while ((numWritten < count) &&
           (status != -666) &&
           ((std::chrono::steady_clock::now() - start) < std::chrono::milliseconds(1000)));
  
  return (numWritten == count);
}

/**
 * Performs the 'slow init' sequence by bit-banging the FTDI's transmit line to
 * clock out the provided address (with optional parity) at the requisite 5 baud.
 */
bool Kwp71::slowInit(uint8_t address, int databits, int parity)
{
  bool status = false;
  unsigned char c;
  int f = 0;
  int bitindex = 0;
  int parityCount = 0;

  if (m_verbose)
  {
    printf("Performing slow init (addr %02X, %d data bits, %d parity)...\n", address, databits, parity);
  }

  // Enable bitbang mode with a single output line (TXD)
  if ((f = ftdi_set_bitmode(&m_ftdi, 0x01, BITMODE_BITBANG)) == 0)
  {
    // start bit
    c = 0;
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
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (parity == 1)
    {
      // odd parity
      c = (parityCount % 2) ? 1 : 0;
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else if (parity == 2)
    {
      // even parity
      c = (parityCount % 2) ? 0 : 1;
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // stop bit
    c = 1;
    ftdi_write_data(&m_ftdi, &c, 1);

    if (ftdi_disable_bitbang(&m_ftdi) == 0)
    {
      ftdi_tcioflush(&m_ftdi);
      status = true;
    }
    else if (m_verbose)
    {
      fprintf(stderr, "Failed to disable bitbang mode\n");
    }
  }
  else if (m_verbose)
  {
    fprintf(stderr, "Failed to set bitbang mode\n");
  }

  return status;
}

/**
 * Reads bytes from the serial port until the entire provided sequence
 * is seen (in order) or the allowed time expires. Some (all?) FTDI
 * devices will have a buffer full of bitbang mode status bytes after
 * switching back to serial/FIFO mode, so we need to simply read until
 * we find the desired sequence.
 */
bool Kwp71::waitForByteSequence(const std::vector<uint8_t>& sequence,
                                std::chrono::milliseconds timeout)
{
  uint8_t curByte = 0;
  int matchedBytes = 0;
  const std::chrono::time_point start = std::chrono::steady_clock::now();
  const std::chrono::time_point end = start + timeout;

  while ((matchedBytes < sequence.size()) && (std::chrono::steady_clock::now() < end))
  {
    if (ftdi_read_data(&m_ftdi, &curByte, 1) == 1)
    {
      if (curByte == sequence.at(matchedBytes))
      {
        matchedBytes++;
      }
      else
      {
        matchedBytes = 0;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  return (matchedBytes == sequence.size());
}

/**
 * Reads the three sync/keyword bytes that are transmitted by the ECU
 * immediately after the 5-baud slow init, and replies with the bitwise
 * inversion of the third byte as an acknowledgement.
 */
bool Kwp71::readAckKeywordBytes()
{
  bool status = false;
  const std::vector<uint8_t> kwp({0x55, 0x00, 0x81});
  
  if (waitForByteSequence(kwp, std::chrono::milliseconds(1500)))
  {
    uint8_t echoByte = ~(kwp[2]);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    status = writeSerial(&echoByte, 1) && readSerial(&echoByte, 1);
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
  std::unique_lock<std::mutex> lock(m_commandMutex);

  m_pendingCmd.type = Kwp71BlockType::RequestID;
  m_pendingCmd.payload = std::vector<uint8_t>();
  m_commandIsPending = true;

  // wait until the response from this command has been completely received
  {
    std::unique_lock<std::mutex> responseLock(m_responseMutex);
    m_responseCondVar.wait(responseLock);
  }

  if (m_responseReadSuccess)
  {
    idResponse = m_responseStringData;
    m_responseStringData.clear();
  }

  return m_responseReadSuccess;
}

bool Kwp71::isValidCommandFromTester(Kwp71BlockType type) const
{
  switch (type)
  {
  case Kwp71BlockType::RequestID:
  case Kwp71BlockType::ReadRAM:
  case Kwp71BlockType::WriteRAM:
  case Kwp71BlockType::ReadROM:
  case Kwp71BlockType::ActivateActuators:
  case Kwp71BlockType::EraseTroubleCodes:
  case Kwp71BlockType::ReadTroubleCodes:
  case Kwp71BlockType::ReadDACChannel:
  case Kwp71BlockType::ReadParamData:
  case Kwp71BlockType::RecordParamData:
  case Kwp71BlockType::RequestSnapshot:
  case Kwp71BlockType::ReadEEPROM:
  case Kwp71BlockType::WriteEEPROM:
    return true;
  default:
    return false;
  }
}

/**
 * Queues a command to be sent to the ECU at the next opportunity, and waits
 * to receive the response.
 */
bool Kwp71::sendCommand(Kwp71Command cmd, std::vector<uint8_t>& response)
{
  if (!isValidCommandFromTester(cmd.type))
  {
    return false;
  }

  // wait until any previous command has been fully processed
  std::unique_lock<std::mutex> lock(m_commandMutex);

  m_pendingCmd = cmd;
  m_commandIsPending = true;

  // wait until the response from this command has been completely received
  {
    std::unique_lock<std::mutex> responseLock(m_responseMutex);
    m_responseCondVar.wait(responseLock);
  }

  if (m_responseReadSuccess)
  {
    response = m_responseBinaryData;
    m_responseBinaryData.clear();
  }

  return m_responseReadSuccess;
}

bool Kwp71::readRAM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;

  if (numBytes <= (UINT8_MAX - 3))
  {
    Kwp71Command cmd;
    cmd.type = Kwp71BlockType::ReadRAM;
    cmd.payload = std::vector<uint8_t>({
      numBytes,
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    status = sendCommand(cmd, data);
  }
  return status;
}

bool Kwp71::readROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;

  if (numBytes <= (UINT8_MAX - 3))
  {
    Kwp71Command cmd;
    cmd.type = Kwp71BlockType::ReadROM;
    cmd.payload = std::vector<uint8_t>({
      numBytes,
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    status = sendCommand(cmd, data);
  }
  return status;
}

bool Kwp71::readEEPROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;

  if (numBytes <= (UINT8_MAX - 3))
  {
    Kwp71Command cmd;
    cmd.type = Kwp71BlockType::ReadEEPROM;
    cmd.payload = std::vector<uint8_t>({
      numBytes,
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    status = sendCommand(cmd, data);
  }
  return status;
}

bool Kwp71::writeRAM(uint16_t addr, const std::vector<uint8_t>& data)
{
  bool status = false;

  // Limit the maximum write payload to the size of the remaining
  // space in a single block (after accounting for the header/trailer).
  if (data.size() <= (UINT8_MAX - 6)) // 249 bytes max
  {
    Kwp71Command cmd;
    cmd.type = Kwp71BlockType::WriteRAM;
    cmd.payload = std::vector<uint8_t>(
    {
      static_cast<uint8_t>(data.size()),
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    cmd.payload.insert(cmd.payload.end(), data.begin(), data.end());

    std::vector<uint8_t> response;
    status = sendCommand(cmd, response);
  }

  return status;
}

bool Kwp71::writeEEPROM(uint16_t addr, const std::vector<uint8_t>& data)
{
  bool status = false;

  // Limit the maximum write payload to the size of the remaining
  // space in a single block (after accounting for the header/trailer).
  if (data.size() <= (UINT8_MAX - 6)) // 249 bytes max
  {
    Kwp71Command cmd;
    cmd.type = Kwp71BlockType::WriteRAM;
    cmd.payload = std::vector<uint8_t>(
    {
      static_cast<uint8_t>(data.size()),
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    cmd.payload.insert(cmd.payload.end(), data.begin(), data.end());

    std::vector<uint8_t> response;
    status = sendCommand(cmd, response);
  }

  return status;
}

/**
 * Parses the data in the received block buffer and takes the appropriate
 * action to package and return the data.
 */
void Kwp71::processReceivedBlock()
{
  m_lastReceivedBlockType = (Kwp71BlockType)(m_recvBlockBuf[2]);
  m_lastUsedSeqNum = m_recvBlockBuf[1];
  const uint8_t payloadLen = m_recvBlockBuf[0] - 3;

  switch (m_lastReceivedBlockType)
  {
  // TODO: what action is required for these two block types?
  case Kwp71BlockType::ParamRecordConf:
  case Kwp71BlockType::Snapshot:
    break;
  case Kwp71BlockType::ASCIIString:
    m_responseStringData.push_back(std::string(m_recvBlockBuf[3], payloadLen));
    break;
  case Kwp71BlockType::DACValue:
  case Kwp71BlockType::BinaryData:
  case Kwp71BlockType::RAMContent:
  case Kwp71BlockType::ROMContent:
  case Kwp71BlockType::EEPROMContent:
  case Kwp71BlockType::ParametricData:
    // capture just the payload data of the block (i.e. the bytes
    // *after* the length, seq. num, and block type, and *before*
    // the 0x03 end-of-block marker
    m_responseBinaryData.insert(m_responseBinaryData.end(),
                                &m_recvBlockBuf[3],
                                &m_recvBlockBuf[3 + payloadLen]);
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
 * Any other commands are sent, one at a time, and the responses is
 * collected before the next command is sent.
 */
void Kwp71::commLoop()
{
  while (!m_shutdown)
  {
    // loop until the initialization sequence has completed,
    // re-attempting the 5-baud slow init if necessary
    while (!m_connectionActive && !m_shutdown)
    {
      if (slowInit(m_ecuAddr, m_initDataBits, m_initParity) &&
          setFtdiSerialProperties())
      {
        m_connectionActive = readAckKeywordBytes();
      }
    }

    // The ECU apparently sends its ID info unsolicited next
    do {
      if (recvBlock())
      {
        // ACK until the ECU sends its first empty block (after the ID info)
        if (m_lastReceivedBlockType != Kwp71BlockType::Empty)
        {
          sendBlock();
        }
      }
      else
      {
        m_connectionActive = false;
      }
    } while ((m_lastReceivedBlockType != Kwp71BlockType::Empty) && !m_shutdown);

    // continue taking turns with the ECU, sending one block per turn
    while (m_connectionActive && !m_shutdown)
    {
      sendBlock();

      if (recvBlock())
      {
        // if the last block received from the ECU was an empty/ack,
        // then we can send a command of our own (if we have one available)
        if (m_lastReceivedBlockType == Kwp71BlockType::Empty)
        {
          if (m_waitingForReply)
          {
            m_waitingForReply = false;
            m_responseReadSuccess = true;
            m_responseCondVar.notify_one();
          }
        }
        else if (m_lastReceivedBlockType == Kwp71BlockType::NACK)
        {
          if (m_waitingForReply)
          {
            m_waitingForReply = false;
            m_responseReadSuccess = false;
            m_responseCondVar.notify_one();
          }
        }
      }
      else
      {
        if (m_waitingForReply)
        {
          m_waitingForReply = false;
          m_responseReadSuccess = false;
          m_responseCondVar.notify_one();
        }
        if (m_verbose)
        {
          fprintf(stderr, "ECU didn't respond during its turn\n");
        }
        m_connectionActive = false;
      }
    }

    // Since we believe that the connection has failed, delay for slightly
    // longer than the maximum time allowed by the spec between responses.
    // This will ensure that the ECU also sees the connection as having
    // been terminated. A re-connection (with slow init) will not work unless
    // we're on the same page as the ECU.
    std::this_thread::sleep_for(std::chrono::milliseconds(255));
  }
}

int Kwp71::getFtdiDeviceInfo(ftdi_device_list* list, int listCount, std::vector<FtdiDeviceInfo>& deviceInfo)
{
  constexpr int usbStrLen = 256;
  char manufacturer[usbStrLen];
  char description[usbStrLen];
  char serial[usbStrLen];
  int index = 0;
  int count = 0;

  while (list && (index < listCount))
  {
    if (ftdi_usb_get_strings(&m_ftdi, list->dev,
                             manufacturer, usbStrLen,
                             description, usbStrLen,
                             NULL, 0) == 0)
    {
      const uint8_t bus_num = libusb_get_bus_number(list->dev);
      const uint8_t device_addr = libusb_get_device_address(list->dev);
      deviceInfo.emplace_back(bus_num, device_addr, manufacturer, description, "");
      count++;
    }
    index++;
    list = list->next;
  }

  return count;
}

std::vector<FtdiDeviceInfo> Kwp71::enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extraPids)
{
  ftdi_device_list* list;
  int count = 0;
  std::vector<FtdiDeviceInfo> deviceInfo;

  // First call used VID:PID of 0:0 to search for all standard VID:PID
  // combinations known to libftdi.
  count = ftdi_usb_find_all(&m_ftdi, &list, 0, 0);
  getFtdiDeviceInfo(list, count, deviceInfo);
  ftdi_list_free(&list);

  // Combine a list of any extra known VID/PID pairs with the list provided
  // by the caller (if any).
  std::set<std::pair<uint16_t,uint16_t>> thirdPartyPids =
  {
    { 0x0403, 0xfa20 } // Ross-Tech interface
  };
  thirdPartyPids.insert(extraPids.begin(), extraPids.end());

  for (auto vidPidPair : thirdPartyPids)
  {
    count = ftdi_usb_find_all(&m_ftdi, &list, vidPidPair.first, vidPidPair.second);
    getFtdiDeviceInfo(list, count, deviceInfo);
    ftdi_list_free(&list);
  }

  return deviceInfo;
}

