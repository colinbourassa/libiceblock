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

Kwp71::Kwp71(bool verbose) : m_verbose(verbose)
{
  ftdi_init(&m_ftdi);
}

void Kwp71::setProtocolVariant(Kwp71Variant variant)
{
  if (variant == Kwp71Variant::Standard)
  {
    m_baudRate = 9600;
    m_initDataBits = 8;
    m_initParity = 0;
    m_timeBeforeReconnectionMs = 260;
    m_bytesEchoedDuringBlockReceipt = true;
    m_lastBlockByteIsChecksum = false;
    m_useSequenceNums = true;
    m_isoKeywordIndexToEcho = 2;
    m_isoKeywordEchoIsInverted = true;
    m_isoKeywordNumBytes = 3;
  }
  else if (variant == Kwp71Variant::FIAT9141)
  {
    m_baudRate = 4800;
    m_initDataBits = 7;
    m_initParity = 1;
    m_timeBeforeReconnectionMs = 2050;
    m_bytesEchoedDuringBlockReceipt = false;
    m_lastBlockByteIsChecksum = true;
    m_useSequenceNums = false;
    m_isoKeywordIndexToEcho = 2;
    m_isoKeywordEchoIsInverted = true;
    m_isoKeywordNumBytes = 6;
  }
}

/**
 * Attempts a connection using provided protocol parameters, which may be
 * different than those used by official/standard implementations of the
 * KWP71 protocol. Performs the slow init sequence using the provided ECU
 * address and then opens the serial port device, using it to read the ECU's
 * keyword bytes.
 * Returns true if all of this is successful; false otherwise.
 */
bool Kwp71::connect(uint16_t vid, uint16_t pid, uint8_t addr, int& err)
{
  bool status = false;
  std::unique_lock<std::mutex> lock(m_connectMutex);

  m_ecuAddr = addr;
  m_connectionActive = false;
  m_shutdown = false;

  if (m_ifThreadPtr == nullptr)
  {
    if (ftdi_set_interface(&m_ftdi, INTERFACE_A) == 0)
    {
      if ((ftdi_usb_open(&m_ftdi, vid, pid) == 0))
      {
        ftdi_tcioflush(&m_ftdi);
        if (slowInit(m_ecuAddr, m_initDataBits, m_initParity))
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
              ftdi_usb_close(&m_ftdi);
            }
          }
          else
          {
            // setFtdiSerialProperties() failed
            err = -2;
            ftdi_usb_close(&m_ftdi);
          }
        }
        else
        {
          // slowInit() failed
          err = -3;
          ftdi_usb_close(&m_ftdi);
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
 * Sends a KWP-71 block by transmitting it one byte at a time. For certain
 * variants of the protocol, the transmitting side must wait for the inverse of
 * each byte echoed by the receiver before transmitting the next byte (with the
 * exception of the last byte, which is not echoed.)
 * Returns true if the entire block was sent successfuly; false otherwise.
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
    // it appears in the Rx buffer (due to the loopback).
    if (writeSerial(&m_sendBlockBuf[index], 1) && readSerial(&loopback, 1))
    {
      if (m_verbose)
      {
        printf("%02X ", m_sendBlockBuf[index]);
      }

      // If the current protocol variant calls for it, read the bitwise
      // inversion of the last sent byte as it is echoed by the receiver.
      if (m_bytesEchoedDuringBlockReceipt && (index < m_sendBlockBuf[0]))
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
 * the first byte. In certain variants of the protocol, each byte (except the
 * last) is positively acknowledged with its bitwise inversion. Returns true if
 * all the expected bytes are received, false otherwise.
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

    if (m_bytesEchoedDuringBlockReceipt)
    {
      // ack the pkt length byte
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      ack = ~(m_recvBlockBuf[0]);
      status = writeSerial(&ack, 1) && readSerial(&loopback, 1);
    }

    while (status && (index <= m_recvBlockBuf[0]))
    {
      if (readSerial(&m_recvBlockBuf[index], 1))
      {
        if (m_verbose)
        {
          printf("%02X ", m_recvBlockBuf[index]);
        }

        // every byte except the last in the block is ack'd
        if (m_bytesEchoedDuringBlockReceipt && (index < m_recvBlockBuf[0]))
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
  else
  {
    status = false;
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

  const uint8_t sizeOfEmptyBlock = m_useSequenceNums ? 3 : 2;
  const uint8_t payloadStartPos = m_useSequenceNums ? 3 : 2;

  switch (type)
  {
  /** TODO: implement missing block types:
   *  ActivateActuators
   *  EraseTroubleCodes
   *  ReadADCChannel
   *  ReadParamData
   *  RecordParamData
   */
  case Kwp71BlockType::Empty:
  case Kwp71BlockType::ReadTroubleCodes:
  case Kwp71BlockType::RequestID:
  case Kwp71BlockType::RequestSnapshot:
  case Kwp71BlockType::Disconnect:
    status = true;
    m_sendBlockBuf[0] = sizeOfEmptyBlock;
    break;
  case Kwp71BlockType::ReadParamData:
    status = true;
    m_sendBlockBuf[0] = sizeOfEmptyBlock + payload.size();
    memcpy(&m_sendBlockBuf[payloadStartPos], &payload[0], payload.size());
    break;
  case Kwp71BlockType::ReadADCChannel:
    if (payload.size() == 1)
    {
      status = true;
      m_sendBlockBuf[0] = sizeOfEmptyBlock + 1;
      m_sendBlockBuf[payloadStartPos] = payload[0];
    }
    break;
  case Kwp71BlockType::ReadRAM:
  case Kwp71BlockType::ReadROM:
  case Kwp71BlockType::ReadEEPROM:
    if (payload.size() == 3)
    {
      status = true;
      m_sendBlockBuf[0] = sizeOfEmptyBlock + 3;
      memcpy(&m_sendBlockBuf[payloadStartPos], &payload[0], payload.size());
    }
    break;
  case Kwp71BlockType::WriteRAM:
  case Kwp71BlockType::WriteEEPROM:
    if (payload.size() == (payload[0] + 3))
    {
      status = true;
      m_sendBlockBuf[0] = sizeOfEmptyBlock + payload.size();
      memcpy(&m_sendBlockBuf[payloadStartPos], &payload[0], payload.size());
    }
  default:
    break;
  }

  if (status)
  {
    if (m_useSequenceNums)
    {
      m_sendBlockBuf[1] = ++m_lastUsedSeqNum;
      m_sendBlockBuf[2] = (uint8_t)type;
    }
    else
    {
      m_sendBlockBuf[1] = (uint8_t)type;
    }

    if (m_lastBlockByteIsChecksum)
    {
      // compute 8-bit checksum and store in the last byte of the block
      m_sendBlockBuf[m_sendBlockBuf[0]] = m_sendBlockBuf[0];
      for (int i = 1; i < m_sendBlockBuf[0]; i++)
      {
        m_sendBlockBuf[m_sendBlockBuf[0]] += m_sendBlockBuf[i];
      }
    }
    else
    {
      m_sendBlockBuf[m_sendBlockBuf[0]] = s_endOfBlock;
    }
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
 * Receives a sequence of bytes being transmitted by the ECU that starts with
 * the value 0x55. Returns true if the expected number of bytes following 0x55
 * are received before the timeout; false otherwise.
 * Note: Some (all?) FTDI devices will have a buffer full of bitbang mode status
 * bytes after switching back to serial/FIFO mode, so we need to simply read
 * until the sequence starting with 0x55 is found.
 */
bool Kwp71::waitForISOSequence(std::chrono::milliseconds timeout,
                               std::vector<uint8_t>& isoBytes)
{
  uint8_t curByte = 0;
  int matchedBytes = 0;
  isoBytes.clear();
  const std::chrono::time_point start = std::chrono::steady_clock::now();
  const std::chrono::time_point end = start + timeout;

  while ((matchedBytes < m_isoKeywordNumBytes) &&
         (std::chrono::steady_clock::now() < end))
  {
    if (ftdi_read_data(&m_ftdi, &curByte, 1) == 1)
    {
      if (((matchedBytes == 0) && (curByte == 0x55)) ||
          (matchedBytes > 0))
      {
        matchedBytes++;
        isoBytes.push_back(curByte);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (m_verbose)
  {
    printf("Got ISO keyword sequence:");
    for (int i = 0; i < isoBytes.size(); i++)
    {
      printf(" %02X", isoBytes.at(i));
    }
    printf("\n");
  }

  return (matchedBytes == m_isoKeywordNumBytes);
}

/**
 * Reads the ISO sync/keyword bytes that are transmitted by the ECU
 * immediately after the 5-baud slow init, and replies with the bitwise
 * inversion of the appropriate byte as an acknowledgement.
 */
bool Kwp71::readAckKeywordBytes()
{
  bool status = false;
  std::vector<uint8_t> isoBytes;
  
  if (waitForISOSequence(std::chrono::milliseconds(1500), isoBytes))
  {
    // check that the ISO keyword sequence is long enough to
    // contain the index of the byte that we need to echo
    if (isoBytes.size() > m_isoKeywordIndexToEcho)
    {
      uint8_t echoByte = isoBytes.at(m_isoKeywordIndexToEcho);
      if (m_isoKeywordEchoIsInverted)
      {
        echoByte = ~echoByte;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      status = writeSerial(&echoByte, 1) && readSerial(&echoByte, 1);
    }
    else
    {
      fprintf(stderr, "Error: ISO byte index to echo (%d) is not within the size of the ISO sequence (%d).\n", m_isoKeywordIndexToEcho, isoBytes.size());
    }
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
  case Kwp71BlockType::ReadADCChannel:
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
  const uint8_t blockOverhead = m_useSequenceNums ? 3 : 2;

  if (numBytes <= (UINT8_MAX - blockOverhead))
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
  const uint8_t blockOverhead = m_useSequenceNums ? 3 : 2;

  if (numBytes <= (UINT8_MAX - blockOverhead))
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
  const uint8_t blockOverhead = m_useSequenceNums ? 3 : 2;

  if (numBytes <= (UINT8_MAX - blockOverhead))
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
  const uint8_t blockOverhead = m_useSequenceNums ? 3 : 2;

  // Limit the maximum write payload to the size of the remaining
  // space in a single block (after accounting for the header/trailer).
  // This is a max of 249 bytes for the standard variant, or 250 bytes
  // for FIAT9141.
  if (data.size() <= (UINT8_MAX - blockOverhead - 3))
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
  const uint8_t blockOverhead = m_useSequenceNums ? 3 : 2;

  // Limit the maximum write payload to the size of the remaining
  // space in a single block (after accounting for the header/trailer).
  // This is a max of 249 bytes for the standard variant, or 250 bytes
  // for FIAT9141.
  if (data.size() <= (UINT8_MAX - blockOverhead - 3))
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
  int payloadLen = 0;
  int blockPayloadStartPos = 0;

  if (m_useSequenceNums)
  {
    m_lastUsedSeqNum = m_recvBlockBuf[1];
    m_lastReceivedBlockType = (Kwp71BlockType)(m_recvBlockBuf[2]);
    payloadLen = m_recvBlockBuf[0] - 3;
    blockPayloadStartPos = 3;
  }
  else
  {
    m_lastReceivedBlockType = (Kwp71BlockType)(m_recvBlockBuf[1]);
    payloadLen = m_recvBlockBuf[0] - 2;
    blockPayloadStartPos = 2;
  }

  switch (m_lastReceivedBlockType)
  {
  // TODO: what action is required for these two block types?
  case Kwp71BlockType::ParamRecordConf:
  case Kwp71BlockType::Snapshot:
    break;
  case Kwp71BlockType::ASCIIString:
    m_responseStringData.push_back(std::string(m_recvBlockBuf[blockPayloadStartPos], payloadLen));
    break;
  case Kwp71BlockType::ADCValue:
  case Kwp71BlockType::BinaryData:
  case Kwp71BlockType::RAMContent:
  case Kwp71BlockType::ROMContent:
  case Kwp71BlockType::EEPROMContent:
  case Kwp71BlockType::ParametricData:
    // capture just the payload data of the block (i.e. the bytes
    // *after* the length, seq. num (if used), and block title,
    // and *before* the 0x03 end-of-block marker (or checksum)
    m_responseBinaryData.insert(m_responseBinaryData.end(),
                                &m_recvBlockBuf[blockPayloadStartPos],
                                &m_recvBlockBuf[blockPayloadStartPos + payloadLen]);
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
        else if ((m_lastReceivedBlockType == Kwp71BlockType::NACK) ||
                 (m_lastReceivedBlockType == Kwp71BlockType::NotSupported))
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
    std::this_thread::sleep_for(std::chrono::milliseconds(m_timeBeforeReconnectionMs));
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

