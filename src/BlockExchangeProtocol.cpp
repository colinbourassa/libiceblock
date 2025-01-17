#include "BlockExchangeProtocol.h"
#include <chrono>
#include <mutex>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>

/**
 * Construct with the baud rate, line used for sending the slow-init byte, and debug verbosity flag.
 */
BlockExchangeProtocol::BlockExchangeProtocol(int baudRate, LineType initLine, bool verbose) :
  m_baudRate(baudRate),
  m_slowInitLine(initLine)
{
  spdlog::set_level(verbose ? spdlog::level::debug : spdlog::level::off);
  ftdi_init(&m_ftdi);
}

BlockExchangeProtocol::~BlockExchangeProtocol()
{
}

/**
 * Attempts a connection using an FTDI device specified by bus ID and bus
 * address. Performs the slow init sequence using the provided ECU address and
 * then opens the serial port device, using it to read the ECU's keyword bytes.
 * Returns true if all of this is successful; false otherwise.
 */
bool BlockExchangeProtocol::connectByBusAddr(uint8_t bus, uint8_t addr, uint8_t ecuAddr)
{
  bool status = false;
  std::unique_lock<std::mutex> lock(m_connectMutex);

  m_ecuAddr = ecuAddr;
  m_connectionActive = false;
  m_shutdown = false;

  if (m_ifThreadPtr == nullptr)
  {
    if (ftdi_set_interface(&m_ftdi, INTERFACE_A) == 0)
    {
      if ((ftdi_usb_open_bus_addr(&m_ftdi, bus, addr) == 0))
      {
        status = initAndStartCommunication();
      }
      else
      {
        spdlog::error("ftdi_usb_open() failed for bus {:03d} / addr {:03d}", bus, addr);
      }
    }
    else
    {
      spdlog::error("ftdi_set_interface() failed.");
    }
  }

  return status;
}

/**
 * Attempts a connection using an FTDI device specified by vendor ID and product
 * ID. Performs the slow init sequence using the provided ECU address and then
 * opens the serial port device, using it to read the ECU's keyword bytes.
 * Returns true if all of this is successful; false otherwise.
 */
bool BlockExchangeProtocol::connectByDeviceId(uint16_t vendorId, uint16_t productId, uint8_t ecuAddr)
{
  bool status = false;
  std::unique_lock<std::mutex> lock(m_connectMutex);

  m_ecuAddr = ecuAddr;
  m_connectionActive = false;
  m_shutdown = false;

  if (m_ifThreadPtr == nullptr)
  {
    if (ftdi_set_interface(&m_ftdi, INTERFACE_A) == 0)
    {
      if ((ftdi_usb_open(&m_ftdi, vendorId, productId) == 0))
      {
        status = initAndStartCommunication();
      }
      else
      {
        spdlog::error("ftdi_usb_open() failed for VID {:04x} / PID {:04x}", vendorId, productId);
      }
    }
    else
    {
      spdlog::error("ftdi_set_interface() failed.");
    }
  }

  return status;
}

/**
 * Performs the slow init sequence, sets up the FTDI device for the normal
 * communication, and reads/acknowledges the keyword byte sequence.
 */
bool BlockExchangeProtocol::initAndStartCommunication()
{
  bool status = false;
  ftdi_tcioflush(&m_ftdi);
  if (slowInit(m_ecuAddr, initDataBits(), initParity()))
  {
    if (setFtdiSerialProperties())
    {
      if (readAckKeywordBytes())
      {
        m_connectionActive = true;
        m_ifThreadPtr = std::make_unique<std::thread>(threadEntry, this);
        status = true;
      }
      else
      {
        spdlog::error("Failed to receive/acknowledge keyword byte sequence.");
      }
    }
    else
    {
      spdlog::error("Failed to set FTDI serial properties after slow init.");
    }
  }
  else
  {
    spdlog::error("Error during slow init sequence.");
  }

  return status;
}

/**
 * Shuts down the connection and waits for the connection thread to
 * finish.
 */
void BlockExchangeProtocol::disconnect()
{
  std::unique_lock<std::mutex> lock(m_connectMutex);
  if (m_ifThreadPtr && m_ifThreadPtr->joinable())
  {
    m_shutdown = true;
    m_ifThreadPtr->join();
  }
  m_ifThreadPtr = nullptr;
  ftdi_usb_close(&m_ftdi);
  ftdi_deinit(&m_ftdi);
}

bool BlockExchangeProtocol::isConnectionActive() const
{
  return m_connectionActive;
}

/**
 * Populates the transmit buffer with all the appropriate parts of
 * a protocol block.
 */
bool BlockExchangeProtocol::populateBlock(bool& usedPendingCmd)
{
  bool status = false;
  uint8_t blockTitle = 0;
  std::vector<uint8_t> payload;

  if (lastReceivedBlockWasEmpty() && m_commandIsPending)
  {
    blockTitle = m_pendingCmd.type;
    payload = m_pendingCmd.payload;
    usedPendingCmd = true;
  }
  else
  {
    blockTitle = blockTitleForEmptyAck();
    usedPendingCmd = false;
  }

  if (checkValidityOfBlockAndPayload(blockTitle, payload))
  {
    setBlockSizePrefix(payload.size());
    setBlockSequenceNum();
    setBlockTitle(blockTitle);
    setBlockPayload(payload);
    setBlockTrailer();
    status = true;
  }

  return status;
}

/**
 * Sets the first byte in the block to reflect the size of the block (in bytes).
 * Depending on the protocol, the count may include (a) the block size byte
 * itself, (b) a sequence number byte, (c) the payload, and (d) the trailer
 * (which may be a fixed value, an 8-bit checksum, or a 16-bit checksum).
 *
 * Note: The GM-developed KW82 protocol actually includes the count byte itself
 * in the total byte count. Support for the KW82 protocol is not currently
 * planned, but if it were added at some point in the future, this routine would
 * need to be adjusted.
 */
void BlockExchangeProtocol::setBlockSizePrefix(int payloadSize)
{
  const uint8_t seqNumSize = (useSequenceNums() ? 1 : 0);
  const uint8_t titleSize = 1;
  const uint8_t trailerSize = (trailerType() == BlockTrailerType::Checksum16Bit) ? 2 : 1;

  m_sendBlockBuf[0] = seqNumSize + titleSize + payloadSize + trailerSize;
}

/**
 * Sets the appropriate byte in the transmit buffer to the block sequence
 * number. If the protocol does not use sequence numbers, this function has no
 * effect.
 */
void BlockExchangeProtocol::setBlockSequenceNum()
{
  if (useSequenceNums())
  {
    m_sendBlockBuf[1] = ++m_lastUsedSeqNum;
  }
}

/**
 * Sets the provided block title in the transmit buffer at the appropriate
 * location (depending on whether the protocol reserves a byte for sequence
 * numbers.)
 */
void BlockExchangeProtocol::setBlockTitle(uint8_t title)
{
  if (useSequenceNums())
  {
    m_sendBlockBuf[2] = title;
  }
  else
  {
    m_sendBlockBuf[1] = title;
  }
}

/**
 * Copies the provided payload bytes to the appropriate location in the
 * transmit buffer.
 */
void BlockExchangeProtocol::setBlockPayload(const std::vector<uint8_t>& payload)
{
  const uint8_t payloadStartPos = useSequenceNums() ? 3 : 2;
  memcpy(&m_sendBlockBuf[payloadStartPos], &payload[0], payload.size());
}

/**
 * Sets the trailer byte(s) in the transmit buffer depending on the type of
 * trailer used in in the protocol implementation.
 */
void BlockExchangeProtocol::setBlockTrailer()
{
  if (trailerType() == BlockTrailerType::Fixed03)
  {
    m_sendBlockBuf[m_sendBlockBuf[0]] = 0x03;
  }
  else if (trailerType() == BlockTrailerType::Checksum8Bit)
  {
    // compute 8-bit checksum and store in the last byte of the block
    m_sendBlockBuf[m_sendBlockBuf[0]] = m_sendBlockBuf[0];
    for (int i = 1; i < m_sendBlockBuf[0]; i++)
    {
      m_sendBlockBuf[m_sendBlockBuf[0]] += m_sendBlockBuf[i];
    }
  }
  else if (trailerType() == BlockTrailerType::Checksum16Bit)
  {
    uint16_t checksum16 = 0;
    for (int i = 0; i < m_sendBlockBuf[0]; i++)
    {
      checksum16 += m_sendBlockBuf[i];
    }
    m_sendBlockBuf[m_sendBlockBuf[0] - 1] = static_cast<uint8_t>(checksum16 >> 8);
    m_sendBlockBuf[m_sendBlockBuf[0]] = static_cast<uint8_t>(checksum16 & 0xff);
  }
}

/**
 * Sends a block by transmitting it one byte at a time. For certain protocols,
 * the transmitting side must wait for the inverse of each byte echoed by the
 * receiver before transmitting the next byte (with the exception of the last
 * byte, which is not echoed.)
 * Returns true if the entire block was sent successfuly; false otherwise.
 */
bool BlockExchangeProtocol::sendBlock(bool sendBufIsPrepopulated)
{
  bool status = true;
  uint8_t index = 0;
  uint8_t loopback = 0;
  uint8_t ack = 0;
  uint8_t ackCompare = 0;
  bool usedPendingCommand = false;
  const bool isDebugLogging = spdlog::should_log(spdlog::level::debug);
  std::string msg("send: ");

  if (!sendBufIsPrepopulated)
  {
    populateBlock(usedPendingCommand);
  }

  while (status && (index <= m_sendBlockBuf[0]))
  {
    // Transmit the block one byte at a time, reading back the same byte as
    // it appears in the Rx buffer (due to the loopback).
    if (writeSerial(&m_sendBlockBuf[index], 1) && readSerial(&loopback, 1))
    {
      if (isDebugLogging)
      {
        msg += fmt::format("{:02X} ", m_sendBlockBuf[index]);
      }

      // If the current protocol variant calls for it, read the bitwise
      // inversion of the last sent byte as it is echoed by the receiver.
      if (bytesEchoedDuringBlockReceipt() && (index < m_sendBlockBuf[0]))
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

  spdlog::debug(msg);

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
bool BlockExchangeProtocol::recvBlock(std::chrono::milliseconds timeout)
{
  bool status = true;
  uint16_t index = 1;
  uint8_t loopback = 0;
  uint8_t ack = 0;
  const bool isDebugLogging = spdlog::should_log(spdlog::level::debug);
  std::string msg("recv: ");

  if (readSerial(&m_recvBlockBuf[0], 1, timeout))
  {
    if (isDebugLogging)
    {
      msg += fmt::format("{:02X} ", m_recvBlockBuf[0]);
    }

    if (bytesEchoedDuringBlockReceipt())
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
        if (isDebugLogging)
        {
          msg += fmt::format("{:02X} ", m_recvBlockBuf[index]);
        }

        // every byte except the last in the block is ack'd
        if (bytesEchoedDuringBlockReceipt() && (index < m_recvBlockBuf[0]))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          ack = ~(m_recvBlockBuf[index]);
          status = writeSerial(&ack, 1) && readSerial(&loopback, 1);
        }
        index++;
      }
      else
      {
        spdlog::error("Timed out while receiving block data!");
        status = false;
      }
    }

    spdlog::debug(msg);
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
 * Sets up the FTDI baud rate, line properties, and latency.
 */
bool BlockExchangeProtocol::setFtdiSerialProperties()
{
  int status = ftdi_set_baudrate(&m_ftdi, m_baudRate);
  if (status == 0)
  {
    status = ftdi_set_line_property(&m_ftdi, BITS_8, STOP_BIT_1, NONE);
    status = status && ftdi_setflowctrl(&m_ftdi, SIO_DISABLE_FLOW_CTRL);
    if (status == 0)
    {
      status = ftdi_set_latency_timer(&m_ftdi, 1);
      if (status != 0)
      {
        spdlog::error("Failed to set FTDI latency timer ('{}')", ftdi_get_error_string(&m_ftdi));
      }
    }
    else
    {
      spdlog::error("Failed to set FTDI line properties ('{}')", ftdi_get_error_string(&m_ftdi));
    }
  }
  else
  {
    spdlog::error("Failed to set FTDI baud rate ('{}')", ftdi_get_error_string(&m_ftdi));
  }
  return (status == 0);
}

/**
 * Reads bytes from the FTDI's receive FIFO until the specified number have been
 * received or a timeout expires.
 */
bool BlockExchangeProtocol::readSerial(uint8_t* buf,
                                       int count,
                                       std::chrono::milliseconds timeout)
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
           ((std::chrono::steady_clock::now() - start) < timeout));
  
  return (readCount == count);
}

/**
 * Writes bytes to the FTDI's transmit FIFO.
 */
bool BlockExchangeProtocol::writeSerial(uint8_t* buf, int count)
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
bool BlockExchangeProtocol::slowInit(uint8_t address, int databits, int parity)
{
  bool status = false;
  unsigned char c;
  int f = 0;
  int bitindex = 0;
  int parityCount = 0;

  spdlog::debug("Performing slow init (addr {:02X}, {} data bits, {} parity)...", address, databits, parity);

  // The bitbang mask *may* need to change to use the L-line (vs K-line) for
  // slow init, although this is entirely dependent on the design of the FTDI
  // cable being used. Testing with a Ross-Tech HEX-USB cable (0403:fa20,
  // purchased circa 2005) showed that a mask value of 0x01 caused activity on
  // both the K-line and L-line, suggesting that they are connected internally.
  // I'm leaving this K/L mask switching mechanism in place for now, and just
  // setting the mask value to the same (0x01) in either case.
  const unsigned char mask = (m_slowInitLine == LineType::KLine) ? 0x01 : 0x01;

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
      c = ((1 << bitindex) & address) ? mask : 0;
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
      c = (parityCount % 2) ? mask : 0;
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    else if (parity == 2)
    {
      // even parity
      c = (parityCount % 2) ? 0 : mask;
      ftdi_write_data(&m_ftdi, &c, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // stop bit
    c = mask;
    ftdi_write_data(&m_ftdi, &c, 1);

    if (ftdi_disable_bitbang(&m_ftdi) == 0)
    {
      ftdi_tcioflush(&m_ftdi);
      status = true;
    }
    else
    {
      spdlog::error("Failed to disable bitbang mode");
    }
  }
  else
  {
    spdlog::error("Failed to set bitbang mode");
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
bool BlockExchangeProtocol::waitForISOSequence(std::chrono::milliseconds timeout,
                                               std::vector<uint8_t>& isoBytes)
{
  uint8_t curByte = 0;
  int matchedBytes = 0;
  isoBytes.clear();
  const std::chrono::time_point start = std::chrono::steady_clock::now();
  const std::chrono::time_point end = start + timeout;
  const bool isDebugLogging = spdlog::should_log(spdlog::level::debug);

  while ((matchedBytes < isoKeywordNumBytes()) &&
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

  if (isDebugLogging && isoBytes.size())
  {
    std::string msg("Got ISO keyword sequence:");
    for (auto isoByte : isoBytes)
    {
      msg += fmt::format(" {:02X}", isoByte);
    }
    spdlog::debug(msg);
  }

  return (matchedBytes == isoKeywordNumBytes());
}

/**
 * Reads the ISO sync/keyword bytes that are transmitted by the ECU
 * immediately after the 5-baud slow init, and replies with the bitwise
 * inversion of the appropriate byte as an acknowledgement.
 */
bool BlockExchangeProtocol::readAckKeywordBytes()
{
  std::vector<uint8_t> isoBytes;
  bool status = waitForISOSequence(std::chrono::milliseconds(1500), isoBytes);

  // If the keyword sequence was successfully received AND there is a valid
  // index of the byte to echo...
  if (status && (isoKeywordIndexToEcho() >= 0))
  {
    // check that the ISO keyword sequence is long enough to
    // contain the index of the byte that we need to echo
    if (isoBytes.size() > isoKeywordIndexToEcho())
    {
      uint8_t echoByte = isoBytes.at(isoKeywordIndexToEcho());
      if (isoKeywordEchoIsInverted())
      {
        echoByte = ~echoByte;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      status = writeSerial(&echoByte, 1) && readSerial(&echoByte, 1);
    }
    else
    {
      spdlog::error("ISO byte index to echo ({}) is not within the size of the ISO sequence ({})",
        isoKeywordIndexToEcho(), isoBytes.size());
    }
  }

  return status;
}

/**
 * Sends the provided command the ECU at the next opportunity (i.e. when the ECU
 * is finished sending data from any previous command), and waits to receive the
 * response.
 */
bool BlockExchangeProtocol::sendCommand(CommandBlock cmd, std::vector<uint8_t>& response)
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
    response = m_lastReceivedPayload;
    m_lastReceivedPayload.clear();
  }

  return m_responseReadSuccess;
}

/**
 * Captures the data from the payload of the block that was most
 * recently received.
 */
void BlockExchangeProtocol::processReceivedBlock()
{
  int payloadLen = 0;
  int blockPayloadStartPos = 0;

  if (useSequenceNums())
  {
    m_lastUsedSeqNum = m_recvBlockBuf[1];
    m_lastReceivedBlockTitle = m_recvBlockBuf[2];
    payloadLen = m_recvBlockBuf[0] - 3;
    blockPayloadStartPos = 3;
  }
  else
  {
    m_lastReceivedBlockTitle = m_recvBlockBuf[1];
    payloadLen = m_recvBlockBuf[0] - 2;
    blockPayloadStartPos = 2;
  }

  m_lastReceivedPayload.insert(m_lastReceivedPayload.end(),
                               &m_recvBlockBuf[blockPayloadStartPos],
                               &m_recvBlockBuf[blockPayloadStartPos + payloadLen]);
}

const std::vector<std::vector<uint8_t>>& BlockExchangeProtocol::getIDInfoStrings() const
{
  return m_idInfoStrings;
}

/**
 * Thread entry point, which calls the main communication loop function.
 */
void BlockExchangeProtocol::threadEntry(BlockExchangeProtocol* iface)
{
  iface->commLoop();
}

/**
 * Loop that maintains a connection with the ECU. When no particular
 * command is queued, the empty/ACK (09) command is sent as a keepalive.
 * Any other commands are sent, one at a time, and the response is
 * collected before the next command is sent.
 */
void BlockExchangeProtocol::commLoop()
{
  while (!m_shutdown)
  {
    // loop until the initialization sequence has completed,
    // re-attempting the 5-baud slow init if necessary
    while (!m_connectionActive && !m_shutdown)
    {
      if (slowInit(m_ecuAddr, initDataBits(), initParity()) &&
          setFtdiSerialProperties())
      {
        m_connectionActive = readAckKeywordBytes();
      }
    }

    // Do any special one-off block exchance sequence that happens
    // immediately after receipt/acknowledgement of the keyword sequence
    m_connectionActive = doPostKeywordSequence();

    // Clear the buffer containing the last-received payload so that any initial
    // ECU ID data doesn't inadvertently get sent along with the response to any
    // subsequent requests for memory content, fault codes, etc.
    m_lastReceivedPayload.clear();

    // continue taking turns with the ECU, sending one block per turn
    while (m_connectionActive && !m_shutdown)
    {
      sendBlock();

      if (recvBlock())
      {
        // if the last block received from the ECU was an empty/ack,
        // then we can send a command of our own (if we have one available)
        if (lastReceivedBlockWasEmpty() && m_waitingForReply)
        {
          m_waitingForReply = false;
          m_responseReadSuccess = true;
          m_responseCondVar.notify_one();
        }
        else if (lastReceivedBlockWasNack() && m_waitingForReply)
        {
          m_waitingForReply = false;
          m_responseReadSuccess = false;
          m_responseCondVar.notify_one();
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
        spdlog::error("ECU didn't respond during its turn");
        m_connectionActive = false;
      }
    }

    if (!m_shutdown)
    {
      // Since we believe that the connection has failed, delay for slightly
      // longer than the maximum time allowed by the spec between responses.
      // This will ensure that the ECU also sees the connection as having
      // been terminated. A re-connection (with slow init) will not work unless
      // we're on the same page as the ECU.
      std::this_thread::sleep_for(std::chrono::milliseconds(timeBeforeReconnectMs()));
    }
  }
}

