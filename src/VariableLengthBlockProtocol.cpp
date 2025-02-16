#include "VariableLengthBlockProtocol.h"
#include <spdlog/spdlog.h>

VariableLengthBlockProtocol::VariableLengthBlockProtocol(int baudRate, LineType initLine, bool verbose) :
  BlockExchangeProtocol(baudRate, initLine, verbose)
{
}

VariableLengthBlockProtocol::~VariableLengthBlockProtocol()
{
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
void VariableLengthBlockProtocol::setBlockSizePrefix(int payloadSize)
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
void VariableLengthBlockProtocol::setBlockSequenceNum()
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
void VariableLengthBlockProtocol::setBlockTitle(uint8_t title)
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
void VariableLengthBlockProtocol::setBlockPayload(const std::vector<uint8_t>& payload)
{
  const uint8_t payloadStartPos = useSequenceNums() ? 3 : 2;
  memcpy(&m_sendBlockBuf[payloadStartPos], &payload[0], payload.size());
}

bool VariableLengthBlockProtocol::setBlockSections(uint8_t blockTitle, std::vector<uint8_t>& payload)
{
  // TODO: We may want to make the return value reflect the success
  // of checkValidityOfBlockAndPayload().
  setBlockSizePrefix(payload.size());
  setBlockSequenceNum();
  setBlockTitle(blockTitle);
  setBlockPayload(payload);
  setBlockTrailer();
  return true;
}

/**
 * Sends a block by transmitting it one byte at a time. For certain protocols,
 * the transmitting side must wait for the inverse of each byte echoed by the
 * receiver before transmitting the next byte (with the exception of the last
 * byte, which is not echoed.)
 * Returns true if the entire block was sent successfuly; false otherwise.
 */
bool VariableLengthBlockProtocol::sendBlock(bool sendBufIsPrepopulated)
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
bool VariableLengthBlockProtocol::recvBlock(std::chrono::milliseconds timeout)
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
  }
  else
  {
    status = false;
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

  if (status)
  {
    spdlog::debug(msg);
    processReceivedBlock();
  }

  return status;
}

/**
 * Captures the data from the payload of the block that was most
 * recently received.
 */
void VariableLengthBlockProtocol::processReceivedBlock()
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
