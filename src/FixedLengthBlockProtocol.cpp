#include "FixedLengthBlockProtocol.h"
#include <spdlog/spdlog.h>

FixedLengthBlockProtocol::FixedLengthBlockProtocol(int baudRate, LineType initLine, bool verbose) :
  BlockExchangeProtocol(baudRate, initLine, verbose)
{
}

FixedLengthBlockProtocol::~FixedLengthBlockProtocol()
{
}

/**
 * Sets the provided block title in the transmit buffer at the first location.
 */
void FixedLengthBlockProtocol::setBlockTitle(uint8_t title)
{
  m_sendBlockBuf[0] = title;
}

/**
 * Copies the provided payload bytes to the appropriate location in the
 * transmit buffer.
 */
void FixedLengthBlockProtocol::setBlockPayload(const std::vector<uint8_t>& payload)
{
  // TODO: add check to verify that the payload is not too large to fit
  // within the fixed block length along with the overhead title/trailer bytes
  const uint8_t payloadStartPos = 1;
  memcpy(&m_sendBlockBuf[payloadStartPos], &payload[0], payload.size());
}

/**
 * Sets the appropriate values in each of the sections of the block to transmit.
 * Because this is a fixed-length block protocol, there is no length prefix byte
 * nor is there a sequence number byte.
 */
bool FixedLengthBlockProtocol::setBlockSections(uint8_t blockTitle, std::vector<uint8_t>& payload)
{
  bool status = false;
  if (checkValidityOfBlockAndPayload(blockTitle, payload))
  {
    setBlockTitle(blockTitle);
    setBlockPayload(payload);
    setBlockTrailer();
    status = true;
  }
  return status;
}

/**
 * Sends a block by transmitting it one byte at a time. For certain protocols,
 * the transmitting side must wait for the inverse of each byte echoed by the
 * receiver before transmitting the next byte (with the exception of the last
 * byte, which is not echoed.)
 * Returns true if the entire block was sent successfuly; false otherwise.
 */
bool FixedLengthBlockProtocol::sendBlock(bool sendBufIsPrepopulated)
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

  while (status && (index < blockLength()))
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
      if (bytesEchoedDuringBlockReceipt() && (index < (blockLength() - 1)))
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
bool FixedLengthBlockProtocol::recvBlock(std::chrono::milliseconds timeout)
{
  bool status = true;
  uint16_t index = 0;
  uint8_t loopback = 0;
  uint8_t ack = 0;
  const bool isDebugLogging = spdlog::should_log(spdlog::level::debug);
  std::string msg("recv: ");

  while (status && (index < blockLength()))
  {
    if (readSerial(&m_recvBlockBuf[index], 1))
    {
      if (isDebugLogging)
      {
        msg += fmt::format("{:02X} ", m_recvBlockBuf[index]);
      }

      // every byte except the last in the block is ack'd
      if (bytesEchoedDuringBlockReceipt() && (index < (blockLength() - 1)))
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
void FixedLengthBlockProtocol::processReceivedBlock()
{
  m_lastReceivedBlockTitle = m_recvBlockBuf[0];
  const int payloadLen = blockLength() - trailerLength() - 1;
  const int blockPayloadStartPos = 1;

  m_lastReceivedPayload.insert(m_lastReceivedPayload.end(),
                               &m_recvBlockBuf[blockPayloadStartPos],
                               &m_recvBlockBuf[blockPayloadStartPos + payloadLen]);
}

