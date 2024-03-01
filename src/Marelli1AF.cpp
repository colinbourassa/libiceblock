#include "Marelli1AF.h"

Marelli1AF::Marelli1AF(bool verbose) :
  BlockExchangeProtocol(4800, verbose)
{
}

Marelli1AF::Marelli1AF(int baudRate, bool verbose) :
  BlockExchangeProtocol(baudRate, verbose)
{
}

/**
 * Returns true if the provided block title value represents a block that can
 * be sent by the Tester (as opposed to the vehicle/ECU); false otherwise.
 */
bool Marelli1AF::isValidCommandFromTester(uint8_t title) const
{
  const Marelli1AFBlockType block = static_cast<Marelli1AFBlockType>(title);
  switch (block)
  {
  case Marelli1AFBlockType::HostBlock:
  case Marelli1AFBlockType::SelectBlock:
  case Marelli1AFBlockType::StopCommunication:
  case Marelli1AFBlockType::ActivateActuator:
  case Marelli1AFBlockType::StopActuator:
  case Marelli1AFBlockType::CheckActuator:
  case Marelli1AFBlockType::ReadMemoryCell:
  case Marelli1AFBlockType::ReadValue:
  case Marelli1AFBlockType::ReadSnapshot:
  case Marelli1AFBlockType::ReadADCChannel:
  case Marelli1AFBlockType::WriteRAM:
  case Marelli1AFBlockType::ReadErrorMemory:
  case Marelli1AFBlockType::ReadIDCode:
  case Marelli1AFBlockType::ReadErrorValue:
  case Marelli1AFBlockType::ClearErrorMemory:
    return true;
  default:
    return false;
  }
}

bool Marelli1AF::checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const
{
  bool status = false;
  const Marelli1AFBlockType type = static_cast<Marelli1AFBlockType>(title);
  switch(type)
  {
  case Marelli1AFBlockType::WriteRAM:
    status = (payload.size() == 6);
    break;
  case Marelli1AFBlockType::ReadMemoryCell:
    status = (payload.size() == 3);
    break;
  case Marelli1AFBlockType::ActivateActuator:
    status = (payload.size() == 2);
    break;
  case Marelli1AFBlockType::ReadIDCode:
  case Marelli1AFBlockType::StopActuator:
  case Marelli1AFBlockType::ReadValue:
  case Marelli1AFBlockType::ReadSnapshot:
  case Marelli1AFBlockType::ReadErrorValue:
    status = (payload.size() == 1);
    break;
  case Marelli1AFBlockType::ReadErrorMemory:
  case Marelli1AFBlockType::ClearErrorMemory:
    status = payload.empty();
    break;
  case Marelli1AFBlockType::ReadADCChannel:
    // Not sure if there's a maximum on the number of ADC channels that can be
    // requested in a single message; for now, we'll just let anything through.
    status = true;
  default:
    break;
  }
  return status;
}

bool Marelli1AF::lastReceivedBlockWasEmpty() const
{
  return (m_lastReceivedBlockTitle == static_cast<uint8_t>(Marelli1AFBlockType::EmptyAck));
}

bool Marelli1AF::lastReceivedBlockWasNack() const
{
  return ((m_lastReceivedBlockTitle == static_cast<uint8_t>(Marelli1AFBlockType::NACK)) ||
          (m_lastReceivedBlockTitle == static_cast<uint8_t>(Marelli1AFBlockType::NotSupported)));
}

/**
 * Sends a special SELECT block.
 */
bool Marelli1AF::sendSelectBlock()
{
  bool status = false;
  m_sendBlockBuf[0] = 0x03;
  m_sendBlockBuf[1] = 0x34;
  m_sendBlockBuf[2] = 0x51;
  m_sendBlockBuf[3] = 0x88;

  // Call sendBlock() with a flag indicating the send buffer is pre-populated
  // (so that it doesn't try to build a normal ACK block or similar.) This is
  // necessary because the SELECT block doesn't follow the normal format for
  // this protocol (i.e. with the 16-bit checksum trailer.)
  status = sendBlock(true);
  return status;
}

/**
 * Waits up to 1200ms for receipt of a block, and verifies that the block is a
 * SELECT ACK. Returns true if exactly one block (a SELECT ACK) was received
 * before the time limit; false otherwise.
 */
bool Marelli1AF::waitForSelectBlockResponse()
{
  bool status = false;
  if (recvBlock(std::chrono::milliseconds(1200)) &&
      (m_recvBlockBuf[0] == 0x03) &&
      (m_recvBlockBuf[1] == 0x34) &&
      (m_recvBlockBuf[2] == 0x51) &&
      (m_recvBlockBuf[3] == 0x88))
  {
    status = true;
  }
  return status;
}

/**
 * Sends a special HOST block. NOTE: Even though there is a single byte of
 * payload identified by the spec as the "Host Number", it seems this byte is
 * intended to have a fixed value of 0x00.
 */
bool Marelli1AF::sendHostBlock()
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::HostBlock);
  cmd.payload = { 0x00 };
  const bool status = sendCommand(cmd, data);
  return status;
}

/**
 * Waits up to 200ms for receipt of an OUT STATUS block, which is expected to
 * be sent by the ECU after the ECU has received the HOST block.
 * NOTE: There are two bytes of payload in this block, the first of which is
 * identified in the spec as "Diagnostic Mode" and can apparently have a
 * variable value, while the second is "Host Number" and is indiciated to have
 * a fixed value of 0x00 (despite the spec having a table of numerous possible
 * Host Number values.)
 */
bool Marelli1AF::waitForHostBlockResponse()
{
  bool status = false;
  if (recvBlock(std::chrono::milliseconds(200)) &&
      (m_recvBlockBuf[0] == 0x05) &&
      (m_recvBlockBuf[1] == static_cast<uint8_t>(Marelli1AFBlockType::OutStatus)))
  {
    status = true;
  }
  return status;
}

/**
 * Performs the block exchange sequence that the ECU expects immediately after
 * initialization. Returns true if the SELECT/SELECT-ACK and HOST/OUT-STATUS
 * block exchanges are done successfully within the time limits given by the
 * protocol spec; false otherwise.
 */
bool Marelli1AF::doPostKeywordSequence()
{
  int attemptsRemaining = 3;
  bool status = true;

  // Per the spec, we need to wait at least 2ms (but not more than 1200ms)
  // after the keyword preamble before we send the SELECT block.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  do {
    sendSelectBlock();
    status = waitForSelectBlockResponse();
    attemptsRemaining--;
  } while (!status && (attemptsRemaining > 0));

  if (status)
  {
    // Per the spec, we need to wait at least 200ms (but not more than 500ms)
    // after the SELECT ACK before we send the SET HOST block.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    attemptsRemaining = 3;
    do {
      sendHostBlock();
      status = waitForHostBlockResponse();
      attemptsRemaining--;
    } while (!status && (attemptsRemaining > 0));
  }
  return status;
}

bool Marelli1AF::activateActuator(uint8_t index, uint8_t parameter)
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ActivateActuator);
  cmd.payload = std::vector<uint8_t>({index, parameter});
  const bool status = sendCommand(cmd, data);
  return status;
}

bool Marelli1AF::stopActuator(uint8_t index)
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::StopActuator);
  cmd.payload = std::vector<uint8_t>({index});
  const bool status = sendCommand(cmd, data);
  return status;
}

bool Marelli1AF::readMemoryCell(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  // TODO: check that the requested number of bytes will fit in
  // a message block that is within the maximum allowed size
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadMemoryCell);
  cmd.payload = std::vector<uint8_t>({
    static_cast<uint8_t>(addr >> 8),
    static_cast<uint8_t>(addr & 0xff),
    numBytes
  });
  const bool status = sendCommand(cmd, data);
  return status;
}

bool Marelli1AF::readValue(uint8_t valueCode, std::vector<uint8_t>& valueSequence)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadValue);
  cmd.payload = std::vector<uint8_t>({valueCode});
  const bool status = sendCommand(cmd, valueSequence);
  return status;
}

bool Marelli1AF::readSnapshot(uint8_t snapshotCode, std::vector<uint8_t>& snapshotData)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadSnapshot);
  cmd.payload = std::vector<uint8_t>({snapshotCode});
  const bool status = sendCommand(cmd, snapshotData);
  return status;
}

bool Marelli1AF::readADCChannel(const std::vector<uint8_t>& channelList,
                                std::vector<uint8_t>& channelValues)
{
  // TODO: check that the provided channel list is not too long.
  // The spec lists only seven possible channels (which are non-contiguously
  // numbered between 00 and 09) but since the 1AF protocol seems to have been
  // used on at least one system that was not an engine, this list may vary
  // between different applications.
  // We may not even want to interpret the returned data, leaving that as an
  // exercise for the application.
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadADCChannel);
  cmd.payload = channelList;
  const bool status = sendCommand(cmd, channelValues);
  return status;
}

bool Marelli1AF::writeSecurityCode(const std::vector<uint8_t>& securityCode)
{
  bool status = false;
  if (securityCode.size() == 5)
  {
    std::vector<uint8_t> response;
    CommandBlock cmd;
    cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::WriteRAM);
    cmd.payload = securityCode;
    cmd.payload.insert(cmd.payload.begin(), {0x0b});
    status = sendCommand(cmd, response);
  }
  return status;
}

bool Marelli1AF::readErrorMemory(std::vector<uint8_t>& data)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadErrorMemory);
  const bool status = sendCommand(cmd, data);
  return status;
}

bool Marelli1AF::readIDCode(Marelli1AFPartNumberType type, std::vector<uint8_t>& idString)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadIDCode);
  cmd.payload.push_back(static_cast<uint8_t>(type));
  const bool status = sendCommand(cmd, idString);
  return status;
}

bool Marelli1AF::readErrorValue(uint8_t code, std::vector<uint8_t>& data)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ReadErrorValue);
  cmd.payload.push_back(code);
  const bool status = sendCommand(cmd, data);
  return status;
}

bool Marelli1AF::clearErrorMemory()
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::ClearErrorMemory);
  const bool status = sendCommand(cmd, data);
  return status;
}

