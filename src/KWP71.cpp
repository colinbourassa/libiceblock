#include "KWP71.h"

KWP71::KWP71(int baud, LineType initLine, bool verbose) :
  VariableLengthBlockProtocol(baud, initLine, verbose)
{
}

bool KWP71::doPostKeywordSequence()
{
  bool status = true;
  // The ECU apparently sends its ID info unsolicited next
  do {
    if (recvBlock())
    {
      // ACK until the ECU sends its first empty block (after the ID info)
      if (m_lastReceivedBlockTitle != static_cast<uint8_t>(KWP71BlockType::Empty))
      {
        // The ECU is (probably) sending identification information as a series of
        // ASCII strings. Save these so that the application can request them later.
        m_idInfoStrings.push_back(m_lastReceivedPayload);
        m_lastReceivedPayload.clear();
        sendBlock();
      }
    }
    else
    {
      status = false;
    }
  } while ((m_lastReceivedBlockTitle != static_cast<uint8_t>(KWP71BlockType::Empty)) &&
           !shutdownRequested());

  return status;
}

bool KWP71::lastReceivedBlockWasEmpty() const
{
  return (m_lastReceivedBlockTitle == static_cast<uint8_t>(KWP71BlockType::Empty));
}

bool KWP71::lastReceivedBlockWasNack() const
{
  return ((m_lastReceivedBlockTitle == static_cast<uint8_t>(KWP71BlockType::NACK)) ||
          (m_lastReceivedBlockTitle == static_cast<uint8_t>(KWP71BlockType::NotSupported)));
}

/**
 * Checks the provided payload length/content against the associated
 * block title and determines if they represent a valid combination.
 */
bool KWP71::checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const
{
  bool status = false;
  const KWP71BlockType type = static_cast<KWP71BlockType>(title);
  switch (type)
  {
  /** TODO: implement missing block types:
   *  ReadParamData
   *  RecordParamData
   */
  case KWP71BlockType::Empty:
  case KWP71BlockType::ReadTroubleCodes:
  case KWP71BlockType::EraseTroubleCodes:
  case KWP71BlockType::RequestID:
  case KWP71BlockType::RequestSnapshot:
  case KWP71BlockType::Disconnect:
    status = (payload.size() == 0);
    break;
  case KWP71BlockType::ReadParamData:
    status = true;
    break;
  case KWP71BlockType::ActivateActuator:
  case KWP71BlockType::ReadADCChannel:
    status = (payload.size() == 1);
    break;
  case KWP71BlockType::ReadRAM:
  case KWP71BlockType::ReadROM:
  case KWP71BlockType::ReadEEPROM:
    status = (payload.size() == 3);
    break;
  case KWP71BlockType::WriteRAM:
  case KWP71BlockType::WriteEEPROM:
    status = (payload.size() == (payload[0] + 3));
    break;
  default:
    break;
  }
  return status;
}

/**
 * Sends a command to activate the specified actuator.
 */
bool KWP71::activateActuator(uint8_t index, const std::vector<uint8_t>& paramData)
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(KWP71BlockType::ActivateActuator);
  cmd.payload = std::vector<uint8_t>({index});
  const bool status = sendCommand(cmd, data);
  return status;
}

/**
 * Sends a command to read fault code data from the ECU. Returns true when the
 * command was successful and data was returned; false otherwise. When
 * successful, the returned data is stored in the provided vector.
 * Note: The FIAT-9141 spec document indicates that the fault code data is
 * arranged in one 5-byte group for each code. This format might hold for other
 * KWP71-supporting ECUs as well.
 */
bool KWP71::readFaultCodes(std::vector<uint8_t>& data)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(KWP71BlockType::ReadTroubleCodes);
  const bool status = sendCommand(cmd, data);
  return status;
}

/**
 * Sends a command to erase the stored fault codes on the ECU.
 * Returns true when successful; false otherwise.
 */
bool KWP71::eraseFaultCodes()
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(KWP71BlockType::EraseTroubleCodes);
  const bool status = sendCommand(cmd, data);
  return status;
}

/**
 * Reads the requested number of bytes from the specified memory device and address.
 * Only reads from RAM, ROM, and EEPROM are supported.
 */
bool KWP71::readMemory(MemoryType type, uint16_t addr, uint16_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;
  if (type == MemoryType::RAM)
  {
    status = readRAM(addr, static_cast<uint8_t>(numBytes), data);
  }
  else if (type == MemoryType::ROM)
  {
    status = readROM(addr, static_cast<uint8_t>(numBytes), data);
  }
  else if (type == MemoryType::EEPROM)
  {
    status = readEEPROM(addr, static_cast<uint8_t>(numBytes), data);
  }

  return status;
}

/**
 * Writes the provided data to the specified memory device and address.
 * Only writes to RAM and EEPROM are supported.
 */
bool KWP71::writeMemory(MemoryType type, uint16_t addr, const std::vector<uint8_t>& data)
{
  bool status = false;
  if (type == MemoryType::RAM)
  {
    status = writeRAM(addr, data);
  }
  else if (type == MemoryType::EEPROM)
  {
    status == writeEEPROM(addr, data);
  }

  return status;
}

/**
 * Sends a command to read data from ECU RAM. Returns true when the command was
 * successful and data was returned; false otherwise. When successful, the
 * returned data is stored in the provided vector.
 */
bool KWP71::readRAM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;

  if (numBytes <= maxPayloadSize())
  {
    CommandBlock cmd;
    cmd.type = static_cast<uint8_t>(KWP71BlockType::ReadRAM);
    cmd.payload = std::vector<uint8_t>({
      numBytes,
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    status = sendCommand(cmd, data);
  }
  return status;
}

/**
 * Sends a command to read data from ECU ROM. Returns true when the command was
 * successful and data was returned; false otherwise. When successful, the
 * returned data is stored in the provided vector.
 */
bool KWP71::readROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;

  if (numBytes <= maxPayloadSize())
  {
    CommandBlock cmd;
    cmd.type = static_cast<uint8_t>(KWP71BlockType::ReadROM);
    cmd.payload = std::vector<uint8_t>({
      numBytes,
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    status = sendCommand(cmd, data);
  }
  return status;
}

/**
 * Sends a command to read data from ECU EEPROM. Returns true when the command
 * was successful and data was returned; false otherwise. When successful, the
 * returned data is stored in the provided vector.
 */
bool KWP71::readEEPROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data)
{
  bool status = false;

  if (numBytes <= maxPayloadSize())
  {
    CommandBlock cmd;
    cmd.type = static_cast<uint8_t>(KWP71BlockType::ReadEEPROM);
    cmd.payload = std::vector<uint8_t>({
      numBytes,
      static_cast<uint8_t>(addr >> 8),
      static_cast<uint8_t>(addr & 0xff)
    });
    status = sendCommand(cmd, data);
  }
  return status;
}

/**
 * Sends a command to write the provided data to the specified address in the
 * ECU's RAM. Returns true when successful; false otherwise.
 */
bool KWP71::writeRAM(uint16_t addr, const std::vector<uint8_t>& data)
{
  bool status = false;

  // Limit the maximum write payload to the size of the remaining
  // space in a single block (after accounting for the header/trailer).
  if (data.size() <= (maxPayloadSize() - 3))
  {
    CommandBlock cmd;
    cmd.type = static_cast<uint8_t>(KWP71BlockType::WriteRAM);
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
 * Sends a command to write the provided data to the specified address in the
 * ECU's EEPROM. Returns true when successful; false otherwise.
 */
bool KWP71::writeEEPROM(uint16_t addr, const std::vector<uint8_t>& data)
{
  bool status = false;

  // Limit the maximum write payload to the size of the remaining
  // space in a single block (after accounting for the header/trailer).
  if (data.size() <= (maxPayloadSize() - 3))
  {
    CommandBlock cmd;
    cmd.type = static_cast<uint8_t>(KWP71BlockType::WriteRAM);
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
 * Returns true if the provided block type is a valid block to be sent from the
 * tester to the ECU (as opposed to a block type that is only ever returned from
 * the ECU to the tester).
 */
bool KWP71::isValidCommandFromTester(uint8_t cmd) const
{
  const KWP71BlockType type = static_cast<KWP71BlockType>(cmd);
  switch (type)
  {
  case KWP71BlockType::RequestID:
  case KWP71BlockType::ReadRAM:
  case KWP71BlockType::WriteRAM:
  case KWP71BlockType::ReadROM:
  case KWP71BlockType::ActivateActuator:
  case KWP71BlockType::EraseTroubleCodes:
  case KWP71BlockType::ReadTroubleCodes:
  case KWP71BlockType::ReadADCChannel:
  case KWP71BlockType::ReadParamData:
  case KWP71BlockType::RecordParamData:
  case KWP71BlockType::RequestSnapshot:
  case KWP71BlockType::ReadEEPROM:
  case KWP71BlockType::WriteEEPROM:
    return true;
  default:
    return false;
  }
}

/**
 * Returns the ID strings that were received immediately after the init sequence.
 */
bool KWP71::readIDCode(std::vector<std::vector<uint8_t>>& idStrings)
{
  idStrings = m_idInfoStrings;
  return true;
}

