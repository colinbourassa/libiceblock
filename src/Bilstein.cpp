#include "Bilstein.h"

Bilstein::Bilstein(int baud, LineType initLine, bool verbose) :
  FixedLengthBlockProtocol(baud, initLine, verbose)
{
}

bool Bilstein::lastReceivedBlockWasEmpty() const
{
  return (m_lastReceivedBlockTitle == static_cast<uint8_t>(BilsteinBlockType::Empty));
}

bool Bilstein::lastReceivedBlockWasNack() const
{
  // TODO: We'll need a protocol spec to determine which block type(s) represent a NACK.
  return false;
}

/**
 * Checks the provided payload length/content against the associated
 * block title and determines if they represent a valid combination.
 */
bool Bilstein::checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const
{
  return true;
}

/**
 * Reads the requested number of bytes from the specified memory device and address.
 */
bool Bilstein::readMemory(MemoryType type, uint16_t addr, uint16_t /*numBytes*/, std::vector<uint8_t>& data)
{
  bool status = false;
  if (type == MemoryType::RAM)
  {
    status = readRAM(addr, data);
  }
  else if (type == MemoryType::Fault)
  {
    status = readFaultMemory(addr, data);
  }

  return status;
}

/**
 * Writes the provided data to the specified memory device and address.
 */
bool Bilstein::writeMemory(MemoryType type, uint16_t addr, const std::vector<uint8_t>& data)
{
  bool status = false;
  if (data.size() == 1)
  {
    if (type == MemoryType::RAM)
    {
      status = writeRAM(addr, data.at(0));
    }
    else if (type == MemoryType::Fault)
    {
      status = writeFaultMemory(addr, data.at(0));
    }
  }
  return status;
}

/**
 * Sends a command to read data from ECU RAM. Returns true when the command was
 * successful and data was returned; false otherwise. When successful, the
 * returned data is stored in the provided vector.
 */
bool Bilstein::readRAM(uint16_t addr, std::vector<uint8_t>& data)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(BilsteinBlockType::ReadRAM);
  cmd.payload = std::vector<uint8_t>({
    static_cast<uint8_t>(addr >> 8),
    static_cast<uint8_t>(addr & 0xff),
    0x00
  });
  const bool status = sendCommand(cmd, data);
  // TODO: Isolate the desired bytes from the response
  return status;
}

/**
 * Sends a command to write the provided data to the specified address in the
 * ECU's RAM. Returns true when successful; false otherwise.
 */
bool Bilstein::writeRAM(uint16_t addr, uint8_t data)
{
  std::vector<uint8_t> response;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(BilsteinBlockType::WriteRAM);
  cmd.payload = std::vector<uint8_t>({
    static_cast<uint8_t>(addr >> 8),
    static_cast<uint8_t>(addr & 0xff),
    data
  });
  const bool status = sendCommand(cmd, response);
  return status;
}

/**
 * Sends a command to read data from ECU fault code memory. Returns true when the command was
 * successful and data was returned; false otherwise. When successful, the
 * returned data is stored in the provided vector.
 */
bool Bilstein::readFaultMemory(uint16_t addr, std::vector<uint8_t>& data)
{
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(BilsteinBlockType::ReadFaultMem);
  cmd.payload = std::vector<uint8_t>({
    static_cast<uint8_t>(addr >> 8),
    static_cast<uint8_t>(addr & 0xff),
    0x00
  });
  const bool status = sendCommand(cmd, data);
  // TODO: Isolate the desired bytes from the response
  return status;
}

/**
 * Sends a command to write data to the ECU fault code memory. Returns true when
 * the command is successful and the data was written; false otherwise.
 */
bool Bilstein::writeFaultMemory(uint16_t addr, uint8_t data)
{
  std::vector<uint8_t> response;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(BilsteinBlockType::WriteFaultMemory);
  cmd.payload = std::vector<uint8_t>({
    static_cast<uint8_t>(addr >> 8),
    static_cast<uint8_t>(addr & 0xff),
    data
  });
  const bool status = sendCommand(cmd, response);
  return status;
}

