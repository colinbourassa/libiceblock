#include "Marelli1AF.h"

Marelli1AF::Marelli1AF(bool verbose) :
  BlockExchangeProtocol(4800, verbose)
{
}

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
  // TODO
  return false;
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

bool Marelli1AF::sendSelectBlock()
{
  bool status = false;
  m_sendBlockBuf[0] = 0x03;
  m_sendBlockBuf[1] = 0x34;
  m_sendBlockBuf[2] = 0x51;
  m_sendBlockBuf[3] = 0x88;

  // Call sendBlock() with a flag indicating the send buffer is
  // pre-populated (so that it doesn't try to build a normal ACK
  // block or similar.) This is necessary because the SELECT block
  // doesn't follow the normal format format for this protocol
  // (with the 16-bit checksum trailer.)
  status = sendBlock(true);
  return status;
}

bool Marelli1AF::waitForSelectBlockResponse()
{
  bool status = false;
  if (recvBlock(std::chrono::milliseconds(1200)) &&
      (m_recvBlockBuf[0] == 0x03) &&
      (m_recvBlockBuf[1] == 0x34) &&
      (m_recvBlockBuf[1] == 0x51) &&
      (m_recvBlockBuf[1] == 0x88))
  {
    status = true;
  }
  return status;
}

bool Marelli1AF::sendHostBlock()
{
  std::vector<uint8_t> data;
  CommandBlock cmd;
  cmd.type = static_cast<uint8_t>(Marelli1AFBlockType::HostBlock);
  // TODO: Do we want to parameterize the "Host Number"? Currently
  // hardcoded to 0x00.
  cmd.payload = { 0x00 };
  const bool status = sendCommand(cmd, data);
  return status;
}

bool Marelli1AF::doPostKeywordSequence()
{
  int attemptsRemaining = 3;
  bool status = true;
  // TODO: add appropriate delays per the spec
  do {
    sendSelectBlock();
    status = waitForSelectBlockResponse();
    attemptsRemaining--;
  } while (!status && (attemptsRemaining > 0));

  if (status)
  {
    attemptsRemaining = 3;
    do {
      sendHostBlock();
      status = waitForHostBlockResponse();
      attemptsRemaining--;
    } while (!status && (attemptsRemaining > 0));
  }
  return status;
}

