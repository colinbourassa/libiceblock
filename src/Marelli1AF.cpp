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
  return (m_lastReceivedBlockType == Marelli1AFBlockType::EmptyAck);
}

bool Marelli1AF::lastReceivedBlockWasNack() const
{
  return ((m_lastReceivedBlockType == Marelli1AFBlockType::NACK) ||
          (m_lastReceivedBlockType == Marelli1AFBlockType::NotSupported));
}

