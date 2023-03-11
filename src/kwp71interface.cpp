#include "kwp71interface.h"
#include <unistd.h>

Kwp71Interface::Kwp71Interface(std::string device) :
  m_shutdown(false),
  m_fd(-1),
  m_lastUsedSeqNum(0)
{
  if (openDevice(device))
  {
    m_ifThread = std::thread(commLoop, this, &m_shutdown);
  }
}

bool Kwp71Interface::openDevice(std::string /*device*/)
{
  return false;
}

void Kwp71Interface::closeDevice()
{
  close(m_fd);
}

void Kwp71Interface::shutdown()
{
  m_shutdown = true;
  m_ifThread.join();
  closeDevice();
}

bool Kwp71Interface::sendPacket()
{
  return false;
}

bool Kwp71Interface::recvPacket()
{
  // TODO: be sure to record m_lastUsedSeqNum
  return false;
}

void Kwp71Interface::populatePacket(Kwp71PacketType type)
{
  switch (type)
  {
  case Kwp71PacketType::Empty:
    m_sendPacketBuf[0] = 0x03;
    m_sendPacketBuf[1] = ++m_lastUsedSeqNum;
    m_sendPacketBuf[2] = 0x09;
    m_sendPacketBuf[3] = s_endOfPacket;
    break;
  default:
    break;
  }
}

void Kwp71Interface::commLoop(Kwp71Interface* iface, bool* shutdown)
{
  // TODO: do we want the 5-baud init here, or should this
  // thread only be started after the init was successful?

  bool isCommandQueued = false;
  Kwp71PacketType command = Kwp71PacketType::Unknown;

  while (!shutdown)
  {
    iface->populatePacket(isCommandQueued ? command : Kwp71PacketType::Empty);
    iface->sendPacket();
  }
  iface->populatePacket(Kwp71PacketType::Disconnect);
  iface->sendPacket();
}

