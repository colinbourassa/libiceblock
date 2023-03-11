#include <string>
#include <thread>
#include <stdint.h>

enum class Kwp71PacketType
{
  Empty,
  Disconnect,
  Unknown
};

class Kwp71Interface
{
public:
  Kwp71Interface(std::string device);
  void shutdown();

private:
  bool m_shutdown;
  std::thread m_ifThread;
  int m_fd;
  uint8_t m_lastUsedSeqNum;
  uint8_t m_sendPacketBuf[256];
  uint8_t m_recvPacketBuf[256];
  static constexpr uint8_t s_endOfPacket = 0x03;

  void populatePacket(Kwp71PacketType type);
  bool openDevice(std::string device);
  void closeDevice();
  bool sendPacket();
  bool recvPacket();
  static void commLoop(Kwp71Interface* iface, bool* shutdown);
};

