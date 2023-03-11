#include <string>
#include <cstring>
#include <thread>
#include <vector>
#include <stdint.h>

enum class Kwp71PacketType
{
  RequestID         = 0x00,
  ReadRAM           = 0x01,
  WriteRAM          = 0x02,
  ReadROM           = 0x03,
  ActivateActuators = 0x04,
  EraseTroubleCodes = 0x05,
  Disconnect        = 0x06,
  ReadTroubleCodes  = 0x07,
  ReadDACChannel    = 0x08,
  Empty             = 0x09,
  ReadParamData     = 0x10,
  RecordParamData   = 0x11,
  RequestSnapshot   = 0x12,
  ReadEEPROM        = 0x19,
  WriteEEPROM       = 0x1A
};

struct Kwp71Command
{
  Kwp71PacketType type;
  std::vector<uint8_t> payload;
};

class Kwp71Interface
{
public:
  Kwp71Interface(std::string device);
  void shutdown();

private:
  bool m_shutdown;
  std::thread m_ifThread;
  std::string m_deviceName;
  int m_fd;
  uint8_t m_lastUsedSeqNum;
  uint8_t m_sendPacketBuf[256];
  uint8_t m_recvPacketBuf[256];
  static constexpr uint8_t s_endOfPacket = 0x03;

  void getNextPendingCommand(Kwp71Command& cmd);
  bool populatePacket(Kwp71PacketType type,
                      const std::vector<uint8_t>& payload);
  bool openSerialPort();
  bool readAckKeywordBytes();
  void closeDevice();
  bool sendPacket();
  bool recvPacket(Kwp71PacketType& type);
  bool slowInit(uint8_t address, int databits, int parity);
  static void commLoop(Kwp71Interface* iface, bool* shutdown);
};

