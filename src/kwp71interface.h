#include <string>
#include <cstring>
#include <thread>
#include <vector>
#include <queue>
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
  NACK              = 0x0A,
  ReadParamData     = 0x10,
  RecordParamData   = 0x11,
  RequestSnapshot   = 0x12,
  ReadEEPROM        = 0x19,
  WriteEEPROM       = 0x1A,
  ParamRecordConf   = 0xEB,
  ParametricData    = 0xEC,
  EEPROMContent     = 0xEF,
  Snapshot          = 0xF4,
  ASCIIString       = 0xF6,
  DACValue          = 0xFB,
  BinaryData        = 0xFC,
  RAMContent        = 0xFD,
  ROMContent        = 0xFE,
};

struct Kwp71Command
{
  Kwp71PacketType type;
  std::vector<uint8_t> payload;
};

class Kwp71Interface
{
public:
  Kwp71Interface(std::string device, uint8_t addr);
  void shutdown();
  std::vector<std::string> requestIDInfo();
  std::vector<uint8_t> sendCommand(Kwp71Command cmd);

private:
  bool m_shutdown;
  std::thread m_ifThread;
  std::string m_deviceName;
  int m_fd;
  uint8_t m_lastUsedSeqNum;
  uint8_t m_sendPacketBuf[256];
  uint8_t m_recvPacketBuf[256];
  std::vector<uint8_t> m_response;
  std::vector<std::string> m_responseStringData;
  std::binary_semaphore m_cmdSemaphore;
  Kwp71Command m_pendingCmd;
  static constexpr uint8_t s_endOfPacket = 0x03;

  bool collectResponsePackets();
  bool populatePacket(Kwp71PacketType type,
                      const std::vector<uint8_t>& payload);
  bool openSerialPort();
  bool readAckKeywordBytes();
  void closeDevice();
  bool sendPacket();
  bool recvPacket(Kwp71PacketType& type);
  void processReceivedPacket();
  bool slowInit(uint8_t address, int databits, int parity);
  static void commLoop(Kwp71Interface* iface, bool* shutdown, uint8_t addr);
};

