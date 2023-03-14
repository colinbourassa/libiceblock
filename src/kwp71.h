#include <string>
#include <cstring>
#include <thread>
#include <vector>
#include <queue>
#include <condition_variable>
#include <stdint.h>
#include "kwp71_version.h"

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

struct Kwp71Version
{
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
};

class Kwp71
{
public:
  Kwp71(std::string device, uint8_t addr);
  void shutdown();
  bool requestIDInfo(std::vector<std::string>& idResponse);
  bool sendCommand(Kwp71Command cmd, std::vector<uint8_t>& response);
  static Kwp71Version getLibraryVersion();

private:
  uint8_t m_ecuAddr;
  bool m_shutdown;
  std::thread m_ifThread;
  std::string m_deviceName;
  int m_fd;
  uint8_t m_lastUsedSeqNum;
  uint8_t m_sendPacketBuf[256];
  uint8_t m_recvPacketBuf[256];

  bool m_readyForCommand;
  bool m_receivingData;
  std::vector<uint8_t> m_responseBinaryData;
  std::vector<std::string> m_responseStringData;
  Kwp71Command m_pendingCmd;
  bool m_responseReadSuccess;
  std::condition_variable m_responseCondVar;
  std::mutex m_responseMutex;

  static constexpr uint8_t s_endOfPacket = 0x03;

  bool populatePacket(bool usePendingCommand);
  bool openSerialPort();
  bool readAckKeywordBytes();
  void closeDevice();
  bool sendPacket();
  bool recvPacket(Kwp71PacketType& type);
  void processReceivedPacket();
  bool slowInit(uint8_t address, int databits, int parity);
  void commLoop();
  static void threadEntry(Kwp71* iface);
};

