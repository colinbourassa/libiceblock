#include <string>
#include <cstring>
#include <thread>
#include <vector>
#include <chrono>
#include <memory>
#include <set>
#include <condition_variable>
#include <stdint.h>
#include <libftdi1/ftdi.h>
#include "kwp71_version.h"

struct FtdiDeviceInfo
{
  uint8_t busNumber;
  uint8_t deviceAddress;
  std::string manufacturer;
  std::string description;
  std::string serial;
  FtdiDeviceInfo(
    uint8_t _busNumber,
    uint8_t _deviceAddress,
    const std::string& _manufacturer,
    const std::string& _description,
    const std::string& _serial) :
    busNumber(_busNumber),
    deviceAddress(_deviceAddress),
    manufacturer(_manufacturer),
    description(_description),
    serial(_serial) {}
};

enum class Kwp71BlockType
{
  RequestID         = 0x00,
  ReadRAM           = 0x01,
  WriteRAM          = 0x02,
  ReadROM           = 0x03,
  ActivateActuators = 0x04,
  EraseTroubleCodes = 0x05,
  Disconnect        = 0x06,
  ReadTroubleCodes  = 0x07,
  ReadADCChannel    = 0x08,
  Empty             = 0x09,
  NACK              = 0x0A,
  NotSupported      = 0x0B,
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
  ADCValue          = 0xFB,
  BinaryData        = 0xFC,
  RAMContent        = 0xFD,
  ROMContent        = 0xFE,
};

enum class Kwp71Variant
{
  Standard,
  FIAT9141
};

struct Kwp71Command
{
  Kwp71BlockType type;
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
  Kwp71(bool verbose = false);
  void setProtocolVariant(Kwp71Variant variant);
  inline void setBaud(int baud)
    { m_baudRate = baud; }
  inline void setSlowInitDataBits(int dataBits)
    { m_initDataBits = dataBits; }
  inline void setSlowInitParity(int parity)
    { m_initParity = parity; }
  inline void setTimeBeforeReconnectionMilliseconds(int timeMs)
    { m_timeBeforeReconnectionMs = timeMs; }
  inline void setEchoDuringBlockReceipt(bool echo)
    { m_bytesEchoedDuringBlockReceipt = echo; }
  inline void setUseChecksumAsTrailer(bool useChecksum)
    { m_lastBlockByteIsChecksum = useChecksum; }
  inline void setUseSequenceNumbers(bool useSequenceNums)
    { m_useSequenceNums = useSequenceNums; }

  bool connect(uint16_t vid, uint16_t pid, uint8_t addr, int& err);
  void disconnect();
  bool requestIDInfo(std::vector<std::string>& idResponse);
  bool sendCommand(Kwp71Command cmd, std::vector<uint8_t>& response);
  bool readRAM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readEEPROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool writeRAM(uint16_t addr, const std::vector<uint8_t>& data);
  bool writeEEPROM(uint16_t addr, const std::vector<uint8_t>& data);
  bool readFaultCodes(std::vector<uint8_t>& data);
  bool eraseFaultCodes();

  static Kwp71Version getLibraryVersion();
  std::vector<FtdiDeviceInfo> enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extrtaPids = {});

private:
  bool m_verbose;
  bool m_connectionActive = false;
  bool m_shutdown = false;
  uint8_t m_ecuAddr = 0x10;

  /////
  // Protocol variant parameters
  bool m_bytesEchoedDuringBlockReceipt = true;
  int m_baudRate = 9600;
  int m_initDataBits = 8;
  int m_initParity = 0;
  int m_timeBeforeReconnectionMs = 260;
  bool m_lastBlockByteIsChecksum = false;
  bool m_useSequenceNums = true;
  int m_isoKeywordIndexToEcho = 2;
  bool m_isoKeywordEchoIsInverted = true;
  int m_isoKeywordNumBytes = 3;
  /////

  std::unique_ptr<std::thread> m_ifThreadPtr = nullptr;
  std::string m_deviceName;
  uint8_t m_lastUsedSeqNum = 0;
  uint8_t m_sendBlockBuf[256];
  uint8_t m_recvBlockBuf[256];
  Kwp71BlockType m_lastReceivedBlockType = Kwp71BlockType::Empty;
  std::vector<uint8_t> m_responseBinaryData;
  std::vector<std::string> m_responseStringData;
  Kwp71Command m_pendingCmd;
  bool m_commandIsPending = false;
  bool m_waitingForReply = false;
  bool m_responseReadSuccess = false;
  std::condition_variable m_responseCondVar;
  std::mutex m_responseMutex;
  std::mutex m_connectMutex;
  std::mutex m_commandMutex;
  struct ftdi_context m_ftdi;

  static constexpr uint8_t s_endOfBlock = 0x03;

  bool waitForISOSequence(std::chrono::milliseconds timeout,
                          std::vector<uint8_t>& isoBytes);
  bool isConnectionActive() const;
  bool populateBlock(bool& usedPendingCommand);
  bool readAckKeywordBytes();
  bool setFtdiSerialProperties();
  void closeFtdi();
  bool sendBlock();
  bool recvBlock();
  void processReceivedBlock();
  bool slowInit(uint8_t address, int databits, int parity);
  void commLoop();
  static void threadEntry(Kwp71* iface);
  bool isValidCommandFromTester(Kwp71BlockType type) const;

  bool readSerial(uint8_t* buf, int count);
  bool writeSerial(uint8_t* buf, int count);

  int getFtdiDeviceInfo(ftdi_device_list* list, int count, std::vector<FtdiDeviceInfo>& deviceInfo);
};

