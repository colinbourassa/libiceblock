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

struct CommandBlock
{
  uint8_t type;
  std::vector<uint8_t> payload;
};

enum class BlockTrailerType
{
  Fixed03,
  Checksum8Bit,
  Checksum16Bit
};

class BlockExchangeProtocol
{
public:
  BlockExchangeProtocol(int baudRate, bool verbose);
  virtual ~BlockExchangeProtocol();

  bool connectByDeviceId(uint16_t vid, uint16_t pid, uint8_t ecuAddr);
  bool connectByBusAddr(uint8_t bus, uint8_t addr, uint8_t ecuAddr);
  void setBaudRate(int baudRate) { m_baudRate = baudRate; }
  void disconnect();
  bool requestIDInfo(std::vector<std::string>& idResponse);
  bool sendCommand(CommandBlock cmd, std::vector<uint8_t>& response);

  std::vector<FtdiDeviceInfo> enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extrtaPids = {});

protected:
  virtual inline bool bytesEchoedDuringBlockReceipt() const = 0;
  virtual inline int initDataBits() const = 0;
  virtual inline int initParity() const = 0;
  virtual inline int timeBeforeReconnectMs() const = 0;
  virtual inline int isoKeywordIndexToEcho() const = 0;
  virtual inline bool isoKeywordEchoIsInverted() const = 0;
  virtual inline int isoKeywordNumBytes() const = 0;
  virtual inline bool useSequenceNums() const = 0;
  virtual inline BlockTrailerType trailerType() const = 0;
  virtual inline uint8_t blockTitleForEmptyAck() const = 0;
  virtual inline uint8_t blockTitleForRequestID() const = 0;
  virtual bool lastReceivedBlockWasEmpty() const = 0;
  virtual bool lastReceivedBlockWasNack() const = 0;

  int m_baudRate;
  uint8_t m_sendBlockBuf[256];
  uint8_t m_recvBlockBuf[256];
  uint8_t m_lastUsedSeqNum = 0;
  std::vector<uint8_t> m_responseBinaryData;
  std::vector<std::string> m_responseStringData;

  bool recvBlock();
  bool sendBlock();
  inline bool shutdownRequested() const { return m_shutdown; }

  virtual bool isValidCommandFromTester(uint8_t type) const { return true; }
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const = 0;
  virtual void processReceivedBlock() = 0;
  virtual bool doPostKeywordSequence() { return true; }

private:
  bool m_verbose;
  bool m_connectionActive = false;
  bool m_shutdown = false;
  uint8_t m_ecuAddr = 0x10;

  std::unique_ptr<std::thread> m_ifThreadPtr = nullptr;
  std::string m_deviceName;
  CommandBlock m_pendingCmd;
  bool m_commandIsPending = false;
  bool m_waitingForReply = false;
  bool m_responseReadSuccess = false;
  std::condition_variable m_responseCondVar;
  std::mutex m_responseMutex;
  std::mutex m_connectMutex;
  std::mutex m_commandMutex;
  struct ftdi_context m_ftdi;

  void setBlockSizePrefix(int payloadSize);
  void setBlockSequenceNum();
  void setBlockTitle(uint8_t title);
  void setBlockPayload(const std::vector<uint8_t>& payload);
  void setBlockTrailer();

  bool initAndStartCommunication();
  bool populateBlock(bool& usedPendingCommand);
  bool waitForISOSequence(std::chrono::milliseconds timeout,
                          std::vector<uint8_t>& isoBytes);
  bool isConnectionActive() const;
  bool readAckKeywordBytes();
  bool setFtdiSerialProperties();
  void closeFtdi();
  bool slowInit(uint8_t address, int databits, int parity);
  void commLoop();
  static void threadEntry(BlockExchangeProtocol* iface);

  bool readSerial(uint8_t* buf, int count);
  bool writeSerial(uint8_t* buf, int count);

  int getFtdiDeviceInfo(ftdi_device_list* list, int count, std::vector<FtdiDeviceInfo>& deviceInfo);
};

