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

class BlockExchangeProtocol
{
public:
  BlockExchangeProtocol(bool verbose);

  bool connectByDeviceId(uint16_t vid, uint16_t pid, uint8_t ecuAddr);
  bool connectByBusAddr(uint8_t bus, uint8_t addr, uint8_t ecuAddr);
  void disconnect();
  bool requestIDInfo(std::vector<std::string>& idResponse);
  bool sendCommand(CommandBlock cmd, std::vector<uint8_t>& response);
  bool readRAM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readEEPROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool writeRAM(uint16_t addr, const std::vector<uint8_t>& data);
  bool writeEEPROM(uint16_t addr, const std::vector<uint8_t>& data);
  bool activateActuator(uint8_t index);
  bool readFaultCodes(std::vector<uint8_t>& data);
  bool eraseFaultCodes();

  std::vector<FtdiDeviceInfo> enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extrtaPids = {});

protected:
  virtual inline bool bytesEchoedDuringBlockReceipt() const = 0;
  virtual inline int baudRate() const = 0;
  virtual inline int initDataBits() const = 0;
  virtual inline int initParity() const = 0;
  virtual inline int timeBeforeReconnectMs() const = 0;
  virtual inline int isoKeywordIndexToEcho() const = 0;
  virtual inline bool isoKeywordEchoIsInverted() const = 0;
  virtual inline int isoKeywordNumBytes() const = 0;

  uint8_t m_sendBlockBuf[256];
  uint8_t m_recvBlockBuf[256];
  std::vector<uint8_t> m_responseBinaryData;
  std::vector<std::string> m_responseStringData;

  bool recvBlock();
  bool sendBlock();
  inline bool shutdownRequested() const { return m_shutdown; }

  virtual bool isValidCommandFromTester(uint8_t type) const { return true; }
  virtual void processReceivedBlock() = 0;
  virtual bool populateBlock(bool& usedPendingCommand) = 0;
  virtual bool doPostKeywordSequence();
  virtual bool lastReceivedBlockWasEmpty() const = 0;
  virtual bool lastReceivedBlockWasNack() const = 0;

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

  bool initAndStartCommunication();
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

