#include <string>
#include <cstring>
#include <thread>
#include <vector>
#include <chrono>
#include <memory>
#include <set>
#include <condition_variable>
#include <cstdint>
#include <libftdi1/ftdi.h>

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

enum class LineType
{
  KLine,
  LLine
};

enum class BlockTrailerType
{
  Fixed03,
  Checksum8Bit,
  Checksum16Bit
};

/**
 * This class forms the basis of "block exchange" diagnostic protocols, which
 * all involve the ECU and test equipment taking turns exchanging message blocks.
 * The initial connection is made with a "slow init" sequence, in which a
 * predetermined ECU address is sent at 5 baud. The ECU will then respond with a
 * sequence of "keyword" bytes (sometimes also known as "ISO" bytes because of
 * this exchange having been described by the ISO-9141 standard.)
 */
class BlockExchangeProtocol
{
public:
  explicit BlockExchangeProtocol(int baudRate, LineType initLine, bool verbose);
  virtual ~BlockExchangeProtocol();

  /**
   * Attempts to connect to a remote ECU at the provided address by using the
   * first FTDI device that can be found with the provided VID/PID.
   */
  bool connectByDeviceId(uint16_t vid, uint16_t pid, uint8_t ecuAddr);

  /**
   * Attempts to connect to a remote ECU at the provided address by using an
   * FTDI device at the specified USB bus/device address.
   */
  bool connectByBusAddr(uint8_t bus, uint8_t addr, uint8_t ecuAddr);

  /**
   * Sets the baud rate that will be used for communicating with the ECU.
   */
  void setBaudRate(int baudRate) { m_baudRate = baudRate; }

  /**
   * Stops the block exchange and disconnects.
   */
  void disconnect();

  /**
   * Sends a request for ID info and populates the response vector. Returns true
   * when a response was received within the timeout; false otherwise.
   */
  bool requestIDInfo(std::vector<std::string>& idResponse);

  /**
   * Sends the provided command block to the ECU and populates the vector with
   * any response data. Returns true when successful; false otherwise.
   */
  bool sendCommand(CommandBlock cmd, std::vector<uint8_t>& response);

  /**
   * Queries the USB subsystem for any devices whose VID/PID match known values
   * for FTDI serial devices. Optionally, the caller may provide a list of
   * additional VID/PID pairs that may be treated as FTDI serial devices. This
   * is useful for commercial FTDI-based K-line adapters that have custom IDs.
   */
  std::vector<FtdiDeviceInfo> enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extraPids = {});

protected:
  /**
   * Returns true when the protocol requires that the receiver echo the inverse
   * of each byte before the sender will transmit the next byte.
   */
  virtual bool bytesEchoedDuringBlockReceipt() const = 0;

  /**
   * Returns the number of data bits in the slow init address transmission.
   */
  virtual int initDataBits() const = 0;

  /**
   * Returns an indication of the parity used when the slow init address byte
   * is sent, with 0 meaning no parity, 1 meaning odd parity, and 2 meaning even.
   */
  virtual int initParity() const = 0;

  /**
   * Returns the time, in milliseconds, that the protocol requires between the
   * last block transmission and a new slow-init attempt.
   */
  virtual int timeBeforeReconnectMs() const = 0;

  /**
   * Zero-based index of the keyword byte to echo after the ECU has sent the
   * keyword sequence.
   */
  virtual int isoKeywordIndexToEcho() const = 0;

  /**
   * Returns true when the keyword byte to be echoed must be inverted, or false
   * if the byte is echoed unmodified.
   */
  virtual bool isoKeywordEchoIsInverted() const = 0;

  /**
   * Returns the number of bytes in the keyword sequence.
   */
  virtual int isoKeywordNumBytes() const = 0;

  /**
   * Returns true when the protocol requires that each block have an incrementing
   * sequence number in byte position 1.
   */
  virtual bool useSequenceNums() const = 0;

  /**
   * Returns the type of trailer required by the protocol on each block.
   */
  virtual BlockTrailerType trailerType() const = 0;

  /**
   * Returns the numeric value of the block title for an empty/ACK block.
   */
  virtual uint8_t blockTitleForEmptyAck() const = 0;

  /**
   * Returns the numeric value of the block title for an ECU ID/info request.
   */
  virtual uint8_t blockTitleForRequestID() const = 0;

  /**
   * Returns true if the last received block was an empty/ACK block (indicating
   * that the other side is free to transmit a new request.)
   */
  virtual bool lastReceivedBlockWasEmpty() const = 0;

  /**
   * Returns true if the last received block was a NACK or other negative
   * response to a request.
   */
  virtual bool lastReceivedBlockWasNack() const = 0;

  /**
   * Returns the maximum number of bytes allowed in the payload of a single block.
   */
  virtual unsigned int maxPayloadSize() const = 0;

  /**
   * Returns true if the provided value represents a block that may be sent from
   * the test equipment; false if it is a block that may only be sent from the ECU.
   */
  virtual bool isValidCommandFromTester(uint8_t type) const { return true; }

  /**
   * Returns true if the provided block title and payload are together a valid
   * combination; false otherwise.
   */
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const = 0;

  /**
   * Takes any action that the protocol requires immediately after the keyword
   * byte sequence and before the free exchange of request blocks. For some
   * protocols, this may be acknowledging the receipt of unsolicited ID/info
   * blocks.
   */
  virtual bool doPostKeywordSequence() { return true; }

  int m_baudRate;
  LineType m_slowInitLine = LineType::KLine;
  uint8_t m_sendBlockBuf[256];
  uint8_t m_recvBlockBuf[256];
  uint8_t m_lastUsedSeqNum = 0;
  uint8_t m_lastReceivedBlockTitle = 0;
  std::vector<uint8_t> m_lastReceivedPayload;

  bool recvBlock(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  bool sendBlock(bool sendBufIsPrepopulated = false);
  bool shutdownRequested() const { return m_shutdown; }
  void processReceivedBlock();

private:
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

  bool readSerial(uint8_t* buf, int count, std::chrono::milliseconds = std::chrono::milliseconds(1000));
  bool writeSerial(uint8_t* buf, int count);

  int getFtdiDeviceInfo(ftdi_device_list* list, int count, std::vector<FtdiDeviceInfo>& deviceInfo);
};

