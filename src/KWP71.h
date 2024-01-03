#include "BlockExchangeProtocol.h"

enum class KWP71BlockType
{
  RequestID         = 0x00,
  ReadRAM           = 0x01,
  WriteRAM          = 0x02,
  ReadROM           = 0x03,
  ActivateActuator  = 0x04,
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

class KWP71 : public BlockExchangeProtocol
{
public:
  explicit KWP71(bool verbose);
  explicit KWP71(int baudRate, bool verbose);

  bool activateActuator(uint8_t index);
  bool readRAM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readEEPROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool writeRAM(uint16_t addr, const std::vector<uint8_t>& data);
  bool writeEEPROM(uint16_t addr, const std::vector<uint8_t>& data);
  bool readFaultCodes(std::vector<uint8_t>& data);
  bool eraseFaultCodes();

protected:
  virtual inline bool bytesEchoedDuringBlockReceipt() const override { return true; }
  virtual inline int initDataBits() const override { return 8; }
  virtual inline int initParity() const override { return 0; }
  virtual inline int timeBeforeReconnectMs() const override { return 260; }
  virtual inline int isoKeywordIndexToEcho() const override { return 2; }
  virtual inline bool isoKeywordEchoIsInverted() const override { return true; }
  virtual inline int isoKeywordNumBytes() const override { return 3; }
  virtual inline bool useSequenceNums() const { return true; }
  virtual inline BlockTrailerType trailerType() const { return BlockTrailerType::Fixed03; }
  virtual inline uint8_t blockTitleForEmptyAck() const { return static_cast<uint8_t>(KWP71BlockType::Empty); }
  virtual inline uint8_t blockTitleForRequestID() const { return static_cast<uint8_t>(KWP71BlockType::RequestID); }

  virtual bool doPostKeywordSequence();
  virtual bool lastReceivedBlockWasEmpty() const;
  virtual bool lastReceivedBlockWasNack() const;
  virtual void processReceivedBlock();
  virtual bool isValidCommandFromTester(uint8_t type) const override;
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const override;

private:
  KWP71BlockType m_lastReceivedBlockType = KWP71BlockType::Empty;

  bool populateBlock(bool& usedPendingCommand);
};

