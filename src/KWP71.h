#pragma once

#include "BlockExchangeProtocol.h"
#include <cstdint>

/**
 * Block title values for KWP-71.
 * There are some ECUs that appear to be using KWP-71 for communication, but
 * that use non-standard block titles (probably for obfuscation of the interface).
 * These include the Ferrari F355 Motronic 5.2, which apparently responds to a
 * block 0x3A request with fault code data.
 */
enum class KWP71BlockType : uint8_t
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
  ROMContent        = 0xFE
};

class KWP71 : public BlockExchangeProtocol
{
public:
  explicit KWP71(int baudRate, LineType initLine, bool verbose);

  bool activateActuator(uint8_t index);
  bool readRAM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool readEEPROM(uint16_t addr, uint8_t numBytes, std::vector<uint8_t>& data);
  bool writeRAM(uint16_t addr, const std::vector<uint8_t>& data);
  bool writeEEPROM(uint16_t addr, const std::vector<uint8_t>& data);
  bool readFaultCodes(std::vector<uint8_t>& data);
  bool eraseFaultCodes();

protected:
  virtual bool bytesEchoedDuringBlockReceipt() const override { return true; }
  virtual int initDataBits() const override { return 8; }
  virtual int initParity() const override { return 0; }
  virtual int timeBeforeReconnectMs() const override { return 260; }
  virtual int isoKeywordIndexToEcho() const override { return 2; }
  virtual bool isoKeywordEchoIsInverted() const override { return true; }
  virtual int isoKeywordNumBytes() const override { return 3; }
  virtual bool useSequenceNums() const override { return true; }
  virtual BlockTrailerType trailerType() const override { return BlockTrailerType::Fixed03; }
  virtual uint8_t blockTitleForEmptyAck() const override { return static_cast<uint8_t>(KWP71BlockType::Empty); }
  virtual uint8_t blockTitleForRequestID() const override { return static_cast<uint8_t>(KWP71BlockType::RequestID); }
  virtual bool lastReceivedBlockWasEmpty() const override;
  virtual bool lastReceivedBlockWasNack() const override;
  virtual bool lastReceivedBlockWasASCII() const override;
  virtual unsigned int maxPayloadSize() const override { return 252; }

  virtual bool doPostKeywordSequence() override;
  virtual bool isValidCommandFromTester(uint8_t type) const override;
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const override;
};

