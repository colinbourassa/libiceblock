#include "BlockExchangeProtocol.h"

enum class Marelli1AFBlockType : uint8_t
{
  // Special blocks used only during post-init sequence
  HostBlock         = 0x00,
  HostBlockResponse = 0x0D,
  SelectBlock       = 0x34,
  SelectBlockNACK   = 0x38,

  // General use blocks
  //SetDiagnosticMode = 0x01, // TODO: Response type is listed as 0x0D - is that correct?
  StopCommunication = 0x05,
  EmptyAck          = 0x09,
  NACK              = 0x0A,
  NotSupported      = 0x0B,
  Wait              = 0x0C,
  ActivateActuator  = 0x20,
  StopActuator      = 0x21,
  CheckActuator     = 0x22,
  ReadMemoryCell    = 0x30,
  ReadValue         = 0x31,
  ReadSnapshot      = 0x32,
  ReadADCChannel    = 0x33,
  WriteRAM          = 0x35,
  ReadErrorMemory   = 0x50,
  ReadIDCode        = 0x51,
  ReadErrorValue    = 0x52,
  ClearErrorMemory  = 0x60,

  ReadErrorValueResponse  = 0xAD,
  ReadIDCodeResponse      = 0xAE,
  ReadErrorMemoryResponse = 0xAF,
  ReadADCChannelResponse  = 0xCC,
  ReadSnapshotResponse    = 0xCD,
  ReadValueResponse       = 0xCE,
  ReadMemoryCellResponse  = 0xCF,
  CheckActuatorError      = 0xDD
};

class Marelli1AF : public BlockExchangeProtocol
{
public:
  Marelli1AF(bool verbose);

protected:
  virtual inline bool bytesEchoedDuringBlockReceipt() const override { return false; }
  virtual inline int initDataBits() const override { return 7; }
  virtual inline int initParity() const override { return 1; }
  virtual inline int timeBeforeReconnectMs() const override { return 550; }
  virtual inline int isoKeywordIndexToEcho() const override { return -1; }
  virtual inline bool isoKeywordEchoIsInverted() const override { return false; }
  virtual inline int isoKeywordNumBytes() const override { return 6; }
  virtual inline bool useSequenceNums() const override { return false; }
  virtual inline BlockTrailerType trailerType() const override { return BlockTrailerType::Checksum16Bit; }
  virtual inline uint8_t blockTitleForEmptyAck() const override { return static_cast<uint8_t>(Marelli1AFBlockType::EmptyAck); }
  virtual inline uint8_t blockTitleForRequestID() const override { return static_cast<uint8_t>(Marelli1AFBlockType::ReadIDCode); }
  virtual bool lastReceivedBlockWasEmpty() const override;
  virtual bool lastReceivedBlockWasNack() const override;

  virtual bool isValidCommandFromTester(uint8_t type) const override;
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const override;
  virtual bool doPostKeywordSequence() override;

private:
  bool sendSelectBlock();
  bool waitForSelectBlockResponse();
  bool sendHostBlock();
  bool waitForHostBlockResponse();
};

