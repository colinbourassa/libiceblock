#pragma once

#include "VariableLengthBlockProtocol.h"

enum class Marelli1AFBlockType : uint8_t
{
  // Special blocks used only during post-init sequence
  HostBlock         = 0x00,
  OutStatus         = 0x0D, // ACK for HostBlock
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

enum class Marelli1AFPartNumberType : uint8_t
{
  MarelliBolognaDrawingNumber = 0,
  HWSWNumber = 1,
  FIATDrawingNumber = 2
};

/**
 * Marelli's 1AF diagnostic protocol is of the block-exchange type, but it has
 * different block titles than KWP-71 or FIAT-9141, and also requires a
 * particular exchange of special blocks immediately after the initialization
 * sequence.
 *
 * Based on its spec (FIAT document 3.00601), this protocol seems to have been
 * designed specifically for four-cylinder engine ECU diagnostics. However, a
 * variant of this protocol also appears to have been used for the Marelli F1
 * gearbox controller on the Ferrari 355 F1. Some of the protocol's engine-
 * specific features must be unsupported in this configuration.
 */
class Marelli1AF : public VariableLengthBlockProtocol
{
public:
  explicit Marelli1AF(int baudRate, LineType initLine, bool verbose);

  virtual bool activateActuator(uint8_t id, const std::vector<uint8_t>& paramData) override;
  virtual bool stopActuator(uint8_t index) override;
  virtual bool readStoredValue(uint8_t id, std::vector<uint8_t>& data) override;
  virtual bool readMemory(MemoryType type, uint16_t addr, uint16_t numBytes, std::vector<uint8_t>& data) override;
  virtual bool readSnapshot(uint8_t snapshotCode, std::vector<uint8_t>& snapshotData) override;
  virtual bool readADCChannel(const std::vector<uint8_t>& channelList,
                              std::vector<uint8_t>& channelValues) override;
  virtual bool readIDCode(std::vector<std::vector<uint8_t>>& idString) override;
  virtual bool eraseFaultCodes() override;
  virtual bool writeSecurityCode(const std::vector<uint8_t>& securityCode) override;

  // TODO: This is an oddball protocol function that doesn't have an analogue
  // in KWP71 or FIAT9141. We might want to just leave it here rather than make
  // it virtual in the base.
  bool readErrorValue(uint8_t code, std::vector<uint8_t>& data);

protected:
  virtual bool bytesEchoedDuringBlockReceipt() const override { return false; }
  virtual int initDataBits() const override { return 7; }
  virtual int initParity() const override { return 1; }
  virtual int timeBeforeReconnectMs() const override { return 550; }
  virtual int isoKeywordIndexToEcho() const override { return -1; }
  virtual bool isoKeywordEchoIsInverted() const override { return false; }
  virtual int isoKeywordNumBytes() const override { return 6; }
  virtual bool useSequenceNums() const override { return false; }
  virtual BlockTrailerType trailerType() const override { return BlockTrailerType::Checksum16Bit; }
  virtual uint8_t blockTitleForEmptyAck() const override { return static_cast<uint8_t>(Marelli1AFBlockType::EmptyAck); }
  virtual uint8_t blockTitleForRequestID() const override { return static_cast<uint8_t>(Marelli1AFBlockType::ReadIDCode); }
  virtual bool lastReceivedBlockWasEmpty() const override;
  virtual bool lastReceivedBlockWasNack() const override;

  /**
   * NOTE: This -- and other parameters of this protocol -- may be
   * different for non-engine implementations of 1AF (e.g. for the
   * Ferrari F1 gearbox controller.)
   */
  virtual unsigned int maxPayloadSize() const override { return 16; }

  virtual bool isValidCommandFromTester(uint8_t type) const override;
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const override;
  virtual bool doPostKeywordSequence() override;

private:
  bool sendSelectBlock();
  bool waitForSelectBlockResponse();
  bool sendHostBlock();
  bool waitForHostBlockResponse();
};

