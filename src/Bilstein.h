#pragma once

#include "FixedLengthBlockProtocol.h"

/**
 * Block title values for the Bilstein protocol.
 */
enum class BilsteinBlockType : uint8_t
{
  ReadRAM           = 0x01,
  WriteRAM          = 0x02,
  RequestID         = 0x06,
  Empty             = 0x0A,
  ActivateActuator  = 0x0B,
  ReadFaultMem      = 0x11,
  WriteFaultMemory  = 0x12 // TODO: check this value
};

/**
 * This protocol is used to communicate with the Bilstein adjustable suspension
 * ECU as used on the Ferrari F355 and 550, and possibly also the Corvette C4
 * (with FX3 suspension option.)
 * The blocks are fixed at five bytes long, with the block title in the first
 * position, and the last byte populated with an XOR of the previous four bytes.
 */
class Bilstein : public FixedLengthBlockProtocol
{
public:
  explicit Bilstein(int baudRate, LineType initLine, bool verbose);

  virtual bool readMemory(MemoryType type, uint16_t addr, uint16_t numBytes, std::vector<uint8_t>& data) override;
  virtual bool writeMemory(MemoryType type, uint16_t addr, const std::vector<uint8_t>& data);

protected:
  virtual int blockLength() const { return 5; }
  virtual bool bytesEchoedDuringBlockReceipt() const override { return false; }
  virtual int initDataBits() const override { return 8; }
  virtual int initParity() const override { return 0; }
  virtual int timeBeforeReconnectMs() const override { return 1000; } // arbitrarily selected; no known public protocol spec
  virtual int isoKeywordIndexToEcho() const override { return -1; }
  virtual bool isoKeywordEchoIsInverted() const override { return false; }
  virtual int isoKeywordNumBytes() const override { return 6; }
  virtual BlockTrailerType trailerType() const override { return BlockTrailerType::XOR; }
  virtual uint8_t blockTitleForEmptyAck() const override { return static_cast<uint8_t>(BilsteinBlockType::Empty); }
  virtual uint8_t blockTitleForRequestID() const override { return static_cast<uint8_t>(BilsteinBlockType::RequestID); }
  virtual bool lastReceivedBlockWasEmpty() const override;
  virtual bool lastReceivedBlockWasNack() const override;
  virtual unsigned int maxPayloadSize() const override { return 3; } // fixed block size of 5, minus title and XOR trailer
  virtual bool checkValidityOfBlockAndPayload(uint8_t title, const std::vector<uint8_t>& payload) const override;

private:
  bool readRAM(uint16_t addr, std::vector<uint8_t>& data);
  bool readFaultMemory(uint16_t addr, std::vector<uint8_t>& data);
  bool writeRAM(uint16_t addr, uint8_t data);
  bool writeFaultMemory(uint16_t addr, uint8_t data);
};

