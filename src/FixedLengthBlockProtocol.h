#pragma once

#include "BlockExchangeProtocol.h"

/**
 */
class FixedLengthBlockProtocol : public BlockExchangeProtocol
{
public:
  explicit FixedLengthBlockProtocol(int baudRate, LineType initLine, bool verbose);

protected:
  virtual uint8_t lastByteIndexOfSendBlock() const override { return blockLength() - 1; }
  virtual uint8_t lastByteIndexOfReceiveBlock() const override { return blockLength() - 1; }
  virtual bool setBlockSections(uint8_t blockTitle, std::vector<uint8_t>& payload) override;
  virtual void setBlockTitle(uint8_t title) override;
  virtual void setBlockPayload(const std::vector<uint8_t>& payload) override;
  virtual bool sendBlock(bool sendBufIsPrepopulated = false) override;
  virtual bool recvBlock(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) override;
  virtual void processReceivedBlock() override;
  virtual int blockLength() const = 0;
};

