#pragma once

#include "BlockExchangeProtocol.h"

/**
 */
class FixedLengthBlockProtocol : public BlockExchangeProtocol
{
public:
  explicit FixedLengthBlockProtocol(int baudRate, LineType initLine, bool verbose);
  virtual ~FixedLengthBlockProtocol();

protected:
  virtual bool setBlockSections(uint8_t blockTitle, std::vector<uint8_t>& payload) override;
  virtual void setBlockTitle(uint8_t title) override;
  virtual void setBlockPayload(const std::vector<uint8_t>& payload) override;
  virtual bool sendBlock(bool sendBufIsPrepopulated = false) override;
  virtual bool recvBlock(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) override;
  virtual void processReceivedBlock() override;
  virtual int blockLength() const = 0;
};

