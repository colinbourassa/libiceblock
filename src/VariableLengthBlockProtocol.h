#pragma once

#include "BlockExchangeProtocol.h"

/**
 */
class VariableLengthBlockProtocol : public BlockExchangeProtocol
{
public:
  explicit VariableLengthBlockProtocol(int baudRate, LineType initLine, bool verbose);
  virtual ~VariableLengthBlockProtocol();

protected:
  /**
   * Returns true when the protocol requires that each block have an incrementing
   * sequence number in byte position 1.
   */
  virtual bool useSequenceNums() const = 0;

  virtual uint8_t lastByteIndexOfSendBlock() const override { return m_sendBlockBuf[0]; }
  virtual uint8_t lastByteIndexOfReceiveBlock() const override { return m_recvBlockBuf[0]; }
  virtual bool setBlockSections(uint8_t blockTitle, std::vector<uint8_t>& payload) override;
  virtual void setBlockTitle(uint8_t title) override;
  virtual void setBlockPayload(const std::vector<uint8_t>& payload) override;
  virtual bool sendBlock(bool sendBufIsPrepopulated = false) override;
  virtual bool recvBlock(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) override;
  virtual void processReceivedBlock() override;

private:
  void setBlockSizePrefix(int payloadSize);
  void setBlockSequenceNum();

  uint8_t m_lastUsedSeqNum = 0;
};

