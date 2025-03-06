#pragma once
#include "KWP71.h"

/**
 * Implementation of the FIAT-9141 protocol. This is mostly identical to KWP-71,
 * with the following exceptions:
 *  - The slow init sequence is done with 7 data bits and odd parity (as opposed to 8/none)
 *  - The ISO keyword sequence is 6 bytes long instead of 3
 *  - Blocks are not echoed byte-for-byte by the receiver
 *  - Block sequence numbers are not used (so each block is one byte shorter than it would be otherwise)
 *  - The timeout (i.e. time before reconnect/re-init) is 2000ms instead of 250ms
 *  - The trailing byte in each block is an 8-bit checksum (rather than fixed at 0x03)
 */
class Fiat9141 : public KWP71
{
public:
  Fiat9141(int baud, LineType initLine, bool verbose);

protected:
  virtual bool bytesEchoedDuringBlockReceipt() const override { return false; }
  virtual int initDataBits() const override { return 7; }
  virtual int initParity() const override { return 1; }
  virtual int timeBeforeReconnectMs() const override { return 2050; }
  virtual int isoKeywordNumBytes() const override { return 6; };
  virtual bool useSequenceNums() const override { return false; }
  virtual BlockTrailerType trailerType() const override { return BlockTrailerType::Checksum8Bit; }
  virtual unsigned int maxPayloadSize() const override { return 29; }
};

