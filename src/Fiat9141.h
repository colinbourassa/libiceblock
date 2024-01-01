#include "KWP71.h"

class Fiat9141 : public KWP71
{
public:
  Fiat9141(bool verbose);

protected:
  virtual inline bool bytesEchoedDuringBlockReceipt() const override { return false; }
  virtual inline int baudRate() const override { return 4800; }
  virtual inline int initDataBits() const override { return 7; }
  virtual inline int initParity() const override { return 1; }
  virtual inline int timeBeforeReconnectMs() const override { return 2050; }
  virtual inline int isoKeywordNumBytes() const override { return 6; };
  virtual inline bool useSequenceNums() const override { return false; }
};

