#include <cstdint>

struct IceblockVersion
{
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
};

IceblockVersion getIceblockVersion();

