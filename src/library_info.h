#include <cstdint>

struct BlockProtocolLibraryVersion
{
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
};

BlockProtocolLibraryVersion getLibraryVersion();

