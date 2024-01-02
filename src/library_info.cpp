#include "library_info.h"
#include "iceblock_version.h"

/**
 * Returns the version of this library build.
 */
IceblockVersion getIceblockVersion()
{
  IceblockVersion ver;

  ver.major = ICEBLOCK_VER_MAJOR;
  ver.minor = ICEBLOCK_VER_MINOR;
  ver.patch = ICEBLOCK_VER_PATCH;

  return ver;
}

