#include "library_info.h"
#include "kwp71_version.h"

/**
 * Returns the version of this library build.
 */
BlockProtocolLibraryVersion getLibraryVersion()
{
  BlockProtocolLibraryVersion ver;

  ver.major = KWP71_VER_MAJOR;
  ver.minor = KWP71_VER_MINOR;
  ver.patch = KWP71_VER_PATCH;

  return ver;
}

