#include <stdio.h>
#include "kwp71.h"

void usage(kwp71_version ver, const char* name)
{
  printf("kwp71read using libkwp71 v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
  printf("Usage: %s <serial device>\n", name);
}

int main(int argc, char** argv)
{
  kwp71_version ver;
  kwp71_info info;
  kwp71_ecuinfo ecuinfo;
  int status = 0;

  ver = kwp71_getLibraryVersion();

  if (argc < 2)
  {
    usage(ver, argv[0]);
    return 0;
  }

  kwp71_init(&info);

  if (kwp71_connect(&info, argv[1]))
  {
    printf("Connected.\n");
    kwp71_disconnect(&info);
  }
  else
  {
    printf("Error: could not connect to ECU on serial device %s.\n", argv[1]);
    status = -1;
  }

  return status;
}

