#include <libftdi1/ftdi.h>
#include <chrono>
#include <thread>
#include <stdio.h>

struct ftdi_context g_ftdi;

bool readSerial(uint8_t* buf, int count)
{
  int readCount = 0;
  int status = 0;
  std::chrono::time_point start = std::chrono::steady_clock::now();

  do {
    status = ftdi_read_data(&g_ftdi, buf + readCount, count - readCount);
    if (status > 0)
    {
      readCount += status;
    }
  } while ((readCount < count) &&
           (status >= 0) &&
           ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start))
             < std::chrono::milliseconds(1000)));
  
  return (readCount == count);
}

int main()
{
  ftdi_init(&g_ftdi);
  ftdi_set_interface(&g_ftdi, INTERFACE_A);
  if ((ftdi_usb_open(&g_ftdi, 0x0403, 0x6001) == 0))
  {
    printf("ftdi_tciflush() returned %d\n", ftdi_tciflush(&g_ftdi));
    printf("enabling bitbang\n");
    ftdi_set_bitmode(&g_ftdi, 0x01, BITMODE_BITBANG);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("disabling bitbang\n");
    ftdi_set_bitmode(&g_ftdi, 0x01, BITMODE_RESET);
    printf("ftdi_tciflush() returned %d\n", ftdi_tciflush(&g_ftdi));

    uint8_t tmpByte;
    while (ftdi_read_data(&g_ftdi, &tmpByte, 1) > 0);

    int count = 260;
    while (count > 0)
    {
      if (readSerial(&tmpByte, 1))
      {
        printf(" %02x", tmpByte);
      }
      else
      {
        printf(" xx");
      }
      fflush(stdout);
      count--;
    }
    printf("\n");
    ftdi_usb_close(&g_ftdi);
  }
  ftdi_deinit(&g_ftdi);

  return 0;
}

