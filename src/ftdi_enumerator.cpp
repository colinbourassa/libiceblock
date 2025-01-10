#include <libusb.h>
#include "ftdi_enumerator.h"

/**
 * Retrieves bus number, device number, and manufacturer description for each
 * FTDI device in the provided ftdi_device_list. Returns the number of devices
 * for which the identifying information was successfully read.
 */
int getFtdiDeviceInfo(struct ftdi_context* ftdiContext, ftdi_device_list* list, int listCount, std::vector<FtdiDeviceInfo>& deviceInfo)
{
  constexpr int usbStrLen = 256;
  char manufacturer[usbStrLen];
  char description[usbStrLen];
  char serial[usbStrLen];
  int index = 0;
  int count = 0;

  while (list && (index < listCount))
  {
    if (ftdi_usb_get_strings(ftdiContext, list->dev,
                             manufacturer, usbStrLen,
                             description, usbStrLen,
                             NULL, 0) == 0)
    {
      const uint8_t bus_num = libusb_get_bus_number(list->dev);
      const uint8_t device_addr = libusb_get_device_address(list->dev);
      deviceInfo.emplace_back(bus_num, device_addr, manufacturer, description, "");
      count++;
    }
    index++;
    list = list->next;
  }

  return count;
}

/**
 * Identifies all FTDI USB devices connected to the system by checking for known
 * combinations of vendor IDs and product IDs, including certain third-party
 * IDs and any IDs provided in the extraPids parameter.
 */
std::vector<FtdiDeviceInfo> enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extraPids)
{
  struct ftdi_context ftdiContext;
  ftdi_device_list* list;
  int count = 0;
  std::vector<FtdiDeviceInfo> deviceInfo;

  ftdi_init(&ftdiContext);

  // First call used VID:PID of 0:0 to search for all standard VID:PID
  // combinations known to libftdi.
  count = ftdi_usb_find_all(&ftdiContext, &list, 0, 0);
  getFtdiDeviceInfo(&ftdiContext, list, count, deviceInfo);
  ftdi_list_free(&list);

  // Combine a list of any extra known VID/PID pairs with the list provided
  // by the caller (if any).
  std::set<std::pair<uint16_t,uint16_t>> thirdPartyPids =
  {
    { 0x0403, 0xfa20 } // Ross-Tech interface
  };
  thirdPartyPids.insert(extraPids.begin(), extraPids.end());

  for (auto vidPidPair : thirdPartyPids)
  {
    count = ftdi_usb_find_all(&ftdiContext, &list, vidPidPair.first, vidPidPair.second);
    getFtdiDeviceInfo(&ftdiContext, list, count, deviceInfo);
    ftdi_list_free(&list);
  }
  ftdi_deinit(&ftdiContext);

  return deviceInfo;
}

