#pragma once

#include <libftdi1/ftdi.h>
#include <vector>
#include <set>
#include <cstdint>
#include <string>

struct FtdiDeviceInfo
{
  uint8_t busNumber;
  uint8_t deviceAddress;
  std::string manufacturer;
  std::string description;
  std::string serial;
  FtdiDeviceInfo(
    uint8_t _busNumber,
    uint8_t _deviceAddress,
    const std::string& _manufacturer,
    const std::string& _description,
    const std::string& _serial) :
    busNumber(_busNumber),
    deviceAddress(_deviceAddress),
    manufacturer(_manufacturer),
    description(_description),
    serial(_serial) {}
};

int getFtdiDeviceInfo(struct ftdi_context* ftdiContext, ftdi_device_list* list, int listCount, std::vector<FtdiDeviceInfo>& deviceInfo);

/**
 * Queries the USB subsystem for any devices whose VID/PID match known values
 * for FTDI serial devices. Optionally, the caller may provide a list of
 * additional VID/PID pairs that may be treated as FTDI serial devices. This
 * is useful for commercial FTDI-based K-line adapters that have custom IDs.
 */
std::vector<FtdiDeviceInfo> enumerateFtdiDevices(const std::set<std::pair<uint16_t,uint16_t>>& extraPids = {});

