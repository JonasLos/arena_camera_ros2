#pragma once
#include <memory>
#include <vector>
#include "ArenaApi.h"

class DeviceInfoHelper
{
 public:
  static size_t get_index_of_serial(std::vector<Arena::DeviceInfo> device_infos,
                                    std::string serial);
  static size_t get_index_of_ip(std::vector<Arena::DeviceInfo> device_infos,
                                std::string ip);
  static std::string info(Arena::DeviceInfo device_info);
};
