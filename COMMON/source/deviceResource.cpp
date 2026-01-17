/**
 * @file deviceResource.cpp
 * @brief Persistent storage instance wiring for device resources.
 */
#include "hardwareConf.hpp"
#include "MFS.hpp"


namespace {
  /// Bridge MFS storage operations to the Persistant storage API.
  Persistant::EepromStoreHandle eepromHandle = {
    .writeFn = &MFS::write,
    .readFn = &MFS::read,
    .eraseFn = &MFS::eraseAll,
    .getLen = &MFS::getlen
  };
  
}

namespace Ressource {
  /// Global storage instance shared by roles.
  Persistant::Storage storage(eepromHandle);
}
