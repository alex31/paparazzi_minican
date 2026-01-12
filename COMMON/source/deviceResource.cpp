#include "hardwareConf.hpp"
#include "MFS.hpp"


namespace {
  Persistant::EepromStoreHandle eepromHandle = {
    .writeFn = &MFS::write,
    .readFn = &MFS::read,
    .eraseFn = &MFS::eraseAll,
    .getLen = &MFS::getlen
  };
  
}

namespace Ressource {
  Persistant::Storage storage(eepromHandle);
}
