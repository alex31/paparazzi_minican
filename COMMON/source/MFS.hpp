#pragma once

#include "hal.h"
#include "hal_mfs.h"
#include <unistd.h>
#include "eeprom_stm_m95p.hpp"


namespace MFS {
  mfs_error_t	start();
  void		stop();
  bool		write(size_t recordIndex, const void *data, size_t size);
  bool		read(size_t recordIndex, void *data, size_t& size);
  size_t	getlen();
  bool		eraseAll();
  Eeprom_M95::Device *getDevice();
}
