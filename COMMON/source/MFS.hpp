/**
 * @file MFS.hpp
 * @brief Minimal flash storage (MFS) wrapper for the external EEPROM.
 */
#pragma once

#include "hal.h"
#include "hal_mfs.h"
#include <unistd.h>
#include "eeprom_stm_m95p.hpp"


namespace MFS {
  /** @brief Initialize the MFS driver and underlying flash device. */
  mfs_error_t start();
  /** @brief Stop the MFS driver. */
  void stop();
  /** @brief Write a record by index. */
  bool write(size_t recordIndex, const void *data, size_t size);
  /** @brief Read a record by index. */
  bool read(size_t recordIndex, void *data, size_t& size);
  /** @brief Return the highest valid record index (0 if none). */
  size_t getlen();
  /** @brief Erase all records. */
  bool eraseAll();
  /** @brief Return the underlying EEPROM device instance. */
  Eeprom_M95::Device *getDevice();
}
