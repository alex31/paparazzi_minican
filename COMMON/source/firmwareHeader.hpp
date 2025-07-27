/**
 * @file firmwareHeader.hpp
 * @author Alex Bustico
 * @brief Defines the structure and constants for the firmware header.
 * @details This header contains the definition of the `FirmwareHeader_t` struct,
 * which holds metadata about the firmware, such as version, size, CRC, and
 * the current state of the flashing process. It also defines constants used
 * throughout the bootloader and application for firmware verification and
 * management.
 */
#pragma once

#include "etl/string.h"
#include "hal_stm32_crc_v1.h"

namespace Firmware {
  /**
   * @brief Enumerates the possible states of the firmware flashing process.
   */
  enum class Flash {
    REQUIRED,                 ///< A new firmware is present and needs to be flashed.
    STARTED,                  ///< The flashing process has begun.
    DONE,                     ///< The firmware has been successfully flashed.
    CRC_ERROR,                ///< CRC validation of the firmware failed.
    LEN_ERROR,                ///< The firmware header length is incorrect.
    MAGIC_ERROR,              ///< The magic number in the header is incorrect.
    INVALID_SIZE,             ///< The firmware size is out of the expected range.
    ADDRESS_MISMATCH,         ///< The firmware's target flash address is incorrect.
    APPLICATION_CORRUPTED,    ///< The application in flash is corrupted (post-flash check failed).
    PROTOCOL_VERSION_MISMATCH ///< The firmware protocol version is incompatible.
  };

  /**
   * @brief Structure defining the firmware header.
   * @details This structure is stored in a specific location in the external EEPROM
   * and contains all the necessary metadata to manage the firmware update process.
   */
  struct FirmwareHeader_t {
    etl::string<32> version = {}; ///< Firmware version string.
    uint32_t magicNumber;         ///< Magic number to identify a valid header.
    uint16_t versionProtocol = 0; ///< Protocol version for compatibility checks.
    uint16_t headerLen = 0;       ///< Length of this header structure.
    uint32_t size = 0;            ///< Size of the firmware image in bytes.
    uint32_t crc32k4 = 0;         ///< CRC32 checksum of the firmware image.
    uint32_t flashAddress = 0x0;  ///< Target address in the internal flash.
    Flash    state;               ///< Current state of the firmware.
    uint8_t  bankInUse = 0;       ///< Currently active firmware bank (not yet used).
    uint32_t eepromCycleCount = 0;///< Counter for the number of flash cycles.
    
    static constexpr uint32_t magicNumberCheck = 0xF0CACC1A; ///< Expected magic number.
    static constexpr uint32_t versionProtocolCheck = 2;     ///< Expected protocol version.
    static constexpr uint32_t headerEepromAddr = 1024*1024; ///< Address of the header in EEPROM.
    static constexpr uint32_t bank1EepromAddr = headerEepromAddr + 512; ///< Start address of the firmware image in EEPROM.
    static constexpr uint32_t bank2EepromAddr = bank1EepromAddr + (512 * 1024); ///< Start address of the second bank (not yet used).
  };
  
  /**
   * @brief CRC configuration for the CRC32-K4 algorithm.
   */
  static constexpr CRCConfig crcK4Config = {
    .poly_size = 32,
    .poly = 0xC9D204F5,
    .initial_val = 0xFFFFFFFF,
    .final_val = 0xFFFFFFFF,
    .reflect_data = true,
    .reflect_remainder = true
  };
}
