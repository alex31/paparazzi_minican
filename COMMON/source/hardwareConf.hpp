#pragma once
#include <stdint.h>
#include <array>
#include <string>
#include "ch.h"
#include "hal.h"
#include "UAVCAN/persistantParam.hpp"
#include "UAVCAN/persistantStorage.hpp"


/*
#                 ______   _____         
#                /  ____| |_   _|        
#                | (___     | |          
#                 \___ \    | |          
#                .____) |  _| |_         
#                \_____/  |_____|        
#                                                              _      _                   
#                                                             | |    (_)                  
#                  ___    ___    _ __   __   __   ___   _ __  | |_    _     ___    _ __   
#                 / __|  / _ \  | '_ \  \ \ / /  / _ \ | '__| | __|  | |   / _ \  | '_ \  
#                | (__  | (_) | | | | |  \ V /  |  __/ | |    \ |_   | |  | (_) | | | | | 
#                 \___|  \___/  |_| |_|   \_/    \___| |_|     \__|  |_|   \___/  |_| |_| 
*/

using namespace std::literals;

static constexpr uint32_t operator""_hz (unsigned long long int freq)
{
  return freq;
}
static constexpr uint32_t operator""_khz (unsigned long long int freq)
{
  return freq * 1000UL;
}
static constexpr uint32_t operator""_mhz (unsigned long long int freq)
{
  return freq * 1000_khz;
}
static constexpr long double operator""_ohm (long double resistance)
{
  return resistance;
}
static constexpr long double operator""_kohm (long double resistance)
{
  return resistance * 1000UL;
}
static constexpr uint32_t operator""_percent (unsigned long long int freq)
{
  return freq * 100UL;
}

/*
#                 _                          _                                           
#                | |                        | |                                          
#                | |__     __ _   _ __    __| |  __      __   __ _   _ __    ___         
#                | '_ \   / _` | | '__|  / _` |  \ \ /\ / /  / _` | | '__|  / _ \        
#                | | | | | (_| | | |    | (_| |   \ V  V /  | (_| | | |    |  __/        
#                |_| |_|  \__,_| |_|     \__,_|    \_/\_/    \__,_| |_|     \___|        
#                                               _                     _            _            
#                                              | |                   (_)          | |           
#                  ___    ___    _ __    ___   | |_    _ __    __ _   _    _ __   | |_          
#                 / __|  / _ \  | '_ \  / __|  | __|  | '__|  / _` | | |  | '_ \  | __|         
#                | (__  | (_) | | | | | \__ \  \ |_   | |    | (_| | | |  | | | | \ |_          
#                 \___|  \___/  |_| |_| |___/   \__|  |_|     \__,_| |_|  |_| |_|  \__|         
*/
#ifdef M95P_RAM_EMULATION
constexpr size_t SECTOR_SIZE_BYTES = 512U;
constexpr size_t MFS_BANK_SIZE_BYTES = (64U * 1024U);
constexpr size_t BLANK_GAP_SIZE_SECTORS = 0U;
#else
constexpr size_t SECTOR_SIZE_BYTES = 512U;
constexpr size_t MFS_BANK_SIZE_BYTES = (256U * 1024U);
constexpr size_t BLANK_GAP_SIZE_SECTORS = ((128U * 1024U) / SECTOR_SIZE_BYTES);
#endif


static constexpr SPIDriver& EepromSPID	  = SPID1;
static inline const SPIConfig eepromSpiCfg = {
    .circular       = false,
    .slave          = false,
    .data_cb        = nullptr,
    .error_cb       = nullptr,
    .ssline         = LINE_SPI_EEPROM_CS,
    .cr1            = SPI_CR1_BR_0, // 42Mhz
    .cr2            = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};  

// Common uart on the minican and microcan connectors
#if HAL_USE_UART
static constexpr UARTDriver& RoleUartDriver = UARTD2;
#endif

/*
#                                                    
#                                                    
#                 _   _   ___     ___   _ __         
#                | | | | / __|   / _ \ | '__|        
#                | |_| | \__ \  |  __/ | |           
#                 \__,_| |___/   \___| |_|           
#                            _    _    _              _       _                 
#                           | |  (_)  | |            | |     | |                
#                  ___    __| |   _   | |_     __ _  | |__   | |    ___         
#                 / _ \  / _` |  | |  | __|   / _` | | '_ \  | |   / _ \        
#                |  __/ | (_| |  | |  \ |_   | (_| | | |_) | | |  |  __/        
#                 \___|  \__,_|  |_|   \__|   \__,_| |_.__/  |_|   \___|        
*/
static constexpr int maxCoreTemp = 60;
static constexpr float psBatMin = 6.5;
static constexpr float psBatMax = 25.2;
static constexpr float ps3vMax = 3.5;
static constexpr float ps3vMin = 3.0;
static constexpr float coreTempMin = -10.0f;
static constexpr float coreTempMax = 60.0f;
