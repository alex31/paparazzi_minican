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

#define CONCAT_NX(st1, st2) st1 ## st2
#define CONCAT(st1, st2) CONCAT_NX(st1, st2)

#define STR_(x) #x
#define STR(x)  STR_(x)
#define NAME_STRING(prefix, sep, version) prefix sep version
#if defined(PLATFORM_MINICAN) && PLATFORM_MINICAN
#define DEVICE_NAME NAME_STRING("org.pprz.minican", "V", STR(HW_VERSION))
#elif defined(PLATFORM_MICROCAN) && PLATFORM_MICROCAN
#define DEVICE_NAME NAME_STRING("org.pprz.microcan", "V", STR(HW_VERSION))
#endif
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

#if (defined PLATFORM_MINICAN) && (defined PLATFORM_MICROCAN)
#if PLATFORM_MINICAN
static constexpr SPIDriver& ExternalSPID  = SPID1;
static constexpr I2CDriver& ExternalI2CD  = I2CD1;
#if HAL_USE_UART
static constexpr UARTDriver &ExternalUARTD = CONCAT(UARTD, UART_TX_USART);
#endif
// External WS2812 strip on PB07 (TIM3_CH4)
#define LED2812_TIM      I2C_SDA_TIM
#define LED2812_TIM_CH   I2C_SDA_TIM_CH
static constexpr PWMDriver& LedStripPWMD  = CONCAT(PWMD, LED2812_TIM);
#elif  PLATFORM_MICROCAN
static constexpr SPIDriver& ExternalSPID  = SPID2;
static constexpr I2CDriver& ExternalI2CD  = I2CD2;
#define SRV1_TIM F1_b_TIM
#if HAL_USE_UART
static constexpr UARTDriver& ExternalUARTD =  CONCAT(UARTD, F2_a_USART);
#endif
// External WS2812 strip on F0_b (PB07, TIM3_CH4)
#define LED2812_TIM      F0_b_TIM
#define LED2812_TIM_CH   F0_b_TIM_CH
static constexpr PWMDriver& LedStripPWMD  = CONCAT(PWMD, LED2812_TIM);
#endif
#elif (!defined BOOTLOADER_MCAN)
#error  PLATFORM_MINICAN and/or PLATFORM_MICROCAN and/or BOOTLOADER_MCAN not defined
#endif

static inline const SPIConfig eepromSpiCfg = {
    .circular       = false,
    .slave          = false,
    .data_cb        = nullptr,
    .error_cb       = nullptr,
    .ssline         = LINE_SPI_EEPROM_CS,
    .cr1            = SPI_CR1_BR_0, // 42Mhz
    .cr2            = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};  

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
static constexpr float coreTempMin = -20.0f;
static constexpr float coreTempMax = 60.0f;
