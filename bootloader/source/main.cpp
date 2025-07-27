#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "ttyConsole.hpp"
#include "led2812.hpp"
#include "rgbLeds.hpp"
#include "M95P/eeprom_stm_m95p.hpp"
#include "firmwareHeader.hpp"
#include "hardwareConf.hpp"

/*
  TODO :


  BOOTLOADER :
  ° check  header @1M
  si flash@next_start -> {
  + calcul CRC32 du firmware stocké dans M95P
  + si CRC correct :
  * copie M95P -> eeprom @address recuperée depuis un marqueur exporté par le linker script
  * flash@next_start = false
  * check CRC32 sur l'eeprom du G4 pour s'assurer que le copie est OK
  * if CRC OK -> {
  ¤ lastFlashStatus = GOOD
  ¤ transfert to application
  } else  {
  ¤ lastFlashStatus = CRC error
  ¤ rbg led pattern -> critical error
  ¤ -> manual action : SWD flash ?
  ¤ wait infinite
  }
  } else { //  flash@next_start == false
  ¤ transfert to application
  }
       
*/

extern uint32_t application_start;

namespace {
  void disable_and_reset_all_stm32g4_rcc_peripherals(void);

  [[noreturn]]
  static void JUMP_TO_AND_RESTART(uint32_t address);

  void flashApplication();

  void _init_chibios() __attribute__ ((constructor(101)));
  void _init_chibios() {
    halInit();
    chSysInit();
  }
  
}



int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  RgbLed::start();
  RgbLed::setColor(HSV{240./360., 1, 0.5});
  RgbLed::setMotif(100, 0b1010100000000000);

#ifdef TRACE
  consoleInit();
  consoleLaunch();
#endif
  flashApplication();
  chThdSleepMilliseconds(20);
  JUMP_TO_AND_RESTART((uint32_t) &application_start);
}





namespace {
  std::array<uint8_t, 16384> buffer;
  Firmware::FirmwareHeader_t firmwareHeader = {};

  // Updates the firmware header in EEPROM
  void updateFirmwareHeader(Eeprom_M95::Device& m95p) {
    m95p.write(firmwareHeader.headerEepromAddr,
               etl::span(reinterpret_cast<const uint8_t*>(&firmwareHeader), sizeof(firmwareHeader)));
  }

  void haltAndCatchFire(systime_t wait) {
    if (wait == TIME_INFINITE) {
      RgbLed::setColor(HSV{0.0f, 1.0f, 0.5f}); // Red
    } else {
      RgbLed::setColor(HSV{60/360.0, 1.0f, 0.5f}); // yellow
    }
    RgbLed::setMotif(100, 0b1010101010101010);
    chThdSleep(wait);
  }

  // Reads and validates the firmware header from EEPROM
  bool readAndValidateHeader(Eeprom_M95::Device& m95p) {
    m95p.read(firmwareHeader.headerEepromAddr,
              etl::span(reinterpret_cast<uint8_t*>(&firmwareHeader),
                        sizeof(firmwareHeader)));

    if (firmwareHeader.magicNumber != firmwareHeader.magicNumberCheck) {
      DebugTrace("magic number not found in header");
      firmwareHeader.state = Firmware::Flash::MAGIC_ERROR;
      return false;
    }

    if (firmwareHeader.headerLen != sizeof(firmwareHeader)) {
      DebugTrace("header structure length mismatch : recompile bootloader and application");
      firmwareHeader.state = Firmware::Flash::LEN_ERROR;
      return false;
    }

    if (firmwareHeader.flashAddress != (uint32_t) &application_start) {
      DebugTrace("application start address mismatch bootloader : 0x%lx application : 0x%lx",
                 firmwareHeader.flashAddress, (uint32_t) &application_start);
      firmwareHeader.state = Firmware::Flash::ADDRESS_MISMATCH;
      return false;
    }
    
    constexpr uint32_t flash_start = 0x08000000;
    uint32_t offset = (uint32_t)&application_start - flash_start;
    if ((firmwareHeader.size < 48000) || (firmwareHeader.size > (512 * 1024) - offset)) {
      firmwareHeader.state = Firmware::Flash::INVALID_SIZE;
      DebugTrace("invalid firmware size");
      return false;
    }
    
    return true;
  }

  // Verifies the CRC of the firmware stored in EEPROM
  bool verifyEepromFirmware(Eeprom_M95::Device& m95p) {
    crcStart(&CRCD1, &Firmware::crcK4Config);
    size_t remain = firmwareHeader.size;
    size_t bytes_read = 0;
    while(remain > 0) {
      const size_t transferSize = std::min(buffer.size(), remain);
      m95p.read(firmwareHeader.bank1EepromAddr + bytes_read,
                std::span(buffer.begin(), transferSize));
      crcCalc(&CRCD1, buffer.data(), transferSize);
      bytes_read += transferSize;
      remain -= transferSize;
    }

    if (crcGetFinalValue(&CRCD1) != firmwareHeader.crc32k4) {
      DebugTrace("Pre-flash CRC check failed. Expected %lu, got %lu",
                 firmwareHeader.crc32k4, crcGetFinalValue(&CRCD1));
      firmwareHeader.state = Firmware::Flash::CRC_ERROR;
      return false;
    }

    DebugTrace("Pre-flash CRC check success");
    return true;
  }

  // Erases the application flash area
  void eraseApplicationFlash() {
    constexpr uint32_t flash_start = 0x08000000;
    constexpr uint32_t sectorSize = 2048;
    uint32_t offset = (uint32_t)&application_start - flash_start;
    const flash_sector_t sectorStart = offset / sectorSize;
    const size_t nbSectorsToErase = (firmwareHeader.size + sectorSize - 1U) / sectorSize;
    
    systime_t now = chVTGetSystemTimeX();
    for(size_t s=0; s < nbSectorsToErase; s++) {
      flash_error_t err = efl_lld_start_erase_sector(&EFLD1, sectorStart + s);
      if (err != FLASH_NO_ERROR) {
        DebugTrace("Error erasing sector %lu", sectorStart + s);
        chThdSleep(TIME_INFINITE);
      }
      while(efl_lld_query_erase(&EFLD1, nullptr) == FLASH_BUSY_ERASING) {
        chThdSleepMilliseconds(STM32_FLASH_WAIT_TIME_MS);
      };
    }
    sysinterval_t elapsed = chTimeDiffX(now, chVTGetSystemTimeX());
    DebugTrace("flash erase sector take %lu milliseconds", chTimeI2MS(elapsed));
  }

  // Programs the flash from EEPROM and verifies the CRC
  bool programFlashAndVerify(Eeprom_M95::Device& m95p) {
    constexpr uint32_t flash_start = 0x08000000;
    uint32_t offset = (uint32_t)&application_start - flash_start;
    flash_error_t err = FLASH_NO_ERROR;

    systime_t now = chVTGetSystemTimeX();
    size_t remain = firmwareHeader.size;
    size_t bytes_written = 0;
    crcReset(&CRCD1); // Re-initialize CRC for the second pass

    while(remain > 0) {
      const size_t transferSize = std::min(buffer.size(), remain);
      m95p.read(firmwareHeader.bank1EepromAddr + bytes_written,
                std::span(buffer.begin(), transferSize));
      err = efl_lld_program(&EFLD1, offset, transferSize, buffer.data());
      chThdYield();
      bytes_written += transferSize;
      offset += transferSize;
      remain -= transferSize;
      crcCalc(&CRCD1, buffer.data(), transferSize);
      if (err != FLASH_NO_ERROR) {
        DebugTrace("Error programming err = %d", err);
        chThdSleep(TIME_INFINITE);
      }
    }

    if (crcGetFinalValue(&CRCD1) != firmwareHeader.crc32k4) {
      DebugTrace("Post-flash CRC error %lu != %lu", crcGetFinalValue(&CRCD1), firmwareHeader.crc32k4);
      firmwareHeader.state =  Firmware::Flash::APPLICATION_CORRUPTED;
      return false;
    }
    
    sysinterval_t elapsed = chTimeDiffX(now, chVTGetSystemTimeX());
    DebugTrace("flash prog sector take %lu milliseconds with err = %d", chTimeI2MS(elapsed), err);
    return true;
  }


  void flashApplication()
  {
    eflStart(&EFLD1, NULL);
    Eeprom_M95::Device m95p(EepromSPID, eepromSpiCfg);
    spiStart(&EepromSPID, &eepromSpiCfg);
    m95p.start();
    crcInit();

    if (!readAndValidateHeader(m95p)) {
      goto exit;
    }

    if (firmwareHeader.state == Firmware::Flash::DONE) {
      return;
    }

    if (firmwareHeader.state == Firmware::Flash::REQUIRED || firmwareHeader.state == Firmware::Flash::STARTED) {
      DebugTrace("New firmware update process started.");
      
      firmwareHeader.state = Firmware::Flash::STARTED;
      
      if (!verifyEepromFirmware(m95p)) {
        goto exit;
      }
      
      eraseApplicationFlash();
      
      if (!programFlashAndVerify(m95p)) {
        goto exit;
      }
      
      firmwareHeader.state = Firmware::Flash::DONE;
      DebugTrace("Firmware update successful.");
    } else {
      DebugTrace("Firmware in an unhandled state: %d", static_cast<int>(firmwareHeader.state));
    }
    
  exit:
    updateFirmwareHeader(m95p);
    switch (firmwareHeader.state) {
    case Firmware::Flash::APPLICATION_CORRUPTED:
      haltAndCatchFire(TIME_INFINITE); break;
    case Firmware::Flash::DONE: break;
    default: haltAndCatchFire(TIME_S2I(10));
    }
  }
    
    
    
    void disable_and_reset_all_stm32g4_rcc_peripherals(void) {
      // --- AHB1 ---
      RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA1EN |
		      RCC_AHB1ENR_DMA2EN |
		      RCC_AHB1ENR_DMAMUX1EN |
		      RCC_AHB1ENR_CRCEN);
    (void)RCC->AHB1ENR;
    RCC->AHB1RSTR |= (RCC_AHB1RSTR_DMA1RST |
		      RCC_AHB1RSTR_DMA2RST |
		      RCC_AHB1RSTR_DMAMUX1RST |
		      RCC_AHB1RSTR_CRCRST);
    RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA1RST |
		       RCC_AHB1RSTR_DMA2RST |
		       RCC_AHB1RSTR_DMAMUX1RST |
		       RCC_AHB1RSTR_CRCRST);

    // --- AHB2 ---
    RCC->AHB2ENR &= ~(RCC_AHB2ENR_ADC12EN |
		      RCC_AHB2ENR_ADC345EN |
		      RCC_AHB2ENR_DAC1EN |
		      RCC_AHB2ENR_DAC3EN |
		      RCC_AHB2ENR_RNGEN);
    (void)RCC->AHB2ENR;
    RCC->AHB2RSTR |= (RCC_AHB2RSTR_ADC12RST |
		      RCC_AHB2RSTR_ADC345RST |
		      RCC_AHB2RSTR_DAC1RST |
		      RCC_AHB2RSTR_DAC3RST |
		      RCC_AHB2RSTR_RNGRST);
    RCC->AHB2RSTR &= ~(RCC_AHB2RSTR_ADC12RST |
		       RCC_AHB2RSTR_ADC345RST |
		       RCC_AHB2RSTR_DAC1RST |
		       RCC_AHB2RSTR_DAC3RST |
		       RCC_AHB2RSTR_RNGRST);

    // --- AHB3 ---
    RCC->AHB3ENR &= ~RCC_AHB3ENR_QSPIEN;
    (void)RCC->AHB3ENR;
    RCC->AHB3RSTR |= RCC_AHB3RSTR_QSPIRST;
    RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST;

    // --- APB1R1 ---
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_FDCANEN |
		       RCC_APB1ENR1_PWREN |
		       RCC_APB1ENR1_I2C1EN |
		       RCC_APB1ENR1_I2C2EN |
		       RCC_APB1ENR1_I2C3EN |
		       RCC_APB1ENR1_SPI2EN |
		       RCC_APB1ENR1_SPI3EN |
		       RCC_APB1ENR1_TIM2EN |
		       RCC_APB1ENR1_TIM3EN |
		       RCC_APB1ENR1_TIM4EN |
		       RCC_APB1ENR1_TIM6EN |
		       RCC_APB1ENR1_TIM7EN |
		       RCC_APB1ENR1_USART2EN |
		       RCC_APB1ENR1_USART3EN |
		       RCC_APB1ENR1_UART4EN |
		       RCC_APB1ENR1_UART5EN |
		       RCC_APB1ENR1_USBEN);
    (void)RCC->APB1ENR1;
    RCC->APB1RSTR1 |= (RCC_APB1RSTR1_FDCANRST |
		       RCC_APB1RSTR1_PWRRST |
		       RCC_APB1RSTR1_I2C1RST |
		       RCC_APB1RSTR1_I2C2RST |
		       RCC_APB1RSTR1_I2C3RST |
		       RCC_APB1RSTR1_SPI2RST |
		       RCC_APB1RSTR1_SPI3RST |
		       RCC_APB1RSTR1_TIM2RST |
		       RCC_APB1RSTR1_TIM3RST |
		       RCC_APB1RSTR1_TIM6RST |
		       RCC_APB1RSTR1_TIM7RST |
		       RCC_APB1RSTR1_USART2RST |
		       RCC_APB1RSTR1_USART3RST |
		       RCC_APB1RSTR1_UART4RST |
		       RCC_APB1RSTR1_UART5RST |
		       RCC_APB1RSTR1_USBRST);
    RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_FDCANRST |
			RCC_APB1RSTR1_PWRRST |
			RCC_APB1RSTR1_I2C1RST |
			RCC_APB1RSTR1_I2C2RST |
			RCC_APB1RSTR1_I2C3RST |
			RCC_APB1RSTR1_SPI2RST |
			RCC_APB1RSTR1_SPI3RST |
			RCC_APB1RSTR1_TIM2RST |
			RCC_APB1RSTR1_TIM3RST |
			RCC_APB1RSTR1_TIM4RST |
			RCC_APB1RSTR1_TIM6RST |
			RCC_APB1RSTR1_TIM7RST |
			RCC_APB1RSTR1_USART2RST |
			RCC_APB1RSTR1_USART3RST |
			RCC_APB1RSTR1_UART4RST |
			RCC_APB1RSTR1_UART5RST |
			RCC_APB1RSTR1_USBRST);

    // --- APB1R2 ---
    RCC->APB1ENR2 &= ~(RCC_APB1ENR2_LPUART1EN);
    (void)RCC->APB1ENR2;
    RCC->APB1RSTR2 |= (RCC_APB1RSTR2_LPUART1RST);
    RCC->APB1RSTR2 &= ~(RCC_APB1RSTR2_LPUART1RST);

    // --- APB2 ---
    RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN |
		      RCC_APB2ENR_TIM1EN |
		      RCC_APB2ENR_TIM8EN |
		      RCC_APB2ENR_TIM15EN |
		      RCC_APB2ENR_TIM16EN |
		      RCC_APB2ENR_TIM17EN |
		      RCC_APB2ENR_TIM20EN |
		      RCC_APB2ENR_USART1EN);
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= (RCC_APB2RSTR_SPI1RST |
		      RCC_APB2RSTR_TIM1RST |
		      RCC_APB2RSTR_TIM8RST |
		      RCC_APB2RSTR_TIM15RST |
		      RCC_APB2RSTR_TIM16RST |
		      RCC_APB2RSTR_TIM17RST |
		      RCC_APB2RSTR_TIM20RST |
		      RCC_APB2RSTR_USART1RST);
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST |
		       RCC_APB2RSTR_TIM1RST |
		       RCC_APB2RSTR_TIM8RST |
		       RCC_APB2RSTR_TIM15RST |
		       RCC_APB2RSTR_TIM16RST |
		       RCC_APB2RSTR_TIM17RST |
		       RCC_APB2RSTR_TIM20RST |
		       RCC_APB2RSTR_USART1RST);
  }

  [[noreturn]]
  static void JUMP_TO_AND_RESTART(uint32_t address) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache();      // Optionnel selon usage
    SCB_DisableDCache();
#endif
#if defined(__ICACHE_PRESENT) && (__ICACHE_PRESENT == 1U)
    SCB_InvalidateICache(); // Optionnel selon usage
    SCB_DisableICache();
#endif
    for(int i=0; i<8; i++)                                    
      NVIC->ICER[i] = 0xFFFFFFFF;
    __set_CONTROL(0); 
    __set_MSP( (uint32_t) (((uint32_t *) address)[0]) ); 
    __DSB();  // Assure l'exécution complète des instructions précédentes
    __ISB();  // Assure que les instructions suivantes sont bien relues
    disable_and_reset_all_stm32g4_rcc_peripherals();
    ( (void (*)(void)) (((uint32_t *) address)[1]) )();
    __builtin_unreachable(); 
  }

}

