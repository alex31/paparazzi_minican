/**
 * @file main.cpp
 * @brief Application entry point and system initialization.
 */
#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "ttyConsole.hpp"
#include "led2812.hpp"
#include "rgbLeds.hpp"
#include "adcSurvey.hpp"
#include "dynamicPinConfig.hpp"
#include "MFS.hpp"
#include "UAVCAN/persistantParam.hpp"
#include "UAVCAN/persistantStorage.hpp"
#include "deviceResource.hpp"
#include "UAVCanSlave.hpp"
#include "hardwareConf.hpp"


namespace {
  /*
    watchdog reset : reset in a low priority thread, so it should catch
    ° hardware fault
    ° RT stalled
    ° thread wild loop that use 100% CPU
   */
#ifdef __OPTIMIZE__
  const WDGConfig wdgcfg = {
    .pr  = STM32_IWDG_PR_32,
    .rlr = STM32_IWDG_RL(300),  // timeout ≈ 300 ms
    .winr = STM32_IWDG_WIN_DISABLED,
  };
  
  THD_WORKING_AREA(waWatchdogReset, 256) __attribute__((section(FAST_SECTION "_clear")));
  /** @brief Periodically reset the watchdog to detect system stalls. */
  void  watchdogReset (void *) {
    wdgStart(&WDGD1, &wdgcfg);
    while(true) {
      wdgReset(&WDGD1);
      chThdSleepMilliseconds(100);
    }
  }
#endif
}

/** @brief Early initializer to set up ChibiOS and the heap. */
void _init_chibios() __attribute__ ((constructor(101)));
void _init_chibios() {
  halInit();
  chSysInit();
  initHeap ();
}


/** @brief Main application entry point. */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
#ifdef __OPTIMIZE__
  chThdCreateStatic(waWatchdogReset, sizeof(waWatchdogReset), NORMALPRIO + 1, &watchdogReset, NULL);
#endif
  RgbLed::start();
  trngStart(&TRNGD1, NULL);
  
#ifdef TRACE
  consoleInit();
  consoleLaunch();
#endif
  
  RgbLed::setColor(HSV{0.3, 1, 0.1});
  RgbLed::setMotif(500, 0b1010101010101010);
  if (not DynPin::isFirmwareMatchHardware()) {
    RgbLed::setWheelOfDeath();
    for (int i = 0; i < 5; i++)
      DebugTrace ("FATAL : FIRMWARE do not match HARDWARE");
    goto end;
  }


  if (mfs_error_t status = MFS::start(); status != MFS_NO_ERROR) {
    DebugTrace("MFS::start has failed with code %d", status);
    RgbLed::setMotif(200, 0b10101010);
    RgbLed::setColor(HSV{0.6, 1, 0.2});
    goto end;
  }
  DebugTrace("MFS::start OK");

  if (not Ressource::storage.start()) {
    DebugTrace ("storage start *FAILED*");
    RgbLed::setMotif(200, 0b10101010);
    RgbLed::setColor(HSV{0.8, 1, 0.2});
    goto end;
  }

  // depend on Ressource::storage.start()
  Adc::start();
  
  if (param_cget<"ROLE.identification">() == true) {
    // mode identification
    DebugTrace ("passage en mode identification");
    RgbLed::setMotif(150, 0b1010100000000000);
    RgbLed::setColor(HSV{0.8, 1, 0.5});
    goto end;
  }

  {
    const int8_t nodeId = param_cget<"uavcan.node_id">();
    DebugTrace("paramètre nodeid = %d", nodeId);
    if (const DeviceStatus status =
	CANSlave::start(nodeId, param_cget<"uavcan.dynid.fd">()); not status) {
      if (status.err == DeviceStatus::CONFLICT) {
	RgbLed::setMotif(100, 0b110011000);
	RgbLed::setColor(HSV{0.0, 1, 0.5});
      } else {
	RgbLed::setMotif(200, 0b10101010);
	RgbLed::setColor(HSV{0.1, 1, 0.5});
      }
      goto end;
    } 
  }
   
  RgbLed::setNodeId(CANSlave::getNodeId());
  Adc::setErrorCB([](float psBat, float coreTemp) {
    // four LSB bits of first byte are for the actual status
    // four MSB bits of first byte keep trace of all event since powered up
    static uint16_t mask = 0;
    uint16_t current = 0;
    if (psBat < psBatMin)
      current |= SPEC_PSBAT_UNDERVOLT_CURRENT;
    if (psBat > psBatMax)
      current |= SPEC_PSBAT_OVERVOLT_CURRENT;
    if (coreTemp > coreTempMax)
      current |= SPEC_CORETEMP_OVER_CURRENT;
    if (coreTemp < coreTempMin)
      current |= SPEC_CORETEMP_UNDER_CURRENT;

    mask = (mask & SPEC_LIFETIME_MASK) | current;
    mask |= static_cast<uint16_t>(current << 4);

    // if we reset from a watchdog reset,
    // we reflect the problem on the nodeInfo specific code
    // TODO : add a mechanism that detect close watchdog reset events
    // to disable ROLE to make UAVCAN update of a fixed firmware possible
    if (RCC->CSR & (RCC_CSR_WWDGRSTF |  RCC_CSR_IWDGRSTF)) {
	mask |= SPEC_WATCHDOG_RESET_FIRED;
	RCC->CSR |= RCC_CSR_RMVF;
    }

    // When node mode is OPERATIONAL, specific_code carries this bitmask.
    CANSlave::getInstance().setSpecificCode(mask);
  });
    
 end:
  chThdSleep(TIME_INFINITE);
}
