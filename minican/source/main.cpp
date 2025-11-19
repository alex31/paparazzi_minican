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
#include "deviceRessource.hpp"
#include "UAVCanSlave.hpp"


/*
  TODO :

  ° role dshot
    + paramètres : mapping, nb_canaux, cmd_rate [100 .. 1000], rpm_freq_divider (0 .. 1000 : 0 is disabled)
    + message UAVCan : equipment/esc/1030.RawCommand
    
  
  ° role proxy Sbus : testé
  
  ° role proxy pprz-link
    + Pour du test -> une devboardH7 joue le role d'AP et de recepteur SBUS
      * Telemetrie : forger des trames et les balancer sur une liaison série
      * AP : protocole UAVCAN avec le bon bitrate CAN
        ¤ abonné à tunnel.telemetry
	¤ on check qu'on reçoit les bonnes trames, et autant qu'on en a envoyé

 
 */

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
  void  watchdogReset (void *) {
    wdgStart(&WDGD1, &wdgcfg);
    while(true) {
      wdgReset(&WDGD1);
      chThdSleepMilliseconds(100);
    }
  }
#endif
}

void _init_chibios() __attribute__ ((constructor(101)));
void _init_chibios() {
  halInit();
  chSysInit();
  initHeap ();
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
    goto end;
  }

  Adc::start([](float psBat, float coreTemp) {
#ifdef TRACE
    DebugTrace("psBat = %.2f, coreTemp = %.1f", psBat, coreTemp);
#else
    (void) psBat;
    (void) coreTemp;
#endif
  });

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

  if (PARAM_CGET("role.identification") == true) {
    // mode identification
    DebugTrace ("passage en mode identification");
    RgbLed::setMotif(150, 0b1010100000000000);
    RgbLed::setColor(HSV{0.8, 1, 0.5});
    goto end;
  }

  {
    const int8_t nodeId = PARAM_CGET("uavcan.node_id");
    DebugTrace("paramètre nodeid = %d", nodeId);
    if (const DeviceStatus status =
	CANSlave::start(nodeId, PARAM_CGET("uavcan.dynid.fd")); not status) {
      DebugTrace("CANSlave error is %s", status.describe().c_str());
      if (status.err == DeviceStatus::CONFLICT) {
	RgbLed::setMotif(100, 0b110011000);
	RgbLed::setColor(HSV{0.0, 1, 0.5});
      } else {
	RgbLed::setMotif(200, 0b10101010);
	RgbLed::setColor(HSV{0.1, 1, 0.5});
      }
      goto end;
    } 
    
    RgbLed::setNodeId(CANSlave::getNodeId());
  }
 end:
  chThdSleep(TIME_INFINITE);
}
