#include <ch.h>
#include <hal.h>
#include "stdutil.h"
#include "ttyConsole.hpp"
#include "led2812.hpp"
#include "rgbLeds.hpp"
#include "M95P/eeprom_stm_m95p.hpp"

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
  RgbLed::start();
  RgbLed::setColor(HSV{240./360., 1, 0.5});
  RgbLed::setMotif(100, 0b1010100000000000);

#ifdef TRACE
  consoleInit();
  consoleLaunch();
#endif

  chThdSleep(TIME_INFINITE);
}
