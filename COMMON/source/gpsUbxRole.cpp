#include <algorithm>

#include "gpsUbxRole.hpp"
#include "hardwareConf.hpp"
#include "ressourceManager.hpp"

#include "stdutil++.hpp"



/*
  dans le subscribe : rien à faire : on ne reçoit pas de messages UAVCan

  dans le start :
  + demarrer la liaison série avec les bon paramètres
  + creer un thread qui :
   ° écoute sur la liaison série avec un buffer de 600 et un timout sur 1 ou 2 caractères
   ° nourrit un objet gpsUbx
   ° dans la CB de gpsUbx : si le message est complet/correct :
     construire un message  et l'envoyer via UAVCAN
 */

namespace {
 UARTConfig gpscfg =
  {
    .txend1_cb =nullptr,
    .txend2_cb = nullptr,
    .rxend_cb = nullptr,
    .rxchar_cb = nullptr,
    .rxerr_cb = nullptr,
    // .timeout_cb = [](hal_uart_driver *) {
    //   palSetLine(LINE_DBG_TIMOUT_CB);
    //   chSysPolledDelayX(1);
    //   palClearLine(LINE_DBG_TIMOUT_CB);
    // },
    .timeout_cb = nullptr,
    .timeout = 6,
    .speed = 0, // will be be at init
    .cr1 = USART_CR1_RTOIE, 
    .cr2 = USART_CR2_STOP1_BITS | USART_CR2_RTOEN,
    .cr3 = 0
  };

}


// It is assumed that thread live forever (until poweroff) dynamic memory allocation is
// done there to not use ram if the role is not enabled. It's normal that the memory is never
// released
DeviceStatus GpsUBX::subscribe(UAVCAN::Node& node)
{
  m_node = &node;

  gpscfg.speed =  PARAM_CGET("role.gnss.ubx.baudrate");
  uartStart(&ExternalUARTD, &gpscfg);
  return DeviceStatus(DeviceStatus::GPS_ROLE);
}


DeviceStatus GpsUBX::start(UAVCAN::Node&)
{
  using HR = HWResource;
  DeviceStatus status(DeviceStatus::GPS_ROLE);
  
  
  // use serial2 tx/rx
  if (not boardResource.tryAcquire(HR::USART_2, HR::PB03, HR::PB04)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
  
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(1024), "gps", NORMALPRIO, 
		      &Trampoline<&GpsUBX::periodic>::fn, this);

  return status;
}

void GpsUBX::periodic(void *)
{
  size_t size;
  while (true) {
    size = sizeof(frame);
    uartReceiveTimeout(&ExternalUARTD, &size, frame, TIME_INFINITE);
    if (size) {
      // feed baby feed
    }
  }
}


