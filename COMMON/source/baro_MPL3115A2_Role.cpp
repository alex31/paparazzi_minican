#include "baro_MPL3115A2_Role.hpp"
#include "stdutil.h"
#include "dynamicPinConfig.hpp"
#include "ressourceManager.hpp"

namespace {
  constexpr uint32_t  STM32_CR1_DNF(uint32_t n) {
    return (n << I2C_CR1_DNF_Pos) & I2C_CR1_DNF_Msk;
  }
  static constexpr uint32_t  OVERSAMPLING = 0x0; // no oversampling conv in 6 ms
  static constexpr uint8_t mplAdr =  0x60;
  static constexpr uint8_t statusReg[] = {0x00};
  static constexpr uint8_t pressureReg[] = {0x01};
  static constexpr uint8_t tempReg[] = {0x04};
  static constexpr uint8_t chipId[] = {0x04};
  static constexpr uint8_t oneShotMode[] = {0x26, 0x2 | OVERSAMPLING};
  static constexpr uint8_t enableEvent[] = {0x13, 0x07};
  static constexpr uint8_t devIdReg[] = {0x0c};

  static constexpr I2CConfig i2ccfg_400  = {
    .timingr = 0xC0310612, // 400 Mhz, FAST DNF(8)
    .cr1 = STM32_CR1_DNF(8), // Digital noise filter activated (timingr should be aware of that)
    .cr2 = 0, // Only the ADD10 bit can eventually be specified here (10-bit addressing mode)
  };
}


void Baro_MPL3115A2_Role::resetI2C1()
{
  const auto config = I2CD1.config;
  i2cStop(&I2CD1);
  if (DynPin::i2cUnhangBus(&I2CD1) == false) {
    DebugTrace("unhang bus I2C1 failed");
  }
     
  i2cStart(&I2CD1, config);
}

void Baro_MPL3115A2_Role::periodic ()
{
  uavcan_equipment_air_data_StaticTemperature msgTemp = {}; 
  uavcan_equipment_air_data_StaticPressure msgPressure = {}; 
  while (true) {
    getPressure(&msgPressure.static_pressure);
    getTemperature(&msgTemp.static_temperature);
    m_node->sendBroadcast(msgPressure, CANARD_TRANSFER_PRIORITY_MEDIUM);
    m_node->sendBroadcast(msgTemp, CANARD_TRANSFER_PRIORITY_MEDIUM);
    chThdSleepMilliseconds(100);
  }
}



DeviceStatus Baro_MPL3115A2_Role::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::MPL3115A2);
}

  
DeviceStatus Baro_MPL3115A2_Role::start(UAVCAN::Node& node)
{
  msg_t status;
  uint8_t devId;

  m_node = &node;
  
  using HR = HWResource;
  
#ifdef     BOARD_ENAC_MINICANv4
  if (not boardResource.try_acquire(HR::PA15, HR::PB07, HR::I2C_1)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
#elifdef BOARD_ENAC_MICROCANv2
  if (not boardResource.try_acquire(HR::I2C_1)) {
    return DeviceStatus(DeviceStatus::RESOURCE, DeviceStatus::CONFLICT);
  }
  DynPin::setScenario(DynPin::Scenario::I2C);
#else
#error only BOARD_ENAC_MINICANv4 and BOARD_ENAC_MICROCANv2 are handled
#endif
  DynPin::i2cActivatePullup();
  i2cStart(&I2CD1, &i2ccfg_400);
  while (not getDevId(&devId)) {
    static int tries = 4;
    chThdSleepMilliseconds(100);
    if (--tries == 0)
      return DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
  }
    
  status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, enableEvent, sizeof(enableEvent),	
				    NULL, 0, 100) ;	
  if (status == MSG_OK) {
    status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, oneShotMode, sizeof(oneShotMode),
				      NULL, 0, 100) ;
  }
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "MPL3115A2", NORMALPRIO, 
		      [](void * arg) {static_cast<Baro_MPL3115A2_Role*>(arg)->periodic();}, this);
  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

DeviceStatus Baro_MPL3115A2_Role::printPressure()
{
#ifdef TRACE
  float pressure;
  const DeviceStatus status = getPressure(&pressure);
  if (status) 
    DebugTrace("Pressure = %.2f millibar", pressure);
  return status;
#else
  return DeviceStatus(DeviceStatus::MPL3115A2);
#endif
}

DeviceStatus Baro_MPL3115A2_Role::getPressure(float *pressure)
{
  uint8_t rxbuf[4];
  bool  notReady;
  msg_t status;
  uint32_t  rawB;
    
  status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, oneShotMode, sizeof(oneShotMode),	
				    NULL, 0, TIME_MS2I(100)) ;
  if (status !=  MSG_OK) {
    DebugTrace("baroGetPressure I²C error");
    return DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
  }
  
  do {
    status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, statusReg, sizeof(statusReg),	
				      rxbuf, 2, TIME_MS2I(100)) ;
    notReady = !(rxbuf[0] & 1<<2);
    if (notReady) {
      chThdSleepMilliseconds(2);
    }
  } while ((notReady) && (status == MSG_OK));

  if (status == MSG_OK) {
    status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, pressureReg, sizeof(pressureReg),	
				      (uint8_t *)&rawB, 3, TIME_MS2I(100)) ;
  }

  if (status == MSG_OK) {
    // pressure in rawB is on 24 bits, so it is safe to right shift these 24 bits
    const uint32_t swapVal = (SWAP_ENDIAN32(rawB<<8)) ;
    *pressure = swapVal / 6400.0f;
  }

  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

DeviceStatus Baro_MPL3115A2_Role::getTemperature(float *temperature)
{
  int16_t mplTemp;
    
  msg_t status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, enableEvent, sizeof(enableEvent),	
					  NULL, 0, 100);	
  if (status == MSG_OK)
    status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, tempReg, sizeof(tempReg),	
				      (uint8_t *) &mplTemp, sizeof(mplTemp), 100);
  if (status == MSG_OK) {
    *temperature = SWAP_ENDIAN16(mplTemp) / 256.0f;
  } else {
    DebugTrace("i2cMasterTransmitTimeout error");
    resetI2C1();
  }
  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

DeviceStatus Baro_MPL3115A2_Role::getDevId(uint8_t *devId)
{
  msg_t status = i2cMasterTransmitTimeout(&I2CD1, mplAdr, devIdReg, sizeof(devIdReg),	
					  devId, sizeof(*devId), 100);
  if (status != MSG_OK) {
    DebugTrace("getDevId : i2cMasterTransmitTimeout error");
    resetI2C1();
  }
  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

