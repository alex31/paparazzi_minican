/**
 * @file baro_MPL3115A2_Role.cpp
 * @brief MPL3115A2 barometer role implementation.
 */
#include "roleConf.h"

#if USE_BARO_MPL3115A2_ROLE

#include "baro_MPL3115A2_Role.hpp"
#include "stdutil.h"
#include "hardwareConf.hpp"
#include "I2C_periph.hpp"

namespace {
  static constexpr uint32_t  OVERSAMPLING = 0x0; // no oversampling conv in 6 ms
  static constexpr uint8_t mplAdr =  0x60;
  static constexpr uint8_t statusReg[] = {0x00};
  static constexpr uint8_t pressureReg[] = {0x01};
  static constexpr uint8_t tempReg[] = {0x04};
  static constexpr uint8_t chipId[] = {0x04};
  static constexpr uint8_t oneShotMode[] = {0x26, 0x2 | OVERSAMPLING};
  static constexpr uint8_t enableEvent[] = {0x13, 0x07};
  static constexpr uint8_t devIdReg[] = {0x0c};

  /** @brief Perform a blocking I2C transfer to the MPL3115A2. */
  msg_t xfer(const uint8_t *tx, size_t txlen, uint8_t *rx, size_t rxlen) {
    i2cAcquireBus(&ExternalI2CD);
    const msg_t ret = i2cMasterTransmitTimeout(&ExternalI2CD, mplAdr, tx, txlen,
                                               rx, rxlen, TIME_MS2I(100));
    i2cReleaseBus(&ExternalI2CD);
    return ret;
  }
}

/** @brief Reset the I2C peripheral in case of bus errors. */
void Baro_MPL3115A2_Role::resetI2C()
{
  I2CPeriph::reset();
}

/** @brief Periodically poll the sensor and publish UAVCAN messages. */
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
/** @brief No subscriptions required for the barometer role. */
DeviceStatus Baro_MPL3115A2_Role::subscribe(UAVCAN::Node&)
{
  return DeviceStatus(DeviceStatus::MPL3115A2);
}

/** @brief Initialize the sensor and spawn the polling thread. */
DeviceStatus Baro_MPL3115A2_Role::start(UAVCAN::Node& node)
{
  msg_t status;
  static IN_DMA_SECTION(uint8_t devId);

  m_node = &node;
  
  if (const auto devstatus = I2CPeriph::start(); not devstatus) {
    return devstatus;
  }
  
  int tries = 4;
  while (not getDevId(&devId)) {
    chThdSleepMilliseconds(100);
    if (--tries == 0)
      return DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
  }
  
  status = xfer(enableEvent, sizeof(enableEvent), nullptr, 0);
  if (status == MSG_OK) {
    status = xfer(oneShotMode, sizeof(oneShotMode), nullptr, 0);
  }
  chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "MPL3115A2", NORMALPRIO, 
		      [](void * arg) {static_cast<Baro_MPL3115A2_Role*>(arg)->periodic();}, this);
  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

/** @brief Print the current pressure value when TRACE is enabled. */
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

/** @brief Read and convert the current pressure measurement. */
DeviceStatus Baro_MPL3115A2_Role::getPressure(float *pressure)
{
  uint8_t rxbuf[4];
  bool  notReady;
  msg_t status;
  uint32_t  rawB;

  status = xfer(oneShotMode, sizeof(oneShotMode), nullptr, 0);
  if (status !=  MSG_OK) {
    DebugTrace("baroGetPressure I²C error");
    return DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
  }
  
  do {
    status = xfer(statusReg, sizeof(statusReg), rxbuf, 2);
    notReady = !(rxbuf[0] & 1<<2);
    if (notReady) {
      chThdSleepMilliseconds(2);
    }
  } while ((notReady) && (status == MSG_OK));

  if (status == MSG_OK) {
    status = xfer(pressureReg, sizeof(pressureReg),
		  reinterpret_cast<uint8_t*>(&rawB), 3);
  }

  if (status == MSG_OK) {
    // pressure in rawB is on 24 bits, so it is safe to right shift these 24 bits
    const uint32_t swapVal = (SWAP_ENDIAN32(rawB<<8)) ;
    *pressure = swapVal / 6400.0f;
  }

  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

/** @brief Read and convert the current temperature measurement. */
DeviceStatus Baro_MPL3115A2_Role::getTemperature(float *temperature)
{
  int16_t mplTemp;

  msg_t status = xfer(enableEvent, sizeof(enableEvent), nullptr, 0);
  if (status == MSG_OK)
    status = xfer(tempReg, sizeof(tempReg),
                  reinterpret_cast<uint8_t*>(&mplTemp), sizeof(mplTemp));
  if (status == MSG_OK) {
    *temperature = SWAP_ENDIAN16(mplTemp) / 256.0f;
  } else {
    DebugTrace("i2cMasterTransmitTimeout error");
    resetI2C();
  }
  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

/** @brief Read the device ID register. */
DeviceStatus Baro_MPL3115A2_Role::getDevId(uint8_t *devId)
{
  msg_t status = xfer(devIdReg, sizeof(devIdReg), devId, sizeof(*devId));
  if (status != MSG_OK) {
    DebugTrace("getDevId : i2cMasterTransmitTimeout error");
    resetI2C();
  }
  return status == MSG_OK ? DeviceStatus(DeviceStatus::MPL3115A2) : DeviceStatus(DeviceStatus::MPL3115A2, DeviceStatus::I2C_TIMOUT);
}

#endif // USE_BARO_MPL3115A2_ROLE
