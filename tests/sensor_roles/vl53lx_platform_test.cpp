#include "vl53l4cxPort.h"
#include "vl53lx_platform.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {
  VL53L4CX_Object_t device = {};
  std::array<uint8_t, 260> scratch = {};
  std::array<uint8_t, 260> written = {};
  uint16_t writtenCount = 0U;
  uint32_t writes = 0U;
  uint32_t reads = 0U;
  uint32_t sleptUs = 0U;
  bool failIo = false;

  int32_t writeRegister(uint16_t address, uint8_t *data, uint16_t size)
  {
    assert(address == 0x52U && size <= written.size());
    ++writes;
    if (failIo) { return -1; }
    std::memcpy(written.data(), data, size);
    writtenCount = size;
    return 0;
  }
  int32_t readRegister(uint16_t address, uint8_t *data, uint16_t size)
  {
    assert(address == 0x52U);
    ++reads;
    if (failIo) { return -1; }
    const uint8_t reply[] = {0x12U, 0x34U, 0x56U, 0x78U};
    for (uint16_t i = 0; i < size; ++i) { data[i] = reply[i % 4U]; }
    return 0;
  }
}

extern "C" uint8_t *vl53l4cx_work_buffer(const void *dev, uint32_t size)
{
  return dev == &device && size <= scratch.size() ? scratch.data() : nullptr;
}
extern "C" void vl53l4cx_sleep_us(uint32_t duration) { sleptUs += duration; }

int main()
{
  device.IO.Address = 0x52U;
  device.IO.WriteReg = writeRegister;
  device.IO.ReadReg = readRegister;
  assert(VL53LX_WrDWord(&device, 0xABCDU, 0x12345678U) == VL53LX_ERROR_NONE);
  const uint8_t expected[] = {0xABU, 0xCDU, 0x12U, 0x34U, 0x56U, 0x78U};
  assert(writtenCount == sizeof(expected));
  assert(std::memcmp(written.data(), expected, sizeof(expected)) == 0);
  uint32_t value = 0U;
  assert(VL53LX_RdDWord(&device, 0x9876U, &value) == VL53LX_ERROR_NONE);
  assert(value == 0x12345678U);
  assert(writtenCount == 2U && written[0] == 0x98U && written[1] == 0x76U);

  std::array<uint8_t, 260> data = {};
  assert(VL53LX_WriteMulti(&device, 0, data.data(), 258U) == VL53LX_ERROR_NONE);
  assert(writtenCount == 260U);
  const auto oldWrites = writes;
  const auto oldReads = reads;
  assert(VL53LX_WriteMulti(&device, 0, data.data(), 259U) == VL53LX_ERROR_INVALID_PARAMS);
  assert(VL53LX_WriteMulti(&device, 0, data.data(), UINT32_MAX) == VL53LX_ERROR_INVALID_PARAMS);
  assert(VL53LX_ReadMulti(&device, 0, data.data(), 261U) == VL53LX_ERROR_INVALID_PARAMS);
  assert(VL53LX_ReadMulti(&device, 0, data.data(), 0U) == VL53LX_ERROR_INVALID_PARAMS);
  assert(VL53LX_WriteMulti(&device, 0, nullptr, 1U) == VL53LX_ERROR_INVALID_PARAMS);
  assert(VL53LX_RdDWord(&device, 0, nullptr) == VL53LX_ERROR_INVALID_PARAMS);
  assert(VL53LX_WrByte(nullptr, 0, 0) == VL53LX_ERROR_INVALID_PARAMS);
  assert(writes == oldWrites && reads == oldReads);

  failIo = true;
  value = 0xDEADBEEFU;
  assert(VL53LX_RdDWord(&device, 0, &value) == VL53LX_ERROR_CONTROL_INTERFACE);
  assert(value == 0xDEADBEEFU && reads == oldReads);
  assert(VL53LX_WrByte(&device, 0, 0) == VL53LX_ERROR_CONTROL_INTERFACE);
  assert(VL53LX_WaitUs(&device, 125) == VL53LX_ERROR_NONE && sleptUs == 125U);
  assert(VL53LX_WaitMs(&device, 3) == VL53LX_ERROR_NONE && sleptUs == 3125U);
  assert(VL53LX_WaitUs(&device, -1) == VL53LX_ERROR_NONE && sleptUs == 3125U);
  std::puts("VL53L4CX platform: endian I/O, bounds, errors and yielding delays OK");
}
