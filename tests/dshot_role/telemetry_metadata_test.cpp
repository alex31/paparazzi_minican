#include <telemetry_test_support.hpp>
#include <cstdio>
#include <cstring>

static void testEdt() {
    static_assert(sizeof(DshotTelemetryFrame) == 10, "Keep the serial wire layout");
    DSHOTDriver driver;
    now = 0;
    auto tlm = dshotGetTelemetry(&driver, 1);
    assert(tlm.valid_mask == 0);
    receiveEdt(driver, 1, EDT_TEMP, 0); // Zero degrees and time zero are valid.
    now = 2999;
    receiveEdt(driver, 1, EDT_VOLT, 80);
    tlm = dshotGetTelemetry(&driver, 1);
    assert(dshotTelemetryIsValid(&tlm, DSHOT_TELEM_TEMP));
    assert(dshotTelemetryIsValid(&tlm, DSHOT_TELEM_VOLTAGE));
    assert(tlm.frame.temp == 0 && tlm.frame.voltage == 2000);
    now = 3000;
    receiveEdt(driver, 1, EDT_DBG1, 255);
    tlm = dshotGetTelemetry(&driver, 1);
    assert(!dshotTelemetryIsValid(&tlm, DSHOT_TELEM_TEMP));
    assert(dshotTelemetryIsValid(&tlm, DSHOT_TELEM_VOLTAGE));
    assert(tlm.updated_mask == 0 && tlm.ts == 2999);
    assert(dshotGetTelemetry(&driver, 3).valid_mask == 0); // Channel isolation.
    receiveEdt(driver, 1, EDT_CURRENT, 3);
    receiveEdt(driver, 1, EDT_STRESS, 0);
    receiveEdt(driver, 1, EDT_STATUS, 0xEB);
    tlm = dshotGetTelemetry(&driver, 1);
    assert(tlm.frame.current == 300 && tlm.stress == 0 && tlm.status == 0xEB);
    assert(dshotTelemetryIsValid(&tlm, DSHOT_TELEM_STRESS));
    assert(dshotTelemetryIsValid(&tlm, DSHOT_TELEM_STATUS));
    assert(tlm.updated_mask == (1U << DSHOT_TELEM_STATUS));
    assert(!dshotTelemetryIsValid(&tlm, DSHOT_TELEM_RPM)); // EDT doesn't fill serial RPM.
    now = 6000;
    assert(dshotGetTelemetry(&driver, 1).valid_mask == 0);
    // Expired fields stay invalid, including after a complete counter wrap.
    now = 3000;
    assert(dshotGetTelemetry(&driver, 1).valid_mask == 0);
    now = UINT32_MAX - 100;
    receiveEdt(driver, 3, EDT_TEMP, 42);
    now += 2999;
    tlm = dshotGetTelemetry(&driver, 3);
    assert(dshotTelemetryIsValid(&tlm, DSHOT_TELEM_TEMP));
    ++now;
    tlm = dshotGetTelemetry(&driver, 3);
    assert(!dshotTelemetryIsValid(&tlm, DSHOT_TELEM_TEMP));
}

// Execute the real serial receiver through good data, CRC error and timeout.
@CRC@
using msg_t = int;
#define noreturn
constexpr unsigned TIME_INFINITE = 0, TIME_IMMEDIATE = 0;
struct Finished {};
static DSHOTDriver* serialDriver;
static unsigned query;
static void chRegSetThreadName(const char*) {}
static void chMBFetchTimeout(int*, msg_t* index, unsigned) {
    if (query != 0) assert(!serialDriver->dshotMotors.onGoingQry);
    if (query == 4) throw Finished{};
    *index = query < 3 ? 1 : 3;
    serialDriver->dshotMotors.onGoingQry = true;
    now = query * 1000;
}
static size_t sdReadTimeout(void*, uint8_t* buffer, size_t length, unsigned timeout) {
    assert(length == 10 && timeout == 100);
    const uint8_t frame[9] = {42, 0x07, 0xD0, 0x01, 0x2C, 0, 10, 0, 60};
    std::memcpy(buffer, frame, sizeof(frame));
    buffer[9] = calculateCrc8(buffer, 9);
    if (query == 1) buffer[9] ^= 1;
    return query++ == 2 ? 3 : length;
}
static int sdGetTimeout(void*, unsigned) { return -1; }
@RECEIVER@
static void testSerial() {
    DSHOTDriver driver;
    serialDriver = &driver;
    query = 0;
    try { dshotTlmRec(&driver); } catch (const Finished&) {}
    assert(driver.tlm_frame_nb == 2 && driver.crc_errors == 1);
    assert(!driver.dshotMotors.onGoingQry);
    const auto& first = driver.dshotMotors.dt[1];
    assert(first.ts == 0 && first.updated_at[DSHOT_TELEM_TEMP] == 0);
    assert(first.frame.temp == 42); // Failed queries preserve the last good data.
    const auto tlm = dshotGetTelemetry(&driver, 3);
    assert(tlm.valid_mask == 0x1F && tlm.updated_mask == 0x1F);
    assert(tlm.frame.temp == 42 && tlm.frame.voltage == 2000);
    assert(tlm.frame.current == 300 && tlm.frame.consumption == 10 && tlm.frame.rpm == 60);
    for (unsigned field = 0; field < DSHOT_TELEM_STRESS; ++field)
        assert(tlm.updated_at[field] == 3000);
    assert(tlm.stress == 0 && tlm.status == 0);
    assert(dshotGetTelemetry(&driver, 1).valid_mask == 0); // CRC/timeout didn't refresh it.
}

int main() {
    testEdt();
    testSerial();
    puts("DShot metadata: independent expiry, zero values, counter wrap, ignored EDT and serial errors OK");
}
