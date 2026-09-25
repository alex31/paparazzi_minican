// Production telemetry types, getter and EDT updater; only RTOS calls are stubbed.
#pragma once
#include <cassert>
#include <cstdint>
#include <dshot_erps.h>
using systime_t = uint32_t;
constexpr unsigned DSHOT_CHANNELS = 4;
#define TIME_MS2I(ms) (ms)
static systime_t now;
static systime_t chVTGetSystemTimeX() { return now; }
static systime_t chTimeDiffX(systime_t start, systime_t end) { return end - start; }
#define chDbgAssert(condition, text) assert(condition)
static void chMtxLock(int* mutex) { assert(*mutex == 0); ++*mutex; }
static void chMtxUnlock(int* mutex) { assert(*mutex == 1); --*mutex; }
@TYPES@
struct DSHOTConfig { void* tlm_sd = nullptr; };
struct DSHOTDriver {
    struct {
        DshotTelemetry dt[DSHOT_CHANNELS]{};
        int tlmMtx[DSHOT_CHANNELS]{};
        bool onGoingQry = false;
    } dshotMotors;
    DSHOTConfig configStorage;
    DSHOTConfig* config = &configStorage;
    int mb = 0;
    unsigned crc_errors = 0, tlm_frame_nb = 0;
};
@GETTER@
@UPDATER@
static void receiveEdt(DSHOTDriver& driver, unsigned channel, EdtType type, uint8_t value) {
    DshotErps erps{};
    erps.ep.rawFrame = (type << 12) | (value << 4);
    updateTelemetryFromBidirEdt(&erps, &driver.dshotMotors.dt[channel]);
}
