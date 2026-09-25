// Host regression test for the actual EscDshot::periodic implementation.
// Hardware/RTOS calls are stubbed; decoder results arrive one cycle at a time.
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

using systime_t = uint32_t;
constexpr unsigned DSHOT_CHANNELS = 4;
constexpr uint32_t DSHOT_BIDIR_ERR_CRC = UINT32_MAX;
constexpr uint32_t DSHOT_BIDIR_TLM_EDT = UINT32_MAX - 1;
constexpr unsigned DSHOT_CMD_MOTOR_STOP = 0, DSHOT_CMD_BIDIR_EDT_MODE_ON = 13;
constexpr unsigned CANARD_TRANSFER_PRIORITY_LOW = 0;
#define TIME_MS2I(ms) (ms)

struct DshotTelemetry {
    struct { uint8_t temp = 0; uint16_t voltage = 0, current = 0; } frame;
};
struct DSHOTDriver { std::array<DshotTelemetry, 4> telemetry{}; };
struct uavcan_equipment_esc_Status {
    uint8_t esc_index = 0;
    uint32_t rpm = 0;
    float voltage = 0, current = 0, temperature = 0;
};
struct Response { uint32_t rpm = DSHOT_BIDIR_ERR_CRC; uint8_t temperature = 0; };
static std::array<std::array<Response, 4>, 6> responses;
static std::array<unsigned, 4> readCount;
static unsigned cycle;
struct Finished {};

static systime_t chVTGetSystemTimeX() { return cycle * 5; }
// Skip the startup command phase, which is outside this regression test.
static systime_t chTimeDiffX(systime_t, systime_t) { return TIME_MS2I(1000); }
static void chThdSleepMicroseconds(unsigned) {}
static void chThdSleepUntilWindowed(systime_t, systime_t) {
    if (++cycle == responses.size()) throw Finished{};
}
static void dshotSendSpecialCommand(DSHOTDriver*, unsigned, unsigned) {}
static void dshotSetThrottle(DSHOTDriver*, unsigned, unsigned) {}
static void dshotSendFrame(DSHOTDriver*) {}
static uint32_t dshotGetRpm(DSHOTDriver* driver, unsigned channel) {
    ++readCount[channel];
    const auto response = responses[cycle][channel];
    if (response.rpm == DSHOT_BIDIR_TLM_EDT) {
        driver->telemetry[channel].frame.temp = response.temperature;
    }
    return response.rpm;
}
static DshotTelemetry dshotGetTelemetry(DSHOTDriver* driver, unsigned channel) {
    return driver->telemetry[channel];
}

struct Node {
    struct Publication { unsigned cycle; uavcan_equipment_esc_Status status; };
    std::vector<Publication> sent;
    void sendBroadcast(const uavcan_equipment_esc_Status& status, unsigned) {
        sent.push_back({cycle, status});
    }
};
struct EscDshot {
    // Non-contiguous physical channels must retain their own telemetry.
    std::array<uint8_t, 4> channelMap{1, 3};
    uint8_t numChannels = 2, mapIndex1 = 4;
    uint16_t rpmFrqDiv = 0;
    systime_t loopPeriod = 5;
    uint16_t throttles[4]{};
    DSHOTDriver dshotd;
    Node node;
    Node* m_node = &node;
    void periodic(void*);
};

@PERIODIC@

static void run(unsigned divider) {
    cycle = 0;
    readCount.fill(0);
    EscDshot role;
    role.rpmFrqDiv = divider;
    try { role.periodic(nullptr); } catch (const Finished&) {}
    const auto& sent = role.node.sent;
#if DSHOT_BIDIR
    assert((readCount == std::array<unsigned, 4>{0, 6, 0, 6}));
    assert(role.dshotd.telemetry[1].frame.temp == 44);
    assert(role.dshotd.telemetry[3].frame.temp == 55);
    if (divider == 0) {
        assert(sent.empty()); // Publication disabled, EDT still processed.
        return;
    }
    assert(sent.size() == 3);
    const unsigned firstCycle = divider == 3 ? 2 : 0;
    assert(sent[0].cycle == firstCycle && sent[0].status.esc_index == 4);
    assert(sent[0].status.rpm == 6000);
    assert(sent[1].cycle == firstCycle && sent[1].status.esc_index == 5);
    assert(sent[1].status.rpm == 9000);
    assert(sent[2].cycle == (divider == 3 ? 5U : 3U));
    assert(sent[2].status.esc_index == 5 && sent[2].status.rpm == 0);
    // The second window has no valid RPM on channel 1: do not repeat 6000.
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
    if (divider == 3) {
        assert(sent[0].status.temperature == 43);
        assert(sent[1].status.temperature == 52);
        assert(sent[2].status.temperature == 55);
    }
#endif
#else
    assert((readCount == std::array<unsigned, 4>{}));
    assert(sent.empty());
#endif
}

int main() {
    responses[0][1] = {6000};
    responses[0][3] = {9000};
    responses[1][1] = {DSHOT_BIDIR_TLM_EDT, 42};
    responses[1][3] = {DSHOT_BIDIR_TLM_EDT, 52};
    responses[2][1] = {DSHOT_BIDIR_TLM_EDT, 43};
    // Channel 3 has a bad response at the first publication boundary.
    responses[3][3] = {0}; // A stopped motor is a valid RPM measurement.
    responses[4][1] = {DSHOT_BIDIR_TLM_EDT, 44};
    responses[4][3] = {DSHOT_BIDIR_TLM_EDT, 54};
    responses[5][3] = {DSHOT_BIDIR_TLM_EDT, 55};
    run(3);
    run(1);
    run(0);
    puts("DShot role: per-cycle decoding, CAN cadence, channel mapping and RPM freshness OK");
}
