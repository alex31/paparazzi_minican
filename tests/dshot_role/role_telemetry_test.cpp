// Host regression test for the actual EscDshot::periodic implementation.
// Hardware/RTOS calls are stubbed; decoder results arrive one cycle at a time.
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <limits>
#include <telemetry_test_support.hpp>

constexpr uint32_t DSHOT_BIDIR_ERR_CRC = UINT32_MAX;
constexpr uint32_t DSHOT_BIDIR_TLM_EDT = UINT32_MAX - 1;
constexpr unsigned DSHOT_CMD_MOTOR_STOP = 0, DSHOT_CMD_BIDIR_EDT_MODE_ON = 13;
constexpr unsigned CANARD_TRANSFER_PRIORITY_LOW = 0;
struct uavcan_equipment_esc_Status {
    uint8_t esc_index = 0;
    uint32_t rpm = 0;
    float voltage = 0, current = 0, temperature = 0;
};
struct dronecan_protocol_FlexDebug {
    uint16_t id = 0;
    struct { uint8_t len = 0, data[255]{}; } u8;
};
struct Response { uint32_t rpm = DSHOT_BIDIR_ERR_CRC; uint8_t value = 0; EdtType type = EDT_TEMP; };
static std::vector<std::array<Response, 4>> responses(6);
static std::array<unsigned, 4> readCount;
static unsigned cycle;
struct Finished {};

static void chThdSleepMicroseconds(unsigned) { ++now; }
static void chThdSleepUntilWindowed(systime_t, systime_t end) {
    now = end;
    if (++cycle == responses.size()) throw Finished{};
}
static void dshotSendSpecialCommand(DSHOTDriver*, unsigned, unsigned) {}
static void dshotSetThrottle(DSHOTDriver*, unsigned, unsigned) {}
static void dshotSendFrame(DSHOTDriver*) {}
static uint32_t dshotGetRpm(DSHOTDriver* driver, unsigned channel) {
    ++readCount[channel];
    const auto response = responses[cycle][channel];
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
    if (response.rpm == DSHOT_BIDIR_TLM_EDT) {
        receiveEdt(*driver, channel, response.type, response.value);
    }
#endif
    return response.rpm;
}
namespace UAVCAN {
struct Node {
    enum { CAN_OK = 0, CAN_ERROR = 1 };
    struct Publication { unsigned cycle; uavcan_equipment_esc_Status status; };
    struct DebugPublication { unsigned cycle; dronecan_protocol_FlexDebug debug; };
    std::vector<Publication> sent;
    std::vector<DebugPublication> debugSent;
    unsigned debugFailures = 0;
    void sendBroadcast(const uavcan_equipment_esc_Status& status, unsigned) {
        sent.push_back({cycle, status});
    }
    int sendBroadcast(const dronecan_protocol_FlexDebug& debug, unsigned) {
        if (debugFailures) { --debugFailures; return CAN_ERROR; }
        debugSent.push_back({cycle, debug});
        return CAN_OK;
    }
};
}
struct EscDshot {
    // Non-contiguous physical channels must retain their own telemetry.
    std::array<uint8_t, 4> channelMap{1, 3};
    uint8_t numChannels = 2, mapIndex1 = 4;
    uint16_t rpmFrqDiv = 0;
    uint8_t polePairs = 1;
    systime_t loopPeriod = 5;
    uint16_t throttles[4]{};
    DSHOTDriver dshotd;
    dronecan_protocol_FlexDebug msgEscDebug{};
    UAVCAN::Node node;
    UAVCAN::Node* m_node = &node;
    void periodic(void*);
};

@PERIODIC@

static void run(unsigned divider, unsigned pairs, uint32_t expectedRpm1, uint32_t expectedRpm3) {
    cycle = 0;
    now = 0;
    readCount.fill(0);
    EscDshot role;
    role.rpmFrqDiv = divider;
    role.polePairs = pairs;
    try { role.periodic(nullptr); } catch (const Finished&) {}
    const auto& sent = role.node.sent;
#if DSHOT_BIDIR
    assert((readCount == std::array<unsigned, 4>{0, 6, 0, 6}));
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
    assert(role.dshotd.dshotMotors.dt[1].frame.temp == 44);
    assert(role.dshotd.dshotMotors.dt[3].frame.temp == 55);
#endif
    if (divider == 0) {
        assert(sent.empty()); // Publication disabled, EDT still processed.
        return;
    }
    assert(sent.size() == 3);
    for (const auto& publication : sent) {
        assert(std::isnan(publication.status.voltage));
        assert(std::isnan(publication.status.current));
    }
    assert(role.node.debugSent.empty());
    const unsigned firstCycle = divider == 3 ? 2 : 0;
    assert(sent[0].cycle == firstCycle && sent[0].status.esc_index == 4);
    assert(sent[0].status.rpm == expectedRpm1);
    assert(sent[1].cycle == firstCycle && sent[1].status.esc_index == 5);
    assert(sent[1].status.rpm == expectedRpm3);
    assert(sent[2].cycle == (divider == 3 ? 5U : 3U));
    assert(sent[2].status.esc_index == 5 && sent[2].status.rpm == 0);
    // The second window has no valid RPM on channel 1: do not repeat its RPM.
#if DSHOT_BIDIR_EXTENTED_TELEMETRY
    if (divider == 3) {
        assert(std::fabs(sent[0].status.temperature - 316.15f) < 0.001f);
        assert(std::fabs(sent[1].status.temperature - 325.15f) < 0.001f);
        assert(std::fabs(sent[2].status.temperature - 328.15f) < 0.001f);
    }
#else
    for (const auto& publication : sent) assert(std::isnan(publication.status.temperature));
#endif
#else
    assert((readCount == std::array<unsigned, 4>{}));
    assert(sent.empty());
#endif
}

#if DSHOT_BIDIR && DSHOT_BIDIR_EXTENTED_TELEMETRY
static void execute(EscDshot& role) {
    cycle = now = 0;
    readCount.fill(0);
    try { role.periodic(nullptr); } catch (const Finished&) {}
}

static void testFreshness() {
    responses.assign(606, {});
    for (auto& response : responses) { response[1] = {6000}; response[3] = {9000}; }
    responses[0][1] = {DSHOT_BIDIR_TLM_EDT, 0, EDT_TEMP};
    responses[1][1] = {DSHOT_BIDIR_TLM_EDT, 80, EDT_VOLT};
    responses[2][1] = {DSHOT_BIDIR_TLM_EDT, 0, EDT_CURRENT};
    responses[599][1] = {DSHOT_BIDIR_TLM_EDT, 84, EDT_VOLT};
    responses[604][1] = {DSHOT_BIDIR_TLM_EDT, 42, EDT_TEMP};
    EscDshot role;
    role.rpmFrqDiv = 1;
    execute(role);
    for (const auto& sent : role.node.sent) {
        const auto& status = sent.status;
        if (status.esc_index == 5) {
            assert(std::isnan(status.temperature) && std::isnan(status.voltage) && std::isnan(status.current));
        } else if (sent.cycle == 3) {
            assert(std::fabs(status.temperature - 273.15f) < 0.001f);
            assert(status.voltage == 20.0f && status.current == 0.0f);
        } else if (sent.cycle == 600) {
            assert(std::isnan(status.temperature) && status.current == 0.0f);
            assert(status.voltage == 21.0f);
        } else if (sent.cycle == 603) {
            assert(std::isnan(status.temperature) && std::isnan(status.current));
            assert(status.voltage == 21.0f); // Only voltage has been refreshed.
        } else if (sent.cycle == 605) {
            assert(std::fabs(status.temperature - 315.15f) < 0.001f);
        }
    }
}

static void testFlexDebug(unsigned failures, unsigned divider) {
    responses.assign(85, {}); // No valid RPM: diagnostics must still be published.
    responses[1][1] = {DSHOT_BIDIR_TLM_EDT, 100, EDT_STRESS};
    responses[2][1] = {DSHOT_BIDIR_TLM_EDT, 20, EDT_STRESS};
    responses[3][1] = {DSHOT_BIDIR_TLM_EDT, 0xAB, EDT_STATUS};
    responses[4][1] = {DSHOT_BIDIR_TLM_EDT, 0x55, EDT_STATUS};
    responses[5][1] = {DSHOT_BIDIR_TLM_EDT, 0, EDT_STATUS};
    responses[6][3] = {DSHOT_BIDIR_TLM_EDT, 0, EDT_STRESS};
    responses[7][3] = {DSHOT_BIDIR_TLM_EDT, 0, EDT_STATUS};
    responses[21][1] = {DSHOT_BIDIR_TLM_EDT, 1, EDT_STRESS};
    responses[22][1] = {DSHOT_BIDIR_TLM_EDT, 0x22, EDT_STATUS};
    responses[23][1] = {DSHOT_BIDIR_TLM_EDT, 1, EDT_STATUS};
    responses[41][3] = {DSHOT_BIDIR_TLM_EDT, 0, EDT_STRESS}; // Stress only in this interval.
    EscDshot role;
    role.rpmFrqDiv = divider;
    role.node.debugFailures = failures;
    execute(role);
    const auto& sent = role.node.debugSent;
    assert(role.node.sent.empty());
    if (divider == 0) { assert(sent.empty()); return; }
    assert(sent.size() == (failures ? 3U : 4U));
    for (const auto& publication : sent) {
        const auto& msg = publication.debug;
        assert(msg.id == 2000U + msg.u8.data[1]);
        assert(msg.u8.len == 5 && msg.u8.data[0] == 1);
        assert(publication.cycle % 20 == 0); // At most 10 Hz with a 5 ms loop.
        if (msg.id == 2004) {
            assert(msg.u8.data[2] == 3);
            const bool accumulated = failures || publication.cycle == 20;
            assert(msg.u8.data[3] == (accumulated ? 100 : 1));
            assert(msg.u8.data[4] == (accumulated ? 0xEB : 0x22));
        } else {
            assert(msg.id == 2005 && msg.u8.data[3] == 0 && msg.u8.data[4] == 0);
            assert(msg.u8.data[2] == (publication.cycle == 20 ? 3 : 1));
        }
    }
    // A slow CAN divider must not publish diagnostics that have expired.
    responses.assign(605, {});
    responses[0][1] = {DSHOT_BIDIR_TLM_EDT, 255, EDT_STRESS};
    responses[1][1] = {DSHOT_BIDIR_TLM_EDT, 255, EDT_STATUS};
    EscDshot slow;
    slow.rpmFrqDiv = 605;
    execute(slow);
    assert(slow.node.debugSent.empty());
}
#endif

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
    for (unsigned divider : {0U, 1U, 3U}) {
        run(divider, 1, 6000, 9000);
        run(divider, 7, 857, 1285); // 14-pole motors; fractional RPM rounded down.
        run(divider, 50, 120, 180); // 100 poles, the maximum configured pole count.
    }
#if DSHOT_BIDIR && DSHOT_BIDIR_EXTENTED_TELEMETRY
    testFreshness();
    testFlexDebug(0, 0);
    testFlexDebug(0, 1);
    testFlexDebug(1, 1);
#endif
    puts("DShot role: cadence, mapping, RPM, NaN, field expiry and FlexDebug aggregation/retry OK");
}
