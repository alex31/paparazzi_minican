// The actual main and CANSlave startup code is inserted by run.py.
// These doubles observe registration/order/LEDs; they do not emulate a CAN bus.
#include <uavcan.protocol.GetNodeInfo_res.h>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#define DebugTrace(...) ((void)0)
#define DEVICE_NAME "org.pprz.microcanV5"
#define SW_VERSION_MAJOR 0
#define SW_VERSION_MINOR 1
#define VCS_COMMIT 0
#define HW_VERSION 5
#define USE_MICROPHONE_ROLE 1
#define USE_SERVO_ROLE 1
#define USE_BARO_MPL3115A2_ROLE 1
#define USE_QMC5883_ROLE 1
#define USE_OPT4060_ROLE 1
#define USE_VL53L4CX_ROLE 1
#define USE_ESC_DSHOT_ROLE 1
#define USE_RC_SBUS_ROLE 1
#define USE_GPS_UBX_ROLE 1
#define USE_SERIAL_STREAM_ROLE 1
#define USE_LED2812_ROLE 1
#define USE_VOLTMETER_ROLE 1
#define USE_TEMPLATE_ROLE 1

static bool identification, changeDuringStart, roleError;
static bool shellEnabled = true;
static bool shellError;
static int8_t configuredId = 10;
static unsigned constructed, subscribed, started, healthStarted, adcCallbacks;
static unsigned shellConstructed, shellSubscribed, shellStarted;
static unsigned nodeStarts, allocations;
static bool terminalResistor;
static uint8_t displayedId;
static uint16_t motif;
struct HSV { float h, s, v; };
static HSV color{};
namespace etl { using string_view = std::string_view; }
namespace RgbLed {
void start() {}
void setMotif(uint16_t, uint16_t value) { motif = value; }
void setColor(HSV value) { color = value; }
void setNodeId(uint8_t value) { displayedId = value; }
}
struct DeviceStatus {
    enum Source { ALL, RESOURCE };
    enum Error { OK, CONFLICT, NB_ROLE_TOO_LARGE };
    Error err;
    DeviceStatus(Source, Error error = OK) : err(error) {}
    explicit operator bool() const { return err == OK; }
    const char* describe() const { return "test error"; }
};
template <size_t N> struct FixedString {
    char value[N];
    constexpr FixedString(const char (&text)[N]) { std::copy_n(text, N, value); }
};
template <FixedString Key> auto param_cget() {
    constexpr std::string_view name(Key.value);
    if constexpr (name == "ROLE.identification") return identification;
    else if constexpr (name == "ROLE.shell") return shellEnabled;
    else if constexpr (name == "uavcan.node_id") return configuredId;
    else if constexpr (name == "uavcan.dynid.fd") return false;
    else if constexpr (name == "hardware.nickname") return std::string("host test");
    else return true; // All roles and the terminal resistor are enabled.
}

using Handler = void (*)();
static std::set<Handler> dispatched;
#define HANDLER(name) void name() { dispatched.insert(name); }
HANDLER(processNodeStatus)
HANDLER(processNodeInfoResponse)
HANDLER(processFileReadResponse)
HANDLER(processNodeInfoRequest)
HANDLER(processGetSetRequest)
HANDLER(processRestartNodeRequest)
HANDLER(processExecuteOpcodeRequest)
HANDLER(processFirmwareUpdateRequest)

constexpr int CAND2 = 2, cancfg = 0;
constexpr int LINE_CAN_TERMR_EN = 1, PAL_HIGH = 1, PAL_LOW = 0;
void palWriteLine(int line, int value) {
    assert(line == LINE_CAN_TERMR_EN);
    terminalResistor = value == PAL_HIGH;
}
namespace UAVCAN {
struct Config {
    int cand, cancfg;
    int8_t nodeId;
    bool dynamicId_fd;
    uavcan_protocol_GetNodeInfoResponse nodeInfo;
    void (*infoCb)(etl::string_view);
};
struct Node {
    std::set<Handler> broadcasts, responses, requests;
    uint8_t mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    uint8_t modeAtStart = mode;
    int8_t id;
    explicit Node(const Config& config) : id(config.nodeId) {}
    template <auto... H> void subscribeBroadcastMessages() { (broadcasts.insert(H), ...); }
    template <auto... H> void subscribeResponseMessages() { (responses.insert(H), ...); }
    template <auto... H> void subscribeRequestMessages() { (requests.insert(H), ...); }
    void setStatusMode(uint8_t value) { mode = value; }
    void setSpecificCode(uint16_t) {}
    void start() {
        assert(terminalResistor);
        ++nodeStarts;
        modeAtStart = mode;
        if (id <= 0) { ++allocations; id = 42; }
        if (changeDuringStart) identification = false;
    }
    uint8_t getNodeId() const { return id; }
    void infoCb(const char*, ...) {}
    void receive(const std::set<Handler>& handlers, Handler callback) {
        assert(nodeStarts == 1 && handlers.contains(callback));
        callback();
    }
};
template <class T> void dsdlAppend(T&, const char*) {}
namespace Helper { void log(Node&, int, const char*, const char*) {} }
}
constexpr int UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR = 3;
static UAVCAN::Node* slaveNode;
namespace HealthSurvey { void start(UAVCAN::Node&) { ++healthStarted; } }
struct FakeRole {
    const bool isShell;
    explicit FakeRole(bool shell = false) : isShell(shell) {
        ++constructed;
        if (isShell) ++shellConstructed;
    }
    virtual ~FakeRole() = default;
    DeviceStatus subscribe(UAVCAN::Node&) {
        assert(nodeStarts == 0);
        ++subscribed;
        if (isShell) ++shellSubscribed;
        return {DeviceStatus::ALL, !isShell && roleError ? DeviceStatus::CONFLICT : DeviceStatus::OK};
    }
    DeviceStatus start(UAVCAN::Node&) {
        assert(nodeStarts == 1);
        ++started;
        if (isShell) ++shellStarted;
        return {DeviceStatus::ALL, isShell && shellError ? DeviceStatus::CONFLICT : DeviceStatus::OK};
    }
};
using MicrophoneRole = FakeRole;
using ServoRole = FakeRole;
using Baro_MPL3115A2_Role = FakeRole;
using Qmc5883Role = FakeRole;
using Opt4060Role = FakeRole;
using Vl53l4cxRole = FakeRole;
using EscDshot = FakeRole;
using RC_Sbus = FakeRole;
using GpsUBX = FakeRole;
using SerialStream = FakeRole;
using RgbLedRole = FakeRole;
using VoltmeterRole = FakeRole;
using TemplateRole = FakeRole;
struct ShellRole : FakeRole { ShellRole() : FakeRole(true) {} };
static std::vector<FakeRole*> roles;
template <class T, FixedString... Names> bool addRole() {
    if ((param_cget<Names>() || ...)) roles.push_back(new T);
    return true;
}
[[maybe_unused]] void printOnceInATime(etl::string_view) {}

@STARTUP@

using mfs_error_t = int;
constexpr int MFS_NO_ERROR = 0;
namespace MFS { int start() { return MFS_NO_ERROR; } }
namespace Ressource { struct Storage { bool start() { return true; } } storage; }
namespace Adc {
void start() {}
template <class Callback> void setErrorCB(Callback) { ++adcCallbacks; }
}
constexpr float psBatMin = 6.5f, psBatMax = 25.2f, coreTempMin = -20, coreTempMax = 60;
@STATUS_BITS@
struct { unsigned CSR = 0; } rcc;
auto* RCC = &rcc;
constexpr unsigned RCC_CSR_WWDGRSTF = 1, RCC_CSR_IWDGRSTF = 2, RCC_CSR_RMVF = 4;
int TRNGD1;
void trngStart(int*, void*) {}
[[maybe_unused]] void consoleInit() {}
[[maybe_unused]] void consoleLaunch() {}
constexpr int TIME_INFINITE = -1;
struct BootComplete {};
[[noreturn]] void chThdSleep(int interval) {
    assert(interval == TIME_INFINITE);
    throw BootComplete{};
}

@MAIN@

int main(int argc, char** argv) {
    assert(argc == 2);
    const std::string_view scenario(argv[1]);
    const bool identifyAtBoot = scenario != "normal" && scenario != "shell-off" && scenario != "role-error";
    identification = identifyAtBoot;
    roleError = identifyAtBoot || scenario == "role-error";
    shellEnabled = scenario != "shell-off" && scenario != "identification-shell-off";
    shellError = scenario == "identification-shell-error";
    changeDuringStart = scenario == "changed-during-start";
    if (scenario == "dynamic") configuredId = 0;
    unsigned expectedShell = 0;
#ifdef TRACE
    if (shellEnabled) expectedShell = 1;
#endif
    try { firmwareMain(); assert(false); } catch (const BootComplete&) {}
    assert(shellConstructed == expectedShell);
    assert(shellSubscribed == expectedShell);

    if (scenario == "role-error") {
        assert(constructed > 0 && subscribed == 1 + expectedShell);
        assert(nodeStarts == 0 && started == 0 && adcCallbacks == 0);
        assert(motif == 0b110011000 && color.h == 0.0f);
    } else {
        assert(slaveNode != nullptr && nodeStarts == 1);
        assert(allocations == (scenario == "dynamic" ? 1U : 0U));
        auto& node = *slaveNode;
        for (auto callback : {processNodeInfoRequest, processGetSetRequest,
                              processExecuteOpcodeRequest, processRestartNodeRequest,
                              processFirmwareUpdateRequest}) {
            node.receive(node.requests, callback);
        }
        node.receive(node.responses, processFileReadResponse);
        node.receive(node.broadcasts, processNodeStatus);
        assert(dispatched.size() == 7);
        if (identifyAtBoot) {
            assert(constructed == expectedShell && subscribed == expectedShell && started == expectedShell);
            assert(shellStarted == expectedShell);
            assert(healthStarted == 0 && adcCallbacks == 0);
            assert(node.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE);
            assert(node.modeAtStart == UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE);
            assert(displayedId == 0);
            if (shellError && expectedShell) {
                // Failed diagnostics must not stop the management services.
                assert(motif == 0b110011000 && color.h == 0.0f);
            } else {
                assert(motif == 0b1010100000000000 && color.h == 0.8f && color.v == 0.5f);
            }
        } else {
            const unsigned expected = 13 + expectedShell;
            assert(constructed == expected && subscribed == constructed && started == constructed);
            assert(shellStarted == expectedShell);
            assert(healthStarted == 1 && adcCallbacks == 1);
            assert(node.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL);
            assert(displayedId == configuredId);
        }
    }
    for (auto role : roles) delete role;
    std::printf("Startup %s: OK\n", argv[1]);
}
