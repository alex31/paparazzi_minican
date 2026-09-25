// run.py inserts production functions. Hardware/RTOS doubles track ownership.
#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <set>
#include <string>
#include <vector>
#include <strings.h>

using msg_t = intptr_t;
constexpr msg_t MSG_OK = 0;
constexpr unsigned NORMALPRIO = 128;
#define THD_FUNCTION(name, arg) void name(void *arg)
#define THD_WORKING_AREA_SIZE(size) ((size) + 480)
#define MUTEX_DECL(name) int name = 0
void chMtxLock(int*) {}
void chMtxUnlock(int*) {}
struct BaseSequentialStream { std::string text; };
size_t streamWrite(BaseSequentialStream *stream, const uint8_t *data, size_t length) {
    assert(stream);
    stream->text.append(reinterpret_cast<const char *>(data), length);
    return length;
}
struct thread_t { void (*entry)(void*); void *arg; };
std::set<thread_t*> threads;
std::set<void*> contexts;
unsigned allocation = 0, failAllocation = 0;
bool failUart = false, uartStarted = false, busy = false;
msg_t message;
thread_t sender{};
struct Yield {};
bool delivered;
thread_t *chMsgWait() {
    if (delivered) throw Yield{}; // Park the simulated worker after one request.
    delivered = true;
    return &sender;
}
msg_t chMsgGet(thread_t*) { return message; }
void chMsgRelease(thread_t*, msg_t) {}
msg_t chMsgSend(thread_t *thread, msg_t data) {
    message = data;
    delivered = false;
    try { thread->entry(thread->arg); } catch (const Yield&) {}
    return MSG_OK;
}
thread_t *chThdCreateFromHeap(void*, size_t, const char*, unsigned,
                              void (*entry)(void*), void *arg) {
    if (++allocation == failAllocation) return nullptr;
    auto *thread = new thread_t{entry, arg};
    threads.insert(thread);
    return thread;
}
void chThdWait(thread_t *thread) { assert(threads.erase(thread) == 1); delete thread; }
void *malloc_m(size_t size) {
    if (++allocation == failAllocation) return nullptr;
    auto *ptr = std::malloc(size);
    contexts.insert(ptr);
    return ptr;
}
void free_m(void *ptr) { assert(contexts.erase(ptr) == 1); std::free(ptr); }
int chvsnprintf(char *buffer, size_t size, const char *format, va_list arguments) {
    return std::vsnprintf(buffer, size, format, arguments);
}
extern "C" void chvprintf(BaseSequentialStream*, const char*, va_list);
extern "C" void chprintf(BaseSequentialStream*, const char*, ...);

@PRINTER@

namespace UAVCAN { struct Node {}; }
struct DeviceStatus {
    enum Source { SHELL_ROLE };
    enum Error { OK, CONFLICT, HEAP_FULL, NOT_RESPONDING };
    Error error;
    DeviceStatus(Source, Error value = OK): error(value) {}
    explicit operator bool() const { return error == OK; }
};
enum class HWResource { LPUART_1, PA02, PA03 };
struct {
    bool tryAcquire(HWResource, HWResource, HWResource) {
        if (busy) return false;
        busy = true;
        return true;
    }
    void release(HWResource, HWResource, HWResource) { assert(busy); busy = false; }
} boardResource;
BaseSequentialStream serialDevice;
BaseSequentialStream *chp = nullptr;
#define CONSOLE_DEV_SD serialDevice
int ftdiConfig;
msg_t sdStart(BaseSequentialStream*, int*) {
    if (failUart) return -1;
    uartStarted = true;
    return MSG_OK;
}
void sdStop(BaseSequentialStream*) { assert(uartStarted); uartStarted = false; }
unsigned commandCalls = 0;
void command(BaseSequentialStream*, int argc, const char *const argv[]) {
    assert(argc == 1 && std::string(argv[0]) == "arg"); ++commandCalls;
}
struct ShellCommand { const char *sc_name; void (*sc_function)(BaseSequentialStream*, int, const char *const[]); };
const ShellCommand commands[] = {{"info", command}, {"mem", command}, {"threads", command}, {nullptr, nullptr}};
struct ShellContext { const char *completions[std::size(commands)]{}; };
ShellContext *shellContext = nullptr;
constexpr size_t SHELL_WA_SIZE = THD_WORKING_AREA_SIZE(2000);
void shellWorker(void*) {}
class ShellRole {
    UAVCAN::Node *m_node = nullptr;
public:
    DeviceStatus subscribe(UAVCAN::Node&);
    DeviceStatus start(UAVCAN::Node&);
};
@CALLBACKS@
@LIFECYCLE@

int main() {
    UAVCAN::Node node;
    // No implicit print-thread allocation when the shell is off, even with a
    // stale stream supplied by an assertion/diagnostic caller.
    chprintf(nullptr, "off");
    chprintf(&serialDevice, "off");
    assert(allocation == 0 && threads.empty() && serialDevice.text.empty());
    for (unsigned fail = 1; fail <= 3; ++fail) {
        failAllocation = fail;
        allocation = 0;
        ShellRole role;
        assert(role.subscribe(node));
        assert(role.start(node).error == DeviceStatus::HEAP_FULL);
        assert(contexts.empty() && threads.empty() && !busy && !uartStarted);
        assert(!shellContext && !chp && !printThread);
    }
    failAllocation = 0;
    allocation = 0;
    failUart = true;
    ShellRole role;
    assert(role.start(node).error == DeviceStatus::NOT_RESPONDING);
    assert(contexts.empty() && threads.empty() && !busy && !chp && !shellContext);
    failUart = false;
    busy = true;
    allocation = 0;
    assert(role.start(node).error == DeviceStatus::CONFLICT);
    assert(allocation == 0 && busy); // Existing owner's reservation is kept.
    busy = false;
    assert(role.start(node));
    assert(contexts.size() == 1 && threads.size() == 2 && busy && uartStarted);
    assert(chp == &serialDevice && allocation == 3);
    chprintf(chp, "%s %d", "value", 42);
    assert(serialDevice.text == "value 42");
    serialDevice.text.clear();
    chprintf(chp, "%s", std::string(500, 'a').c_str());
    assert(serialDevice.text == std::string(159, 'a')); // No over-read on truncation.
    const char *argv[] = {"MeM", "arg"};
    shellExecute(2, argv);
    assert(commandCalls == 1);
    shellExecute(0, nullptr);
    const char *unknown[] = {"absent"};
    shellExecute(1, unknown);
    assert(serialDevice.text.ends_with("Commande inconnue : absent\r\n"));
    auto matches = shellComplete(0, nullptr);
    assert(std::string(matches[0]) == "info" && matches[3] == nullptr);
    const char *prefix[] = {"th"};
    matches = shellComplete(1, prefix);
    assert(std::string(matches[0]) == "threads" && matches[1] == nullptr);
    matches = shellComplete(2, argv);
    assert(matches[0] == nullptr);
    // Simulated power-down; production roles persist until reset.
    chp = nullptr;
    consolePrintfStop();
    chThdWait(*threads.begin());
    shellContext->~ShellContext();
    free_m(shellContext);
    puts("Shell: allocation rollback, UART conflict, disabled output, formatting and commands OK");
}
