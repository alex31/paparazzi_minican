/** @brief Serial formatting worker whose stack exists only with ROLE.shell. */
#include "consolePrintf.hpp"
#include "printf.h"
#include <algorithm>

namespace {
  thread_t *printThread = nullptr;
  MUTEX_DECL(printMutex);
  struct PrintRequest {
    BaseSequentialStream *stream;
    const char *format;
    va_list *arguments;
  };

  THD_FUNCTION(printWorker, unused) {
    (void)unused;
    while (true) {
      thread_t *sender = chMsgWait();
      auto *request = reinterpret_cast<PrintRequest *>(chMsgGet(sender));
      if (!request) {
        chMsgRelease(sender, MSG_OK);
        return;
      }
      char buffer[160];
      va_list arguments;
      va_copy(arguments, *request->arguments);
      const int length = chvsnprintf(buffer, sizeof(buffer), request->format, arguments);
      va_end(arguments);
      if (length > 0) {
        streamWrite(request->stream, reinterpret_cast<const uint8_t *>(buffer),
                    std::min(static_cast<size_t>(length), sizeof(buffer) - 1U));
      }
      chMsgRelease(sender, MSG_OK);
    }
  }
}

bool consolePrintfStart() {
  if (!printThread) {
    printThread = chThdCreateFromHeap(nullptr, THD_WORKING_AREA_SIZE(2048),
                                    "serialPrint", NORMALPRIO + 1, printWorker, nullptr);
  }
  return printThread != nullptr;
}

void consolePrintfStop() {
  if (printThread) {
    chMsgSend(printThread, 0);
    chThdWait(printThread);
    printThread = nullptr;
  }
}

extern "C" void chvprintf(BaseSequentialStream *stream, const char *format, va_list arguments) {
  // Also protects unguarded callers (assertions) while the UART is stopped.
  if (!stream || !printThread) return;
  chMtxLock(&printMutex);
  va_list copy;
  va_copy(copy, arguments);
  PrintRequest request{stream, format, &copy};
  chMsgSend(printThread, reinterpret_cast<msg_t>(&request));
  va_end(copy);
  chMtxUnlock(&printMutex);
}

extern "C" void chprintf(BaseSequentialStream *stream, const char *format, ...) {
  va_list arguments;
  va_start(arguments, format);
  chvprintf(stream, format, arguments);
  va_end(arguments);
}
