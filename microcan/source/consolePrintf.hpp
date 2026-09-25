#pragma once
#include <hal.h>

// Called only by ShellRole, before publishing the stream through chp.
bool consolePrintfStart();
// Roll back a failed shell startup. No client may submit prints at this point.
void consolePrintfStop();
