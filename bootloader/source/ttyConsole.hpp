/**
 * @file ttyConsole.hpp
 * @brief Bootloader console initialization helpers.
 */
#pragma once



// fonctions exportees par le module

#ifdef CONSOLE_DEV_SD
/** @brief Initialize the bootloader console. */
void consoleInit (void);
/** @brief Launch the bootloader console thread. */
void consoleLaunch (void);

#if defined TRACE 
#include "stdutil.h"
#include "usb_serial.h"
#define CDCTrace(fmt, ...) {if (isUsbConnected()) chprintf (chp, fmt "\r\n", ## __VA_ARGS__ );}
#else
#define CDCTrace(...) 
#endif // TRACE


#endif
