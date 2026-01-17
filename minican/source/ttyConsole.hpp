/**
 * @file ttyConsole.hpp
 * @brief TTY console initialization helpers.
 */
#pragma once



// fonctions exportees par le module

#ifdef CONSOLE_DEV_SD
/** @brief Initialize the console subsystem. */
void consoleInit (void);
/** @brief Launch the console thread. */
void consoleLaunch (void);

#if defined TRACE 
#include "stdutil.h"
#include "usb_serial.h"
#define CDCTrace(fmt, ...) {if (isUsbConnected()) chprintf (chp, fmt "\r\n", ## __VA_ARGS__ );}
#else
#define CDCTrace(...) 
#endif // TRACE


#endif
