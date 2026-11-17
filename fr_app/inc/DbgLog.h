/*
 * DbgLog.h
 *
 * Application-level debug and log macro layer.
 *
 * All macros format with vsnprintf and push the result into a single RAM
 * ring buffer; one background "DbgLog" consumer task drains the ring and does
 * the slow output (UART byte-stream and/or external-flash write). Producers
 * therefore never block on the slow transports and concurrent callers from
 * different tasks cannot corrupt or lose each other's lines (each message is
 * enqueued atomically; only on a full ring is a whole message dropped).
 *
 * Two verbosity levels are available, in two families:
 *
 *   DBG(fmt, ...)      — debug transport only (debug UART)
 *   LOG(fmt, ...)      — external-flash text log only
 *   DBG_LOG(fmt, ...)  — both
 *
 *     These are "verbose" — every line is prefixed with the current date/time
 *     and battery voltage, e.g. "1970-01-01 00:00:00 3800mV <message>".
 *
 *   _DBG_(fmt, ...), _LOG_(fmt, ...), _DBG_LOG_(fmt, ...)
 *
 *     Plain — printed exactly as formatted, with no extra prefix. Use these
 *     where the message already carries its own timestamp/context, or where
 *     the prefix would just be noise (e.g. multi-line dumps).
 */

#ifndef INC_DBGLOG_H_
#define INC_DBGLOG_H_

#include <stdint.h>
#include <stdbool.h>

#include "Log.h"     /* LogSink_t, LogStreamResult_t */

/* Output destinations (bitmask) */
#define DBGLOG_DEST_UART   (1U << 0)
#define DBGLOG_DEST_FLASH  (1U << 1)
#define DBGLOG_DEST_BOTH   (DBGLOG_DEST_UART | DBGLOG_DEST_FLASH)

void DBGLOG_vInit(void);

/* Format and enqueue one line for the given destination(s). Non-blocking. */
void DBGLOG_vPut(uint8_t dest, const char *format, ...);

/* Like DBGLOG_vPut, but prefixes the line with "<date> <time> <battery>mV ". */
void DBGLOG_vPutVerbose(uint8_t dest, const char *format, ...);

/* Ask the consumer task to stream the newest u8Percent % of the external-flash
 * log (100 = all of it) over the debug transport. Done from the consumer so it
 * stays the single UART writer (no interleaving with live log output).
 * Returns false, and requests nothing, while a stream is already pending or
 * running - see DBGLOG_bDumpBusy. */
bool DBGLOG_bRequestDump(uint8_t u8Percent);

/* Same as DBGLOG_bRequestDump, but streams via the given sink instead of the
 * debug UART -- still run from the consumer task, so it can't race the
 * consumer's own log writes on the shared storage device. Used by FrKernel's
 * LoRa interface, which has no UART session to read a UART-routed dump from.
 * pfnDone (may be NULL) is called on the consumer task once the stream has
 * finished or been aborted, with what it did - FrKernel uses it to send the
 * end-of-stream frame. */
bool DBGLOG_bRequestDumpVia(LogSink_t sink, uint8_t u8Percent,
                            void (*pfnDone)(const LogStreamResult_t *pRes));

/* True from a dump request until that stream has finished. */
bool DBGLOG_bDumpBusy(void);

/* Stop a running stream at its next chunk (LOG_vAbortStream), or cancel one
 * that was requested but has not started yet. */
void DBGLOG_vCancelDump(void);

/* Ask the consumer task to erase the whole text log (the external-flash log
 * region, via LOG_vErase). Done from the consumer so it never races the
 * consumer's own log writes on the shared storage device. */
void DBGLOG_vRequestErase(void);

/* Flash backend only: ask the consumer task to erase the ENTIRE ext-flash
 * device -- log partition AND the OTA image scratchpad + metadata below it
 * (LOG_vEraseAll), not just the log. Destroys any staged-but-not-yet-
 * installed OTA image. Same consumer-task rationale as DBGLOG_vRequestErase. */
void DBGLOG_vRequestEraseAll(void);

/* Drop the UART half of every VERBOSE line (DBG / DBG_LOG) while bMute is
 * true; the flash half of DBG_LOG/LOG and all plain _DBG_ lines are
 * unaffected. The kernel console mutes these so its terminal is an operator
 * UI rather than a scrolling debug trace, and prints its own output through
 * _DBG_ - which keeps the DbgLog consumer the only writer of the UART. */
void DBGLOG_vMuteVerboseUart(bool bMute);

/* Compatibility wrappers (UART-only / flash-only). */
void DBGLOG_vPutDebug(const char *format, ...);
void DBGLOG_vPutLog(const char *format, ...);

#define DBG(x, ...)    DBGLOG_vPutVerbose(DBGLOG_DEST_UART, x, ##__VA_ARGS__)
#define _DBG_(x, ...)  DBGLOG_vPut(DBGLOG_DEST_UART, x, ##__VA_ARGS__)

#define LOG(x, ...)        DBGLOG_vPutVerbose(DBGLOG_DEST_FLASH, x, ##__VA_ARGS__)
#define DBG_LOG(x, ...)    DBGLOG_vPutVerbose(DBGLOG_DEST_BOTH,  x, ##__VA_ARGS__)

#define _LOG_(x, ...)      DBGLOG_vPut(DBGLOG_DEST_FLASH, x, ##__VA_ARGS__)
#define _DBG_LOG_(x, ...)  DBGLOG_vPut(DBGLOG_DEST_BOTH,  x, ##__VA_ARGS__)

#endif /* INC_DBGLOG_H_ */
