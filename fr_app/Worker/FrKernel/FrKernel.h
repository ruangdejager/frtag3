/*
 * FrKernel.h
 *
 * Device command interface Worker (kernelV2).
 *
 * Transport selected at compile time in build_config.h:
 *   FRKERNEL_INTERFACE_UART          tag: single-letter commands on the debug UART
 *   FRKERNEL_INTERFACE_LORA          tag: v2 frames over LoRa (FrKernel_Protocol.h)
 *   FRKERNEL_INTERFACE_LORA_CONSOLE  the operator console (FrKernelConsole.c);
 *                                    exclusive of the two above
 *
 * Session model on a tag
 * ----------------------
 * A tag is either IDLE or has an ACTIVE kernel session. While a session is
 * active the kernel owns the device: DeviceDiscovery runs no campaign, starts
 * no scheduled wake, GPS pre-trigger or basic beacon, and MeshNetwork drops
 * every packet that is not FrKernel or OTA traffic. Nothing but the session
 * ending - "x", 5 minutes without a command, or a terminal command such as
 * production sleep - gives the device back.
 *
 * Over LoRa a session can only be OPENED (by a CONNECT frame) while a connect
 * window is open. DeviceDiscovery opens the windows:
 *   - FRKERNEL_WINDOW_SHAKE for FRKERNEL_SHAKE_WINDOW_MS (2 min) after the
 *     shake sequence;
 *   - FRKERNEL_WINDOW_WAKE  for FRKERNEL_WAKE_WINDOW_MS (5 s) at the start of
 *     every discovery wake, inside the quiet wake buffer before any campaign
 *     traffic. A session opened there takes the device out of that campaign.
 * Outside a window a CONNECT is ignored, so a console left beaconing can never
 * pull a tag out of its schedule at an arbitrary moment.
 *
 * Over the debug UART (bench) there is no window: any command opens a session.
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_H_
#define WORKER_FRKERNEL_FRKERNEL_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FRKERNEL_WINDOW_NONE  = 0,
    FRKERNEL_WINDOW_WAKE  = 1,   /* first seconds of a discovery wake */
    FRKERNEL_WINDOW_SHAKE = 2,   /* after the shake sequence          */
} FrKernelWindow_e;

void FRKERNEL_vInit(void);

/* Open (or extend) a connect window of the given kind for u32Ms. A window
 * never shortens an open one, and a SHAKE window is never downgraded to WAKE.
 * No-op while a session is already active. Safe from any task. */
void FRKERNEL_vOpenWindow(FrKernelWindow_e eWindow, uint32_t u32Ms);

/* Close any open connect window now. */
void FRKERNEL_vCloseWindow(void);

/* True while a connect window is open (and not yet expired). */
bool FRKERNEL_bWindowOpen(void);

/* True while a kernel session is active: the device belongs to the kernel. */
bool FRKERNEL_bIsActive(void);

/* True while the kernel holds the device: a session is active, or the shake
 * window is still waiting for a connect. The wake-schedule task stands down
 * (no discovery wake, basic beacon or GPS pre-trigger) while this is true. */
bool FRKERNEL_bHoldsDevice(void);

/* End the session from outside the kernel (e.g. the device is going into a
 * mode that cannot serve one). pcReason is logged. No-op when idle. */
void FRKERNEL_vEndSession(const char *pcReason);

#endif /* WORKER_FRKERNEL_FRKERNEL_H_ */
