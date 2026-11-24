/*
 * FrKernel_Config.h
 *
 * Compile-time tuning for the FrKernel command interface (kernelV2). WHICH
 * transport is built is decided in fr_app/inc/config/build_config.h:
 *   FRKERNEL_INTERFACE_UART          tag, single-letter commands on the debug UART
 *   FRKERNEL_INTERFACE_LORA          tag, v2 frames over LoRa
 *   FRKERNEL_INTERFACE_LORA_CONSOLE  the operator console rig (FrKernelConsole.c)
 * UART and LORA may be combined; they then share one session. The console is
 * exclusive of both.
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_CONFIG_H_
#define WORKER_FRKERNEL_FRKERNEL_CONFIG_H_

#include "build_config.h"

/* ---- Tag: session and connect windows ---------------------------------- */

/* A session ends after this long without a command addressed to the tag
 * (single or bulk). Outbound traffic the operator is waiting on - a log
 * stream, an OTA receive or distribute - counts as
 * activity while it runs, so a long download is never cut off by its own
 * silence. A bulk console's repeated CONNECT beacon deliberately does NOT
 * count: a console left on would otherwise hold a whole herd out of its
 * schedule indefinitely. */
#define FRKERNEL_INACTIVITY_TIMEOUT_MS  (5U * 60U * 1000U)

/* How long the shake sequence keeps the tag listening for a CONNECT before it
 * gives up and goes back to its schedule. */
#define FRKERNEL_SHAKE_WINDOW_MS        (2U * 60U * 1000U)

/* How long every discovery wake listens for a CONNECT before the campaign.
 * It must fit inside APP_WAKEUP_BUFFER_MS (DeviceDiscovery.h; checked by a
 * _Static_assert there): that buffer is the quiet part of every wake, before
 * the primary's first DReq, so a CONNECT is not competing with campaign
 * traffic and costs the campaign nothing when no console is about. */
#define FRKERNEL_WAKE_WINDOW_MS         5000U

/* ---- Tag: presence beacon (ENABLE_KERNEL_PRESENCE_BEACON) ----------------
 *
 * Between scheduled wakes a tag beacons its id every MIN..MAX ms (uniform
 * random per beacon, from MESHNETWORK_u32Random, so a herd never lines up)
 * and listens for PRESENCE_LISTEN_MS after each one. A console that wants the
 * tag answers inside that listen with a CONNECT (LORA_FAST_REPLY_* in
 * LoraRadio_Config.h), so connecting takes at most MAX ms with no shake and no
 * waiting for the schedule.
 *
 * Cost, estimated from datasheet currents (measure it on the Otii before
 * trusting it): one beacon is ~18 ms of TX at ~20-25 mA, ~30 ms of RX at
 * ~5 mA and a few ms of radio re-init and TCXO start, ~0.6 mC in all. At
 * 10..20 s (15 s on average) that is ~40 uA averaged (~1 mAh a day) - far more
 * than a sleeping tag's floor, so these two numbers are THE power knob of this
 * feature: doubling both halves it. 5..15 s was the first choice, ~60 uA. */
#define FRKERNEL_PRESENCE_MIN_MS        10000U
#define FRKERNEL_PRESENCE_MAX_MS        20000U

/* Listen after each beacon. The receiver only has to catch the START of the
 * console's long-preamble CONNECT (stop-on-preamble keeps it receiving to the
 * end), and that starts ~4-5 ms after the beacon on both sides; 30 ms leaves
 * room for a slow console without paying for much idle RX. */
#define FRKERNEL_PRESENCE_LISTEN_MS     30U

/* How long the kernel accepts a CONNECT after asking for a beacon: the beacon,
 * the listen and the parser hand-off all fit in well under this. Longer would
 * only let a stray CONNECT that arrived some other way open a session. */
#define FRKERNEL_PRESENCE_CONNECT_MS    500U

/* Task poll while a session is active (inactivity check). Idle, the task
 * blocks until a command arrives - the v1 task woke every 10 s forever. */
#define FRKERNEL_POLL_INTERVAL_MS       1000U

/* Upper bound on the random back-off before a SCAN answer, so several shaken
 * tags do not answer on top of each other. Drawn from MESHNETWORK_u32Random
 * (per-node seeded): v1 used rand(), which FreeRTOS's per-task newlib reent
 * left unseeded on the FrKernel task, so every tag drew the SAME delay and
 * collided anyway. */
#define FRKERNEL_SCAN_JITTER_MAX_MS     1500U

/* Before giving the device back after a final reply ("x", a timeout notice, a
 * sleep command), wait up to this long for that reply to actually leave the
 * radio. v1 cleared the session first and the AppTask then deep-slept the
 * radio with the reply still in carrier sense. */
#define FRKERNEL_TX_DRAIN_MAX_MS        3000U

/* ---- Tag: RX path ------------------------------------------------------- */

/* Longest command frame a tag accepts: header + letter + space + argument.
 * The longest argument is "park 10000". Frames are filtered by version,
 * direction and address in the parser callback BEFORE they are queued, so
 * other tags' replies and log streams never occupy these slots. */
#define FRKERNEL_RX_ITEM_LEN            40U
#define FRKERNEL_LORA_QUEUE_LEN          4U

/* UART command line ("z park 10000" is the longest). */
#define FRKERNEL_LINE_BUF_LEN           32U
#define FRKERNEL_RESP_BUF_LEN          256U

/* ---- Tag: log stream ---------------------------------------------------- */

/* Longest a stream frame may wait for room in the radio TX queue. When it
 * cannot get in, the session or the radio has gone away and the stream stops
 * rather than blocking the DbgLog consumer (and so all logging) forever. */
#define FRKERNEL_STREAM_TX_WAIT_MS      10000U

/* Quiet time after each stream frame's transmission before the next flash
 * read. The stream used to refill the TX queue the instant a slot opened, so
 * every log read ran right on top of a PA pulse - the condition under which
 * frtag2 measured corrupt NOR reads that even a back-to-back re-read did not
 * catch (OTA_LORA_CHUNK_GAP_MS, Fota_Config.h, same 100 ms). It also kept one
 * stream frame at most in the TX queue, so a reply to "s" (stop) is not stuck
 * behind eight of them. Costs throughput (~0.7 KB/s instead of ~1 KB/s at
 * SF7/BW250), which is what the half/quarter/tenth stream commands are for. */
#define FRKERNEL_STREAM_READ_SETTLE_MS  100U

/* ---- Console ------------------------------------------------------------ */

/* CONNECT repeat period while connecting. At 1 s a tag's 5 s wake window hears
 * four or five of them; a ~15 B frame at SF7/BW250 is ~20 ms of air. */
#define FRKERNEL_CON_CONNECT_PERIOD_MS  1000U

/* A bulk console stops repeating CONNECT after this long without operator
 * input, so a forgotten console cannot keep pulling a herd into the kernel at
 * every discovery wake. Matches the tag-side inactivity timeout. */
#define FRKERNEL_CON_BULK_IDLE_MS       FRKERNEL_INACTIVITY_TIMEOUT_MS

/* Single connection: resend an unanswered command (same seq, so the tag never
 * runs it twice) after this long, up to this many attempts in total. */
#define FRKERNEL_CON_REPLY_TIMEOUT_MS   2500U
#define FRKERNEL_CON_CMD_TRIES          3U

/* Bulk: there is no reply to retry on, so every command is sent this many
 * times, this far apart. The tags dedupe on seq. */
#define FRKERNEL_CON_BULK_REPEATS       3U
#define FRKERNEL_CON_BULK_GAP_MS        400U

/* Scan list on the console: how many answering tags one scan numbers, so the
 * operator can connect with "c <n>" instead of typing an 8-digit id. Answers
 * past this still print, unnumbered. 16 x 4 B of console RAM; only the console
 * build has the list at all. */
#define FRKERNEL_CON_SCAN_MAX           16U

/* While scanning, how often the console re-sends SCAN so a tag shaken after
 * the scan began is still listed. Each is ~15 ms on air; tags answer only from
 * the shake window or a session, so this costs the herd nothing. */
#define FRKERNEL_CON_SCAN_QUERY_MS      3000U

/* Presence beacons queued from the parser to the console task. A burst of a
 * whole herd's beacons is spread over 10-20 s, so a few slots are plenty; an
 * overflow only delays a tag's listing to its next beacon. */
#define FRKERNEL_CON_PRES_QUEUE_LEN     8U

/* Log stream on the console: no stream frame for this long ends stream mode
 * (the END frame can be lost like any other) with an "incomplete" notice. */
#define FRKERNEL_CON_STREAM_IDLE_MS     8000U

/* Status line cadence while waiting for a single tag to answer CONNECT. */
#define FRKERNEL_CON_WAIT_NOTE_MS       30000U

/* Console inbound frame queue (largest frame is a full stream frame). */
#define FRKERNEL_CON_RX_QUEUE_LEN       8U

#endif /* WORKER_FRKERNEL_FRKERNEL_CONFIG_H_ */
