/*
 * FrKernel_Protocol.h
 *
 * The kernelV2 over-the-air frame and the single-letter command set, shared by
 * the two ends of a FrKernel conversation:
 *
 *   - the tag (FRKERNEL_INTERFACE_LORA and/or _UART, FrKernel.c), which
 *     executes commands, and
 *   - the console (FRKERNEL_INTERFACE_LORA_CONSOLE, FrKernelConsole.c), a
 *     dedicated rig whose only job is to turn what an operator types on its
 *     debug UART into frames for one tag or for every tag in earshot.
 *
 * Both ends include this one header so the command letters, the frame layout
 * and the help text cannot drift apart between the image that sends a command
 * and the image that runs it.
 *
 * Why a binary frame instead of the old "tag <ID> <cmd>" text
 * -----------------------------------------------------------
 * The v1 protocol carried the target address as hex text inside the command
 * line, so every tag had to strtoul() every FrKernel packet it heard to find
 * out whether it was the addressee, and nothing distinguished a command from a
 * response except that responses happened not to start with "tag". It had no
 * sequence number, so a lost reply could not be retried without running an
 * action twice, and a lost stream packet left a silent hole in a downloaded
 * log. The v2 frame puts the address, the direction and a sequence number in
 * a fixed header:
 *
 *   [0]    MeshPktType_FrKernel      mesh-level type (added/stripped by mesh)
 *   [1]    FRK_PROTO_VERSION (0xA2)  never a printable byte, so a v1 "tag ..."
 *                                    line and a v2 frame can never be confused
 *                                    in either direction: a v1 tag in a mixed
 *                                    herd ignores v2 frames (they do not start
 *                                    with "tag") and a v2 tag ignores v1 lines
 *   [2]    kind (FrkKind_e)          ASCII so a raw sniff is still readable
 *   [3..6] id, little-endian u32     console->tag: target, or FRK_ID_BULK
 *                                    tag->console: the replying tag's own id
 *   [7]    seq                       console->tag: command sequence number, so
 *                                    a retried command is recognised as a
 *                                    retry and an action is never run twice
 *                                    tag->console REPLY: echoes the command's
 *                                    seq so the console can match it
 *                                    tag->console STREAM: running packet
 *                                    counter so the console can count and
 *                                    report lost log packets
 *   [8..]  payload                   kind-specific, see FrkKind_e
 *
 * Offsets above are within the LoRa packet. MESHNETWORK_vOnFrKernelPacket()
 * hands FrKernel the packet with byte [0] already stripped, so the FRK_OFS_*
 * constants below are relative to that stripped buffer.
 *
 * Single vs bulk
 * --------------
 * A tag replies to a frame if and only if the frame was addressed to its own
 * id. A frame addressed to FRK_ID_BULK is executed silently - every tag in
 * earshot answering at once would only collide on a shared channel - and the
 * result goes to that tag's own DBG_LOG. That one rule is the whole difference
 * between a single and a bulk connection on the tag side; the console is what
 * keeps track of which kind of connection the operator opened.
 */

#ifndef WORKER_FRKERNEL_FRKERNEL_PROTOCOL_H_
#define WORKER_FRKERNEL_FRKERNEL_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "LoraRadio_Config.h"    /* LORA_MAX_PACKET_SIZE */

/* -------------------------------------------------------------------------- */
/* Frame layout (offsets in the buffer FrKernel sees: mesh type byte removed) */
/* -------------------------------------------------------------------------- */
#define FRK_PROTO_VERSION      0xA2U

#define FRK_OFS_VERSION        0U
#define FRK_OFS_KIND           1U
#define FRK_OFS_ID             2U     /* 4 bytes, little-endian */
#define FRK_OFS_SEQ            6U
#define FRK_OFS_PAYLOAD        7U
#define FRK_HDR_LEN            FRK_OFS_PAYLOAD

/* Payload bytes one frame can carry. The radio appends a 2-byte CRC inside the
 * uint8_t length field and the mesh type byte comes first, hence the "- 3",
 * the same bound every other FrKernel LoRa sender has always used. */
#define FRK_MAX_PAYLOAD        ((uint16_t)(LORA_MAX_PACKET_SIZE - 3 - FRK_HDR_LEN))

/* "Every tag that hears this". No real tag id is all-ones: ids are the low
 * 16 bits of the radio's unique id (see LORARADIO_u32GetUniqueId). */
#define FRK_ID_BULK            0xFFFFFFFFUL

typedef enum {
    /* console -> tag. No payload. Opens a kernel session on a tag that is in a
     * connect window (2 min after the shake sequence, or the first 5 s of a
     * discovery wake) or that already has a session. A singly-addressed tag
     * answers FRK_KIND_REPLY with its status line; a bulk connect is silent.
     * The console repeats this frame until it is answered (single) or the
     * operator stops (bulk), because the console cannot know when a tag's
     * window opens. */
    FRK_KIND_CONNECT    = 'C',

    /* console -> tag, always FRK_ID_BULK. No payload. A tag that is in its
     * shake window, or already in a session, answers with its id after a
     * random 0-2 s back-off so several shaken tags do not collide. Tags in the
     * 5 s discovery-wake window deliberately stay quiet: that window is open
     * on the whole herd at once. Never opens a session. */
    FRK_KIND_SCAN       = 'S',

    /* console -> tag. Payload: one command letter, optionally followed by a
     * space and an argument ("d 60"). Only acted on by a tag that already has
     * a session. */
    FRK_KIND_CMD        = 'K',

    /* tag -> console. Payload: text. seq echoes the command's seq (0 for
     * unsolicited lines such as the session-timeout notice). */
    FRK_KIND_REPLY      = 'R',

    /* tag -> console. Payload: raw log bytes, possibly containing NULs. seq is
     * a running per-stream counter starting at 0. */
    FRK_KIND_STREAM     = 'D',

    /* tag -> console. Payload: text summary. seq = the number of STREAM frames
     * that were sent (mod 256), so the console can check what it missed. */
    FRK_KIND_STREAM_END = 'E',

    /* tag -> console, single sessions only. Payload: why the session closed
     * ("5 min inactivity", ...). Lets the console drop back to idle instead
     * of retrying commands into a tag that has gone back to sleep. */
    FRK_KIND_BYE        = 'X',
} FrkKind_e;

/* -------------------------------------------------------------------------- */
/* Command set                                                                */
/* -------------------------------------------------------------------------- */

/* Command flags */
#define FRK_CMDF_SINGLE   0x01U  /* needs a single connection: a query whose
                                  * only output is its reply, or an action that
                                  * must never hit a whole herd at once */
#define FRK_CMDF_ARG      0x02U  /* takes an optional argument after a space */

typedef struct {
    char        cLetter;
    uint8_t     u8Flags;
    const char *pcArgs;   /* shown in the help line, "" if none */
    const char *pcHelp;
} FrkCmd_t;

/* Stream fractions: the "latest N %" of the persisted log. 100 = all of it. */
#define FRK_STREAM_ALL_PCT      100U
#define FRK_STREAM_HALF_PCT      50U
#define FRK_STREAM_QUARTER_PCT   25U
#define FRK_STREAM_TENTH_PCT     10U

/* Look up a command letter. Case-sensitive on purpose: the upper-case letters
 * are the destructive or terminal commands (erase, sleep, radio test), so a
 * stray lower-case keystroke can never reach one. NULL if not a command. */
const FrkCmd_t *FRK_ptFindCmd(char cLetter);

/* Number of entries in the command table and entry i, for printing help. */
size_t          FRK_szCmdCount(void);
const FrkCmd_t *FRK_ptCmdAt(size_t i);

/* Emit the help table one line at a time through pfnPut. bBulk == true leaves
 * out the FRK_CMDF_SINGLE commands, so a bulk session's help shows only what
 * it can actually send. */
void FRK_vPrintHelp(void (*pfnPut)(const char *pcLine), bool bBulk);

/* Little-endian u32 helpers for the id field. */
static inline void FRK_vPutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline uint32_t FRK_u32GetU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#endif /* WORKER_FRKERNEL_FRKERNEL_PROTOCOL_H_ */
