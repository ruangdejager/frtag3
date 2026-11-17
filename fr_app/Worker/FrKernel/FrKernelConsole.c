/*
 * FrKernelConsole.c
 *
 * The kernelV2 operator console: a tag built with
 * FRKERNEL_INTERFACE_LORA_CONSOLE, whose only job is to turn what an operator
 * types on its debug UART into FrKernel v2 frames (FrKernel_Protocol.h) and to
 * print what comes back. It replaces the v1 "LoRa bridge", which forwarded
 * whatever was typed verbatim ("tag 1234 flash stream") and printed whatever
 * arrived, with no notion of a connection, no retry and no way to tell one
 * tag's answer from another's.
 *
 * Operator view
 * -------------
 *   c 1234     connect to tag 1234. The console repeats CONNECT every second
 *              until the tag answers - the operator does not have to time it
 *              against the tag's 2 min shake window or its 5 s discovery-wake
 *              window. Once connected, commands need no id.
 *   c bulk     connect to every tag in earshot. CONNECT(bulk) repeats every
 *              second, so tags join as their windows open, until 5 minutes
 *              pass with no typing. Bulk tags never reply; queries are refused
 *              locally. Every bulk command is sent three times (tags dedupe).
 *   c off      stop connecting / forget the connection (tags time out).
 *   scan       list tags that are just-shaken or already connected.
 *   dbg        toggle this console's own debug output.
 *   ?          help.
 *   <letter> [arg]   a tag command (FrKernel_Commands.c), sent on Enter.
 *
 * Single connection: a command is resent (same seq, so the tag never runs it
 * twice) if no reply arrives in FRKERNEL_CON_REPLY_TIMEOUT_MS, up to
 * FRKERNEL_CON_CMD_TRIES times in all.
 *
 * Log streams: STREAM frames are printed raw. Their seq is checked so lost
 * frames are reported ("[console: 3 frame(s) lost]") rather than leaving a
 * silent hole, and stream mode ends on the END frame or after
 * FRKERNEL_CON_STREAM_IDLE_MS of silence - v1's bridge stayed in raw mode for
 * good if its END marker was lost.
 *
 * Output path: everything this console prints goes through DbgLog's plain
 * _DBG_ path, so the DbgLog consumer stays the one and only writer of the
 * debug UART. v1's bridge wrote the UART from the mesh parser task, its own
 * task and the consumer at once, and HAL_UART_vTxPutByte's unlocked ring head
 * update garbled exactly the log streams the rig existed to capture.
 */

#include "build_config.h"

#ifdef FRKERNEL_INTERFACE_LORA_CONSOLE

#include "FrKernel.h"
#include "FrKernel_Config.h"
#include "FrKernel_Protocol.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "Debug.h"
#include "DbgLog.h"

#define CON_FLAG_UART  0x01U
#define CON_FLAG_LORA  0x02U
#define CON_TICK_MS    100U

typedef enum {
    CON_IDLE,
    CON_CONNECTING,   /* single: repeating CONNECT, waiting for an answer */
    CON_SINGLE,
    CON_BULK,
} ConMode_e;

typedef struct {
    uint8_t len;
    uint8_t data[LORA_MAX_PACKET_SIZE - 1U];
} ConFrame_t;

static osThreadId_t       s_taskHandle = NULL;
static osMessageQueueId_t s_rxQueue    = NULL;

static ConMode_e s_eMode            = CON_IDLE;
static uint32_t  s_u32Target        = 0U;
static uint8_t   s_u8Seq            = 0U;
static uint32_t  s_u32NextConnect   = 0U;
static uint32_t  s_u32ConnectStart  = 0U;
static uint32_t  s_u32NextNote      = 0U;
static uint32_t  s_u32LastInput     = 0U;
static bool      s_bBulkBeacon      = false;

/* Single connection: the one command awaiting its reply. */
static bool      s_bPending         = false;
static uint8_t   s_u8PendSeq        = 0U;
static uint8_t   s_u8PendTries      = 0U;
static uint32_t  s_u32PendSent      = 0U;
static char      s_acPend[FRKERNEL_LINE_BUF_LEN];

/* Stream from the connected tag. */
static bool      s_bStreaming       = false;
static uint8_t   s_u8StreamExpect   = 0U;
static uint32_t  s_u32StreamFrames  = 0U;
static uint32_t  s_u32StreamLost    = 0U;
static uint32_t  s_u32StreamLast    = 0U;

/* Typed line. */
static char      s_acLine[FRKERNEL_LINE_BUF_LEN];
static uint8_t   s_u8LineLen        = 0U;

static bool      s_bDbgMuted        = true;

static void CON_vTask(void *arg);

/* -------------------------------------------------------------------------- */
/* Output                                                                     */
/* -------------------------------------------------------------------------- */

static void CON_vPut(const char *pcText)
{
    _DBG_("%s", pcText);
}

static void CON_vPrompt(void)
{
    const char *pcMode = "";
    char acId[12];
    switch (s_eMode)
    {
        case CON_SINGLE:     (void)snprintf(acId, sizeof(acId), "%04" PRIX32, s_u32Target); pcMode = acId; break;
        case CON_CONNECTING: (void)snprintf(acId, sizeof(acId), "%04" PRIX32 "?", s_u32Target); pcMode = acId; break;
        case CON_BULK:       pcMode = "bulk"; break;
        default:             break;
    }
    _DBG_("%s> %.*s", pcMode, (int)s_u8LineLen, s_acLine);
}

/* Print a line that arrived while the operator may be mid-typing: start it on
 * a fresh line, then redraw the prompt and whatever was typed so far. */
static void CON_vAsync(const char *pcText)
{
    _DBG_("\r\n%s", pcText);
    size_t n = strlen(pcText);
    if ((n == 0U) || (pcText[n - 1U] != '\n'))
        _DBG_("\r\n");
    if (!s_bStreaming)
        CON_vPrompt();
}

/* -------------------------------------------------------------------------- */
/* Transmit                                                                   */
/* -------------------------------------------------------------------------- */

static bool CON_bSend(FrkKind_e eKind, uint32_t u32Id, uint8_t u8Seq, const char *pcPayload)
{
    LoraRadio_Packet_t tPkt;
    memset(&tPkt, 0, sizeof(tPkt));
    tPkt.buffer[0] = (uint8_t)MeshPktType_FrKernel;
    uint8_t *pF = &tPkt.buffer[1];
    pF[FRK_OFS_VERSION] = FRK_PROTO_VERSION;
    pF[FRK_OFS_KIND]    = (uint8_t)eKind;
    FRK_vPutU32(&pF[FRK_OFS_ID], u32Id);
    pF[FRK_OFS_SEQ]     = u8Seq;
    uint16_t u16Len = 0U;
    if (pcPayload != NULL)
    {
        u16Len = (uint16_t)strlen(pcPayload);
        if (u16Len > FRK_MAX_PAYLOAD)
            u16Len = FRK_MAX_PAYLOAD;
        memcpy(&pF[FRK_OFS_PAYLOAD], pcPayload, u16Len);
    }
    tPkt.length = (uint8_t)(1U + FRK_HDR_LEN + u16Len);
    return LORARADIO_bTxPacket(&tPkt);
}

static uint8_t CON_u8NextSeq(void)
{
    if (++s_u8Seq == 0U)     /* 0 is reserved for CONNECT / unsolicited */
        s_u8Seq = 1U;
    return s_u8Seq;
}

/* -------------------------------------------------------------------------- */
/* Help                                                                       */
/* -------------------------------------------------------------------------- */

static void CON_vHelp(void)
{
    /* One CON_vPut per line or two: a DbgLog message is capped at 255 B, and
     * this block as a single string was nearly twice that. */
    static const char *const apcLines[] = {
        "\r\nConsole:\r\n",
        "  c <id>     connect to one tag (repeats until it answers)\r\n",
        "  c bulk     connect to every tag in range (no replies)\r\n",
        "  c off      stop / forget the connection\r\n",
        "  scan       list tags that were just shaken or are connected\r\n",
        "  dbg        toggle this console's own debug output\r\n",
        "A tag only listens for a connect for 2 min after its shake\r\n",
        "sequence, and for the first 5 s of each discovery wake.\r\n\r\n",
    };
    for (size_t i = 0U; i < (sizeof(apcLines) / sizeof(apcLines[0])); i++)
        CON_vPut(apcLines[i]);
    FRK_vPrintHelp(CON_vPut, s_eMode == CON_BULK);
}

/* -------------------------------------------------------------------------- */
/* Typed lines                                                                */
/* -------------------------------------------------------------------------- */

static void CON_vResetConnection(void)
{
    s_eMode       = CON_IDLE;
    s_bPending    = false;
    s_bBulkBeacon = false;
    s_bStreaming  = false;
}

static void CON_vConnectCmd(const char *pcArg)
{
    if (strcmp(pcArg, "bulk") == 0)
    {
        CON_vResetConnection();
        s_eMode          = CON_BULK;
        s_bBulkBeacon    = true;
        s_u32NextConnect = osKernelGetTickCount();
        CON_vPut("bulk: joining every tag whose connect window opens (shake, or\r\n"
                 "the first 5 s of a discovery wake). Joined tags stay 5 min after\r\n"
                 "the last command. Join beacon stops after 5 min without typing.\r\n");
        FRK_vPrintHelp(CON_vPut, true);
        return;
    }

    if (strcmp(pcArg, "off") == 0)
    {
        CON_vResetConnection();
        CON_vPut("connection dropped (tags time out after 5 min; 'x' tells them now)\r\n");
        return;
    }

    if (*pcArg == '\0')
    {
        CON_vPut("usage: c <id> | c bulk | c off\r\n");
        return;
    }

    char *pcEnd;
    /* 4 digits (the legacy id, still printed on older tags and labels) or 8
     * (the full id). A 4-digit target reaches every tag sharing those 16 bits;
     * the first one to answer pins the connection to its full id - see
     * CON_vHandleFrame. */
    uint32_t u32Id = (uint32_t)strtoul(pcArg, &pcEnd, 16);
    if ((pcEnd == pcArg) || (*pcEnd != '\0') || (strlen(pcArg) > 8U) ||
        (u32Id == FRK_ID_BULK))
    {
        CON_vPut("tag id is 4 or 8 hex digits, e.g. c 1A2B or c 0A3C1A2B\r\n");
        return;
    }

    CON_vResetConnection();
    s_eMode          = CON_CONNECTING;
    s_u32Target      = u32Id;
    s_u32ConnectStart = osKernelGetTickCount();
    s_u32NextConnect = s_u32ConnectStart;
    s_u32NextNote    = s_u32ConnectStart + FRKERNEL_CON_WAIT_NOTE_MS;
    _DBG_("connecting to %04" PRIX32 " - shake it, or wait for its next discovery\r\n"
          "wake. 'c off' stops calling.\r\n", u32Id);
}

static void CON_vSendCommand(const char *pcLine)
{
    const FrkCmd_t *pCmd = FRK_ptFindCmd(pcLine[0]);
    if ((pCmd == NULL) || ((pcLine[1] != '\0') && (pcLine[1] != ' ')))
    {
        CON_vPut("not a command - '?' lists them\r\n");
        return;
    }
    if ((pcLine[1] == ' ') && !(pCmd->u8Flags & FRK_CMDF_ARG))
    {
        _DBG_("'%c' takes no argument\r\n", pcLine[0]);
        return;
    }

    if ((s_eMode == CON_IDLE) || (s_eMode == CON_CONNECTING))
    {
        CON_vPut("not connected - 'c <id>' or 'c bulk' first\r\n");
        return;
    }

    if (s_eMode == CON_BULK)
    {
        if (pCmd->u8Flags & FRK_CMDF_SINGLE)
        {
            _DBG_("'%c' needs a single connection (its answer is the point)\r\n", pcLine[0]);
            return;
        }
        uint8_t u8Seq = CON_u8NextSeq();
        for (uint8_t i = 0U; i < FRKERNEL_CON_BULK_REPEATS; i++)
        {
            if (i > 0U)
                osDelay(FRKERNEL_CON_BULK_GAP_MS);
            (void)CON_bSend(FRK_KIND_CMD, FRK_ID_BULK, u8Seq, pcLine);
        }
        _DBG_("sent to all (x%u, no replies)\r\n", (unsigned)FRKERNEL_CON_BULK_REPEATS);
        if (pcLine[0] == 'x')
        {
            CON_vResetConnection();
            CON_vPut("bulk session ended\r\n");
        }
        return;
    }

    /* Single. One command in flight at a time - a second one would make the
     * replies ambiguous; the operator waits, or the retry gives up. */
    if (s_bPending)
    {
        CON_vPut("still waiting for the previous reply\r\n");
        return;
    }

    s_u8PendSeq   = CON_u8NextSeq();
    s_u8PendTries = 1U;
    s_u32PendSent = osKernelGetTickCount();
    (void)snprintf(s_acPend, sizeof(s_acPend), "%s", pcLine);
    s_bPending    = true;
    (void)CON_bSend(FRK_KIND_CMD, s_u32Target, s_u8PendSeq, s_acPend);
}

static void CON_vRunLine(char *pcLine)
{
    while (*pcLine == ' ')
        pcLine++;
    size_t n = strlen(pcLine);
    while ((n > 0U) && (pcLine[n - 1U] == ' '))
        pcLine[--n] = '\0';

    s_u32LastInput = osKernelGetTickCount();
    if ((s_eMode == CON_BULK) && !s_bBulkBeacon)
    {
        /* Operator is back: resume joining stragglers. */
        s_bBulkBeacon    = true;
        s_u32NextConnect = s_u32LastInput;
    }

    if (n == 0U)
        return;

    if ((pcLine[0] == 'c') && ((pcLine[1] == '\0') || (pcLine[1] == ' ')))
    {
        const char *pcArg = &pcLine[1];
        while (*pcArg == ' ')
            pcArg++;
        CON_vConnectCmd(pcArg);
    }
    else if (strcmp(pcLine, "scan") == 0)
    {
        (void)CON_bSend(FRK_KIND_SCAN, FRK_ID_BULK, CON_u8NextSeq(), NULL);
        CON_vPut("scan sent - shaken/connected tags answer within ~2 s\r\n");
    }
    else if (strcmp(pcLine, "dbg") == 0)
    {
        s_bDbgMuted = !s_bDbgMuted;
        DBGLOG_vMuteVerboseUart(s_bDbgMuted);
        CON_vPut(s_bDbgMuted ? "console debug output off\r\n" : "console debug output on\r\n");
    }
    else if (strcmp(pcLine, "?") == 0)
    {
        CON_vHelp();
    }
    else
    {
        CON_vSendCommand(pcLine);
    }
}

static void CON_vServiceUart(void)
{
    uint8_t u8;
    while (DEBUG_bReadByte(&u8))
    {
        if ((u8 == '\r') || (u8 == '\n'))
        {
            if ((u8 == '\n') && (s_u8LineLen == 0U))
                continue;                  /* the LF of a CRLF pair */
            s_acLine[s_u8LineLen] = '\0';
            s_u8LineLen = 0U;
            CON_vPut("\r\n");
            CON_vRunLine(s_acLine);
            s_acLine[0] = '\0';
            if (!s_bStreaming)
                CON_vPrompt();
        }
        else if ((u8 == 0x08U) || (u8 == 0x7FU))
        {
            if (s_u8LineLen > 0U)
            {
                s_u8LineLen--;
                CON_vPut("\b \b");
            }
        }
        else if ((u8 >= 0x20U) && (u8 < 0x7FU) && (s_u8LineLen < (sizeof(s_acLine) - 1U)))
        {
            /* Typing keeps the character on screen; nothing is sent until
             * Enter. */
            s_acLine[s_u8LineLen++] = (char)u8;
            _DBG_("%c", (char)u8);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Received frames                                                            */
/* -------------------------------------------------------------------------- */

static void CON_vEndStream(const char *pcWhy)
{
    char acMsg[112];
    (void)snprintf(acMsg, sizeof(acMsg),
                   "\r\n---- stream %s: %lu frame(s) received, %lu lost ----\r\n",
                   pcWhy, (unsigned long)s_u32StreamFrames, (unsigned long)s_u32StreamLost);
    s_bStreaming = false;
    CON_vAsync(acMsg);
}

static void CON_vHandleFrame(const ConFrame_t *pF)
{
    const uint8_t  *pu8   = pF->data;
    const FrkKind_e eKind = (FrkKind_e)pu8[FRK_OFS_KIND];
    const uint32_t  u32Id = FRK_u32GetU32(&pu8[FRK_OFS_ID]);
    const uint8_t   u8Seq = pu8[FRK_OFS_SEQ];
    uint16_t        u16N  = (uint16_t)(pF->len - FRK_HDR_LEN);
    if (u16N > FRK_MAX_PAYLOAD)
        u16N = FRK_MAX_PAYLOAD;

    /* Payload as printable text: NUL (and other control bytes a log can
     * hold) would cut or garble a %s, so they become '.'. CR/LF/TAB pass. */
    char acText[FRK_MAX_PAYLOAD + 1U];
    for (uint16_t i = 0U; i < u16N; i++)
    {
        char c = (char)pu8[FRK_OFS_PAYLOAD + i];
        acText[i] = ((c >= 0x20 && c < 0x7F) || c == '\r' || c == '\n' || c == '\t') ? c : '.';
    }
    acText[u16N] = '\0';

    /* Same-tag test, not equality: the operator may have typed only the 16-bit
     * id, and a tag on newer firmware answers with its full 32-bit one. */
    const bool bFromTarget = (s_eMode != CON_IDLE) && (s_eMode != CON_BULK) &&
                             LORARADIO_bSameDevice(u32Id, s_u32Target);

    switch (eKind)
    {
    case FRK_KIND_REPLY:
    {
        if ((s_eMode == CON_CONNECTING) && bFromTarget)
        {
            /* Pin to the full id of whichever tag answered first. Commands
             * addressed by a 16-bit id would reach every tag that shares it;
             * after this they reach only the one actually connected, and a
             * second tag with the same low half no longer passes bFromTarget. */
            s_u32Target = u32Id;
            s_eMode = CON_SINGLE;
            char acMsg[FRK_MAX_PAYLOAD + 24U];
            (void)snprintf(acMsg, sizeof(acMsg), "[%04" PRIX32 "] %s", u32Id, acText);
            CON_vAsync(acMsg);
            CON_vHelp();
            CON_vPrompt();
            break;
        }
        bool bExitDone = false;
        if (bFromTarget && s_bPending && (u8Seq == s_u8PendSeq))
        {
            s_bPending = false;
            /* The tag also sends a BYE after "x", but that frame can be lost
             * like any other: the reply alone is enough to know it is gone. */
            bExitDone = (s_acPend[0] == 'x');
        }

        char acMsg[FRK_MAX_PAYLOAD + 24U];
        (void)snprintf(acMsg, sizeof(acMsg), "[%04" PRIX32 "] %s", u32Id, acText);
        if (bExitDone)
            CON_vResetConnection();
        CON_vAsync(acMsg);
        break;
    }

    case FRK_KIND_STREAM:
        if (!bFromTarget)
            break;                       /* someone else's stream */
        if (!s_bStreaming)
        {
            s_bStreaming      = true;
            s_u8StreamExpect  = 0U;
            s_u32StreamFrames = 0U;
            s_u32StreamLost   = 0U;
            _DBG_("\r\n---- stream from %04" PRIX32 " ('s' + Enter stops it) ----\r\n", u32Id);
        }
        if (u8Seq != s_u8StreamExpect)
        {
            uint8_t u8Gap = (uint8_t)(u8Seq - s_u8StreamExpect);
            s_u32StreamLost += u8Gap;
            _DBG_("\r\n[console: %u frame(s) lost]\r\n", (unsigned)u8Gap);
        }
        s_u8StreamExpect = (uint8_t)(u8Seq + 1U);
        s_u32StreamFrames++;
        s_u32StreamLast  = osKernelGetTickCount();
        _DBG_("%s", acText);
        break;

    case FRK_KIND_STREAM_END:
        if (!bFromTarget)
            break;
        if (s_bStreaming && (u8Seq != s_u8StreamExpect))
        {
            uint8_t u8Gap = (uint8_t)(u8Seq - s_u8StreamExpect);
            s_u32StreamLost += u8Gap;
        }
        _DBG_("\r\n[%04" PRIX32 "] %s", u32Id, acText);
        s_bStreaming = true;             /* so CON_vEndStream prints the tally */
        CON_vEndStream("end");
        break;

    case FRK_KIND_BYE:
    {
        char acMsg[FRK_MAX_PAYLOAD + 24U];
        (void)snprintf(acMsg, sizeof(acMsg), "[%04" PRIX32 "] %s", u32Id, acText);
        if (bFromTarget)
        {
            if (s_bStreaming)
                CON_vEndStream("cut short");
            CON_vResetConnection();
        }
        CON_vAsync(acMsg);
        break;
    }

    default:
        break;   /* CONNECT/SCAN/CMD from another console */
    }
}

/* Override the weak MeshNetwork callback - parser task. Only queue: printing
 * happens on the console task (via DbgLog), never here. */
void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len)
{
    if ((s_rxQueue == NULL) || (len < FRK_HDR_LEN) ||
        (buf[FRK_OFS_VERSION] != FRK_PROTO_VERSION))
        return;

    ConFrame_t tF;
    tF.len = len;
    memcpy(tF.data, buf, len);
    if (osMessageQueuePut(s_rxQueue, &tF, 0U, 0U) == osOK && (s_taskHandle != NULL))
        osThreadFlagsSet(s_taskHandle, CON_FLAG_LORA);
}

/* Override the weak HAL callback - USART2 ISR. */
void UART2_vNotifyOnRX(void)
{
    if (s_taskHandle != NULL)
        osThreadFlagsSet(s_taskHandle, CON_FLAG_UART);
}

/* -------------------------------------------------------------------------- */
/* Timers                                                                     */
/* -------------------------------------------------------------------------- */

static void CON_vTimers(void)
{
    const uint32_t u32Now = osKernelGetTickCount();

    /* CONNECT beacon: single until answered, bulk until idle. */
    if (((s_eMode == CON_CONNECTING) || ((s_eMode == CON_BULK) && s_bBulkBeacon)) &&
        ((int32_t)(u32Now - s_u32NextConnect) >= 0))
    {
        s_u32NextConnect = u32Now + FRKERNEL_CON_CONNECT_PERIOD_MS;
        (void)CON_bSend(FRK_KIND_CONNECT,
                        (s_eMode == CON_BULK) ? FRK_ID_BULK : s_u32Target, 0U, NULL);
    }

    if ((s_eMode == CON_CONNECTING) && ((int32_t)(u32Now - s_u32NextNote) >= 0))
    {
        s_u32NextNote = u32Now + FRKERNEL_CON_WAIT_NOTE_MS;
        char acMsg[96];
        (void)snprintf(acMsg, sizeof(acMsg), "still calling %04" PRIX32 " (%lu s) - 'c off' to stop\r\n",
                       s_u32Target, (unsigned long)((u32Now - s_u32ConnectStart) / 1000U));
        CON_vAsync(acMsg);
    }

    if ((s_eMode == CON_BULK) && s_bBulkBeacon &&
        ((u32Now - s_u32LastInput) >= FRKERNEL_CON_BULK_IDLE_MS))
    {
        s_bBulkBeacon = false;
        CON_vAsync("bulk join beacon paused after 5 min without typing (any line resumes it)\r\n");
    }

    if ((s_eMode == CON_SINGLE) && s_bPending &&
        ((u32Now - s_u32PendSent) >= FRKERNEL_CON_REPLY_TIMEOUT_MS))
    {
        if (s_u8PendTries < FRKERNEL_CON_CMD_TRIES)
        {
            s_u8PendTries++;
            s_u32PendSent = u32Now;
            (void)CON_bSend(FRK_KIND_CMD, s_u32Target, s_u8PendSeq, s_acPend);
        }
        else if (s_acPend[0] == 'x')
        {
            /* The tag closes its session after "x" whether or not its reply
             * got through; at worst it times out on its own in 5 min. */
            s_bPending = false;
            CON_vResetConnection();
            CON_vAsync("no reply to 'x' - connection dropped (the tag times out within 5 min)\r\n");
        }
        else
        {
            s_bPending = false;
            char acMsg[112];
            (void)snprintf(acMsg, sizeof(acMsg),
                           "no reply from %04" PRIX32 " after %u tries - it may have left the\r\n"
                           "kernel ('c %04" PRIX32 "' reconnects)\r\n",
                           s_u32Target, (unsigned)FRKERNEL_CON_CMD_TRIES, s_u32Target);
            CON_vAsync(acMsg);
        }
    }

    if (s_bStreaming && ((u32Now - s_u32StreamLast) >= FRKERNEL_CON_STREAM_IDLE_MS))
        CON_vEndStream("INCOMPLETE (no frames for 8 s, end marker lost?)");
}

/* -------------------------------------------------------------------------- */
/* Init + task                                                                */
/* -------------------------------------------------------------------------- */

void FRKERNEL_vInit(void)
{
    s_rxQueue = osMessageQueueNew(FRKERNEL_CON_RX_QUEUE_LEN, sizeof(ConFrame_t), NULL);
    configASSERT(s_rxQueue != NULL);

    static const osThreadAttr_t attr = {
        .name       = "FrkConsole",
        .stack_size = configMINIMAL_STACK_SIZE * 8U * sizeof(StackType_t),
        .priority   = osPriorityLow,
    };
    s_taskHandle = osThreadNew(CON_vTask, NULL, &attr);
    configASSERT(s_taskHandle != NULL);
}

static void CON_vTask(void *arg)
{
    (void)arg;

    /* Let the boot log drain before taking over the terminal, then silence
     * the 1 Hz heartbeat marker and the verbose debug lines: this terminal is
     * an operator UI. "dbg" brings the debug lines back. */
    osDelay(1500);
    DEBUG_vSetQuiet(true);
    DBGLOG_vMuteVerboseUart(true);

    _DBG_("\r\n==== FrKernel console (kernelV2) - console id %04" PRIX32 " ====\r\n",
          LORARADIO_u32GetUniqueId());
    CON_vHelp();
    CON_vPrompt();

    for (;;)
    {
        (void)osThreadFlagsWait(CON_FLAG_UART | CON_FLAG_LORA, osFlagsWaitAny, CON_TICK_MS);

        CON_vServiceUart();

        ConFrame_t tF;
        while (osMessageQueueGet(s_rxQueue, &tF, NULL, 0U) == osOK)
            CON_vHandleFrame(&tF);

        CON_vTimers();
    }
}

#endif /* FRKERNEL_INTERFACE_LORA_CONSOLE */
