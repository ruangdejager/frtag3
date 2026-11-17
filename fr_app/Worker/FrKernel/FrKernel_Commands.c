/*
 * FrKernel_Commands.c
 *
 * The single-letter FrKernel command table, shared by the tag (FrKernel.c),
 * which executes the commands, and the console (FrKernelConsole.c), which
 * validates what the operator typed and prints the help. Keeping the table in
 * one place is what stops the help text, the console's bulk check and the
 * tag's dispatcher from drifting apart - in v1 there were three hand-written
 * help strings (UART, LoRa, LoRa second half) and they had already diverged
 * from the commands actually implemented.
 *
 * Letter choice:
 *   - lower case for reads and for actions that are cheap to undo;
 *   - UPPER case for destructive or terminal commands (erase, sleep, radio
 *     test). The table lookup is case-sensitive, so a stray lower-case key can
 *     never reach one of these.
 *   - 'c' is deliberately unused: on the console "c <id>" / "c bulk" opens a
 *     connection and never reaches a tag.
 *
 * The letters for the log streams are mnemonic for the fraction they send:
 * a = all, h = half, q = quarter, t = tenth.
 */

#include "FrKernel_Protocol.h"
#include "build_config.h"

#include <stdio.h>

static const FrkCmd_t s_atCmds[] = {
    /* ---- queries (single only: the reply is the whole point) ---- */
    { '?', 0U,                              "",          "list commands" },
    { 'i', FRK_CMDF_SINGLE,                 "",          "status: id, role, fw, battery/solar, interval, log use" },
    { 'e', FRK_CMDF_SINGLE,                 "",          "self-test results (gps/acc/flash + boot detail)" },
    { 'f', FRK_CMDF_SINGLE,                 "",          "ext-flash health (gate, id, fault counters)" },
    { 'k', FRK_CMDF_SINGLE,                 "",          "ext-flash read-verify counters (reading clears them)" },
    { 'n', FRK_CMDF_SINGLE,                 "",          "serial number, radio id and where the serial is stored" },

    /* ---- log streaming ---- */
    { 'a', FRK_CMDF_SINGLE,                 "",          "stream the whole ext-flash log" },
    { 'h', FRK_CMDF_SINGLE,                 "",          "stream the latest half of the log" },
    { 'q', FRK_CMDF_SINGLE,                 "",          "stream the latest quarter of the log" },
    { 't', FRK_CMDF_SINGLE,                 "",          "stream the latest 10 % of the log" },
    { 's', FRK_CMDF_SINGLE,                 "",          "stop a running log stream" },

    /* ---- flash diagnostics ---- */
    { 'r', FRK_CMDF_SINGLE,                 "",          "force an ext-flash re-probe + bring-up" },
    { 'p', FRK_CMDF_SINGLE,                 "",          "run the ext-flash self-test now" },
#ifdef ENABLE_FLASH_STRESS_TEST
    { 'z', FRK_CMDF_SINGLE | FRK_CMDF_ARG,  "[tx|park] <N>", "read a known page N times (optionally TX / park between)" },
#endif

    /* ---- configuration / OTA ---- */
    { 'd', FRK_CMDF_ARG,                    "[min]",     "show, or set, the discovery interval (15/30/60/120/240)" },
    { 'w', FRK_CMDF_ARG,                    "[off]",     "arm (or disarm) firmware acceptance - secondary" },
    { 'o', 0U,                              "",          "distribute the staged firmware - primary" },

    /* ---- session ---- */
    { 'x', 0U,                              "",          "exit the kernel; the tag resumes its schedule" },

    /* ---- destructive / terminal (upper case) ---- */
    { 'E', 0U,                              "",          "ERASE the ext-flash log" },
    { 'W', 0U,                              "",          "WIPE the whole ext-flash (log + staged OTA image)" },
    { 'P', 0U,                              "",          "production sleep - wakes on solar or shake (secondary)" },
    { 'S', 0U,                              "",          "solar sleep - wakes on shake only (secondary)" },
    { 'R', FRK_CMDF_SINGLE,                 "",          "radio range test (secondary beacons, shake to end;"
                                                         " primary listens)" },
    { 'Q', 0U,                              "",          "quit a radio range test (listening primary)" },
    { 'N', FRK_CMDF_SINGLE | FRK_CMDF_ARG,  "<serial> <radio id> [fix]",
                                                         "WRITE the serial number to OTP - permanent;"
                                                         " 'fix' replaces a wrong one" },
};

const FrkCmd_t *FRK_ptFindCmd(char cLetter)
{
    for (size_t i = 0U; i < (sizeof(s_atCmds) / sizeof(s_atCmds[0])); i++)
        if (s_atCmds[i].cLetter == cLetter)
            return &s_atCmds[i];
    return NULL;
}

size_t FRK_szCmdCount(void)
{
    return sizeof(s_atCmds) / sizeof(s_atCmds[0]);
}

const FrkCmd_t *FRK_ptCmdAt(size_t i)
{
    return (i < FRK_szCmdCount()) ? &s_atCmds[i] : NULL;
}

void FRK_vPrintHelp(void (*pfnPut)(const char *pcLine), bool bBulk)
{
    char acLine[96];

    pfnPut(bBulk ? "Bulk commands (no replies - check results with a single connect):\r\n"
                 : "Commands (letter, then Enter):\r\n");

    for (size_t i = 0U; i < FRK_szCmdCount(); i++)
    {
        const FrkCmd_t *pt = &s_atCmds[i];
        if (bBulk && (pt->u8Flags & FRK_CMDF_SINGLE))
            continue;
        (void)snprintf(acLine, sizeof(acLine), "  %c %-14s %s\r\n",
                       pt->cLetter, pt->pcArgs, pt->pcHelp);
        pfnPut(acLine);
    }
}
