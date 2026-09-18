/*
 * Flash_Config.h
 *
 * AT25XE041B-MAHN-T — 4 Mbit (512 KB) SPI NOR flash, 8-pad UDFN 2x3.
 * Manufacturer: Renesas (formerly Adesto/Dialog/Atmel), JEDEC mfr ID 0x1F.
 *
 * EVERY constant in this file is traceable to
 *   DS-AT25XE041B-062 Rev. I, Mar 15 2023
 * and carries the section or table it came from. Nothing here is inherited
 * from the AT25EU0041A this board's predecessor carried, and nothing is
 * "what SPI NOR usually does". Three values that could not be read
 * unambiguously out of that document are marked TODO rather than guessed —
 * see the bottom of this file.
 *
 * Why that matters here specifically: the V2 board carried an AT25EU0041A and
 * the driver was written against a Winbond-style two-register protection model
 * that this part does not have. Several of those assumptions happened to still
 * work on this part for the wrong reasons (see the Status Register block
 * below), which is a worse state than failing outright — it would have held
 * until the first time something was genuinely protected.
 *
 * Board notes (V3):
 *   WP#   (pin 3) — tied to GND. Asserted. See the SPRL discussion below:
 *                   this makes SPRL a one-way latch.
 *   HOLD# (pin 7) — tied to VCC. Deasserted, never used.
 *   Neither has an MCU GPIO, so neither can be driven out of a bad state.
 *
 * Pinout (datasheet Figure 3, 8-UDFN, top view) — matches the board:
 *   1 CS#   2 SO   3 WP#   4 GND   5 SI   6 SCK   7 HOLD#   8 VCC
 */

#ifndef DEVICE_FLASH_FLASH_CONFIG_H_
#define DEVICE_FLASH_FLASH_CONFIG_H_

/* --------------------------------------------------------------------------
 * Geometry — §4 Memory Array, Figure 6
 *
 * Unchanged from the previous part, which is the one thing that did carry
 * over: 512 KB in 128 uniform 4 KB blocks (B0..B127), also addressable as
 * 32 KB and 64 KB blocks, 256-byte pages. Three address bytes; A23-A19 are
 * ignored because the array tops out at 07FFFFh (§6).
 * -------------------------------------------------------------------------- */
#define FLASH_CAPACITY_BYTES        (512U * 1024U)
#define FLASH_PAGE_SIZE_BYTES       256U
#define FLASH_SECTOR_SIZE_BYTES     (4U   * 1024U)   /* 4 KB  block erase, 20h */
#define FLASH_BLOCK_SIZE_BYTES      (64U  * 1024U)   /* 64 KB block erase, D8h */
#define FLASH_NUM_SECTORS           128U
#define FLASH_NUM_BLOCKS            8U
#define FLASH_NUM_PAGES             2048U

/* --------------------------------------------------------------------------
 * Command set — §6, Table 2 "Command Listing"
 *
 * The full set is listed, including the commands this driver does not issue,
 * so that "what can this part do" is answerable from one file instead of from
 * a PDF. Unsupported opcodes are ignored by the device and start no operation
 * (§6), which is exactly why the old 35h Read-Status-Register-2 read returned
 * nothing useful rather than failing loudly.
 * -------------------------------------------------------------------------- */

/* Read — §7. Note the three different clock ceilings; see FLASH_SPI_CLOCK_HZ
 * in hal_spi.h for which one actually binds this driver. */
#define FLASH_CMD_READ              0x03U   /* Read Array, low frequency (fRDLF) */
#define FLASH_CMD_FAST_READ         0x0BU   /* Read Array + 1 dummy byte (fCLK)  */
#define FLASH_CMD_DUAL_READ         0x3BU   /* Dual-Output Read Array (fRDDO)    */

/* Program and erase — §8 */
#define FLASH_CMD_PAGE_PROGRAM      0x02U   /* Byte/Page Program, 1-256 B, §8.1 */
#define FLASH_CMD_PAGE_ERASE        0x81U   /* 256-byte page erase, §8.4        */
#define FLASH_CMD_SECTOR_ERASE      0x20U   /* Block Erase, 4 KB,  §8.5         */
#define FLASH_CMD_BLOCK_ERASE_32K   0x52U   /* Block Erase, 32 KB, §8.5         */
#define FLASH_CMD_BLOCK_ERASE       0xD8U   /* Block Erase, 64 KB, §8.5         */
#define FLASH_CMD_CHIP_ERASE        0x60U   /* Chip Erase, §8.6 (C7h is an alias) */
#define FLASH_CMD_CHIP_ERASE_ALT    0xC7U

/* Protection — §9 */
#define FLASH_CMD_WRITE_ENABLE      0x06U   /* §9.1 */
#define FLASH_CMD_WRITE_DISABLE     0x04U   /* §9.2 */
#define FLASH_CMD_PROTECT_SECTOR    0x36U   /* §9.3 — not used, see TODO 2     */
#define FLASH_CMD_UNPROTECT_SECTOR  0x39U   /* §9.4 — not used, see TODO 2     */
#define FLASH_CMD_READ_SECTOR_PROT  0x3CU   /* §9.6 — returns FFh or 00h       */

/* Status register — §11 */
#define FLASH_CMD_READ_STATUS       0x05U   /* §11.1 — byte 1 then byte 2, repeating */
#define FLASH_CMD_ACTIVE_STATUS_INT 0x25U   /* §11.2 — not used                */
#define FLASH_CMD_WRITE_STATUS      0x01U   /* §11.3 — SPRL + global protect/unprotect */
#define FLASH_CMD_WRITE_STATUS2     0x31U   /* §11.4 — RSTE only               */

/* Miscellaneous — §12 */
#define FLASH_CMD_JEDEC_ID          0x9FU   /* §12.1 */
#define FLASH_CMD_DEEP_PWR_DOWN     0xB9U   /* §12.2 */
#define FLASH_CMD_RESUME            0xABU   /* §12.3 — Resume from Deep Power-Down */
#define FLASH_CMD_ULTRA_DEEP_PWR_DN 0x79U   /* §12.4 — exit is a CS pulse, no opcode */
#define FLASH_CMD_RESET             0xF0U   /* §12.7 — needs RSTE set via 31h first */
#define FLASH_RESET_CONFIRM_BYTE    0xD0U   /* §12.7 — mandatory second byte    */

/* --------------------------------------------------------------------------
 * JEDEC identification — §12.1, Table 14
 *
 * 9Fh returns FOUR bytes: mfr, device ID part 1, device ID part 2, then the
 * Extended Device Information String Length, which is 00h (no EDI follows).
 *
 *   1Fh  Manufacturer — JEDEC code for Renesas
 *   44h  Family 010 (AT25F/AT25XExxx) + density 00100 (4 Mbit)
 *   02h  Sub code 000 (standard series) + product version 00010
 *   00h  EDI string length — none
 *
 * All three meaningful bytes are checked. The previous driver compared the
 * manufacturer byte alone, which passed ANY 0x1F part of any density — so a
 * mis-stuffed reel of a different Renesas flash would have been accepted and
 * then failed much later, at an address the wrong part does not have.
 * -------------------------------------------------------------------------- */
#define FLASH_MANUFACTURER_ID       0x1FU
#define FLASH_DEVICE_ID1            0x44U
#define FLASH_DEVICE_ID2            0x02U
#define FLASH_EDI_LENGTH            0x00U
#define FLASH_JEDEC_ID_BYTES        4U      /* read all four, report all four  */

/* --------------------------------------------------------------------------
 * Status Register Byte 1 — §11.1, Table 10
 *
 *   bit7  SPRL     R/W  Sector Protection Registers Locked
 *   bit6  SPM      R    1 = Sequential Program Mode entered
 *   bit5  EPE      R    1 = the last erase or program FAILED
 *   bit4  WPP      R    WP# pin status; 0 = WP# ASSERTED (low)
 *   bit3:2 SWP     R    00 none / 01 some / 11 ALL sectors protected
 *   bit1  WEL      R    Write Enable Latch
 *   bit0  RDY/BSY  R    1 = busy with an internal operation
 *
 * Three things here are different from the part this driver was written for,
 * and all three matter:
 *
 * 1. There are no BP0..BP4 bits and no SRP0. Bits 6:2 are read-only STATUS on
 *    this part. The old FLASH_STATUS_PROTECTED mask (BP|SRP0) therefore read
 *    as clean on a healthy chip purely by coincidence — every bit it tested
 *    happens to sit at 0 after a successful unprotect with WP# grounded. It
 *    was testing the right thing for the wrong reason.
 *
 * 2. EPE is new and free. The device itself reports a failed erase or program
 *    (§11.1.3). The driver previously had to infer that from a read-back
 *    sample, which cannot distinguish "the erase did nothing" from "the erase
 *    worked and the read-back was corrupted".
 *
 * 3. WPP lets the firmware REPORT the WP# strap instead of assuming it. On
 *    this board WP# is grounded, so a healthy part must read WPP = 0. A WPP
 *    of 1 means the strap is not what this file says it is.
 *
 * WEL and RDY/BSY did land in the same bit positions as the old part, so the
 * busy-poll and write-enable logic was correct — again, by coincidence.
 * -------------------------------------------------------------------------- */
#define FLASH_SR1_RDY_BSY          (1U << 0)
#define FLASH_SR1_WEL              (1U << 1)
#define FLASH_SR1_SWP_MASK         (0x03U << 2)
#define FLASH_SR1_SWP_NONE         (0x00U << 2)   /* no sector protected      */
#define FLASH_SR1_SWP_SOME         (0x01U << 2)
#define FLASH_SR1_SWP_ALL          (0x03U << 2)   /* power-on default         */
#define FLASH_SR1_WPP              (1U << 4)      /* 0 = WP# asserted (low)   */
#define FLASH_SR1_EPE              (1U << 5)
#define FLASH_SR1_SPM              (1U << 6)
#define FLASH_SR1_SPRL             (1U << 7)

/* "The array is not writable as it stands." SWP non-zero means at least one
 * sector is protected; SPRL means the protection registers are locked and a
 * Global Unprotect will be refused. Either one blocks a program or erase while
 * leaving reads working perfectly, which is the failure mode that presents as
 * "the log just froze". */
#define FLASH_STATUS_PROTECTED     (FLASH_SR1_SWP_MASK | FLASH_SR1_SPRL)

/* --------------------------------------------------------------------------
 * Status Register Byte 2 — §11.1.8, Table 11
 *
 * Reached by CONTINUING to clock after byte 1 of a single 05h command — the
 * device emits byte 1, byte 2, byte 1, ... for as long as CS stays asserted
 * (§11.1). There is no separate opcode. 35h, which the previous driver used,
 * is not a command on this part at all and is silently ignored (§6).
 *
 *   bit4  RSTE     R/W  Reset command enabled (default 0 = disabled)
 *   bit0  RDY/BSY  R    mirror of byte 1 bit 0
 *   all others reserved, read 0
 * -------------------------------------------------------------------------- */
#define FLASH_SR2_RDY_BSY          (1U << 0)
#define FLASH_SR2_RSTE             (1U << 4)

/* --------------------------------------------------------------------------
 * Protection model — §9.5, Table 4, and §11.3
 *
 * THE DEFAULT IS PROTECTED. Table 3: a Sector Protection Register's default
 * value is 1, "Sector is protected and cannot be programmed or erased". That
 * applies to a virgin part, to a part after a power cycle, AND — see
 * FLASH_TXUDPD_US below — to a part that has just exited Ultra-Deep
 * Power-Down. A Global Unprotect is not a recovery step on this part, it is
 * part of normal bring-up.
 *
 * Global Unprotect = WREN, then WRSR (01h) with ONE data byte of 00h.
 * Global Protect   = the same with 7Fh.
 * Table 4, row WP#=0 / current SPRL=0 / data 0x0000xx:
 *     "Global Unprotect - all Sector Protection Registers reset to 0",
 *     new SPRL = 0.
 * So the unprotect works with WP# grounded, provided SPRL is currently 0.
 *
 * SPRL IS A ONE-WAY LATCH ON THIS BOARD. §11.3: while WP# is asserted, SPRL
 * can only be set to 1; an attempt to clear it is ignored. WP# is grounded
 * here and has no MCU GPIO, so a single WRSR data byte with bit 7 set
 * permanently locks the protection registers of that unit — reads keep
 * working, every write and erase is refused forever, and the part cannot be
 * recovered without physically lifting the WP# pin. This is why the driver
 * never writes a WRSR byte with bit 7 set, and why FLASH_vWriteDisable() on
 * every error path (keeping WEL=0 at idle, so a stray WRSR is refused) is
 * load-bearing rather than tidiness.
 * -------------------------------------------------------------------------- */
#define FLASH_WRSR_GLOBAL_UNPROTECT 0x00U   /* SPRL=0, bits 5:2 = 0000        */
#define FLASH_WRSR_GLOBAL_PROTECT   0x7FU   /* SPRL=0, bits 5:2 = 1111        */
#define FLASH_WRSR_SPRL_MASK        0x80U   /* must never be set — see above  */

/* Back-compat name for the unprotect data byte. */
#define FLASH_STATUS_UNPROTECT      FLASH_WRSR_GLOBAL_UNPROTECT

/* --------------------------------------------------------------------------
 * AC timing — §13.3, Table 18 (1.65-3.6 V, -40 to +85 C)
 *
 * This board runs at 1.8 V; a sibling board runs the same firmware at 3.0 V.
 * Every constant below is therefore taken from the 1.65-3.6 V column, which
 * is the worse of the two, so one set of numbers is correct on both boards.
 * Where Table 19 (-40 to +125 C) is worse still, the timeout is sized off
 * Table 19 instead and says so.
 * -------------------------------------------------------------------------- */

/* Clock ceilings. fCLK is the headline number and does NOT apply to the
 * opcode this driver uses. See hal_spi.h. */
#define FLASH_FCLK_MAX_HZ           85000000UL  /* all ops incl. 0Bh          */
#define FLASH_FRDLF_MAX_HZ          25000000UL  /* 03h Read Array @1.65-3.6 V */
#define FLASH_FRDDO_MAX_HZ          40000000UL  /* 3Bh Dual-Output Read       */

/* Bus timing worth naming. tCSH is the only one the software can violate:
 * a back-to-back deselect/select at 12 MHz leaves far more than 35 ns, but
 * it is the constraint that exists, so it is recorded. */
#define FLASH_TCSH_NS               35U     /* min CS# high between commands  */

/* Power-down entry/exit. These are the numbers the driver actually waits on;
 * see FLASH_vPark / FLASH_vEnsureAwake in Flash.c. */
#define FLASH_TEDPD_US              3U      /* CS# high -> Deep Power-Down    */
#define FLASH_TRDPD_US              8U      /* ABh -> standby (DPD exit)      */
#define FLASH_TEUDPD_US             3U      /* CS# high -> Ultra-Deep PD      */
#define FLASH_TCSLU_US              1U      /* spec is tCSLU = 20 ns MIN; the
                                             * busy-wait cannot resolve below
                                             * 1 us and erring long is the safe
                                             * direction for a settle          */
#define FLASH_TXUDPD_US             70U     /* CS# pulse -> standby (UDPD exit) */
#define FLASH_TSWRST_US             60U     /* F0h+D0h -> idle (unused)       */

/* Program and erase times, Table 18 typ/max. Present so the timeouts below
 * can be read as "spec x margin" instead of as round numbers somebody liked. */
#define FLASH_TPP_TYP_MS            2U      /* 1.85 ms typ, 2.75 ms max       */
#define FLASH_TPP_MAX_MS            3U      /* 3.75 ms max on the 125 C part  */
#define FLASH_TBP_US                8U      /* single byte program            */
#define FLASH_TPE_MAX_MS            20U     /* 256-byte page erase, 81h       */
#define FLASH_TBLKE_4K_MAX_MS       60U     /* 75 ms on the 125 C part        */
#define FLASH_TBLKE_32K_MAX_MS      500U    /* 550 ms on the 125 C part       */
#define FLASH_TBLKE_64K_MAX_MS      900U    /* 1100 ms on the 125 C part      */
#define FLASH_TCHPE_MAX_MS          7200U   /* 7600 ms on the 125 C part      */

/* Power-up — §14.1, Table 20. Not enforced in software: FLASH_vInit runs
 * milliseconds into boot, well past tVCSL, and the first operation is a read
 * rather than a program, so tPUW is satisfied long before anything writes.
 * Recorded because "the first read at boot is the most exposed one" is a
 * claim this project makes repeatedly (see FLASH_BOOT_PROBE_ROUNDS) and these
 * are the numbers behind it. */
#define FLASH_TVCSL_US              70U     /* VCC valid -> CS# low for a read */
#define FLASH_TPUW_MS               3U      /* VCC valid -> program/erase OK   */

/* --------------------------------------------------------------------------
 * Driver timeouts
 *
 * All three are far above the datasheet maxima on purpose. They are not
 * "how long the operation takes" — the driver polls RDY/BSY for that — they
 * are "how long before we declare the bus wedged and give up", and the thing
 * that actually consumes them is a stalled SPI transfer, not a slow flash.
 * FLASH_bDeviceBusy() issues two transfers, each bounded by SPI_TIMEOUT
 * (1000 ms in hal_spi.h), so one poll iteration can itself cost ~2 s.
 * -------------------------------------------------------------------------- */

/* Covers a page program (tPP 3.75 ms worst case) and a 4 KB erase (tBLKE 4K,
 * 75 ms worst case) with ~40x margin, and is the general pre-command wait.
 * Bounded at all because the old unbounded `while busy` loop is what turned a
 * single bad SPI transaction into "logging just stopped". */
#define FLASH_WAIT_READY_TIMEOUT_MS   3000U

/* 64 KB block erase: tBLKE 64K is 1100 ms worst case, so ~9x margin. */
#define FLASH_BLOCK_ERASE_TIMEOUT_MS  10000U

/* Chip erase: tCHPE is 7.6 s worst case, so ~4x margin. This one is genuinely
 * close to the spec number rather than orders above it — a chip erase really
 * does take seconds. */
#define FLASH_CHIP_ERASE_TIMEOUT_MS   30000U

/* --------------------------------------------------------------------------
 * Probe and recovery policy
 *
 * Unchanged from the previous part. The reasoning is about this BOARD, not
 * about the flash: a single read is not trustworthy here, and the driver has
 * to survive that without declaring a healthy chip dead.
 * -------------------------------------------------------------------------- */

/* JEDEC-ID read attempts before declaring the part absent/faulty. A single
 * read is not trustworthy on this board — a transient (supply-rail sag from a
 * concurrent radio/GPS current spike) can corrupt one read of an otherwise
 * healthy chip, and calling that "flash FAIL" is both wrong and dangerous if
 * anything acts on it. */
#define FLASH_JEDEC_READ_ATTEMPTS   5U

/* Cooldown between automatic re-probes once the device has been declared
 * absent. The driver's bDevicePresent gate used to be latched once at boot and
 * never revisited, so ONE bad JEDEC read disabled every write and erase for the
 * whole power cycle - the log froze and FOTA_bEraseScratch failed instantly,
 * reporting FWDONE=ERR with 0 B delivered on every campaign until the unit was
 * power-cycled by hand. Field units cannot be reprogrammed by wire, so a
 * boot-lifetime latch on a transient is not survivable: the gate has to be able
 * to re-open by itself.
 *
 * Rate-limited because the gate exists to keep a genuinely absent chip cheap -
 * LOG_vInit alone probes every sector, and an unthrottled retried probe
 * (FLASH_JEDEC_READ_ATTEMPTS x the 2 ms settle delay) would put ~10 ms into
 * every one of those calls. One probe per interval bounds that to nothing
 * measurable while still recovering well inside a single FOTA session. */
#define FLASH_REPROBE_INTERVAL_MS   5000U

/* Extra probe rounds at boot only, each a full FLASH_JEDEC_READ_ATTEMPTS burst
 * separated by a longer settle. Boot is the one moment where getting this wrong
 * is most expensive and retrying is cheapest: FLASH_vInit runs a few
 * milliseconds after power-up, in the worst part of the rail for a clean SPI
 * read, and LOG_vInit immediately behind it scans every sector to find the log
 * head. Entering that scan with the gate wrongly shut costs a whole boot's
 * logging; spending ~150 ms up front to avoid it is free by comparison. */
#define FLASH_BOOT_PROBE_ROUNDS     3U
#define FLASH_BOOT_PROBE_SETTLE_MS  50U

/* Bytes sampled per erased region to confirm an erase actually took effect.
 * A protected device ACCEPTS the erase command, never sets RDY/BSY and
 * returns to ready - so FLASH_bWaitReady() succeeds and the erase silently
 * does nothing. Without a read-back the driver reports success and the old
 * contents survive; FOTA would then stage an image over stale bytes and only
 * discover it at the whole-image XOR check, or not at all.
 *
 * On this part EPE (SR1 bit 5) also reports a failed erase directly, and the
 * driver now checks both: EPE catches the case where the erase was attempted
 * and failed, the read-back catches the case where it was never attempted. */
#define FLASH_ERASE_VERIFY_BYTES    32U

/* Erased NOR reads back all-ones. */
#define FLASH_ERASED_BYTE           0xFFU

/* --------------------------------------------------------------------------
 * Self-test scratch sector
 *
 * The device is otherwise fully allocated (see Fota_Config.h): OTA scratchpad
 * 0x00000-0x3AFFF, OTA metadata 0x3B000, text log 0x3C000-0x7FFFF. The boot
 * self-test needs somewhere it can erase and program destructively, and
 * neither the staged firmware image nor the field-collected log can be it.
 *
 * So the log gives up its top 4 KB: LOG_FLASH_END_ADDR drops from 0x80000 to
 * 0x7F000 (272 KB -> 268 KB) and sector 127 belongs to the self-test. The
 * bootloader contract is untouched — it only ever reads the scratchpad and
 * the metadata sector.
 *
 * One 4 KB erase per boot against a 100,000-cycle endurance rating (Features,
 * -40 to +85 C) is ~270 years of daily reboots, but the destructive half of
 * the test is still compile-time optional; see ENABLE_FLASH_SELFTEST_DESTRUCTIVE
 * in build_config.h.
 * -------------------------------------------------------------------------- */
#define FLASH_SELFTEST_ADDR         0x07F000UL
#define FLASH_SELFTEST_LEN          FLASH_SECTOR_SIZE_BYTES
#define FLASH_SELFTEST_PATTERN_LEN  256U        /* one full page              */
#define FLASH_SELFTEST_PRNG_SEED    0xA5C33C5AUL /* fixed: the pattern must be
                                                  * reproducible across boots
                                                  * and across units           */

/* --------------------------------------------------------------------------
 * Read-verify (CRC) policy — see FLASH_vRead in Flash.c
 *
 * A CRC over a buffer that was read once proves nothing: it is a function of
 * whatever bytes came back, so it matches corrupt data just as happily as
 * good data. Detection needs a second read to compare against. That is what
 * these bound.
 * -------------------------------------------------------------------------- */
#define FLASH_READ_CRC_MAX_RETRIES  3U      /* extra read attempts on mismatch */
#define FLASH_READ_CRC_MAX_LEN      256U    /* verify buffer; longer reads are
                                             * verified over their first
                                             * FLASH_READ_CRC_MAX_LEN bytes    */

/* ---- Durable flash-health word (TAMP->BKP4R) ----
 * Backup registers survive reset and brownout and need no external flash,
 * which is precisely why the health state lives here: when the log device is
 * the casualty, the log cannot be where the fault is recorded. BKP0R..BKP2R
 * belong to the bootloader handshake (see Fota_Config.h); BKP4R is free.
 *
 * Layout: [31:16] magic  [15:8] last JEDEC mfr byte  [7:0] status bits. */
#define FLASH_HEALTH_BKP_MAGIC      0xF1A50000UL
#define FLASH_HEALTH_BKP_MAGIC_MASK 0xFFFF0000UL
#define FLASH_HEALTH_BIT_PRESENT     (1UL << 0)  /* gate open              */
#define FLASH_HEALTH_BIT_EVERABSENT  (1UL << 1)  /* gate has been shut     */
#define FLASH_HEALTH_BIT_PROTECTED   (1UL << 2)  /* SWP/SPRL seen set      */
#define FLASH_HEALTH_BIT_UNPROT_FAIL (1UL << 3)  /* unprotect did not take */
#define FLASH_HEALTH_BIT_ERASEVFAIL  (1UL << 4)  /* erase did not blank    */
#define FLASH_HEALTH_BIT_EPE         (1UL << 5)  /* device reported EPE    */

/* --------------------------------------------------------------------------
 * Values NOT confirmed from DS-AT25XE041B-062 Rev. I
 *
 * 1. tWRSR ("Write Status Register Time", §13.3 Table 18) reads as 200 with
 *    the units column aligning to ns, which is implausibly short for an
 *    operation the same document describes as self-timed. Immaterial here —
 *    the driver waits on RDY/BSY, never on a fixed delay — so no constant is
 *    published for it rather than publishing one that may be a
 *    column-alignment artefact.
 *    TODO: confirm from datasheet §13.3 Table 18.
 *
 * 2. The span of a protection "sector" for 36h / 39h / 3Ch (§9.3-9.6) is not
 *    stated in a form that could be read back reliably. Per-sector
 *    protect/unprotect is therefore declared above but deliberately never
 *    issued; only Global Unprotect is used, whose behaviour is fully
 *    specified in Table 4.
 *    TODO: confirm from datasheet §9.3.
 *
 * 3. Whether RSTE survives an Ultra-Deep Power-Down exit. §12.7 says the
 *    Reset command does not change RSTE; §12.5.3 says all internal registers
 *    return to the power-on default on UDPD exit. The two are not reconciled
 *    for RSTE specifically. Immaterial because the Reset command is never
 *    issued, but noted rather than assumed.
 *    TODO: confirm from datasheet §12.5.3.
 * -------------------------------------------------------------------------- */

#endif /* DEVICE_FLASH_FLASH_CONFIG_H_ */
