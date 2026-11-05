/*
 * Flash_Config.h
 *
 * AT25EU0041A — 4 Mbit (512 KB) ultra-low-energy SPI NOR flash, 1.65-3.6 V.
 * Manufacturer: Renesas (formerly Adesto/Dialog), JEDEC mfr ID 0x1F.
 * Fitted as U1 on SPI2 on BOTH the Tagskaap V3 RevA primary and secondary.
 *
 * EVERY constant in this file is traceable to
 *   DS-AT25EU0041A-212 Rev. E, Jul 10 2025
 * and carries the section or table it came from. Values that could not be
 * read unambiguously out of that document are listed at the bottom of this
 * file rather than guessed.
 *
 * ---- Part history: why this file has been rewritten twice ----
 *
 * frtag2 (V2) carried an AT25EU0041A, and the inherited driver was written
 * against its Winbond-style two-register protection model. Early V3 bring-up
 * assumed an AT25XE041B and commit 55b85be retargeted the driver to that
 * part's completely different register set (SPRL/SWP/EPE, protected-by-
 * default, Ultra-Deep Power-Down). The FINAL V3 RevA schematics fit the
 * AT25EU0041A again, so this file returns to that part — but sourced from the
 * current datasheet revision, not copied back from the frtag2 driver, and with
 * the XE041B lessons kept where they still apply (3-byte JEDEC compare, EPE-
 * free erase verify, bounded waits).
 *
 * The XE041B assumptions that MUST NOT survive on this part, each of which
 * would be wrong in the dangerous direction:
 *   - Bits 6:2 of SR1 are BP4..BP0 here, not read-only status. Bit 5 is BP3,
 *     not EPE, so testing "EPE" after an erase would read a protection bit.
 *   - There is no Ultra-Deep Power-Down (no 79h) and no CS-pulse exit. A
 *     CS pulse does NOT wake this part; only ABh does (§6.3.7).
 *   - Protection is NOT the power-on default. Table 3: "on power-up the
 *     device is unprotected", and deep power-down does not touch the SR. So
 *     the XE041B's "unprotect on the first write after every wake" would here
 *     be a NON-VOLATILE status-register write (tW up to 12 ms, and SR wear)
 *     on nearly every wake, for nothing.
 *   - 35h IS a command here (Read Status Register 2), and WRSR takes one OR
 *     two data bytes (§6.1.6) — exactly 8 or 16 bits, anything else is not
 *     executed.
 *
 * ---- Board notes (V3 RevA, both roles) ----
 *
 *   CS#   (pin 1) — PA15, with R1 = 10k pull-up to VCC. That is exactly what
 *                   Table 1 recommends "to ensure CS ramps together with VCC
 *                   during power-up", and it keeps the part deselected if PA15
 *                   is ever left undriven.
 *   SO    (pin 2) — PA5 (MISO). SI (pin 5) — PA10 (MOSI). SCK (pin 6) — PA8.
 *   WP#   (pin 3) — NOT CONNECTED on the PCB (no-connect marker).
 *   HOLD# (pin 7) — NOT CONNECTED on the PCB (no-connect marker).
 *
 *   Table 1: "The WP pin is internally pulled high and can be left floating
 *   if not used", and the same sentence for HOLD. So with QE = 0 (factory
 *   default) both pins read as DEASSERTED. That makes the floating pins safe
 *   provided firmware guarantees three things, all enforced or relied on by
 *   Flash.c:
 *     1. SRP1:SRP0 stays 0:0 ("Software Protected", Table 5). WP# is only
 *        consulted in the 0:1 mode; at 0:0 a noise pulse on the floating
 *        WP# pad cannot lock the status register. Global Unprotect writes
 *        both SRP bits to 0, and bring-up treats either SRP bit set as a
 *        protection fault to clear (FLASH_STATUS_PROTECTED / _STATUS2_).
 *     2. No quad command is ever issued (6Bh, EBh, 32h, 94h). With QE = 1
 *        pins 3 and 7 become IO2/IO3, and on this board they are wired to
 *        nothing, so a quad transfer would read the internal pull-ups.
 *     3. QE stays 0, i.e. pins 3/7 stay WP#/HOLD# held high internally. QE
 *        does not protect the array, so a QE = 1 seen at bring-up is
 *        REPORTED but does not on its own trigger a non-volatile SR write;
 *        any Global Unprotect clears it as a side effect (SR2 data 00h).
 *   HOLD# floating with only an internal pull-up is the one residual
 *   exposure: a HOLD# pulse low with SCK low pauses the transfer (§4.5),
 *   which from the host looks like a shifted read. The pull-up strength is not
 *   specified; see the open items at the bottom of this file.
 *
 *   VCC is 1.8 V on the secondary board and 3.0 V on the primary. Every
 *   voltage-dependent figure below is taken from the 1.65-3.6 V tables
 *   (Table 20, Table 23), which bind at 1.8 V and are also valid at 3.0 V.
 *
 * Pinout (datasheet Figure 2, 8-pad DFN 2x3, top view; SOIC Figure 1 is the
 * same) — matches the board:
 *   1 CS#   2 SO   3 WP#   4 GND   5 SI   6 SCK   7 HOLD#   8 VCC
 */

#ifndef DEVICE_FLASH_FLASH_CONFIG_H_
#define DEVICE_FLASH_FLASH_CONFIG_H_

/* --------------------------------------------------------------------------
 * Geometry — §3, Table 2 "Block Addresses"
 *
 * Identical to the AT25XE041B, which is why no caller (Log, Fota, flashLog,
 * the self-test sector) needed to change: 512 KB in 128 uniform 4 KB blocks
 * (0..127), also addressable as 16 x 32 KB and 8 x 64 KB blocks, with a
 * 256-byte page for program and for Page Erase (§6.4.4). Three address
 * bytes; the array tops out at 07FFFFh.
 * -------------------------------------------------------------------------- */
#define FLASH_CAPACITY_BYTES        (512U * 1024U)
#define FLASH_PAGE_SIZE_BYTES       256U
#define FLASH_SECTOR_SIZE_BYTES     (4U   * 1024U)   /* 4 KB  block erase, 20h */
#define FLASH_BLOCK_SIZE_BYTES      (64U  * 1024U)   /* 64 KB block erase, D8h */
#define FLASH_NUM_SECTORS           128U
#define FLASH_NUM_BLOCKS            8U
#define FLASH_NUM_PAGES             2048U

/* --------------------------------------------------------------------------
 * Command set — §6, Table 8 "Command Set Table 1"
 *
 * The single-I/O set is listed in full, including the commands this driver
 * does not issue, so that "what can this part do" is answerable from one file.
 * Dual/quad opcodes are deliberately NOT listed: quad needs WP#/HOLD# as
 * IO2/IO3 and those pads are not connected on this board (see above).
 *
 * §6: for Page Program, Page Erase, Block Erase, Chip Erase, WRSR, WREN,
 * WRDI and Deep Power-Down, CS# must rise exactly on a byte boundary or the
 * command is rejected. Every transaction in Flash.c is whole bytes.
 * -------------------------------------------------------------------------- */

/* Read — §6.2. See FLASH_FR_MAX_HZ / FLASH_FC_MAX_HZ below and hal_spi.h for
 * which clock ceiling binds this driver. */
#define FLASH_CMD_READ              0x03U   /* Normal Read Data (fR), §6.2.1     */
#define FLASH_CMD_FAST_READ         0x0BU   /* Fast Read + 8 dummy clocks (fC), §6.2.2 */

/* Program and erase — §6.4 */
#define FLASH_CMD_PAGE_PROGRAM      0x02U   /* Page Program, 1-256 B, §6.4.1    */
#define FLASH_CMD_PAGE_ERASE        0x81U   /* 256-byte Page Erase, §6.4.4      */
#define FLASH_CMD_PAGE_ERASE_ALT    0xDBU   /*   (alias, §6.4.4)                */
#define FLASH_CMD_SECTOR_ERASE      0x20U   /* Block Erase, 4 KB,  §6.4.5       */
#define FLASH_CMD_BLOCK_ERASE_32K   0x52U   /* Block Erase, 32 KB, §6.4.6       */
#define FLASH_CMD_BLOCK_ERASE       0xD8U   /* Block Erase, 64 KB, §6.4.7       */
#define FLASH_CMD_CHIP_ERASE        0x60U   /* Chip Erase, §6.4.8               */
#define FLASH_CMD_CHIP_ERASE_ALT    0xC7U   /*   (alias, §6.4.8)                */
#define FLASH_CMD_PE_SUSPEND        0x75U   /* §6.4.9 — not used                */
#define FLASH_CMD_PE_RESUME         0x7AU   /* §6.4.10 — not used               */

/* Configuration and status — §6.1 */
#define FLASH_CMD_WRITE_ENABLE      0x06U   /* §6.1.1                           */
#define FLASH_CMD_VOLATILE_SR_WREN  0x50U   /* §6.1.2 — not used, see below     */
#define FLASH_CMD_WRITE_DISABLE     0x04U   /* §6.1.3                           */
#define FLASH_CMD_READ_STATUS       0x05U   /* §6.1.4 — Status Register 1       */
#define FLASH_CMD_READ_STATUS2      0x35U   /* §6.1.4 — Status Register 2       */
#define FLASH_CMD_ACTIVE_STATUS_INT 0x25U   /* §6.1.5 — not used                */
#define FLASH_CMD_WRITE_STATUS      0x01U   /* §6.1.6 — SR1, or SR1 then SR2    */

/* ID and power — §6.3 */
#define FLASH_CMD_JEDEC_ID          0x9FU   /* §6.3.5                           */
#define FLASH_CMD_MFR_DEVICE_ID     0x90U   /* §6.3.2 — not used (returns 1F 14) */
#define FLASH_CMD_DEEP_PWR_DOWN     0xB9U   /* §6.3.7                           */
#define FLASH_CMD_RESUME            0xABU   /* §6.3.1 — Release Power-down / ID */

/* Other — §6.4.14. Not used: see the open items at the bottom. */
#define FLASH_CMD_ENABLE_RESET      0x66U
#define FLASH_CMD_RESET             0x99U

/* --------------------------------------------------------------------------
 * JEDEC identification — §6.3, Table 10 "AT25EU0041A ID Definition Table"
 *
 * 9Fh returns THREE bytes: 1Fh 14h 01h. Unlike the AT25XE041B there is no
 * fourth "EDI length" byte; Table 8 note 9 says the ID simply repeats while
 * CS# stays low, so clocking a fourth byte would read back a repeat, not a
 * field. Exactly three are read.
 *
 * All three are checked. Comparing the manufacturer byte alone (what the
 * frtag2 driver did) passes ANY 0x1F part of any density — including the
 * AT25XE041B (1F 44 02) this board was briefly drawn with — so a mis-stuffed
 * reel would be accepted here and fail much later, at an address or a
 * register bit the fitted part does not have.
 *
 * 90h returns 1F 14 and ABh (with three dummy bytes) returns 14; neither is
 * used for identification, but the ABh value is why a release-from-DPD that
 * is accidentally clocked further reads back 14h.
 * -------------------------------------------------------------------------- */
#define FLASH_MANUFACTURER_ID       0x1FU
#define FLASH_DEVICE_ID1            0x14U
#define FLASH_DEVICE_ID2            0x01U
#define FLASH_JEDEC_ID_BYTES        3U

/* --------------------------------------------------------------------------
 * Status Register 1 — §5.1, Table 3 (read 05h)
 *
 *   bit7    SRP0     R/W  NV  Status Register Protect 0 (with SRP1, Table 5)
 *   bit6:2  BP4..BP0 R/W  NV  Block Protect size (Tables 6/7)
 *   bit1    WEL      R        Write Enable Latch
 *   bit0    RDY/BSY  R        1 = program / erase / WRSR in progress
 *
 * Defaults (Table 3): SRP0 = 0, BP = 0 — "on power-up the device is
 * unprotected". The bits are NON-VOLATILE: a value written stays across power
 * cycles and across deep power-down, which is why a protection state, once
 * seen, is a real finding on this part rather than the boot-time norm it was
 * on the AT25XE041B.
 *
 * WEL and RDY/BSY sit in the same positions on both parts, which is the one
 * reason the busy-poll and WREN logic carried across the part changes
 * untouched.
 * -------------------------------------------------------------------------- */
#define FLASH_SR1_RDY_BSY          (1U << 0)
#define FLASH_SR1_WEL              (1U << 1)
#define FLASH_SR1_BP_SHIFT         2U
#define FLASH_SR1_BP_MASK          (0x1FU << FLASH_SR1_BP_SHIFT)
#define FLASH_SR1_SRP0             (1U << 7)

/* --------------------------------------------------------------------------
 * Status Register 2 — §5.2, Table 4 (read 35h — a separate opcode on this
 * part; NOT reached by clocking past SR1, which just repeats SR1, §6.1.4)
 *
 *   bit7    SUS      R        1 = a program/erase is SUSPENDED (75h)
 *   bit6    CMP      R/W  NV  Complement Protect (inverts the BP map)
 *   bit5:3  LB3..LB1 R    OTP Security-register lock bits — ONE-TIME
 *   bit2    —        R        reserved
 *   bit1    QE       NV       Quad Enable; 0 = pins 3/7 are WP#/HOLD#
 *   bit0    SRP1     R/W  NV  Status Register Protect 1
 *
 * CMP matters to "is the array protected" as much as BP does: Table 7, with
 * CMP = 1 and BP2:0 = 000 the protected portion is the WHOLE array. A check
 * that looked at SR1 alone would call that chip clean.
 *
 * SUS is reported at bring-up because this driver never issues 75h: a set SUS
 * means a stray command did, the page/block it was working on is half-done,
 * and RDY/BSY reads 0 regardless (§6.4.9).
 * -------------------------------------------------------------------------- */
#define FLASH_SR2_SRP1             (1U << 0)
#define FLASH_SR2_QE               (1U << 1)
#define FLASH_SR2_LB_SHIFT         3U
#define FLASH_SR2_LB_MASK          (0x07U << FLASH_SR2_LB_SHIFT)
#define FLASH_SR2_CMP              (1U << 6)
#define FLASH_SR2_SUS              (1U << 7)

/* "The status registers are not in the state this driver requires."
 *
 * SR1: any BP bit protects some portion of the array against program and
 * erase while leaving reads working perfectly — the failure mode that
 * presents as "the log just froze". SRP0 alone does not protect the array,
 * but it hands write-control of the status register to the floating WP# pad
 * (Table 5, row 0:1), which is the one thing the board notes above say must
 * never matter.
 *
 * SR2: CMP inverts the BP map (CMP = 1 with BP = 0 protects everything);
 * SRP1 = 1 is Power Supply Lock-Down (with SRP0 = 0) or One-Time-Program
 * (with SRP0 = 1), in both of which a WRSR is refused. QE, SUS and the OTP LB
 * bits are deliberately NOT here: none of them stops a program or erase of
 * the main array, and an NV status-register write is not free (tW, wear). */
#define FLASH_STATUS_PROTECTED     (FLASH_SR1_BP_MASK | FLASH_SR1_SRP0)
#define FLASH_STATUS2_PROTECTED    (FLASH_SR2_CMP | FLASH_SR2_SRP1)

/* --------------------------------------------------------------------------
 * Protection model — §5.3 Table 5, §5.4 Tables 6/7, §6.1.6
 *
 * Global Unprotect = WREN (06h), then WRSR (01h) with TWO data bytes:
 *   SR1 = 00h  -> SRP0 = 0, BP4..BP0 = 0
 *   SR2 = 00h  -> SRP1 = 0, QE = 0, CMP = 0 (LB/SUS/reserved are read-only)
 * §6.1.6: one data byte writes SR1 only; two write SR1 then SR2; any other
 * bit count is NOT executed. Both bytes are needed: an SR1-only write leaves
 * CMP, and CMP = 1 with BP = 0 is "everything protected".
 *
 * This works whenever SRP1 = 0. With SRP1:SRP0 = 0:1 it needs WP# high,
 * which the internal pull-up provides on this board. With SRP1 = 1 it is
 * refused:
 *   1:0 Power Supply Lock-Down — until the next power-down/power-up cycle
 *       (Table 5 note 1). A battery tag effectively never gets one.
 *   1:1 One-Time Program — permanent ("available upon special order").
 *
 * WHAT MUST NEVER BE WRITTEN AS 1 — checked at compile time below:
 *   SR2 bit 0 (SRP1): the two lock modes above.
 *   SR2 bits 5:3 (LB3..LB1): OTP; once 1, the security registers are
 *   read-only for ever (Table 4). Harmless to the main array, but
 *   irreversible, and nothing in this firmware has any reason to set them.
 * Everything else a stray WRSR could set (BP, CMP, SRP0, QE) is recoverable
 * by the next Global Unprotect, which is the point of the WP# analysis above:
 * with WP# floating high, SRP0 is no longer the one-way latch that SPRL was on
 * the AT25XE041B with WP# grounded.
 *
 * The SR bits are written NON-VOLATILE (tW = 6.5 ms typ / 12 ms max, Table
 * 23), so the driver only issues Global Unprotect when bring-up or an erase
 * verify actually finds protection set — never as a reflex on wake.
 *
 * 50h (Volatile SR Write Enable) is listed for completeness and not used:
 * a volatile clear would be undone by the next power cycle while the NV bits
 * stayed set. Note that 50h does NOT set WEL (§6.1.2), so WEL = 0 at idle
 * does not by itself refuse a 50h+01h pair; two specific commands in sequence
 * is a much smaller stray-command target than one, and a volatile write is
 * undone by a power cycle or a 66h/99h reset (§6.4.14).
 * -------------------------------------------------------------------------- */
#define FLASH_WRSR_SR1_UNPROTECT    0x00U   /* SRP0 = 0, BP = 00000             */
#define FLASH_WRSR_SR2_UNPROTECT    0x00U   /* SRP1 = 0, QE = 0, CMP = 0        */
#define FLASH_WRSR_SR2_NEVER_SET    (FLASH_SR2_SRP1 | FLASH_SR2_LB_MASK)

/* Back-compat name for the SR1 unprotect data byte. */
#define FLASH_STATUS_UNPROTECT      FLASH_WRSR_SR1_UNPROTECT

#if (FLASH_WRSR_SR2_UNPROTECT & FLASH_WRSR_SR2_NEVER_SET) != 0
#error "Flash_Config.h: the SR2 unprotect byte sets SRP1 or an OTP LB bit"
#endif

/* --------------------------------------------------------------------------
 * AC timing — §7.6, Table 23 "AC Electrical Characteristics for 1.65 V to
 * 3.6 V"
 *
 * VCC is 1.8 V on the secondary and 3.0 V on the primary. 1.8 V is below the
 * 2.3 V floor of Table 24, so every figure here is from Table 23, which also
 * covers 3.0 V; one set of numbers holds on both roles. Where Table 24
 * (2.3-3.6 V) differs it is noted, but never used.
 * -------------------------------------------------------------------------- */

/* Clock ceilings. fR is the one that binds: this driver reads with 03h. The
 * 12 MHz SPI2 clock (hal_spi.h) is a quarter of it. */
#define FLASH_FC_MAX_HZ             80000000UL  /* all ops except 03h/6Bh/EBh;
                                                 * 108 MHz only at >= 2.3 V   */
#define FLASH_FR_MAX_HZ             50000000UL  /* 03h Normal Read, both ranges */

/* tSHSL, CS# deselect time. The only bus timing software could violate; a
 * back-to-back deselect/select at 12 MHz leaves far more than this. The
 * larger of the two is recorded: 15 ns read-to-read, 30 ns after a write,
 * program or erase before the next status read. */
#define FLASH_TCSH_NS               30U

/* Deep power-down entry/exit. These are the numbers the driver actually
 * waits on; see FLASH_vDeepPowerDown / FLASH_vEnsureAwake in Flash.c.
 * Both are maxima ("guaranteed by design", Table 23 note 2). */
#define FLASH_TDP_US                3U      /* B9h, CS# high -> deep PD        */
#define FLASH_TRES1_US              8U      /* ABh, CS# high -> standby        */
#define FLASH_TRST_US               300U    /* 66h+99h -> next command (unused) */

/* Program, erase and WRSR times, Table 23 typ/max. Present so the timeouts
 * below can be read as "spec x margin" instead of as round numbers somebody
 * liked.
 *
 * Note what this part does that the AT25XE041B did not: §1, "the short erase
 * times are constant and independent of the size of memory block being
 * erased". A 4 KB erase, a 64 KB erase and a CHIP erase are all 8 ms typ /
 * 12 ms max. */
#define FLASH_TPP_TYP_MS            2U      /* page program, 1-256 B           */
#define FLASH_TPP_MAX_MS            3U
#define FLASH_TPE_MAX_MS            12U     /* 256-byte page erase, 81h        */
#define FLASH_TBLKE_4K_MAX_MS       12U
#define FLASH_TBLKE_32K_MAX_MS      12U
#define FLASH_TBLKE_64K_MAX_MS      12U
#define FLASH_TCE_MAX_MS            12U     /* chip erase, 60h/C7h             */
#define FLASH_TW_MAX_MS             12U     /* WRSR, non-volatile SR write     */

/* Power-up — §7.3, Table 19 (characterised only). Not enforced in software:
 * FLASH_vInit runs tens of milliseconds into boot, far past tVSL, and the
 * datasheet gives no separate "power-up to first write" figure (the AT25XE041B
 * had tPUW; this part's Table 19 has only tVSL). Recorded because "the first
 * read at boot is the most exposed one" is a claim this project makes
 * repeatedly (see FLASH_BOOT_PROBE_ROUNDS). §4.3.2: the device also ignores
 * CS# until it has seen a FALLING edge after power-up, which R1 (10k to VCC)
 * guarantees by holding CS# high while VCC ramps. */
#define FLASH_TVSL_US               300U    /* VCC(min) -> CS# low, MIN        */
#define FLASH_VWI_MAX_MV            1400U   /* write-inhibit threshold, max;
                                             * 400 mV below the secondary's
                                             * 1.8 V rail                      */

/* --------------------------------------------------------------------------
 * Driver timeouts
 *
 * All three are far above the datasheet maxima on purpose. They are not
 * "how long the operation takes" — the driver polls RDY/BSY for that — they
 * are "how long before we declare the bus wedged and give up", and the thing
 * that actually consumes them is a stalled SPI transfer, not a slow flash.
 * FLASH_bDeviceBusy() issues two transfers (opcode, then SR1), each bounded by
 * SPI_TIMEOUT (1000 ms in hal_spi.h), so one poll iteration can itself cost
 * ~2 s.
 *
 * On this part every program, erase (of any size) and WRSR completes within
 * 12 ms worst case, so the three are kept distinct only so the call sites do
 * not churn and a future part with genuinely size-dependent erase times slots
 * back in. They were sized against the AT25XE041B (1.1 s 64 KB erase, 7.6 s
 * chip erase) and are deliberately NOT shrunk: §1's "constant erase time"
 * claim is unusual enough that it has not earned a tight timeout until a
 * bench has timed a real chip erase (see the open items below).
 * -------------------------------------------------------------------------- */

/* General pre-command wait and the 4 KB erase budget. 3 s against a 12 ms
 * worst case. Bounded at all because the old unbounded `while busy` loop is
 * what turned a single bad SPI transaction into "logging just stopped". */
#define FLASH_WAIT_READY_TIMEOUT_MS   3000U

/* 64 KB block erase: tBLKE-64kB is 12 ms max on this part. */
#define FLASH_BLOCK_ERASE_TIMEOUT_MS  10000U

/* Chip erase: tCE is 12 ms max on this part (Table 23) — the same as a 4 KB
 * erase. Kept long; see the block comment above. */
#define FLASH_CHIP_ERASE_TIMEOUT_MS   30000U

/* --------------------------------------------------------------------------
 * Probe and recovery policy
 *
 * Unchanged across both part changes. The reasoning is about the BOARD, not
 * about the flash: a single read was not trustworthy on frtag2, V3 RevA has
 * not yet been shown to be any different, and the driver has to survive that
 * without declaring a healthy chip dead.
 * -------------------------------------------------------------------------- */

/* JEDEC-ID read attempts before declaring the part absent/faulty. A single
 * read was not trustworthy on frtag2 (V2) hardware, and V3 RevA has not yet
 * been shown to be any different: a transient can corrupt one read of
 * an otherwise healthy chip, and calling that "flash FAIL" is both wrong and
 * dangerous if anything acts on it. On frtag2 (V2) hardware the transients
 * correlated with radio TX and were blamed on supply-rail sag from a radio/GPS
 * current spike, but that mechanism was never tested and TX was not shown to
 * sag the rail; noise on the very long MISO/MOSI runs over a non-continuous
 * ground plane is the likelier cause (see OTA_LORA_CHUNK_GAP_MS in
 * Fota_Config.h). V3 RevA has not been characterised for it. */
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
 * is most expensive and retrying is cheapest: FLASH_vInit runs early in boot,
 * when the supply and the part have had the least time to settle, and
 * LOG_vInit immediately behind it scans every sector to find the log
 * head. Entering that scan with the gate wrongly shut costs a whole boot's
 * logging; spending ~150 ms up front to avoid it is free by comparison. */
#define FLASH_BOOT_PROBE_ROUNDS     3U
#define FLASH_BOOT_PROBE_SETTLE_MS  50U

/* Bytes sampled per erased region to confirm an erase actually took effect.
 * §6.4.5-6.4.8: an erase whose region is protected by BP/CMP "is not
 * executed" - the device accepts the command, never raises RDY/BSY and
 * returns to ready, so FLASH_bWaitReady() succeeds and the erase silently does
 * nothing. Without a read-back the driver reports success and the old
 * contents survive; FOTA would then stage an image over stale bytes and only
 * discover it at the whole-image XOR check, or not at all.
 *
 * This read-back is the ONLY erase-failure signal on this part. The
 * AT25XE041B's EPE bit (device-reported erase/program failure) does not exist
 * here - SR1 bit 5 is BP3 - so the driver no longer checks it. */
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
 * ENDURANCE IS 10,000 CYCLES on this part (Features; Rev. E of the datasheet
 * corrected an earlier "100k" to "10k"), a tenth of the AT25XE041B's. One
 * 4 KB erase per boot is still ~27 years of daily reboots, but a boot loop
 * would burn through it in hours, which is one more reason the destructive
 * half of the test is compile-time optional; see
 * ENABLE_FLASH_SELFTEST_DESTRUCTIVE in build_config.h.
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
 * Layout: [31:16] magic  [15:8] last JEDEC mfr byte  [7:0] status bits.
 * Bit 5 was the AT25XE041B's EPE flag; that part is gone and the bit is now
 * unused, left reserved rather than reassigned so a word written by an
 * XE041B-era image is never misread as something else. */
#define FLASH_HEALTH_BKP_MAGIC      0xF1A50000UL
#define FLASH_HEALTH_BKP_MAGIC_MASK 0xFFFF0000UL
#define FLASH_HEALTH_BIT_PRESENT     (1UL << 0)  /* gate open              */
#define FLASH_HEALTH_BIT_EVERABSENT  (1UL << 1)  /* gate has been shut     */
#define FLASH_HEALTH_BIT_PROTECTED   (1UL << 2)  /* BP/SRP0/CMP/SRP1 seen  */
#define FLASH_HEALTH_BIT_UNPROT_FAIL (1UL << 3)  /* unprotect did not take */
#define FLASH_HEALTH_BIT_ERASEVFAIL  (1UL << 4)  /* erase did not blank    */
/*      bit 5 reserved (was AT25XE041B EPE)                                */

/* --------------------------------------------------------------------------
 * Open items — not confirmed from DS-AT25EU0041A-212 Rev. E
 *
 * 1. Table 23's min/typ/max columns are read from a text extraction; single-
 *    value entries (tDP 3 us, tRES1 8 us, tRST 300 us, fR 50 MHz, fC 80 MHz)
 *    are taken as maxima and tVSL 300 us as a minimum, which is how every
 *    other Winbond-style datasheet lays out those rows. Immaterial in the
 *    direction that matters: the driver's waits are at or above them and the
 *    bus runs at a quarter of fR.
 *    TODO: confirm against the PDF table.
 *
 * 2. Whether deep power-down (B9h) is refused while RDY/BSY = 1. §6.3.1 says
 *    ABh is ignored while busy; §6.3.7 says nothing for B9h. frtag2 field
 *    history on this same part number is that B9h issued mid-program ABORTED
 *    the program and left the page half-written, so FLASH_vDeepPowerDown
 *    checks busy first regardless.
 *
 * 3. The strength of the internal WP#/HOLD# pull-ups (Table 1 says only
 *    "internally pulled high"). A weak pull-up on the unconnected HOLD# pad
 *    is a plausible coupling path for the TX-correlated read corruption this
 *    project is still chasing: HOLD# low pauses the transfer (§4.5) and the
 *    result would classify as "shifted" in Flash_SelfTest.
 *    TODO: bench - scope pin 7 during a LoRa TX burst.
 *
 * 4. Whether a 66h/99h software reset clears Power Supply Lock-Down
 *    (SRP1:SRP0 = 1:0). Table 5 note 1 names only a power cycle; §6.4.14 says
 *    reset returns the device to "its default power-on state" and drops
 *    VOLATILE settings. If it did, it would be a field recovery path for a
 *    locked SR on a battery unit that never power-cycles. Not used until
 *    confirmed.
 *    TODO: confirm with Renesas or on a bench part.
 *
 * 5. §1's constant erase time (8 ms typ / 12 ms max for page, 4/32/64 KB and
 *    chip erase alike). The timeouts above do not depend on it, but the
 *    self-test and OTA erase durations in field logs will look very different
 *    from XE041B-era bench notes.
 *    TODO: bench - time a 4 KB, 64 KB and chip erase on a real unit.
 * -------------------------------------------------------------------------- */

#endif /* DEVICE_FLASH_FLASH_CONFIG_H_ */
