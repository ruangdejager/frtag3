/*
 * Fota_Config.h
 *
 * PARTLY SHARED with frtag2_bootloader/fr_bootloader/inc/Fota_Config.h.
 * The flash layout and the application/bootloader handoff contract live in
 * both copies and MUST stay identical in both: the internal flash split,
 * the scratchpad and metadata addresses, the metadata record offsets, magic
 * and markers, OTA_BOOT_MAGIC and the backup-register usage, OTA_FW_INFO_ADDR
 * and FwVersion_t. Any change to one of those has to be made in both files
 * together, because the bootloader is frozen in the field and reads them
 * from its own copy. The two files are, however, no longer byte-identical:
 * this copy carries application-only blocks (the fw_info identity offset, the
 * install-failure counter, the pending-Prep expiry, the pre-send erase
 * threshold, multi-primary coexistence and the verify quiet window), rewrites
 * several comments, and describes the AT25XE041B flash part and the self-test
 * sector, none of which the bootloader reads. Do not try to re-sync the files
 * byte for byte; keep the contract constants equal.
 *
 * The OTA file is the raw application binary — the very same bytes that
 * get programmed into internal flash at OTA_APP_BASE_ADDR — hosted
 * publicly on GitHub Pages under
 *   https://ruangdejager.github.io/farmranger-firmware/frtag3-firmware/
 * as frtag3_<M>_<m>_<p>.bin, named by the "file" field of a sibling
 * version.json (there is no latest.bin; see
 * build_scripts/create_release_hex_files.ps1, which also publishes the same
 * pair to an in-house mirror for bench units). The fr9 fetches version.json,
 * decides based on the tag's own current version, then downloads the .bin
 * that the manifest names to the modem filesystem and streams it
 * byte-for-byte to the primary over UART. On the primary each block is
 * written straight into the external scratchpad at the same offset (file
 * offset N -> scratch offset N); no encoding, no address remap. Validation
 * is an 8-bit XOR over the full image; a metadata record plus a
 * backup-register boot flag hand the image to the bootloader, which
 * programs internal flash and jumps to the application.
 *
 * ---- Internal flash map (STM32WLE5CC, 256 KB, 2 KB pages) ----
 *
 *   0x08000000 - 0x08004FFF   bootloader (20 KB, frtag2_bootloader project)
 *   0x08005000 - 0x0803FFFF   application (236 KB)
 *
 * ---- External NOR map (AT25XE041B, 512 KB, 4 KB sectors) ----
 *
 *   0x00000 - 0x3AFFF   OTA image scratchpad (sectors 0-58, 236 KB;
 *                       scratch offset N maps to OTA_APP_BASE_ADDR + N)
 *   0x3B000 - 0x3BFFF   OTA metadata (sector 59)
 *   0x3C000 - 0x7EFFF   text log (sectors 60-126; see Services/Log/Log.h)
 *   0x7F000 - 0x7FFFF   flash self-test scratch (sector 127; see
 *                       Device/Flash/Flash_Config.h). Not part of the
 *                       app/bootloader contract - the bootloader never
 *                       reads above the metadata sector.
 *
 * ---- Metadata record (at OTA_META_ADDR) ----
 *
 *   +0x00  u32  magic "FROT" (little-endian in flash)
 *   +0x04  u32  fwVersion (MMmmpp)
 *   +0x08  u32  imageStopAddr (last written STM32 address, incl. base)
 *   +0x0C  u32  imageSizeBytes (= stopAddr - base + 1)
 *   +0x10  u8   xor8 (XOR over scratch bytes 0..size-1)
 *   +0x11  u8   VALID marker 0xA5 — written LAST by the application
 *               after its own verify pass
 *   +0x12  u8   DISTRIBUTED marker 0xA5 — written by the primary once
 *               the LoRa distribution session for this image has run
 *
 * ---- Bootloader contract ----
 *
 *   The application arms the handoff with:
 *     TAMP->BKP0R = OTA_BOOT_MAGIC, TAMP->BKP1R = fwVersion,
 *     then NVIC_SystemReset().
 *   The bootloader compares meta.fwVersion against the CURRENTLY INSTALLED
 *   app's own version, read directly out of internal flash at
 *   OTA_FW_INFO_ADDR (a FwVersion_t compiled into the app image at a fixed
 *   linker address — see below). If strictly newer, it recomputes the
 *   XOR-8 over scratch 0..size-1 and — on a match — programs
 *   0x08005000..stopAddr from the scratchpad (which, being a raw byte
 *   copy, carries the new image's own .fw_info along with it — no special
 *   handling needed), then clears BKP0R and jumps to the app. On any
 *   mismatch or same-or-older version it clears BKP0R and jumps to the
 *   existing application unchanged.
 *
 *   This deliberately does NOT use a backup register for the installed
 *   version (an earlier revision did, via TAMP->BKP3R written by the app
 *   on every boot): writing TAMP registers requires the RTC backup-domain
 *   clock to be enabled first, and the app's boot sequence calls
 *   FOTA_vInit() before HAL_RTC_vInit() — the write silently no-op'd,
 *   always reading back 0. Reading a fixed flash address has no such
 *   ordering dependency.
 */

#ifndef WORKER_FOTA_FOTA_CONFIG_H_
#define WORKER_FOTA_FOTA_CONFIG_H_

#include <stdint.h>

/* ---- Internal flash split ---- */
#define OTA_BOOTLOADER_SIZE      0x5000UL                   /* 20 KB          */
#define OTA_APP_BASE_ADDR        0x08005000UL               /* == fr9's base  */
#define OTA_APP_MAX_SIZE         0x3B000UL                  /* 236 KB         */

/* ---- External NOR partitions ---- */
#define OTA_SCRATCH_START_ADDR   0x000000UL
#define OTA_SCRATCH_SIZE         OTA_APP_MAX_SIZE           /* byte-mapped    */
#define OTA_META_ADDR            (OTA_SCRATCH_START_ADDR + OTA_SCRATCH_SIZE)
#define OTA_META_SECTOR_ADDR     OTA_META_ADDR              /* sector-aligned */

/* ---- Metadata field offsets (relative to OTA_META_ADDR) ---- */
#define OTA_META_OFF_MAGIC       0x00U
#define OTA_META_OFF_VERSION     0x04U
#define OTA_META_OFF_STOP_ADDR   0x08U
#define OTA_META_OFF_SIZE        0x0CU
#define OTA_META_OFF_XOR8        0x10U
#define OTA_META_OFF_VALID       0x11U
#define OTA_META_OFF_DISTRIBUTED 0x12U
#define OTA_META_RECORD_LEN      0x13U

#define OTA_META_MAGIC           0x46524F54UL               /* "FROT"         */
#define OTA_META_MARKER          0xA5U

/* ---- Application/bootloader boot flag (TAMP backup registers) ----
 *   BKP0R = OTA_BOOT_MAGIC when app wants bootloader attention
 *   BKP1R = staged image version (informational, set alongside magic)
 *   BKP2R = bootloader's own version (written by bootloader on every boot) */
#define OTA_BOOT_MAGIC           0x4F544152UL               /* "OTAR"         */

/* ---- Embedded firmware-version header ----
 * The app's own version is compiled directly into its image at this fixed
 * address (fr_app's linker script places a KEEP'd ".fw_info" section
 * here — see STM32WLE5CCUX_FLASH.ld) so the bootloader can read "what's
 * currently installed" straight out of internal flash with a plain
 * pointer dereference. Offset 0x200 sits comfortably past the vector
 * table (83 entries * 4 B = 332 B on this build) with margin for growth.
 * An erased app region reads back {0xFFFF,0xFFFF,0xFFFF} — the bootloader
 * treats that as "no app installed" (version 0), not as a huge version. */
#define OTA_FW_INFO_ADDR         (OTA_APP_BASE_ADDR + 0x200UL)

/* Same record's offset WITHIN an image, i.e. into the OTA scratch copy. The
 * scratch is byte-mapped onto the app region (scratch 0 == OTA_APP_BASE_ADDR),
 * so a staged image carries its own version at this offset and can be asked
 * what it actually is - as opposed to what a manifest, an fr9 or a LoRa Prep
 * packet CLAIMS it is. Those are the only sources of version identity the tag
 * had until now, and on 2026-09-01 they were all wrong together: fr9 adopted a
 * byte-identical 2.1.8 file as v20109 (its staleness check is size+xor8, and
 * 2.1.7/2.1.8/2.1.9 all built to 109068 B with xor8 0xC5), served it under the
 * new version number, and every layer downstream agreed because every layer
 * was checking integrity, not identity. The image itself was the one witness
 * that could not be fooled. */
#define OTA_FW_INFO_OFFSET       0x200UL

/* An image whose fw_info does not match the version it is being staged,
 * distributed or installed under is REJECTED. Fail-closed is safe here: every
 * published artifact from 2.0.1 onward carries a correct fw_info record (the
 * linker KEEPs the .fw_info section), so a mismatch means the bytes are not
 * the firmware they are labelled as - which is exactly the condition that
 * cost a fleet-wide FOTA window. */

/* Consecutive boots where the bootloader was handed a newer staged image and
 * did NOT install it, before the app erases that image and re-acquires.
 *
 * This exists because the bootloader is frozen in the field and re-evaluates
 * the metadata record on EVERY boot: while meta.version > installed version it
 * will keep trying, so an image it cannot or will not install loops forever
 * with no backoff, burning battery and every FOTA window. The app is the only
 * layer left that can break that loop, and it can only do so by invalidating
 * the record the bootloader keeps acting on. 3 gives a genuine transient (one
 * bad SPI read during the bootloader's verify pass) room to recover on its
 * own before the image is thrown away. */
#define OTA_INSTALL_FAIL_MAX     3U

/* Backup register holding that counter across the resets it is counting.
 * BKP0R/1R = app->bootloader handoff, BKP2R = bootloader version,
 * BKP4R = flash health word. BKP5R is free. Magic in the top bits so an
 * uninitialised or unrelated value is not read as a count. */
#define OTA_INSTALL_FAIL_BKP_MAGIC      0xF0A70000UL
#define OTA_INSTALL_FAIL_BKP_MASK       0xFFFF0000UL
#define OTA_INSTALL_FAIL_COUNT_MASK     0x000000FFUL

typedef struct
{
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
} FwVersion_t;

/* ---- Verify pass ---- */
#define OTA_XOR_BUF_LEN          64U    /* streamed checksum read chunk       */

/* ---- UART acquire (fr9 -> primary) ---- */
#define OTA_UART_BLOCK_LEN        1024U   /* raw file bytes per AT+FWGET pull  */
#define OTA_UART_BLOCK_RETRIES    3U      /* attempts per block                */
#define OTA_UART_RETRY_DELAY_MS   250U    /* settle time between attempts      */
#define OTA_FWREQ_WAIT_POLL_MS    2000U   /* re-poll period on FW,WAIT         */
#define OTA_FWREQ_WAIT_MAX_MS     90000U  /* covers fr9's own AT+FWCHECK round
                                             trip (modem wake + PSD activate +
                                             two HTTPS GETs)                  */
#define OTA_FWCHECK_ATSEND_TIMEOUT_MS 2000U

/* ---- LoRa distribution (primary -> secondaries) ---- */
#define OTA_LORA_CHUNK_LEN        224U    /* image payload bytes per OtaChunk  */
#define OTA_LORA_WINDOW_CHUNKS    64U     /* chunks per blast/repair window    */
#define OTA_LORA_MAX_TARGETS      8U      /* secondaries served per session    */
#define OTA_LORA_PREP_REPEATS     5U      /* OtaPrep announcements, 1 s apart  */
#define OTA_LORA_PREP_GAP_MS      1000U

/* Pause after each chunk TX, before the next chunk's flash read.
 *
 * This is the canonical write-up of the TX-correlated flash read problem;
 * every other site in the codebase that guards against it points here.
 *
 * It is not just radio pacing. On the V2 (frtag2) board, before the frtag3
 * fork, a flash read issued soon after a TX came back corrupted while the
 * stored bytes were fine. In an isolated single-TX bench test the read right
 * after the TX was wrong, a second back-to-back re-read was ALSO wrong and
 * different again, and a third read ~65 ms after the TX matched the known-good
 * value exactly. 15 ms of settle time was not enough; 100 ms gives comfortable
 * margin. The field evidence of 2026-08-31 (see OTA_VERIFY_TX_SETTLE_MS below)
 * and a chunk-for-chunk comparison of a full-image PRE scan against the
 * FOTA_bSendChunk output (see that function) tie the corruption to TX timing in
 * the same way. That correlation with TX timing is measured.
 *
 * The MECHANISM is not. This used to be written up as the SX126x PA's current
 * spike during TX sagging the shared supply rail. That theory was never
 * tested, and TX was never shown to sag the rail. The likelier cause is noise
 * coupling onto the very long MISO/MOSI lines, which run over a non-continuous
 * ground plane. None of that makes a TX harmless to a nearby flash read: the
 * observed corruption is real whatever the coupling path turns out to be.
 *
 * This delay, and every other quiet window, settle delay and retry count that
 * points here, is sized from the observed timing rather than from either
 * mechanism, so all of them are kept unchanged. V3 RevA (frtag3) is a new
 * layout running SPI2 at 12 MHz instead of V2's 750 kHz, and it has not yet
 * been characterised for this problem; until it has been shown to be free of
 * it, the guards stay. */
#define OTA_LORA_CHUNK_GAP_MS     100U
#define OTA_LORA_POLL_TIMEOUT_MS  6000U   /* wait for one OtaReport. Needs to
                                             cover the primary's own poll TX
                                             sitting behind remaining chunks
                                             in the radio TX queue (up to
                                             ~1 s under a 64-chunk blast),
                                             plus secondary CAD + reply
                                             airtime + primary RX. 2500 ms
                                             was the boundary and every
                                             post-mid-session window was
                                             timing out just as the reply
                                             arrived on the air.            */
#define OTA_LORA_REPAIR_ROUNDS    3U      /* repair passes per window          */
#define OTA_LORA_RX_IDLE_MS       20000U  /* secondary: session silence abort  */

/* How long a latched-but-unserviced OtaPrep stays valid on a secondary.
 *
 * bPrepPending is set by the mesh-parser task and consumed by the AppTask.
 * If the AppTask has already left its campaign loop when the Prep lands (a
 * congested campaign can push the primary's TimeSync/OtaPrep phase out past
 * the secondary's silence timeout), nothing ever consumes it — and the
 * first-primary-wins latch then rejects EVERY later OtaPrep because its
 * session id differs. Field case: one unit sat locked to a dead session id
 * for ~18 h, refusing an update all 37 of its peers took, and only recovered
 * when a shake-wake happened to disarm acceptance.
 *
 * So a pending Prep expires: past this age with no receive started, a
 * different-session OtaPrep is allowed to take over. Sized off
 * OTA_LORA_RX_IDLE_MS — the same "this session is dead" scale used for an
 * in-progress transfer. */
#define OTA_PREP_PENDING_MAX_MS   OTA_LORA_RX_IDLE_MS

/* Consecutive PRE-SEND verify failures (each in a separate campaign) before
 * the primary gives up on its staged image and erases it to re-acquire.
 *
 * Deliberately not 1. The per-pass retry budget only rules out a glitch
 * within one verify; a TX-correlated read disturbance (see
 * OTA_LORA_CHUNK_GAP_MS; the mechanism is untested) can spoil a whole pass
 * while the stored bytes are fine, and erasing on that evidence throws away a
 * good 100 kB image and re-downloads it. A field primary looped exactly that
 * way — verify-fail, erase, re-acquire, repeat every few minutes, never
 * distributing once. Requiring the failure to reproduce across campaigns
 * separates a real corruption from a transient, and costs only a delayed
 * re-acquire when it is real. */
#define OTA_PRESEND_FAIL_ERASE_THRESHOLD  3U
#define OTA_LORA_SESSION_MAX_MS   (12UL * 60UL * 1000UL)

/* ---- Multi-primary coexistence (listen-before-distribute) ----
 * Two primaries driven by the same fr9 schedule wake, TimeSync and start
 * distributing in the same second, so their OtaPrep/chunk/ack exchanges
 * collide and secondaries between them join neither. Before broadcasting
 * its first OtaPrep a primary waits a random backoff and listens; if it
 * hears another primary's live OTA session (any Ota* packet whose session
 * id isn't ours) it defers this campaign and lets the other finish. The
 * random backoff breaks the phase symmetry so one primary always Preps
 * first; per-campaign randomness keeps it fair and self-healing. */
#define OTA_DISTRIBUTE_BACKOFF_SPREAD_MS  5000U   /* random pre-Prep backoff; kept
                                                     well under APP_OTA_PREP_WAIT_MS
                                                     (15 s) so the winner's Prep
                                                     still lands in the secondary's
                                                     arm window                    */
#define OTA_FOREIGN_ACTIVE_MS             6000U   /* foreign OTA counts as "another
                                                     primary active" if heard this
                                                     recently (~one PREP burst:
                                                     REPEATS 5 * GAP 1000 ms)       */

/* ---- Radio quiet window before a whole-image XOR verify pass ---------------
 *
 * A flash read issued too soon after a TX can read back corrupted bytes while
 * the stored bytes are fine. That is observed on hardware, not explained: see
 * OTA_LORA_CHUNK_GAP_MS above for the measurements (15 ms of settle was not
 * enough, ~65 ms was, 100 ms was called comfortable) and for what is and is
 * not known about the cause. The old "PA current spike sags the shared supply
 * rail" explanation was never tested and TX was not shown to sag the rail;
 * noise on the long MISO/MOSI traces over a non-continuous ground plane is the
 * likelier cause. Neither changes the behaviour here. At the 750 kHz SPI2
 * rate of the time, a whole-image XOR pass was ~1.5 s of gap-free reads
 * (~95 ms at today's 12 MHz, see FLASH_SPI_CLOCK_HZ), so a single TX landing
 * anywhere inside it spoiled the pass.
 *
 * SEEN IN THE FIELD, 2026-08-31 (V2 board): two bad pre-send passes, both with
 * every chunk read reporting success (readOk=1 rdFails=0 hal=0 - so corrupted
 * bytes, not a transport failure) and the PA firing 270 ms and 336 ms into a
 * ~1550 ms pass. The one campaign in the same run whose random backoff
 * happened to let the TimeSync transmit before the pass started verified first
 * time. This ties the corruption to TX timing; it does not identify the
 * mechanism.
 *
 * Note this cannot be fixed by reordering. The offending TX is not one the
 * FOTA code issues: DeviceDiscovery queues a TimeSync ~20 ms before calling
 * FOTA_vDistribute, MESHNETWORK_bSendPacket only stamps it with 20-1500 ms of
 * jitter, and the radio task fires the PA whenever carrier sense lets it. The
 * packet is already in flight before any FOTA code runs, so the verify has to
 * wait it out - the same thing every other whole-image read here already does. */
#define OTA_VERIFY_TX_SETTLE_MS            150U   /* quiet time required before
                                                     starting a pass; 1.5x the
                                                     100 ms already judged
                                                     comfortable margin for
                                                     OTA_LORA_CHUNK_GAP_MS     */
#define OTA_VERIFY_TX_QUIET_MAX_MS        1500U   /* stop waiting and verify
                                                     anyway past this - just
                                                     over MESH_TX_JITTER_MAX_MS.
                                                     A busy radio must never
                                                     stall a campaign; the
                                                     retry budget still covers
                                                     a spoiled pass.           */

#endif /* WORKER_FOTA_FOTA_CONFIG_H_ */
