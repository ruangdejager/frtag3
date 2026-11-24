/*
 * LoraRadio_Config.h
 *
 * LoRa radio configuration — frequency, RF parameters and packet size.
 */

#ifndef DEVICE_LORARADIO_LORARADIO_CONFIG_H_
#define DEVICE_LORARADIO_LORARADIO_CONFIG_H_

#define RF_FREQUENCY                868000000   /* Hz        */
#define TX_OUTPUT_POWER             15          /* dBm       */
#define LORA_BANDWIDTH              LORA_BW_250 /* kHz       */
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1
#define LORA_PREAMBLE_LENGTH        8           /* Tx and Rx */
#define LORA_SYMBOL_TIMEOUT         5           /* Symbols   */
#define LORA_SYNCWORD               0x1424

#define LORA_MAX_PACKET_SIZE        (256)

/* ---- Beacon + listen window, and the fast reply that answers it ----
 *
 * A sleeping tag announces itself with a tiny beacon and then listens for a
 * few tens of ms (LORARADIO_bStartBeaconWindow). A listener that wants it -
 * the kernel console - answers inside that window (LORARADIO_bFastReply).
 * Timeline, measured from the end of the beacon on both sides:
 *
 *   tag      TX done -> ~4-5 ms TCXO + SetRx -> receiver live -> window timer
 *   console  RX done -> read + build (~1 ms) -> ~4 ms TCXO + PLL -> preamble
 *
 * The two start within a few ms of each other, so the answer goes out with a
 * long preamble (32 symbols = 16.4 ms at SF7/BW250) that is still on the air
 * whenever the tag's receiver comes up, and the tag's timer only has to
 * cover the start of that preamble (stop-on-preamble-detect does the rest). */
#define LORA_FAST_REPLY_PREAMBLE_LENGTH 32U

/* Longest fast reply a listener may build (a kernel CONNECT is 11 B). */
#define LORA_FAST_REPLY_MAX_LEN         24U

/* Longest beacon frame LORARADIO_bStartBeaconWindow takes (type + id + flags
 * is 6 B); kept tiny because every byte is airtime on a sleeping tag. */
#define LORA_BEACON_FRAME_MAX           8U

/* After the listen window's timer expires (or a preamble stopped it), how long
 * the radio task waits for the chip to report the outcome. Covers a full
 * long-preamble CONNECT (~30 ms on air) with margin. */
#define LORA_BEACON_WINDOW_TAIL_MS      150U

/* How long LORARADIO_vWakeUp / LORARADIO_vEnterDeepSleep wait for a beacon
 * window in progress to finish before they take the chip anyway. A window is
 * ~60 ms end to end; this is only reached if something went wrong. */
#define LORA_BEACON_WAIT_MAX_MS         300U

/* ---- Radio device id (see LORARADIO_u32GetUniqueId) ----
 *
 * The id's high half is [15:8] wafer number, [7:0] a CRC-8 over the 7 lot
 * number bytes. The CRC is CRC-8/SMBUS: poly 0x07, init 0x00, no reflection,
 * no final XOR, bytes fed in the order UID w1[15:8], w1[23:16], w1[31:24],
 * w2[7:0], w2[15:8], w2[23:16], w2[31:24]. Anything that has to reproduce a
 * tag's id from its UID off the board (the production script, the fr9 or the
 * backend) must use exactly this, so do not change either constant without
 * accepting that every tag in the field gets a new id. */
#define LORA_ID_CRC8_POLY           0x07U

/* A high half of zero is reserved: it means "only the legacy 16 bits are known"
 * (a peer on older firmware, or a beacon relayed by one), see
 * LORARADIO_bSameDevice. A real tag whose wafer number and lot CRC are both
 * zero gets this instead. Wafer 255 cannot occur - a lot has 25 wafers - so the
 * substitute cannot collide with a naturally derived id. */
#define LORA_ID_HI_ZERO_SUBST       0xFF00U

#endif /* DEVICE_LORARADIO_LORARADIO_CONFIG_H_ */
