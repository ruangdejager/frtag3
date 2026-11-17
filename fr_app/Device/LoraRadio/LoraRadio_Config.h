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
