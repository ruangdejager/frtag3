/*
 * SettingsStore.h
 *
 * Persistent backing store for the Platform settings engine, on the external
 * NOR (partition and record format in SettingsStore_Config.h).
 *
 * This module has no API of its own for callers. It provides the two storage
 * hooks Platform/inc/settings.h declares - SETTINGS_bStorageLoad() and
 * SETTINGS_bStorageSave() - replacing their weak no-op defaults, so the
 * settings engine stays free of any knowledge of the flash part or its map.
 * Application code uses SETTINGS_vInit() and SETTINGS_bCommit() as before.
 *
 * Diagnostics are the one exception: SETTINGS_STORE_vGetStatus() reports
 * where the current record lives, for the boot banner and the kernel.
 */

#ifndef SETTINGS_SETTINGSSTORE_H_
#define SETTINGS_SETTINGSSTORE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool     bHaveRecord;    /* a valid record exists on flash            */
    uint8_t  u8Bank;         /* 0 = A, 1 = B; meaningful with bHaveRecord */
    uint8_t  u8Slot;
    uint16_t u16Layout;      /* layout version the record was written with */
    uint16_t u16Len;         /* payload bytes                              */
    uint32_t u32Seq;         /* commit sequence number                     */
    uint16_t u16WriteFails;  /* failed write-verifies since boot           */
} SettingsStore_Status_t;

void SETTINGS_STORE_vGetStatus(SettingsStore_Status_t *ptStatus);

#endif /* SETTINGS_SETTINGSSTORE_H_ */
