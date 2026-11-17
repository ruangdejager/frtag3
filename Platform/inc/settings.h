/*
 * settings.h
 *
 * Runtime settings module.
 * Settings live in RAM, are initialised from the compile-time defaults in
 * settings_default.h, and are then overlaid with the stored record if the
 * application's storage hook has one (fr_app/settings/SettingsStore.c keeps it
 * on the external NOR). Accessor macros translate field names to byte offsets
 * so the underlying storage can be changed without touching callers.
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <stdint.h>
#include <stddef.h>
#include "settings_default.h"

typedef Settings_EE settings_struct_t;

/* const: lives in flash, not .data. RAM on this part is nearly full, and the
 * defaults are only ever copied from. */
extern const Settings_EE settings_def;

#define SETTINGS_CONF_DEFAULT_STRUCT    settings_def

/* Load: compiled-in defaults first, then whatever the storage hook can supply
 * on top of them (see SETTINGS_bStorageLoad). Called once at boot. */
void SETTINGS_vInit(void);
void SETTINGS_vReadValues(void);

/* Persist the current RAM settings through the storage hook. Returns true when
 * the stored copy matches RAM afterwards, including when nothing had changed
 * and no write was needed - so it is cheap to call after every batch of
 * SETTINGS_vSet*() calls. Call it from one task at a time per batch: setters
 * write RAM unlocked, so a commit racing a setter can store a half-applied
 * batch (it would still be a consistent record, just not the intended one). */
bool SETTINGS_bCommit(void);

/* Put every field back to its compiled-in default, in RAM only; follow with
 * SETTINGS_bCommit() to make it stick. */
void SETTINGS_vRestoreDefaults(void);

/* ---- Storage hooks ----
 * The engine keeps no knowledge of where settings are stored. The application
 * provides these two; the weak defaults in settings.c store nothing, which
 * leaves a RAM-only engine running on its defaults.
 *
 * SETTINGS_bStorageLoad: fill pvDst with up to u16Size bytes of the stored
 *   record if one exists whose layout version equals u16Layout, and return
 *   true. pvDst already holds the defaults; a stored record shorter than
 *   u16Size leaves the remainder untouched. On false, pvDst may have been
 *   partly overwritten and the caller restores the defaults.
 * SETTINGS_bStorageSave: store u16Size bytes from pvSrc tagged with u16Layout.
 *   True when stored (or already stored unchanged). */
bool SETTINGS_bStorageLoad(void *pvDst, uint16_t u16Size, uint16_t u16Layout);
bool SETTINGS_bStorageSave(const void *pvSrc, uint16_t u16Size, uint16_t u16Layout);

/* uint8_t */
uint8_t  SETTINGS_u8GetByteAtOffset(uint32_t offset);
void     SETTINGS_vSetByteAtOffset(uint32_t offset, uint8_t value);
bool     SETTINGS_bCheckByteAtOffset(uint32_t offset);
void     SETTINGS_vResetByteAtOffset(uint32_t offset);

/* uint16_t */
uint16_t SETTINGS_u16GetShortAtOffset(uint32_t offset);
void     SETTINGS_vSetShortAtOffset(uint32_t offset, uint16_t value);
bool     SETTINGS_bCheckShortAtOffset(uint32_t offset);
void     SETTINGS_vResetShortAtOffset(uint32_t offset);

/* uint32_t */
uint32_t SETTINGS_u32GetWordAtOffset(uint32_t offset, uint32_t extra);
void     SETTINGS_vSetWordAtOffset(uint32_t offset, uint32_t extra, uint32_t value);
bool     SETTINGS_bCheckWordAtOffset(uint32_t offset, uint32_t extra, bool is_signed);
void     SETTINGS_vResetWordAtOffset(uint32_t offset, uint32_t extra);

/* strings */
void  SETTINGS_vSetStringAtOffset(uint32_t offset, size_t length_at_offset, const char *new_value, size_t length);
char *SETTINGS_vGetStringAtOffset(uint32_t offset, size_t length_at_offset, char *buf, size_t length);
bool  SETTINGS_bCheckStringAtOffset(uint32_t offset, size_t length_at_offset, bool mayBeEmpty);
void  SETTINGS_vResetStringAtOffset(uint32_t offset, size_t length);

/* bool */
void SETTINGS_vSetBoolAtOffset(uint32_t offset, uint32_t extra, bool value);
bool SETTINGS_bGetBoolAtOffset(uint32_t offset, uint32_t extra);
bool SETTINGS_bCheckBoolAtOffset(uint32_t offset, uint32_t extra);
void SETTINGS_vResetBoolAtOffset(uint32_t offset, uint32_t extra);

/* struct pointer */
void *SETTINGS_vpGetStructureAtOffset(uint32_t offset);

settings_struct_t *SETTINGS_pGetSettingsStruct(void);

/* ---- Helper macros ---- */
#define ADR(parameter)          (offsetof(settings_struct_t, parameter))
#define PARAM_SIZE(parameter)   ((size_t)sizeof(((settings_struct_t *)0)->parameter))

/* uint8_t */
#define SETTINGS_u8GetByte(p)           SETTINGS_u8GetByteAtOffset(ADR(p))
#define SETTINGS_vSetByte(p, v)         SETTINGS_vSetByteAtOffset(ADR(p), v)
#define SETTINGS_bCheckByte(p)          SETTINGS_bCheckByteAtOffset(ADR(p))
#define SETTINGS_vResetByte(p)          SETTINGS_vResetByteAtOffset(ADR(p))

/* uint16_t */
#define SETTINGS_u16GetShort(p)         SETTINGS_u16GetShortAtOffset(ADR(p))
#define SETTINGS_vSetShort(p, v)        SETTINGS_vSetShortAtOffset(ADR(p), v)
#define SETTINGS_bCheckShort(p)         SETTINGS_bCheckShortAtOffset(ADR(p))
#define SETTINGS_vResetShort(p)         SETTINGS_vResetShortAtOffset(ADR(p))

/* uint32_t */
#define SETTINGS_u32GetWord(p)          SETTINGS_u32GetWordAtOffset(ADR(p), 0)
#define SETTINGS_vSetWord(p, v)         SETTINGS_vSetWordAtOffset(ADR(p), 0, v)
#define SETTINGS_bCheckWord(p)          SETTINGS_bCheckWordAtOffset(ADR(p), 0, false)
#define SETTINGS_bCheckSignedWord(p)    SETTINGS_bCheckWordAtOffset(ADR(p), 0, true)
#define SETTINGS_vResetWord(p)          SETTINGS_vResetWordAtOffset(ADR(p), 0)

/* uint32_t with extra offset (iterating over repeated structs) */
#define SETTINGS_u32GetWordExtra(st, p, e)      SETTINGS_u32GetWordAtOffset(offsetof(st, p), e)
#define SETTINGS_vSetWordExtra(st, p, e, v)     SETTINGS_vSetWordAtOffset(offsetof(st, p), e, v)
#define SETTINGS_bCheckWordExtra(st, p, e)      SETTINGS_bCheckWordAtOffset(offsetof(st, p), e, false)
#define SETTINGS_vResetWordExtra(st, p, e)      SETTINGS_vResetWordAtOffset(offsetof(st, p), e)

/* string */
#define SETTINGS_pauGetString(p)                SETTINGS_vGetStringAtOffset(ADR(p), PARAM_SIZE(p), NULL, 0)
#define SETTINGS_pauGetStringInto(p, b, l)      SETTINGS_vGetStringAtOffset(ADR(p), PARAM_SIZE(p), b, l)
#define SETTINGS_vSetString(p, v, l)            SETTINGS_vSetStringAtOffset(ADR(p), PARAM_SIZE(p), v, l)
#define SETTINGS_vResetString(p)                SETTINGS_vResetStringAtOffset(ADR(p), PARAM_SIZE(p))
#define SETTINGS_bCheckString(p, e)             SETTINGS_bCheckStringAtOffset(ADR(p), PARAM_SIZE(p), e)

/* bool */
#define SETTINGS_bGetBool(p)                    SETTINGS_bGetBoolAtOffset(ADR(p), 0)
#define SETTINGS_vSetBool(p, v)                 SETTINGS_vSetBoolAtOffset(ADR(p), 0, v)
#define SETTINGS_vResetBool(p)                  SETTINGS_vResetBoolAtOffset(ADR(p), 0)
#define SETTINGS_bCheckBool(p)                  SETTINGS_bCheckBoolAtOffset(ADR(p), 0)

#define SETTINGS_bGetBoolExtra(st, p, e)        SETTINGS_bGetBoolAtOffset(offsetof(st, p), e)
#define SETTINGS_vSetBoolExtra(st, p, e, v)     SETTINGS_vSetBoolAtOffset(offsetof(st, p), e, v)
#define SETTINGS_bCheckBoolExtra(st, p, e)      SETTINGS_bCheckBoolAtOffset(offsetof(st, p), e)
#define SETTINGS_vResetBoolExtra(st, p, e)      SETTINGS_vResetBoolAtOffset(offsetof(st, p), e)

/* struct pointer */
#define SETTINGS_vpGetStructure(p)              SETTINGS_vpGetStructureAtOffset(ADR(p))

#endif /* SETTINGS_H_ */
