/*
 * uart_callbacks.c
 *
 * USART1 RX dispatch for frtag3.
 * USART1 is shared between GPS (secondary role) and Farmranger (primary role).
 * This file overrides the hal_uart weak callback and routes to the correct handler.
 *
 * Runs in the USART1 ISR. The role it reads (eDeviceRole, via
 * DEVICE_DISCOVERY_eGetDeviceRole) is a plain enum written once at boot by
 * DEVICE_DISCOVERY_vConfigDeviceRole(), before any USART1 interrupt can be
 * enabled, and never changed after, so the read is ISR-safe.
 */

#include "DeviceDiscovery.h"
#include "Farmranger.h"
#include "GPS.h"

void UART1_vNotifyOnRX(void)
{
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY)
        FARMRANGER_vNotifyOnRX();
    else
        GPS_vNotifyOnRX();
}
