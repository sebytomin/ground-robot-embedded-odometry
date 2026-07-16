#include "app_usb_stream.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

void APP_USB_SendGyroSample(const GyroSample_t *sample)
{
    char msg[96];

    int len = snprintf(msg, sizeof(msg),"%lu,%.3f,%.3f,%.3f\r\n", sample->timestamp_ms,sample->gx,sample->gy,sample->gz);

    if (len > 0)
    {
        CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
    }
}
