#ifndef APP_GYRO_H
#define APP_GYRO_H

#include <stdint.h>

typedef struct
{
    uint32_t timestamp_ms;
    float gx;
    float gy;
    float gz;
} GyroSample_t;

void APP_GYRO_Init(void);
void APP_GYRO_Read(GyroSample_t *sample);

#endif
