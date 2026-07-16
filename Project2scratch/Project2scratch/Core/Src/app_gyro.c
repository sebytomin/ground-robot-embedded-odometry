#include "app_gyro.h"
#include "main.h"
#include "stm32f3_discovery_gyroscope.h"

void APP_GYRO_Init(void)
{
    BSP_GYRO_Init();
}

void APP_GYRO_Read(GyroSample_t *sample)
{
    float xyz[3] = {0};

    sample->timestamp_ms = HAL_GetTick();

    BSP_GYRO_GetXYZ(xyz);

    sample->gx = xyz[0];
    sample->gy = xyz[1];
    sample->gz = xyz[2];
}
