/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : IMU Dead Reckoning – corrected firmware
  *
  * ACCELEROMETER CONVERSION (important to understand)
  * ---------------------------------------------------
  * LSM303DLHC_AccReadXYZ() returns raw 16-bit left-justified counts.
  * At ±2g HR mode: 1g = 16384 counts (12-bit left-justified in 16-bit).
  * Original code:  ax = acc_raw[0] / 16384.0f  → outputs in g-units
  * Python imu_receiver.py then multiplies by 9.81 → correct m/s²
  * This chain was CORRECT. We keep it.
  *
  * What was actually wrong
  * -----------------------
  * 1. Only 100 gyro calibration samples (~1 s) → poor bias estimate.
  *    Increased to 512 samples (~5.4 s) for a far better bias.
  *
  * 2. No accelerometer bias calibration.
  *    Added: collect 512 accel samples, subtract mean XY bias.
  *    Gravity axis is auto-detected and removed from bias.
  *
  * 3. HAL_Delay(10) was AFTER the CDC busy-wait → loop ran slower than
  *    100 Hz when USB was congested. Moved to TOP of loop.
  *
  * 4. Added on-board ZUPT flag as 8th CSV field (0/1).
  *    Python can use this directly without recomputing.
  *
  * 5. Added LED feedback: LD3 blinks during calibration, LD4 flashes
  *    3× when done so you know when to start moving.
  *
  * CSV format (one extra field vs original):
  *   timestamp_ms, ax_g, ay_g, az_g, gx_rads, gy_rads, gz_rads, stationary
  *   ax/ay/az : g-units  (Python × 9.81 → m/s²)
  *   gx/gy/gz : rad/s    (bias subtracted on-board)
  *   stationary : 1 when board is stationary, 0 otherwise
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stm32f3_discovery.h"
#include "l3gd20.h"
#include "lsm303dlhc.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* Gyro bias in deg/s (subtracted before deg→rad conversion) */
static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};

/* Accel bias in g-units (subtracted after /16384 conversion) */
static float acc_bias[3]    = {0.0f, 0.0f, 0.0f};

/* On-board ZUPT ring buffer – tracks |accel_g| - 1.0 variance */
#define ZUPT_WIN   30
static float   zupt_buf[ZUPT_WIN];
static uint8_t zupt_idx    = 0;
static uint8_t zupt_filled = 0;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
static float zupt_variance(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static float zupt_variance(void)
{
    uint8_t n = zupt_filled ? ZUPT_WIN : zupt_idx;
    if (n < 2) return 1.0f;
    float sum = 0.0f, var = 0.0f, mean;
    for (uint8_t i = 0; i < n; i++) sum += zupt_buf[i];
    mean = sum / (float)n;
    for (uint8_t i = 0; i < n; i++)
        var += (zupt_buf[i] - mean) * (zupt_buf[i] - mean);
    return var / (float)n;
}
/* USER CODE END 0 */


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* ── Peripheral init (ALL must be called before any sensor access) ──────── */
  MX_GPIO_Init();
  MX_I2C1_Init();   /* ← was missing: LSM303DLHC accelerometer uses I2C1     */
  MX_SPI1_Init();   /* ← was missing: L3GD20 gyroscope uses SPI1             */
  MX_USB_DEVICE_Init();

  HAL_Delay(1500);  /* Wait for USB enumeration and sensor power-on */

  /* Force SPI mode for MEMS */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /* ===== Gyroscope init =====
   *
   * L3GD20_Init(uint16_t): LOW byte → CTRL_REG1, HIGH byte → CTRL_REG4
   *
   * Previous attempt put L3GD20_BlockDataUpdate_Single (0x80) in the
   * OR expression with the other 8-bit values, so it ended up in
   * CTRL_REG1 (bit 7, which corrupts ODR) — NOT in CTRL_REG4.
   * BDU was never actually set.
   *
   * FIX: shift BDU and FS into the HIGH byte explicitly.
   *   CTRL_REG1 (low byte)  = ODR2 | ACTIVE | AXES | BW4
   *                         = 0x40 | 0x08   | 0x07 | 0x30 = 0x7F
   *   CTRL_REG4 (high byte) = BDU  | BLE_LSB | FS_250
   *                         = 0x80 | 0x00    | 0x00  = 0x80
   *   Combined uint16_t     = 0x807F
   */
  GYRO_IO_Init();
  L3GD20_Init(0x807F);
  /* 0x807F:
   *   CTRL_REG1 = 0x7F: DR=01(190Hz) PD=1 Zen=Yen=Xen=1 BW=11(70Hz cutoff)
   *   CTRL_REG4 = 0x80: BDU=1 BLE=0 FS=00(250dps) ST=0
   */

  /* ===== Accelerometer init =====
   *
   * LSM303DLHC_AccInit(uint16_t): LOW byte → CTRL_REG1,
   *   (uint8_t)(InitStruct << 8) → CTRL_REG4  (this is a driver bug —
   *   the shift produces 0x00 always when all values are 8-bit).
   *
   * FIX: call LSM303DLHC_AccInit with only CTRL_REG1 values (low byte),
   * then write CTRL_REG4 directly via COMPASSACCELERO_IO_Write.
   *
   *   CTRL_REG1 = ODR_200Hz | AXES_ENABLE | NORMAL_MODE
   *             = 0x60      | 0x07        | 0x00 = 0x67
   *   CTRL_REG4 = BDU | HR | FS_2G
   *             = 0x80| 0x08| 0x00 = 0x88   (written directly)
   */
  LSM303DLHC_AccInit(0x0067);          /* CTRL_REG1 only: ODR200+axes */
  COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS,
                           LSM303DLHC_CTRL_REG4_A,
                           0x88);      /* BDU=1, HR=1, FS=±2g */

  /* =========================================================
   * CALIBRATION  (512 samples ≈ 5.4 s at ~95 Hz gyro ODR)
   *
   * Keep the board COMPLETELY STILL on a flat surface.
   * LD3 (orange) blinks every 32 samples.
   * LD4 (green)  flashes 3× when calibration is complete.
   * ========================================================= */
#define CAL_N  512

  float    tmp_g[3]  = {0};
  int16_t  tmp_a[3]  = {0};
  float    sum_g[3]  = {0.0f, 0.0f, 0.0f};
  float    sum_a[3]  = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < CAL_N; i++)
  {
    /* Blink LD3 every 32 samples so user sees progress */
    if ((i & 31) == 0)
      BSP_LED_Toggle(LED3);

    /* Gyro – read raw deg/s */
    L3GD20_ReadXYZAngRate(tmp_g);
    sum_g[0] += tmp_g[0];
    sum_g[1] += tmp_g[1];
    sum_g[2] += tmp_g[2];

    /* Accel – read raw counts, convert to g-units (/16384) */
    LSM303DLHC_AccReadXYZ(tmp_a);
    sum_a[0] += tmp_a[0] / 16384.0f;
    sum_a[1] += tmp_a[1] / 16384.0f;
    sum_a[2] += tmp_a[2] / 16384.0f;

    HAL_Delay(10);
  }
  BSP_LED_Off(LED3);

  /* Gyro bias: mean reading while stationary */
  gyro_offset[0] = sum_g[0] / (float)CAL_N;
  gyro_offset[1] = sum_g[1] / (float)CAL_N;
  gyro_offset[2] = sum_g[2] / (float)CAL_N;

  /* Accel bias:
   *   Horizontal axes: mean ≈ 0 g  → bias = mean
   *   Gravity axis   : mean ≈ ±1 g → bias = mean - sign(mean)*1.0
   *
   * Auto-detect gravity axis = axis with largest |mean|.
   */
  float ma[3];
  ma[0] = sum_a[0] / (float)CAL_N;
  ma[1] = sum_a[1] / (float)CAL_N;
  ma[2] = sum_a[2] / (float)CAL_N;

  uint8_t gax = 0;
  if (fabsf(ma[1]) > fabsf(ma[gax])) gax = 1;
  if (fabsf(ma[2]) > fabsf(ma[gax])) gax = 2;

  acc_bias[0] = ma[0];
  acc_bias[1] = ma[1];
  acc_bias[2] = ma[2];
  /* Remove the 1 g of gravity from the vertical axis bias */
  acc_bias[gax] -= (ma[gax] >= 0.0f ? 1.0f : -1.0f);

  /* Flash LD4 (green) 3× = done, safe to move */
  for (int i = 0; i < 3; i++)
  {
    BSP_LED_On(LED4);  HAL_Delay(200);
    BSP_LED_Off(LED4); HAL_Delay(200);
  }

  /* ===== Main loop ===== */
  while (1)
  {
    /* Delay at TOP for stable 100 Hz regardless of USB busy time */
    HAL_Delay(10);

    /* --- Gyro ---
     * Read deg/s, subtract calibrated bias, convert to rad/s.
     */
    float g[3] = {0.0f};
    L3GD20_ReadXYZAngRate(g);
    g[0] = (g[0] - gyro_offset[0]) * 0.0174533f;
    g[1] = (g[1] - gyro_offset[1]) * 0.0174533f;
    g[2] = (g[2] - gyro_offset[2]) * 0.0174533f;

    /* --- Accelerometer ---
     * Read raw counts → divide by 16384 → g-units → subtract bias.
     * Python imu_receiver.py multiplies by 9.81 to get m/s².
     * This is the SAME conversion as the original firmware.
     */
    int16_t acc_raw[3] = {0};
    LSM303DLHC_AccReadXYZ(acc_raw);

    float ax = acc_raw[0] / 16384.0f - acc_bias[0];
    float ay = acc_raw[1] / 16384.0f - acc_bias[1];
    float az = acc_raw[2] / 16384.0f - acc_bias[2];

    /* --- On-board ZUPT detector ---
     * Variance of (|accel_g| - 1.0) over 30 samples.
     * Low variance + low gyro magnitude = stationary.
     */
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    zupt_buf[zupt_idx] = a_mag - 1.0f;   /* residual vs gravity in g */
    zupt_idx = (zupt_idx + 1) % ZUPT_WIN;
    if (zupt_idx == 0) zupt_filled = 1;

    float g_mag  = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    float var    = zupt_variance();
    /* ZUPT thresholds — must fire reliably when robot is stationary.
     * Too tight → never fires → gyro bias drift accumulates → yaw ramp.
     * Too loose → fires while moving → position jumps to wrong place.
     *
     * var  < 0.01  : accel variance in g² (fan+vibration ≈ 0.001–0.005 on flat surface)
     * g_mag< 0.10  : raw gyro magnitude rad/s (L3GD20 bias ≈ 0.01–0.05 rad/s)
     */
    uint8_t stat = (var < 0.01f && g_mag < 0.10f) ? 1u : 0u;

    /* --- Transmit CSV ---
     * Format: timestamp_ms, ax, ay, az, gx, gy, gz, stationary
     * ax/ay/az in g-units (Python multiplies by 9.81 for m/s²)
     * gx/gy/gz in rad/s
     */
    uint32_t t = HAL_GetTick();
    char msg[128];
    int len = sprintf(msg,
                      "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u\r\n",
                      t, ax, ay, az, g[0], g[1], g[2], stat);

    uint8_t retries = 5;
    while (CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len) == USBD_BUSY && retries--)
    {
      HAL_Delay(1);
    }
  }
}

/* ── Clock and peripheral init (unchanged from original) ─────────────────── */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                           |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin,
                   GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin
                       |MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                       |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
