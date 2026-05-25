/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "st7789.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_TEMP_THRESHOLD_C          38.0f
#define APP_PAUSE_DISPLAY_MS          10000UL
#define APP_TOUCH_DEBOUNCE_MS         80UL
#define APP_TOUCH_POLL_MS             20UL
#define APP_TOUCH_SAMPLE_COUNT        7U
#define APP_TOUCH_CONFIRM_COUNT       2U
#define APP_TOUCH_RELEASE_COUNT       2U
#define APP_TOUCH_STABLE_DELTA        120U
#define APP_TOUCH_HIT_MARGIN          8

#define APP_ADC_TIMEOUT_MS            10UL
#define APP_SPI_TIMEOUT_MS            10UL
#define APP_SENSOR_POLL_MS            200UL
#define APP_LCD_UPDATE_MS             500UL
#define APP_FRAM_TIMEOUT_MS           30UL
#define APP_FRAM_SAVE_PERIOD_MS       1000UL
#define APP_VREFINT_TYPICAL_V         1.21f
#define APP_TEMP_V25                  0.76f
#define APP_TEMP_AVG_SLOPE            0.0025f
#define APP_HEAT_BUDGET_MS            5UL
#define APP_HEAT_INNER_ITER           256UL

//#define APP_TP_CS_PORT                LCD_PA9_GPIO_Port
//#define APP_TP_CS_PIN                 LCD_PA9_Pin
#define APP_TP_CS_PORT                GPIOB
#define APP_TP_CS_PIN                 GPIO_PIN_9
#define APP_TP_IRQ_PORT               GPIOB
#define APP_TP_IRQ_PIN                GPIO_PIN_4

#define APP_TOUCH_CMD_X               0xD0
#define APP_TOUCH_CMD_Y               0x90
/* Default calibration window for the XPT2046 on this board.
   If touch is still shifted after flashing, tune these four values once. */
#define APP_TOUCH_RAW_X_MIN           200U
#define APP_TOUCH_RAW_X_MAX           3900U
#define APP_TOUCH_RAW_Y_MIN           200U
#define APP_TOUCH_RAW_Y_MAX           3900U
#define APP_TOUCH_SWAP_XY             1
#define APP_TOUCH_INV_X               0
#define APP_TOUCH_INV_Y               1

#define APP_FRAM_ADDR                 (0x50 << 1)
/* Khong dat header o 0x0000 nua. Neu doc nham header nhu record thi se ra ADC=16717, T=180.02 vi 0x4652414D = "FRAM". */
#define APP_FRAM_HEADER_ADDR          0x0020U
#define APP_FRAM_RECORD_ADDR          0x0040U
/* Khong dung magic ASCII "FRAM" de tranh hien thanh ADC/T neu bi doc nham. */
#define APP_FRAM_MAGIC                0xA55A33CCU
/* Chi can vung 0..255 de luu records. Cach nay dung duoc voi ca FM24/AT24 dang dia chi 8-bit. */
#define APP_FRAM_TOTAL_BYTES          256U
/* Sau khi da xoa du lieu rac cu, de = 0 de KHONG xoa FRAM khi reset/ghi runtime.
   Chi doi tam thanh 1U neu muon format sach FRAM mot lan. */
#define APP_FRAM_FORCE_CLEAR_ON_BOOT  0U

#define BTN_PLAY_X                    10
#define BTN_PLAY_Y                    185
#define BTN_PLAY_W                    140
#define BTN_PLAY_H                    45
#define BTN_PAUSE_X                   170
#define BTN_PAUSE_Y                   185
#define BTN_PAUSE_W                   140
#define BTN_PAUSE_H                   45
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
/* USER CODE BEGIN PV */
typedef enum {
    APP_MODE_HOME = 0,
    APP_MODE_PLAY,
    APP_MODE_PAUSE
} AppMode_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t head;
    uint16_t count;
} FramHeader_t;

typedef struct __attribute__((packed)) {
    uint16_t adc_value;
    int16_t temp_x100;
    uint32_t tick_ms;
} FramRecord_t;

volatile AppMode_t g_appMode = APP_MODE_HOME;
volatile float g_tempC = 0.0f;
volatile uint16_t g_adcInternal = 0;
volatile uint32_t g_lastTouchTick = 0;
volatile uint32_t g_lastFramSaveTick = 0;
volatile uint8_t g_touchHeld = 0;
volatile uint8_t g_framReady = 0;
static uint16_t g_framMemAddrSize = I2C_MEMADD_SIZE_8BIT;
static uint8_t g_playStatusLcd = 0xFFU;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */
static uint8_t APP_ReadAdcChannel(uint32_t channel, uint32_t samplingTime, uint16_t *value);
static uint8_t APP_ReadInternalSensors(float *tempC, uint16_t *adcRaw);
static void APP_TouchSelect(void);
static void APP_TouchUnselect(void);
static uint16_t APP_TouchRead12(uint8_t cmd);
static void APP_SortU16(uint16_t *values, uint8_t count);
static uint8_t APP_TouchReadRaw(uint16_t *rawX, uint16_t *rawY);
static uint8_t APP_TouchGetXY(uint16_t *x, uint16_t *y);
static uint16_t APP_MapU16(uint16_t value, uint16_t inMin, uint16_t inMax, uint16_t outMax);
static uint8_t APP_PointInRect(uint16_t x, uint16_t y, int16_t rx, int16_t ry, int16_t rw, int16_t rh);
static void APP_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
static void APP_DrawButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *label, uint16_t fill, uint16_t border);
static void APP_FormatTemp(char *buffer, float temp);
static HAL_StatusTypeDef APP_FramMemRead(uint16_t address, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef APP_FramMemWrite(uint16_t address, const uint8_t *data, uint16_t length);
static uint8_t APP_FramRecordIsValid(const FramRecord_t *record);
static uint8_t APP_FramTryFormatWithAddrSize(uint16_t memAddrSize);
static void APP_FramFormat(void);
static void APP_FramInit(void);
static void APP_FramWriteRecord(uint16_t adcValue, float tempC, uint32_t tickMs);
static uint16_t APP_FramReadLatest(FramRecord_t *records, uint16_t maxCount);
static void APP_DrawHomeScreen(void);
static void APP_DrawPlayScreen(void);
static void APP_DrawPauseScreen(void);
static void APP_UpdateHomeValues(void);
static void APP_UpdatePlayValues(void);
static void APP_RunHeatChunk(void);
uint8_t APP_IsPlayMode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
uint8_t APP_IsPlayMode(void)
{
    return (g_appMode == APP_MODE_PLAY) ? 1U : 0U;
}

static void APP_RunHeatChunk(void)
{
    static volatile uint32_t heatState = 0x13579BDFU;
    static volatile float heatFloat = 1.2345f;
    uint32_t startTick = HAL_GetTick();

    do {
        for (uint32_t i = 0; i < APP_HEAT_INNER_ITER; i++) {
            heatState = (heatState * 1664525UL) + 1013904223UL + i;
            heatState ^= (heatState << 13);
            heatState ^= (heatState >> 17);
            heatState ^= (heatState << 5);

            heatFloat = (heatFloat * 1.00024414f) + ((float)(heatState & 0xFFU) * 0.000001f);
            if (heatFloat > 1000.0f) {
                heatFloat = 1.2345f;
            }
        }
    } while ((HAL_GetTick() - startTick) < APP_HEAT_BUDGET_MS);
}

static uint8_t APP_ReadAdcChannel(uint32_t channel, uint32_t samplingTime, uint16_t *value)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t raw;

    if (value == NULL) {
        return 0U;
    }

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = samplingTime;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 0U;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 0U;
    }

    if (HAL_ADC_PollForConversion(&hadc1, APP_ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    raw = HAL_ADC_GetValue(&hadc1) & 0x0FFFUL;
    HAL_ADC_Stop(&hadc1);

    *value = (uint16_t)raw;
    return 1U;
}

static uint16_t APP_ReadAdcAverage(uint32_t channel, uint32_t samplingTime, uint8_t samples)
{
    uint32_t sum = 0U;
    uint8_t okCount = 0U;
    uint16_t raw = 0U;

    (void)APP_ReadAdcChannel(channel, samplingTime, &raw);

    for (uint8_t i = 0U; i < samples; i++) {
        if (APP_ReadAdcChannel(channel, samplingTime, &raw)) {
            sum += raw;
            okCount++;
        }
    }

    if (okCount == 0U) {
        return 0U;
    }

    return (uint16_t)(sum / okCount);
}

static uint8_t APP_ReadInternalSensors(float *tempC, uint16_t *adcRaw)
{
    uint16_t rawTemp;
    uint16_t rawVref;
    float vdda;
    float vSense;
    float temp;

    if ((tempC == NULL) || (adcRaw == NULL)) {
        return 0U;
    }

    ADC->CCR |= ADC_CCR_TSVREFE;
    for (volatile uint32_t i = 0U; i < 8000U; i++) {
        __NOP();
    }

    rawTemp = APP_ReadAdcAverage(ADC_CHANNEL_TEMPSENSOR, ADC_SAMPLETIME_480CYCLES, 8U);
    rawVref = APP_ReadAdcAverage(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_480CYCLES, 4U);

    if ((rawTemp == 0U) || (rawTemp > 4095U) || (rawVref < 500U) || (rawVref > 2500U)) {
        return 0U;
    }

    vdda = (APP_VREFINT_TYPICAL_V * 4095.0f) / (float)rawVref;
    vSense = ((float)rawTemp * vdda) / 4095.0f;
    temp = ((vSense - APP_TEMP_V25) / APP_TEMP_AVG_SLOPE) + 25.0f;

    if ((temp < -20.0f) || (temp > 120.0f)) {
        return 0U;
    }

    *tempC = temp;
    *adcRaw = rawTemp;
    return 1U;
}

static void APP_TouchSelect(void)
{
    HAL_GPIO_WritePin(APP_TP_CS_PORT, APP_TP_CS_PIN, GPIO_PIN_RESET);
}

static void APP_TouchUnselect(void)
{
    HAL_GPIO_WritePin(APP_TP_CS_PORT, APP_TP_CS_PIN, GPIO_PIN_SET);
}

static uint16_t APP_TouchRead12(uint8_t cmd)
{
    uint8_t tx[3] = {cmd, 0x00, 0x00};
    uint8_t rx[3] = {0, 0, 0};

    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, APP_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    return (uint16_t)((((uint16_t)rx[1] << 8) | rx[2]) >> 3);
}

static void APP_SortU16(uint16_t *values, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = (uint8_t)(i + 1U); j < count; j++) {
            if (values[j] < values[i]) {
                uint16_t temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
}

static uint8_t APP_TouchReadRaw(uint16_t *rawX, uint16_t *rawY)
{
    uint16_t xSamples[APP_TOUCH_SAMPLE_COUNT];
    uint16_t ySamples[APP_TOUCH_SAMPLE_COUNT];

    if ((rawX == NULL) || (rawY == NULL)) {
        return 0;
    }

    if (HAL_GPIO_ReadPin(APP_TP_IRQ_PORT, APP_TP_IRQ_PIN) == GPIO_PIN_SET) {
        return 0;
    }

    APP_TouchSelect();

    /* Throw away the first conversion after CS goes low. */
    (void)APP_TouchRead12(APP_TOUCH_CMD_X);
    (void)APP_TouchRead12(APP_TOUCH_CMD_Y);

    for (uint8_t i = 0; i < APP_TOUCH_SAMPLE_COUNT; i++) {
        xSamples[i] = APP_TouchRead12(APP_TOUCH_CMD_X);
        ySamples[i] = APP_TouchRead12(APP_TOUCH_CMD_Y);
    }

    APP_TouchUnselect();

    if (HAL_GPIO_ReadPin(APP_TP_IRQ_PORT, APP_TP_IRQ_PIN) == GPIO_PIN_SET) {
        return 0;
    }

    APP_SortU16(xSamples, APP_TOUCH_SAMPLE_COUNT);
    APP_SortU16(ySamples, APP_TOUCH_SAMPLE_COUNT);

    if ((uint16_t)(xSamples[APP_TOUCH_SAMPLE_COUNT - 1U] - xSamples[0]) > APP_TOUCH_STABLE_DELTA) {
        return 0;
    }
    if ((uint16_t)(ySamples[APP_TOUCH_SAMPLE_COUNT - 1U] - ySamples[0]) > APP_TOUCH_STABLE_DELTA) {
        return 0;
    }

    *rawX = xSamples[APP_TOUCH_SAMPLE_COUNT / 2U];
    *rawY = ySamples[APP_TOUCH_SAMPLE_COUNT / 2U];
    return 1;
}

static uint16_t APP_MapU16(uint16_t value, uint16_t inMin, uint16_t inMax, uint16_t outMax)
{
    if (value <= inMin) return 0;
    if (value >= inMax) return outMax;
    return (uint16_t)(((uint32_t)(value - inMin) * outMax) / (uint32_t)(inMax - inMin));
}

static uint8_t APP_TouchGetXY(uint16_t *x, uint16_t *y)
{
    uint16_t rawX, rawY;

    if ((x == NULL) || (y == NULL)) {
        return 0;
    }

    if (!APP_TouchReadRaw(&rawX, &rawY)) {
        return 0;
    }

#if APP_TOUCH_SWAP_XY
    {
        uint16_t tmp = rawX;
        rawX = rawY;
        rawY = tmp;
    }
#endif
#if APP_TOUCH_INV_X
    rawX = (uint16_t)(4095U - rawX);
#endif
#if APP_TOUCH_INV_Y
    rawY = (uint16_t)(4095U - rawY);
#endif

    if (rawX < APP_TOUCH_RAW_X_MIN) rawX = APP_TOUCH_RAW_X_MIN;
    if (rawX > APP_TOUCH_RAW_X_MAX) rawX = APP_TOUCH_RAW_X_MAX;
    if (rawY < APP_TOUCH_RAW_Y_MIN) rawY = APP_TOUCH_RAW_Y_MIN;
    if (rawY > APP_TOUCH_RAW_Y_MAX) rawY = APP_TOUCH_RAW_Y_MAX;

    *x = APP_MapU16(rawX, APP_TOUCH_RAW_X_MIN, APP_TOUCH_RAW_X_MAX, ST7789_WIDTH - 1U);
    *y = APP_MapU16(rawY, APP_TOUCH_RAW_Y_MIN, APP_TOUCH_RAW_Y_MAX, ST7789_HEIGHT - 1U);
    return 1;
}

static uint8_t APP_PointInRect(uint16_t x, uint16_t y, int16_t rx, int16_t ry, int16_t rw, int16_t rh)
{
    if ((int16_t)x < rx) return 0;
    if ((int16_t)y < ry) return 0;
    if ((int16_t)x > (int16_t)(rx + rw - 1)) return 0;
    if ((int16_t)y > (int16_t)(ry + rh - 1)) return 0;
    return 1;
}

static void APP_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if ((w == 0U) || (h == 0U)) return;
    ST7789_Fill(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U), color);
}

static void APP_DrawButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *label, uint16_t fill, uint16_t border)
{
    APP_FillRect(x, y, w, h, fill);
    ST7789_DrawRectangle(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U), border);
    ST7789_WriteString((uint16_t)(x + 36U), (uint16_t)(y + 15U), label, Font_7x10, WHITE, fill);
}

static void APP_FormatTemp(char *buffer, float temp)
{
    int32_t value = (int32_t)(temp * 100.0f);
    int32_t absValue;
    int32_t integerPart;
    int32_t decimalPart;

    if (value < 0) {
        absValue = -value;
        integerPart = absValue / 100;
        decimalPart = absValue % 100;
        sprintf(buffer, "-%ld.%02ld", (long)integerPart, (long)decimalPart);
    } else {
        integerPart = value / 100;
        decimalPart = value % 100;
        sprintf(buffer, "%ld.%02ld", (long)integerPart, (long)decimalPart);
    }
}

static HAL_StatusTypeDef APP_FramMemRead(uint16_t address, uint8_t *data, uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            APP_FRAM_ADDR,
                            address,
                            g_framMemAddrSize,
                            data,
                            length,
                            APP_FRAM_TIMEOUT_MS);
}

static HAL_StatusTypeDef APP_FramMemWrite(uint16_t address, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c2,
                               APP_FRAM_ADDR,
                               address,
                               g_framMemAddrSize,
                               (uint8_t *)data,
                               length,
                               APP_FRAM_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    /* FM24 la FRAM nen gan nhu ghi xong ngay. Neu module thuc te la AT24 EEPROM,
       no can vai ms write-cycle; cho ready o day de header/count khong bi mat sau khi ghi record. */
    for (uint8_t retry = 0U; retry < 10U; retry++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, APP_FRAM_ADDR, 1U, APP_FRAM_TIMEOUT_MS) == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(2);
    }

    return HAL_TIMEOUT;
}

static uint8_t APP_FramRecordIsValid(const FramRecord_t *record)
{
    if (record == NULL) {
        return 0U;
    }

    /* ADC STM32 12-bit chi hop le 0..4095. Gia tri 16717 = 0x414D la chu 'AM'
       trong magic cu "FRAM", nen phai loai bo. */
    if (record->adc_value > 4095U) {
        return 0U;
    }

    /* Nhiet do noi chip hop ly cho bai nay, khong chap nhan 180.02 C = 0x4652/100. */
    if ((record->temp_x100 < -2000) || (record->temp_x100 > 12000)) {
        return 0U;
    }

    return 1U;
}

static void APP_FramFormat(void)
{
    FramHeader_t header;
    uint8_t zero[16] = {0};
    uint16_t clearEnd = (uint16_t)(APP_FRAM_RECORD_ADDR + 5U * sizeof(FramRecord_t));

    /* Xoa ca vung 0x0000..record dau tien de diet du lieu cu, dac biet magic cu
       0x4652414D = "FRAM" nam o 0x0000. */
    for (uint16_t addr = 0U; addr < clearEnd; addr = (uint16_t)(addr + sizeof(zero))) {
        (void)APP_FramMemWrite(addr, zero, sizeof(zero));
        HAL_Delay(2);
    }

    header.magic = APP_FRAM_MAGIC;
    header.head = 0U;
    header.count = 0U;

    if (APP_FramMemWrite(APP_FRAM_HEADER_ADDR, (const uint8_t *)&header, sizeof(header)) == HAL_OK) {
        HAL_Delay(2);
        g_framReady = 1U;
    } else {
        g_framReady = 0U;
    }
}

static uint8_t APP_FramTryFormatWithAddrSize(uint16_t memAddrSize)
{
    FramHeader_t check;

    g_framMemAddrSize = memAddrSize;
    APP_FramFormat();

    memset(&check, 0, sizeof(check));
    if (APP_FramMemRead(APP_FRAM_HEADER_ADDR, (uint8_t *)&check, sizeof(check)) != HAL_OK) {
        return 0U;
    }

    if ((check.magic == APP_FRAM_MAGIC) && (check.head == 0U) && (check.count == 0U)) {
        g_framReady = 1U;
        return 1U;
    }

    g_framReady = 0U;
    return 0U;
}

static void APP_FramInit(void)
{
    FramHeader_t header;
    uint16_t capacity = (uint16_t)((APP_FRAM_TOTAL_BYTES - APP_FRAM_RECORD_ADDR) / sizeof(FramRecord_t));

#if (APP_FRAM_FORCE_CLEAR_ON_BOOT != 0U)
    /* Thu dia chi bo nho 8-bit truoc vi module AT24/FM24 trong kit Waveshare
       co the la dong FM24Cxx/AT24Cxx nho, neu dung 16-bit se doc nham byte 0. */
    if (APP_FramTryFormatWithAddrSize(I2C_MEMADD_SIZE_8BIT)) {
        return;
    }

    /* Neu chip cua ban la FM24CL64/AT24C64 dung dia chi o nho 16-bit, fallback o day. */
    if (APP_FramTryFormatWithAddrSize(I2C_MEMADD_SIZE_16BIT)) {
        return;
    }

    g_framReady = 0U;
    return;
#else
    g_framMemAddrSize = I2C_MEMADD_SIZE_8BIT;
    if (APP_FramMemRead(APP_FRAM_HEADER_ADDR,
                        (uint8_t *)&header,
                        sizeof(header)) == HAL_OK &&
        header.magic == APP_FRAM_MAGIC &&
        header.head < capacity &&
        header.count <= capacity) {
        g_framReady = 1U;
        return;
    }

    g_framMemAddrSize = I2C_MEMADD_SIZE_16BIT;
    if (APP_FramMemRead(APP_FRAM_HEADER_ADDR,
                        (uint8_t *)&header,
                        sizeof(header)) == HAL_OK &&
        header.magic == APP_FRAM_MAGIC &&
        header.head < capacity &&
        header.count <= capacity) {
        g_framReady = 1U;
        return;
    }

    (void)capacity;
    if (!APP_FramTryFormatWithAddrSize(I2C_MEMADD_SIZE_8BIT)) {
        (void)APP_FramTryFormatWithAddrSize(I2C_MEMADD_SIZE_16BIT);
    }
#endif
}

static void APP_FramWriteRecord(uint16_t adcValue, float tempC, uint32_t tickMs)
{
    FramHeader_t header;
    FramRecord_t record;
    uint16_t capacity = (uint16_t)((APP_FRAM_TOTAL_BYTES - APP_FRAM_RECORD_ADDR) / sizeof(FramRecord_t));
    uint16_t address;

    if ((g_framReady == 0U) || (adcValue > 4095U) || (tempC < -20.0f) || (tempC > 120.0f)) {
        return;
    }

    if (APP_FramMemRead(APP_FRAM_HEADER_ADDR, (uint8_t *)&header, sizeof(header)) != HAL_OK) {
        /* Khong format FRAM trong luc dang chay: neu I2C loi thoang qua ma format
           thi tat ca record vua luu se bien mat, gay hien tuong Pause = No records. */
        return;
    }

    if (header.magic != APP_FRAM_MAGIC ||
        header.head >= capacity ||
        header.count > capacity) {
        /* Header sai trong runtime thi bo qua lan ghi nay, KHONG goi APP_FramInit()
           vi APP_FramInit() co the format va xoa record. Header chi nen format luc boot. */
        return;
    }

    record.adc_value = adcValue;
    record.temp_x100 = (int16_t)(tempC * 100.0f);
    record.tick_ms = tickMs;

    address = (uint16_t)(APP_FRAM_RECORD_ADDR + (header.head * sizeof(FramRecord_t)));
    if (APP_FramMemWrite(address, (const uint8_t *)&record, sizeof(record)) != HAL_OK) {
        return;
    }

    header.head = (uint16_t)((header.head + 1U) % capacity);
    if (header.count < capacity) {
        header.count++;
    }

    (void)APP_FramMemWrite(APP_FRAM_HEADER_ADDR, (const uint8_t *)&header, sizeof(header));
}

static uint16_t APP_FramReadLatest(FramRecord_t *records, uint16_t maxCount)
{
    FramHeader_t header;
    FramRecord_t tempRecord;
    uint16_t capacity = (uint16_t)((APP_FRAM_TOTAL_BYTES - APP_FRAM_RECORD_ADDR) / sizeof(FramRecord_t));
    uint16_t count;
    uint16_t validCount = 0U;

    if ((records == NULL) || (maxCount == 0U) || (g_framReady == 0U)) {
        return 0U;
    }

    if (APP_FramMemRead(APP_FRAM_HEADER_ADDR, (uint8_t *)&header, sizeof(header)) != HAL_OK ||
        header.magic != APP_FRAM_MAGIC ||
        header.head >= capacity ||
        header.count > capacity) {
        return 0U;
    }

    count = header.count;
    for (uint16_t i = 0U; (i < count) && (validCount < maxCount); i++) {
        int32_t index = (int32_t)header.head - 1 - (int32_t)i;
        uint16_t address;
        while (index < 0) {
            index += capacity;
        }
        address = (uint16_t)(APP_FRAM_RECORD_ADDR + ((uint16_t)index * sizeof(FramRecord_t)));

        if (APP_FramMemRead(address, (uint8_t *)&tempRecord, sizeof(FramRecord_t)) == HAL_OK &&
            APP_FramRecordIsValid(&tempRecord)) {
            records[validCount] = tempRecord;
            validCount++;
        }
    }

    return validCount;
}

static void APP_DrawHomeScreen(void)
{
    char tempText[20];
    char adcText[20];

    APP_FormatTemp(tempText, g_tempC);
    sprintf(adcText, "%u", g_adcInternal);

    ST7789_Fill_Color(BLACK);
    ST7789_WriteString(56, 10, "OPENX05R-C RTOS", Font_7x10, CYAN, BLACK);
    ST7789_WriteString(12, 45, "Chip temperature:", Font_7x10, WHITE, BLACK);
    ST7789_WriteString(170, 45, tempText, Font_7x10, (g_tempC > APP_TEMP_THRESHOLD_C) ? RED : GREEN, BLACK);
    ST7789_WriteString(230, 45, "C", Font_7x10, WHITE, BLACK);

    ST7789_WriteString(12, 75, "ADC internal:", Font_7x10, WHITE, BLACK);
    ST7789_WriteString(170, 75, adcText, Font_7x10, YELLOW, BLACK);

    ST7789_WriteString(12, 110, "PLAY: monitor internal ADC + heat chip", Font_7x10, GRAY, BLACK);
    ST7789_WriteString(12, 128, "save ADC to FRAM if Temp > 38C", Font_7x10, GRAY, BLACK);
    ST7789_WriteString(12, 146, "PAUSE: show saved FRAM records", Font_7x10, GRAY, BLACK);

    APP_DrawButton(BTN_PLAY_X, BTN_PLAY_Y, BTN_PLAY_W, BTN_PLAY_H, "PLAY", GREEN, WHITE);
    APP_DrawButton(BTN_PAUSE_X, BTN_PAUSE_Y, BTN_PAUSE_W, BTN_PAUSE_H, "PAUSE", RED, WHITE);
}

static void APP_DrawPlayScreen(void)
{
    char tempText[20];
    char adcText[20];

    APP_FormatTemp(tempText, g_tempC);
    sprintf(adcText, "%u", g_adcInternal);

    ST7789_Fill_Color(BLACK);
    ST7789_WriteString(108, 10, "PLAY MODE", Font_7x10, YELLOW, BLACK);
    ST7789_WriteString(12, 48, "ADC internal:", Font_7x10, WHITE, BLACK);
    ST7789_WriteString(170, 48, adcText, Font_7x10, CYAN, BLACK);

    ST7789_WriteString(12, 78, "Chip temperature:", Font_7x10, WHITE, BLACK);
    ST7789_WriteString(170, 78, tempText, Font_7x10, (g_tempC > APP_TEMP_THRESHOLD_C) ? RED : GREEN, BLACK);
    ST7789_WriteString(230, 78, "C", Font_7x10, WHITE, BLACK);

    ST7789_WriteString(12, 120, "Heating STM32...", Font_7x10, YELLOW, BLACK);
    if (g_tempC > APP_TEMP_THRESHOLD_C) {
        ST7789_WriteString(12, 138, "Temp > 38C: ADC saved to FRAM", Font_7x10, RED, BLACK);
        g_playStatusLcd = 1U;
    } else {
        ST7789_WriteString(12, 138, "Waiting temperature > 38C", Font_7x10, GRAY, BLACK);
        g_playStatusLcd = 0U;
    }

    ST7789_WriteString(12, 156, "Touch PAUSE to show saved data", Font_7x10, GRAY, BLACK);
    APP_DrawButton(BTN_PLAY_X, BTN_PLAY_Y, BTN_PLAY_W, BTN_PLAY_H, "PLAY", DARKBLUE, WHITE);
    APP_DrawButton(BTN_PAUSE_X, BTN_PAUSE_Y, BTN_PAUSE_W, BTN_PAUSE_H, "PAUSE", RED, WHITE);
}

static void APP_DrawPauseScreen(void)
{
    FramRecord_t records[5];
    uint16_t count;
    char line[48];
    char tempText[20];

    ST7789_Fill_Color(BLACK);
    ST7789_WriteString(72, 10, "FRAM OVER-THRESHOLD", Font_7x10, YELLOW, BLACK);
    ST7789_WriteString(110, 24, "RECORDS", Font_7x10, YELLOW, BLACK);

    count = APP_FramReadLatest(records, 5);
    if (count == 0U) {
        ST7789_WriteString(24, 80, "No record stored in FRAM yet", Font_7x10, WHITE, BLACK);
    } else {
        for (uint16_t i = 0; i < count; i++) {
            APP_FormatTemp(tempText, (float)records[i].temp_x100 / 100.0f);
            sprintf(line, "%u) ADC=%u  T=%sC", (unsigned)(i + 1U), records[i].adc_value, tempText);
            ST7789_WriteString(12, (uint16_t)(55U + i * 22U), line, Font_7x10, WHITE, BLACK);
        }
    }

    ST7789_WriteString(44, 210, "Auto return HOME after 10s", Font_7x10, GRAY, BLACK);
}

static void APP_UpdateHomeValues(void)
{
    char tempText[20];
    char adcText[20];

    APP_FormatTemp(tempText, g_tempC);
    sprintf(adcText, "%u", g_adcInternal);

    APP_FillRect(168, 43, 90, 12, BLACK);
    ST7789_WriteString(170, 45, tempText, Font_7x10, (g_tempC > APP_TEMP_THRESHOLD_C) ? RED : GREEN, BLACK);
    ST7789_WriteString(230, 45, "C", Font_7x10, WHITE, BLACK);

    APP_FillRect(168, 73, 90, 12, BLACK);
    ST7789_WriteString(170, 75, adcText, Font_7x10, YELLOW, BLACK);
}

static void APP_UpdatePlayValues(void)
{
    char tempText[20];
    char adcText[20];
    uint8_t status = (g_tempC > APP_TEMP_THRESHOLD_C) ? 1U : 0U;

    APP_FormatTemp(tempText, g_tempC);
    sprintf(adcText, "%u", g_adcInternal);

    APP_FillRect(168, 46, 90, 12, BLACK);
    ST7789_WriteString(170, 48, adcText, Font_7x10, CYAN, BLACK);

    APP_FillRect(168, 76, 90, 12, BLACK);
    ST7789_WriteString(170, 78, tempText, Font_7x10, status ? RED : GREEN, BLACK);
    ST7789_WriteString(230, 78, "C", Font_7x10, WHITE, BLACK);

    if (status != g_playStatusLcd) {
        APP_FillRect(12, 136, 300, 14, BLACK);
        if (status) {
            ST7789_WriteString(12, 138, "Temp > 38C: ADC saved to FRAM", Font_7x10, RED, BLACK);
        } else {
            ST7789_WriteString(12, 138, "Waiting temperature > 38C", Font_7x10, GRAY, BLACK);
        }
        g_playStatusLcd = status;
    }
}
/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  APP_FramInit();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, StartTask01, osPriorityLow, 0, 256);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityNormal, 0, 384);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, StartTask03, osPriorityAboveNormal, 0, 768);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|LCD_RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin, GPIO_PIN_SET);

  /* Keep touch controller deselected while LCD uses the shared SPI bus */
//  HAL_GPIO_WritePin(LCD_PA9_GPIO_Port, LCD_PA9_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB6 LCD_CS_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_6|LCD_CS_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB0/PB1 no longer used for external ADC in this version */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_PA9_Pin */
//  GPIO_InitStruct.Pin = LCD_PA9_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LCD_PA9_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 as touch IRQ input */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
 * @brief  Function implementing the Task01 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN 5 */
    (void)argument;
    while (1) {
        if (g_appMode == APP_MODE_PLAY) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
            APP_RunHeatChunk();
            osThreadYield();
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
            osDelay(50);
        }
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the Task02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
    (void)argument;
    while (1) {
        float temperature;
        uint16_t adcValue;
        uint32_t now = HAL_GetTick();

        if (APP_ReadInternalSensors(&temperature, &adcValue)) {
            g_tempC = temperature;
            g_adcInternal = adcValue;

            if ((g_appMode == APP_MODE_PLAY) && (temperature > APP_TEMP_THRESHOLD_C)) {
                if ((now - g_lastFramSaveTick) >= APP_FRAM_SAVE_PERIOD_MS) {
                    APP_FramWriteRecord(adcValue, temperature, now);
                    g_lastFramSaveTick = now;
                }
            }
        }

        osDelay(APP_SENSOR_POLL_MS);
    }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the Task03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
    uint16_t touchX = 0U, touchY = 0U;
    uint16_t stableX = 0U, stableY = 0U;
    uint8_t pressCount = 0U;
    uint8_t releaseCount = APP_TOUCH_RELEASE_COUNT;
    uint8_t touchLatched = 0U;
    uint8_t waitTouchRelease = 0U;
    uint32_t now;
    uint32_t lastLcdUpdate = 0U;
    uint32_t pauseStartTick = 0U;
    AppMode_t lastMode = APP_MODE_HOME;
    float lastTemp = -1000.0f;
    uint16_t lastAdc = 0xFFFFU;
    (void)argument;

    APP_TouchUnselect();
    ST7789_Init();
    APP_DrawHomeScreen();

    while (1)
    {
        now = HAL_GetTick();

        if (g_appMode == APP_MODE_PAUSE) {
            uint8_t touched = APP_TouchGetXY(&touchX, &touchY);

            if (waitTouchRelease != 0U) {
                pressCount = 0U;
                touchLatched = 1U;
                if (!touched) {
                    waitTouchRelease = 0U;
                    touchLatched = 0U;
                    releaseCount = APP_TOUCH_RELEASE_COUNT;
                }
            } else {
                pressCount = 0U;
                if (releaseCount < 0xFFU) {
                    releaseCount++;
                }
                if (releaseCount >= APP_TOUCH_RELEASE_COUNT) {
                    touchLatched = 0U;
                }
            }

            if ((now - pauseStartTick) >= APP_PAUSE_DISPLAY_MS) {
                g_appMode = APP_MODE_HOME;
                waitTouchRelease = touched ? 1U : 0U;
                touchLatched = touched ? 1U : 0U;
                pressCount = 0U;
                releaseCount = touched ? 0U : APP_TOUCH_RELEASE_COUNT;
            }
        } else {
            uint8_t touched = APP_TouchGetXY(&touchX, &touchY);

            if (waitTouchRelease != 0U) {
                pressCount = 0U;
                touchLatched = 1U;
                if (!touched) {
                    waitTouchRelease = 0U;
                    touchLatched = 0U;
                    releaseCount = APP_TOUCH_RELEASE_COUNT;
                }
            } else if (touched) {
                stableX = touchX;
                stableY = touchY;
                releaseCount = 0U;
                if (pressCount < 0xFFU) {
                    pressCount++;
                }

                if ((touchLatched == 0U) &&
                    (pressCount >= APP_TOUCH_CONFIRM_COUNT) &&
                    ((now - g_lastTouchTick) >= APP_TOUCH_DEBOUNCE_MS)) {
                    g_lastTouchTick = now;
                    touchLatched = 1U;

                    if (APP_PointInRect(stableX, stableY,
                                        (int16_t)(BTN_PLAY_X - APP_TOUCH_HIT_MARGIN),
                                        (int16_t)(BTN_PLAY_Y - APP_TOUCH_HIT_MARGIN),
                                        (int16_t)(BTN_PLAY_W + 2 * APP_TOUCH_HIT_MARGIN),
                                        (int16_t)(BTN_PLAY_H + 2 * APP_TOUCH_HIT_MARGIN))) {
                        g_appMode = APP_MODE_PLAY;
                    } else if (APP_PointInRect(stableX, stableY,
                                               (int16_t)(BTN_PAUSE_X - APP_TOUCH_HIT_MARGIN),
                                               (int16_t)(BTN_PAUSE_Y - APP_TOUCH_HIT_MARGIN),
                                               (int16_t)(BTN_PAUSE_W + 2 * APP_TOUCH_HIT_MARGIN),
                                               (int16_t)(BTN_PAUSE_H + 2 * APP_TOUCH_HIT_MARGIN))) {
                        g_appMode = APP_MODE_PAUSE;
                        waitTouchRelease = 1U;
                        pressCount = 0U;
                        releaseCount = APP_TOUCH_RELEASE_COUNT;
                        touchLatched = 1U;
                    }
                }
            } else {
                pressCount = 0U;
                if (releaseCount < 0xFFU) {
                    releaseCount++;
                }
                if (releaseCount >= APP_TOUCH_RELEASE_COUNT) {
                    touchLatched = 0U;
                }
            }
        }

        if (g_appMode != lastMode) {
            if (g_appMode == APP_MODE_HOME) {
                APP_DrawHomeScreen();
            } else if (g_appMode == APP_MODE_PLAY) {
                g_playStatusLcd = 0xFFU;
                APP_DrawPlayScreen();
            } else {
                pauseStartTick = now;
                APP_DrawPauseScreen();
            }

            lastMode = g_appMode;
            lastTemp = -1000.0f;
            lastAdc = 0xFFFFU;
            lastLcdUpdate = now;
        }

        if (((now - lastLcdUpdate) >= APP_LCD_UPDATE_MS) && (g_appMode == APP_MODE_HOME)) {
            float diff = g_tempC - lastTemp;
            if (diff < 0.0f) diff = -diff;
            if ((diff >= 0.10f) || (g_adcInternal != lastAdc)) {
                APP_UpdateHomeValues();
                lastTemp = g_tempC;
                lastAdc = g_adcInternal;
            }
            lastLcdUpdate = now;
        } else if (((now - lastLcdUpdate) >= APP_LCD_UPDATE_MS) && (g_appMode == APP_MODE_PLAY)) {
            float diff = g_tempC - lastTemp;
            if (diff < 0.0f) diff = -diff;
            if ((diff >= 0.10f) || (g_adcInternal != lastAdc)) {
                APP_UpdatePlayValues();
                lastTemp = g_tempC;
                lastAdc = g_adcInternal;
            }
            lastLcdUpdate = now;
        }

        osDelay(APP_TOUCH_POLL_MS);
    }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
