/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "lcd_i2c.h"
#include "usbd_cdc_if.h" // Diperlukan untuk VCP
#include "HCSR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Struktur untuk state kontrol pengisian (histeresis)
typedef enum
{
  STATE_IDLE,
  STATE_FILLING,
  STATE_FILLING_B, // isi drum B
  STATE_FILLING_C  // isi drum C
} FillState;

// Struktur untuk menyimpan semua parameter yang dapat dikonfigurasi
typedef struct
{
  uint32_t magic_number; // Penanda untuk verifikasi data di Flash

  float tinggi_A;
  float ambang_bawah_A;
  float target_penuh_A;

  float tinggi_B;
  float ambang_bawah_B;
  float target_penuh_B;

  float tinggi_C;
  float ambang_bawah_C;
  float target_penuh_C;

  float sumber_kosong;

  uint8_t moving_avg_size; // Jumlah nilai yang akan dirata-rata
} ControlParams;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_SIZE 6         // Define the size of the first-stage sampling
#define MAX_MOVING_AVG_SIZE 6 // Ukuran maksimum buffer

#define GUNAKAN_LCD 1     // Aktifkan jika menggunakan LCD I2C
#define GUNAKAN_SD_CARD 0 // Aktifkan jika menggunakan SD Card

#define HCSR04_SENSOR1 0
#define HCSR04_SENSOR2 1
#define HCSR04_SENSOR3 2

#define NUM_SENSORS 3

// Definisi Konstanta Kontrol Drum
#define TINGGI_A 85.0f // dalam cm
#define TINGGI_B 85.0f // dalam cm
#define TINGGI_C 85.0f // dalam cm

#define AMBANG_BAWAH 25.0f // Level 25% untuk memicu pengisian
#define TARGET_PENUH 95.0f // Target 95% untuk menghentikan pengisian
#define SUMBER_KOSONG 1.0f // Anggap sumber kosong jika level <= 1%

// Alamat di memori Flash untuk menyimpan parameter.
// Pilih sektor terakhir agar aman dari kode program utama.
// Untuk STM32F103C8 (64KB), alamat page terakhir adalah 0x0800FC00.
#define FLASH_STORAGE_ADDRESS 0x0800FC00
#define FLASH_MAGIC_NUMBER 0xDEADBEEF // Penanda data valid
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// Variables for the new two-stage filter
float sample_buffer1[SAMPLE_SIZE];
float sample_buffer2[SAMPLE_SIZE];
float sample_buffer3[SAMPLE_SIZE];
uint8_t sample_counter = 0;

// New buffers for the two-stage filter
float Stage2_buffer1[MAX_MOVING_AVG_SIZE];
float Stage2_buffer2[MAX_MOVING_AVG_SIZE];
float Stage2_buffer3[MAX_MOVING_AVG_SIZE];

uint8_t stage2_index1 = 0;
uint8_t stage2_index2 = 0;
uint8_t stage2_index3 = 0;

uint8_t is_stage2_full1 = 0;
uint8_t is_stage2_full2 = 0;
uint8_t is_stage2_full3 = 0;

// variabel moving average
float Distance1_buffer[MAX_MOVING_AVG_SIZE];
float Distance2_buffer[MAX_MOVING_AVG_SIZE];
float Distance3_buffer[MAX_MOVING_AVG_SIZE];

float filtered_distance1, filtered_distance2, filtered_distance3;

uint8_t buffer_index_1 = 0;
uint8_t buffer_index_2 = 0;
uint8_t buffer_index_3 = 0;

uint8_t is_buffer_full_1 = 0;
uint8_t is_buffer_full_2 = 0;
uint8_t is_buffer_full_3 = 0;

// Buffer untuk LCD (misalnya, 20 karakter)
char lcd_buffer[20];

// Variabel untuk level air
float LevelA_persen = 0, LevelB_persen = 0, LevelC_persen = 0;

// Variabel state untuk kontrol histeresis
FillState state_pompa_A = STATE_IDLE;
FillState state_pompa_B = STATE_IDLE;

// Variabel global untuk menyimpan parameter
ControlParams g_params;

// Variabel untuk CLI via USB VCP
#define RX_BUFFER_SIZE 128
uint8_t g_usb_rx_buffer[RX_BUFFER_SIZE];
volatile uint32_t g_rx_index = 0;
volatile uint8_t g_command_ready = 0;

// variabel untuk sensor ultrasonik
uint16_t TRIG_Ticks = 0;
uint16_t LCD_Ticks = 0;
float Distance1 = 0.0, Distance2 = 0.0, Distance3 = 0.0;
char TEXT_L1[16] = {0};
char TEXT_L2[16] = {0};

// Timers for non-blocking loops
uint32_t last_trig_time = 0;
uint32_t trig_time = 25;
uint8_t counter_trig = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Run_Control_Logic(void);
void Update_LCD_Display(void);
void Log_Data_To_SD(void);

// Fungsi untuk CLI dan Parameter
void VCP_printf(const char *format, ...);
void Process_Command(uint8_t *cmd_buffer);
void Save_Parameters_To_Flash(void);
void Load_Parameters_From_Flash(void);
void Set_Default_Parameters(void);
float calculate_moving_average(float *buffer, uint8_t size, uint8_t is_full);
float calculate_weighted_moving_average(float *buffer, uint8_t size, uint8_t is_full);
float find_max_in_sample(float *sample_buffer, float tinggi_drum);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
#if GUNAKAN_LCD
  lcd_init(&hi2c1); // Inisialisasi LCD I2C
  lcd_clear();      // Bersihkan LCD

  // pesan awal di LCD
  lcd_set_cursor(0, 0);
  sprintf(lcd_buffer, "hai dunia");
  lcd_send_string(lcd_buffer);
#endif

  // inisialisasi sensor ultrasonik
  HCSR04_Init(HCSR04_SENSOR1, &htim2);
  HCSR04_Init(HCSR04_SENSOR2, &htim3);
  HCSR04_Init(HCSR04_SENSOR3, &htim4);

  // Muat parameter dari Flash saat startup
  Load_Parameters_From_Flash();

  // Beri sedikit waktu agar USB siap
  HAL_Delay(2000);
  VCP_printf("\r\nSistem Kontrol Drum Siap. Ketik '$H' untuk bantuan.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Distance1 = HCSR04_Read(HCSR04_SENSOR1);
    Distance2 = HCSR04_Read(HCSR04_SENSOR2);
    Distance3 = HCSR04_Read(HCSR04_SENSOR3);

    uint32_t current_time = HAL_GetTick();

    // --- Task-like functions execution ---

    // Run HCSR04_Trigger every 100ms
    if (current_time - last_trig_time >= trig_time)
    {
      // if (counter_trig == 1)
      HCSR04_Trigger(HCSR04_SENSOR1);
      // else if (counter_trig == 2)
      HCSR04_Trigger(HCSR04_SENSOR2);
      // else if (counter_trig == 3)
      HCSR04_Trigger(HCSR04_SENSOR3);
      // else
      //   counter_trig = 0;
      // counter_trig++;
      last_trig_time = current_time;
    }

    // Cek apakah ada perintah baru dari USB VCP untuk diproses
    if (g_command_ready)
    {
      Process_Command(g_usb_rx_buffer);
      // Reset buffer dan flag
      memset(g_usb_rx_buffer, 0, RX_BUFFER_SIZE);
      g_rx_index = 0;
      g_command_ready = 0;
    }

    Run_Control_Logic();

#if GUNAKAN_LCD == 1
    Update_LCD_Display();
#endif

    HAL_Delay(100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
   */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USER_LED_Pin | LED_GREEN_Pin | LED_RED_Pin | POMPA_BA_Pin | POMPA_CB_Pin | TRIG_1_Pin | TRIG_2_Pin | TRIG_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POMPA_1_ON_Pin POMPA_2_ON_Pin */
  GPIO_InitStruct.Pin = POMPA_1_ON_Pin | POMPA_2_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_Pin LED_GREEN_Pin LED_RED_Pin POMPA_BA_Pin
                           POMPA_CB_Pin TRIG_1_Pin TRIG_2_Pin TRIG_3_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin | LED_GREEN_Pin | LED_RED_Pin | POMPA_BA_Pin | POMPA_CB_Pin | TRIG_1_Pin | TRIG_2_Pin | TRIG_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Function to find the maximum value from a small sample
float find_max_in_sample(float *sample_buffer, float tinggi_drum)
{
  float max_val = 0.0f;
  for (int i = 0; i < SAMPLE_SIZE; i++)
  {
    if (sample_buffer[i] > max_val)
    {
      max_val = sample_buffer[i];
    }
  }

  if (max_val > tinggi_drum)
  {
    max_val = tinggi_drum; // jangan sampai nilainya melebihi max tinggi A
  }

  return max_val;
}

// kalkukasi moving average
float calculate_moving_average(float *buffer, uint8_t size, uint8_t is_full)
{
  float sum = 0.0f;
  uint8_t count = is_full ? size : buffer_index_1; // Gunakan jumlah data yang ada

  if (count == 0)
    return 0.0f;

  for (int i = 0; i < count; i++)
  {
    sum += buffer[i];
  }
  return sum / count;
}

float calculate_weighted_moving_average(float *buffer, uint8_t size, uint8_t is_full)
{
  float sum_weighted_values = 0.0f;
  float sum_of_weights = 0.0f;
  uint8_t count = is_full ? size : buffer_index_1;

  if (count == 0)
  {
    return 0.0f;
  }

  for (int i = 0; i < count; i++)
  {
    // Simple linear weighting: use the value itself as the weight.
    // You can adjust this to your needs. For example, add a base value
    // to prevent zero weights.
    float weight = buffer[i];

    // Or if you want a more explicit weighting:
    // float weight = 1.0f + (buffer[i] / 10.0f); // Example: a value of 90 has a weight of 10

    sum_weighted_values += buffer[i] * weight;
    sum_of_weights += weight;
  }

  if (sum_of_weights == 0.0f)
  {
    return 0.0f;
  }

  return sum_weighted_values / sum_of_weights;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  HCSR04_TMR_IC_ISR(htim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HCSR04_TMR_OVF_ISR(htim);
}

/**
 * @brief Mengirim string terformat melalui USB VCP.
 */
void VCP_printf(const char *format, ...)
{
  char buf[126];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  CDC_Transmit_FS((uint8_t *)buf, strlen(buf));
}

/**
 * @brief Mengatur parameter ke nilai default.
 */
void Set_Default_Parameters(void)
{
  g_params.magic_number = FLASH_MAGIC_NUMBER;
  g_params.tinggi_A = 85.0f;
  g_params.ambang_bawah_A = 70.0f;
  g_params.target_penuh_A = 74.0f;
  g_params.tinggi_B = 85.0f;
  g_params.ambang_bawah_B = 70.0f;
  g_params.target_penuh_B = 74.0f;
  g_params.tinggi_C = 85.0f;
  g_params.ambang_bawah_C = 70.0f;
  g_params.target_penuh_C = 74.0f;
  g_params.sumber_kosong = 1.0f;
  g_params.moving_avg_size = 30; // Nilai default 5
}

/**
 * @brief Menyimpan parameter ke Flash.
 */
void Save_Parameters_To_Flash(void)
{
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_STORAGE_ADDRESS;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    VCP_printf("Error: Gagal menghapus Flash!\r\n");
    HAL_FLASH_Lock();
    return;
  }

  uint32_t address = FLASH_STORAGE_ADDRESS;
  uint32_t *data_ptr = (uint32_t *)&g_params;
  for (size_t i = 0; i < sizeof(ControlParams) / 4; i++)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data_ptr[i]) != HAL_OK)
    {
      VCP_printf("Error: Gagal menulis ke Flash!\r\n");
      break;
    }
    address += 4;
  }

  HAL_FLASH_Lock();
  VCP_printf("OK: Parameter berhasil disimpan ke Flash.\r\n");
}

/**
 * @brief Memuat parameter dari Flash. Jika gagal, gunakan nilai default.
 */
void Load_Parameters_From_Flash(void)
{
  ControlParams params_from_flash;
  memcpy(&params_from_flash, (void *)FLASH_STORAGE_ADDRESS,
         sizeof(ControlParams));

  if (params_from_flash.magic_number == FLASH_MAGIC_NUMBER)
  {
    memcpy(&g_params, &params_from_flash, sizeof(ControlParams));
    // VCP belum tentu siap di sini, jadi jangan print
  }
  else
  {
    Set_Default_Parameters();
  }
}

/**
 * @brief Memproses perintah yang diterima dari USB VCP.
 */
void Process_Command(uint8_t *cmd_buffer)
{
  char *cmd = (char *)cmd_buffer;
  float min, max;
  uint16_t val;

  if (strncmp(cmd, "$$", 2) == 0)
  {
    char large_buffer[2048]; // Gunakan buffer yang lebih besar
    int len = 0;

    len += sprintf(large_buffer + len, "--- Parameter Saat Ini ---\r\n");
    len += sprintf(large_buffer + len, "Drum A: Tinggi=%.1f, Min=%.1f%%, Max=%.1f%%\r\n", g_params.tinggi_A, g_params.ambang_bawah_A, g_params.target_penuh_A);
    len += sprintf(large_buffer + len, "Drum B: Tinggi=%.1f, Min=%.1f%%, Max=%.1f%%\r\n", g_params.tinggi_B, g_params.ambang_bawah_B, g_params.target_penuh_B);
    len += sprintf(large_buffer + len, "Drum C: Tinggi=%.1f, Min=%.1f%%, Max=%.1f%%\r\n", g_params.tinggi_C, g_params.ambang_bawah_C, g_params.target_penuh_C);
    len += sprintf(large_buffer + len, "Ambang Sumber Kosong: %.1f%%\r\n", g_params.sumber_kosong);

    CDC_Transmit_FS((uint8_t *)large_buffer, len);
  }
  else if (strncmp(cmd, "$P", 2) == 0)
  {
    char large_buffer[2048]; // Gunakan buffer yang lebih besar
    int len = 0;

    len += sprintf(large_buffer + len, "--- Daftar Pin ---\r\n");
    len += sprintf(large_buffer + len, "Sensor A (TIM2_CH1): PB3 (TRIG), PA15 (ECHO)\r\n");
    len += sprintf(large_buffer + len, "Sensor B (TIM3_CH1): PB5 (TRIG), PB4 (ECHO)\r\n");
    len += sprintf(large_buffer + len, "Sensor C (TIM4_CH1): PB7 (TRIG), PB6 (ECHO)\r\n");
    len += sprintf(large_buffer + len, "Pompa B->A: PB14\r\n");
    len += sprintf(large_buffer + len, "Pompa C->B: PB15\r\n");
    len += sprintf(large_buffer + len, "LED Merah (Alert): PB13\r\n");
    len += sprintf(large_buffer + len, "LCD I2C (I2C1): PB6 (SCL), PB7 (SDA)\r\n");

    CDC_Transmit_FS((uint8_t *)large_buffer, len);
  }
  else if (strncmp(cmd, "$H", 2) == 0)
  {
    char large_buffer[2048]; // Gunakan buffer yang lebih besar
    int len = 0;

    len += sprintf(large_buffer + len, "--- Bantuan ---\r\n");
    len += sprintf(large_buffer + len, "$$          : Lihat semua parameter\r\n");
    len += sprintf(large_buffer + len, "$P          : Lihat daftar pin\r\n");
    len += sprintf(large_buffer + len, "$H          : Tampilkan pesan ini\r\n");
    len += sprintf(large_buffer + len, "$1=<min>,<max> : Set ambang batas Drum A\r\n");
    len += sprintf(large_buffer + len, "$2=<min>,<max> : Set ambang batas Drum B\r\n");
    len += sprintf(large_buffer + len, "$3=<min>,<max> : Set ambang batas Drum C\r\n");
    len += sprintf(large_buffer + len, "$M=<num>       : Set nilai mov avrg sensor\r\n");
    len += sprintf(large_buffer + len, "$S          : Simpan parameter ke Flash\r\n");
    len += sprintf(large_buffer + len, "$L          : Muat parameter dari Flash\r\n");
    len += sprintf(large_buffer + len, "$D          : Kembalikan ke pengaturan default\r\n");

    CDC_Transmit_FS((uint8_t *)large_buffer, len);
  }
  else if (sscanf(cmd, "$1=%f,%f", &min, &max) == 2)
  {
    g_params.ambang_bawah_A = min;
    g_params.target_penuh_A = max;
    VCP_printf("OK: Parameter Drum A diubah -> Min=%.1f, Max=%.1f\r\n", min, max);
  }
  else if (sscanf(cmd, "$2=%f,%f", &min, &max) == 2)
  {
    g_params.ambang_bawah_B = min;
    g_params.target_penuh_B = max;
    VCP_printf("OK: Parameter Drum B diubah -> Min=%.1f, Max=%.1f\r\n", min, max);
  }
  else if (sscanf(cmd, "$3=%f,%f", &min, &max) == 2)
  {
    g_params.ambang_bawah_C = min;
    g_params.target_penuh_C = max;
    VCP_printf("OK: Parameter Drum C diubah -> Min=%.1f, Max=%.1f\r\n", min, max);
  }
  else if (sscanf(cmd, "$M=%hd", &val) == 1)
  {
    if (val > 0 && val <= MAX_MOVING_AVG_SIZE)
    {
      g_params.moving_avg_size = (uint8_t)val;
      VCP_printf("OK: Ukuran Moving Average diubah -> %d\r\n", val);
      // Reset buffers setelah mengubah ukuran
      memset(Distance1_buffer, 0, sizeof(Distance1_buffer));
      memset(Distance2_buffer, 0, sizeof(Distance2_buffer));
      memset(Distance3_buffer, 0, sizeof(Distance3_buffer));
      buffer_index_1 = 0;
      buffer_index_2 = 0;
      buffer_index_3 = 0;
      is_buffer_full_1 = 0;
      is_buffer_full_2 = 0;
      is_buffer_full_3 = 0;
    }
    else
    {
      VCP_printf("Error: Ukuran Moving Average tidak valid (1-%d).\r\n", MAX_MOVING_AVG_SIZE);
    }
  }
  else if (strncmp(cmd, "$S", 2) == 0)
  {
    Save_Parameters_To_Flash();
    VCP_printf("OK: Parameter disimpan ke Flash.\r\n");
  }
  else if (strncmp(cmd, "$L", 2) == 0)
  {
    Load_Parameters_From_Flash();
    VCP_printf("OK: Parameter dimuat dari Flash.\r\n");
  }
  else if (strncmp(cmd, "$D", 2) == 0)
  {
    Set_Default_Parameters();
    VCP_printf("OK: Parameter dikembalikan ke default.\r\n");
  }
  else
  {
    VCP_printf(
        "Error: Perintah tidak dikenali. Ketik '$H' untuk bantuan.\r\n");
  }
}

void Run_Control_Logic(void)
{
  static uint8_t stage2_count1 = 0;
  static uint8_t stage2_count2 = 0;
  static uint8_t stage2_count3 = 0;

  // Collect 8 samples before processing
  sample_buffer1[sample_counter] = Distance1;
  sample_buffer2[sample_counter] = Distance2;
  sample_buffer3[sample_counter] = Distance3;
  sample_counter++;

  // Only process the filter when 8 samples are collected
  if (sample_counter >= SAMPLE_SIZE)
  {
    sample_counter = 0; // Reset the counter

    // --- Stage 1: Find the maximum value from the 8 samples ---
    float max_distance1 = find_max_in_sample(sample_buffer1, g_params.tinggi_A);
    float max_distance2 = find_max_in_sample(sample_buffer2, g_params.tinggi_B);
    float max_distance3 = find_max_in_sample(sample_buffer3, g_params.tinggi_C);

    // --- Stage 2: Push the max value into the WMA buffer ---
    Stage2_buffer1[stage2_index1] = max_distance1;
    stage2_index1 = (stage2_index1 + 1) % g_params.moving_avg_size;
    if (stage2_count1 < g_params.moving_avg_size)
    {
      stage2_count1++;
      if (stage2_count1 >= g_params.moving_avg_size)
      {
        is_stage2_full1 = 1;
      }
    }
    filtered_distance1 = calculate_moving_average(Stage2_buffer1, g_params.moving_avg_size, is_stage2_full1);

    Stage2_buffer2[stage2_index2] = max_distance2;
    stage2_index2 = (stage2_index2 + 1) % g_params.moving_avg_size;
    if (stage2_count2 < g_params.moving_avg_size)
    {
      stage2_count2++;
      if (stage2_count2 >= g_params.moving_avg_size)
      {
        is_stage2_full2 = 1;
      }
    }
    filtered_distance2 = calculate_moving_average(Stage2_buffer2, g_params.moving_avg_size, is_stage2_full2);

    Stage2_buffer3[stage2_index3] = max_distance3;
    stage2_index3 = (stage2_index3 + 1) % g_params.moving_avg_size;
    if (stage2_count3 < g_params.moving_avg_size)
    {
      stage2_count3++;
      if (stage2_count3 >= g_params.moving_avg_size)
      {
        is_stage2_full3 = 1;
      }
    }
    filtered_distance3 = calculate_moving_average(Stage2_buffer3, g_params.moving_avg_size, is_stage2_full3);
  }

  // Gunakan nilai jarak yang sudah difilter
  float LevelA_cm = g_params.tinggi_A - filtered_distance1;
  float LevelB_cm = g_params.tinggi_B - filtered_distance2;
  float LevelC_cm = g_params.tinggi_C - filtered_distance3;

  LevelA_persen = (LevelA_cm / g_params.tinggi_A) * 100.0f;
  LevelB_persen = (LevelB_cm / g_params.tinggi_B) * 100.0f;
  LevelC_persen = (LevelC_cm / g_params.tinggi_C) * 100.0f;
  if (LevelA_persen < 0)
    LevelA_persen = 0;
  else if (LevelA_persen > 100)
    LevelA_persen = 100;
  if (LevelB_persen < 0)
    LevelB_persen = 0;
  else if (LevelB_persen > 100)
    LevelB_persen = 100;
  if (LevelC_persen < 0)
    LevelC_persen = 0;
  else if (LevelC_persen > 100)
    LevelC_persen = 100;

  // --- LED merah jika salah satu drum di bawah ambang ---
  if (LevelA_persen <= g_params.ambang_bawah_A ||
      LevelB_persen <= g_params.ambang_bawah_B ||
      LevelC_persen <= g_params.ambang_bawah_C)
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  // --- Kontrol POMPA_BA (drum A) ---
  if (state_pompa_A == STATE_IDLE)
  {
    if (LevelA_persen <= g_params.ambang_bawah_A &&
        HAL_GPIO_ReadPin(POMPA_1_ON_GPIO_Port, POMPA_1_ON_Pin) == GPIO_PIN_RESET) // tombol ditekan
    {
      state_pompa_A = STATE_FILLING;
      HAL_GPIO_WritePin(POMPA_BA_GPIO_Port, POMPA_BA_Pin, GPIO_PIN_SET);
    }
  }
  else if (state_pompa_A == STATE_FILLING)
  {
    if (LevelA_persen >= g_params.target_penuh_A)
    {
      state_pompa_A = STATE_IDLE;
      HAL_GPIO_WritePin(POMPA_BA_GPIO_Port, POMPA_BA_Pin, GPIO_PIN_RESET);
    }
  }

  // --- Kontrol POMPA_CB (drum B dan drum C) ---
  if (state_pompa_B == STATE_IDLE)
  {
    if (LevelB_persen <= g_params.ambang_bawah_B &&
        HAL_GPIO_ReadPin(POMPA_2_ON_GPIO_Port, POMPA_2_ON_Pin) == GPIO_PIN_RESET) // tombol ditekan
    {
      state_pompa_B = STATE_FILLING_B;
      HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_SET);
    }
    else if (LevelC_persen <= g_params.ambang_bawah_C &&
             HAL_GPIO_ReadPin(POMPA_2_ON_GPIO_Port, POMPA_2_ON_Pin) == GPIO_PIN_RESET) // tombol ditekan
    {
      state_pompa_B = STATE_FILLING_C;
      HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_SET);
    }
  }
  else if (state_pompa_B == STATE_FILLING_B)
  {
    if (LevelB_persen >= g_params.target_penuh_B)
    {
      state_pompa_B = STATE_IDLE;
      HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_RESET);
    }
  }
  else if (state_pompa_B == STATE_FILLING_C)
  {
    if (LevelC_persen >= g_params.target_penuh_C)
    {
      state_pompa_B = STATE_IDLE;
      HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_RESET);
    }
  }

  // --- LED hijau jika idle, mati jika menyalakan pompa ---
  if (state_pompa_A == STATE_IDLE && state_pompa_B == STATE_IDLE)
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
}

void Update_LCD_Display(void)
{
  lcd_set_cursor(0, 0);
  sprintf(lcd_buffer, "%3.0f%% %3.0f%% %3.0f%%", LevelA_persen, LevelB_persen, LevelC_persen);
  lcd_send_string(lcd_buffer);

  lcd_set_cursor(1, 0);
  char status[17] = {0};

  if (state_pompa_A == STATE_IDLE && state_pompa_B == STATE_IDLE)
  {
    // Cek apakah ada drum yang habis
    if (LevelA_persen <= g_params.ambang_bawah_A)
      strcpy(status, "drum A habis    ");
    else if (LevelB_persen <= g_params.ambang_bawah_B)
      strcpy(status, "drum B habis    ");
    else if (LevelC_persen <= g_params.ambang_bawah_C)
      strcpy(status, "drum C habis    ");
    else
      strcpy(status, "idle...         ");
  }
  else
  {
    // Pompa sedang bekerja
    if (state_pompa_A == STATE_FILLING)
      strcpy(status, "isi drum A      ");
    else if (state_pompa_B == STATE_FILLING_B)
      strcpy(status, "isi drum B      ");
    else if (state_pompa_B == STATE_FILLING_C)
      strcpy(status, "isi drum C      ");
  }

  lcd_set_cursor(1, 0);
  lcd_send_string(status);
}

/**
 * @brief  Callback ini dipanggil dari usbd_cdc_if.c setiap kali data diterima dari USB VCP.
 */
void CDC_On_Receive(uint8_t *Buf, uint32_t Len)
{
  for (uint32_t i = 0; i < Len; i++)
  {
    // Cek jika buffer penuh
    if (g_rx_index >= RX_BUFFER_SIZE - 1)
    {
      g_rx_index = 0; // Reset jika overflow
      memset(g_usb_rx_buffer, 0, RX_BUFFER_SIZE);
    }

    // Echo karakter kembali ke terminal
    CDC_Transmit_FS(&Buf[i], 1);

    // Jika menerima Enter (CR) atau Newline (LF)
    if (Buf[i] == '\r' || Buf[i] == '\n')
    {
      // Abaikan jika buffer kosong
      if (g_rx_index > 0)
      {
        g_usb_rx_buffer[g_rx_index] = '\0'; // Null-terminate string
        g_command_ready = 1;                // Set flag untuk diproses di main loop
      }
    }
    else
    {
      g_usb_rx_buffer[g_rx_index++] = Buf[i]; // Tambahkan karakter ke buffer
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
