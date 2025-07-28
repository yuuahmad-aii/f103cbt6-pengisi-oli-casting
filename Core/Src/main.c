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
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Struktur untuk menangani state dan data dari setiap sensor
typedef enum
{
  STATE_WAIT_FOR_START,
  STATE_READ_H_DATA,
  STATE_READ_L_DATA,
  STATE_READ_SUM
} UART_Receive_State;

typedef struct
{
  UART_HandleTypeDef *huart;  // Handle UART yang digunakan
  uint8_t rx_buffer;          // Buffer untuk 1 byte data masuk
  UART_Receive_State state;   // State machine untuk parsing
  uint8_t raw_data[4];        // Buffer untuk frame data [0xFF, H, L, SUM]
  uint16_t distance;          // Hasil jarak dalam mm
  uint8_t new_data_available; // Flag penanda data baru
} Sensor_HandleTypeDef;

// Struktur untuk state kontrol pengisian (histeresis)
typedef enum
{
  STATE_IDLE,
  STATE_FILLING
} FillState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GUNAKAN_LCD 0     // Aktifkan jika menggunakan LCD I2C
#define GUNAKAN_SD_CARD 0 // Aktifkan jika menggunakan SD Card

#define NUM_SENSORS 3
#define START_BYTE 0xFF

// Definisi Konstanta Kontrol Drum
#define TINGGI_A 100.0f // dalam cm
#define TINGGI_B 150.0f // dalam cm
#define TINGGI_C 120.0f // dalam cm

#define AMBANG_BAWAH 25.0f // Level 25% untuk memicu pengisian
#define TARGET_PENUH 95.0f // Target 95% untuk menghentikan pengisian
#define SUMBER_KOSONG 1.0f // Anggap sumber kosong jika level <= 1%
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Array untuk menangani 3 sensor
Sensor_HandleTypeDef sensors[NUM_SENSORS];

// Buffer untuk LCD (misalnya, 20 karakter)
char lcd_buffer[20];

// Variabel untuk level air
float LevelA_persen = 0, LevelB_persen = 0, LevelC_persen = 0;

// Variabel state untuk kontrol histeresis
FillState state_A = STATE_IDLE;
FillState state_B = STATE_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void Initialize_Sensors(void);
void Process_UART_Byte(Sensor_HandleTypeDef *sensor);
void Run_Control_Logic(void);
void Update_LCD_Display(void);
void Log_Data_To_SD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Memperbarui fungsi Run_Control_Logic dengan kontrol histeresis, prioritas, dan pengaman sumber kosong.
 * @note   Drum A memiliki prioritas tertinggi.
 */
void Run_Control_Logic(void)
{
  // 1. Hitung level air aktual dalam cm dan % dari data sensor
  float LevelA_cm = TINGGI_A - ((float)sensors[0].distance / 10.0f);
  float LevelB_cm = TINGGI_B - ((float)sensors[1].distance / 10.0f);
  float LevelC_cm = TINGGI_C - ((float)sensors[2].distance / 10.0f);

  LevelA_persen = (LevelA_cm / TINGGI_A) * 100.0f;
  LevelB_persen = (LevelB_cm / TINGGI_B) * 100.0f;
  LevelC_persen = (LevelC_cm / TINGGI_C) * 100.0f;

  // Pastikan persentase tidak di bawah 0 atau di atas 100
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

  // 2. Logika Kontrol dengan Prioritas untuk Drum A
  // Cek kondisi untuk memulai pengisian Drum A
  if (state_A == STATE_IDLE && LevelA_persen <= AMBANG_BAWAH)
  {
    // Hanya mulai mengisi jika Drum B memiliki cukup air
    if (LevelB_persen > SUMBER_KOSONG)
    {
      state_A = STATE_FILLING;
      HAL_GPIO_WritePin(POMPA_BA_GPIO_Port, POMPA_BA_Pin, GPIO_PIN_SET); // BUKA Valve B ke A
    }
  }
  // Cek kondisi untuk menghentikan pengisian Drum A
  else if (state_A == STATE_FILLING)
  {
    // Hentikan pengisian jika Drum A sudah penuh ATAU jika sumber (Drum B) kosong
    if (LevelA_persen >= TARGET_PENUH || LevelB_persen <= SUMBER_KOSONG)
    {
      state_A = STATE_IDLE;
      HAL_GPIO_WritePin(POMPA_BA_GPIO_Port, POMPA_BA_Pin, GPIO_PIN_RESET); // TUTUP Valve B ke A
    }
  }

  // 3. Logika Kontrol untuk Drum B (HANYA JIKA DRUM A TIDAK SEDANG DIISI)
  if (state_A == STATE_IDLE)
  {
    // Cek kondisi untuk memulai pengisian Drum B
    if (state_B == STATE_IDLE && LevelB_persen <= AMBANG_BAWAH)
    {
      // Hanya mulai mengisi jika Drum C memiliki cukup air
      if (LevelC_persen > SUMBER_KOSONG)
      {
        state_B = STATE_FILLING;
        HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_SET); // BUKA Valve C ke B
      }
    }
    // Cek kondisi untuk menghentikan pengisian Drum B
    else if (state_B == STATE_FILLING)
    {
      // Hentikan pengisian jika Drum B sudah penuh ATAU jika sumber (Drum C) kosong
      if (LevelB_persen >= TARGET_PENUH || LevelC_persen <= SUMBER_KOSONG)
      {
        state_B = STATE_IDLE;
        HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_RESET); // TUTUP Valve C ke B
      }
    }
  }
  else
  {
    // Jika Drum A sedang diisi, pastikan valve C->B mati untuk keamanan.
    state_B = STATE_IDLE;
    HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_RESET);
  }

  // 4. Logika Peringatan untuk Operator (independen dari logika pengisian)
  if (LevelC_persen <= AMBANG_BAWAH)
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET); // NYALAKAN Alert
  }
  else
  {
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET); // MATIKAN Alert
  }
}

/**
 * @brief Menginisialisasi struktur handle untuk setiap sensor.
 */
void Initialize_Sensors(void)
{
  // Sensor 1 menggunakan USART1
  sensors[0].huart = &huart1;
  sensors[0].state = STATE_WAIT_FOR_START;
  sensors[0].new_data_available = 0;

  // Sensor 2 menggunakan USART3
  sensors[1].huart = &huart2;
  sensors[1].state = STATE_WAIT_FOR_START;
  sensors[1].new_data_available = 0;

  // Sensor 3 menggunakan UART4
  sensors[2].huart = &huart3;
  sensors[2].state = STATE_WAIT_FOR_START;
  sensors[2].new_data_available = 0;
}

void Update_LCD_Display(void)
{
  lcd_set_cursor(0, 0);
  sprintf(lcd_buffer, "A:%3.0f%% B:%3.0f%%", LevelA_persen, LevelB_persen);
  lcd_send_string(lcd_buffer);
  lcd_set_cursor(1, 0);
  sprintf(lcd_buffer, "C:%3.0f%% S:%c%c", LevelC_persen, (state_A == STATE_FILLING) ? 'A' : ' ', (state_B == STATE_FILLING) ? 'B' : ' ');
  lcd_send_string("                ");
  lcd_set_cursor(1, 0);
  lcd_send_string(lcd_buffer);
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Inisialisasi handle sensor
  Initialize_Sensors();

  // Mulai menerima data dari semua sensor via interrupt
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    HAL_UART_Receive_IT(sensors[i].huart, &sensors[i].rx_buffer, 1);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Run_Control_Logic();

#if (GUNAKAN_LCD)
    {
      // Perbarui tampilan LCD setiap iterasi
      Update_LCD_Display();
    }
#endif

    if (sensors[0].new_data_available && sensors[1].new_data_available && sensors[2].new_data_available)
    {
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        sensors[i].new_data_available = 0;
      }
    }

    HAL_Delay(1000); // Mengurangi delay agar sistem lebih responsif
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin | USER_LED_Pin | LED_GREEN_Pin | LED_RED_Pin | POMPA_BA_Pin | POMPA_CB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin USER_LED_Pin LED_GREEN_Pin LED_RED_Pin
                           POMPA_BA_Pin POMPA_CB_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin | USER_LED_Pin | LED_GREEN_Pin | LED_RED_Pin | POMPA_BA_Pin | POMPA_CB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Callback yang dipanggil setiap kali 1 byte data diterima via UART.
 * @param  huart: pointer ke handle UART.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Cari tahu sensor mana yang mengirim data
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (huart->Instance == sensors[i].huart->Instance)
    {
      Process_UART_Byte(&sensors[i]);
      // Aktifkan lagi interrupt untuk menerima byte berikutnya
      HAL_UART_Receive_IT(sensors[i].huart, &sensors[i].rx_buffer, 1);
      return; // Keluar dari loop setelah menemukan sensor yang sesuai
    }
  }
}

/**
 * @brief  Memproses setiap byte yang masuk menggunakan state machine.
 * @param  sensor: pointer ke handle sensor yang sedang diproses.
 * @retval None
 */
void Process_UART_Byte(Sensor_HandleTypeDef *sensor)
{
  uint8_t received_byte = sensor->rx_buffer;

  switch (sensor->state)
  {
  case STATE_WAIT_FOR_START:
    if (received_byte == START_BYTE)
    {
      sensor->raw_data[0] = received_byte;
      sensor->state = STATE_READ_H_DATA;
    }
    break;

  case STATE_READ_H_DATA:
    sensor->raw_data[1] = received_byte;
    sensor->state = STATE_READ_L_DATA;
    break;

  case STATE_READ_L_DATA:
    sensor->raw_data[2] = received_byte;
    sensor->state = STATE_READ_SUM;
    break;

  case STATE_READ_SUM:
    sensor->raw_data[3] = received_byte;
    uint8_t checksum = (sensor->raw_data[0] + sensor->raw_data[1] + sensor->raw_data[2]) & 0xFF;

    if (checksum == sensor->raw_data[3])
    {
      // Checksum valid, hitung jarak
      sensor->distance = (uint16_t)(sensor->raw_data[1] << 8) | sensor->raw_data[2];
      sensor->new_data_available = 1; // Set flag data baru
    }
    // Reset state untuk frame berikutnya, baik checksum valid maupun tidak
    sensor->state = STATE_WAIT_FOR_START;
    break;

  default:
    // Jika terjadi state yang tidak dikenal, reset
    sensor->state = STATE_WAIT_FOR_START;
    break;
  }
}

/**
 * @brief  Callback untuk menangani error UART.
 * @param  huart: pointer ke handle UART.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  // Jika terjadi error (misal: Overrun), coba aktifkan kembali interrupt
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (huart->Instance == sensors[i].huart->Instance)
    {
      HAL_UART_Receive_IT(sensors[i].huart, &sensors[i].rx_buffer, 1);
      return;
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
