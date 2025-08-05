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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Struktur untuk menangani state dan data dari setiap sensor
typedef enum
{
	STATE_WAIT_FOR_START,
	STATE_READ_H_DATA,
	STATE_READ_M_DATA, // Tambahkan state untuk Middle Byte
	STATE_READ_L_DATA,
	STATE_READ_SUM // Jika masih ada checksum, jika tidak bisa dihapus
} UART_Receive_State;

typedef struct
{
	UART_HandleTypeDef *huart;	 // Handle UART yang digunakan
	uint8_t rx_buffer;			 // Buffer untuk 1 byte data masuk
	UART_Receive_State state;	 // State machine untuk parsing
	uint8_t raw_data[5];		 // Buffer untuk frame data [0xA0, H, M, L, SUM (optional)]
	uint32_t raw_distance_value; // Nilai mentah dari H, M, L (maksimal 24-bit, gunakan uint32_t)
	float distance_mm;			 // Hasil jarak dalam mm
	uint8_t new_data_available;	 // Flag penanda data baru
								 // GPIO_TypeDef *de_port;       // Port GPIO untuk pin DE
								 // uint16_t de_pin;             // Pin GPIO untuk pin DE
} Sensor_HandleTypeDef;

// Struktur untuk state kontrol pengisian (histeresis)
typedef enum
{
	STATE_IDLE,
	STATE_FILLING
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
} ControlParams;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GUNAKAN_LCD 1	  // Aktifkan jika menggunakan LCD I2C
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

// Variabel global untuk menyimpan parameter
ControlParams g_params;

// Variabel untuk CLI via USB VCP
#define RX_BUFFER_SIZE 128
uint8_t g_usb_rx_buffer[RX_BUFFER_SIZE];
volatile uint32_t g_rx_index = 0;
volatile uint8_t g_command_ready = 0;
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

// Fungsi untuk CLI dan Parameter
void VCP_printf(const char *format, ...);
void Process_Command(uint8_t *cmd_buffer);
void Save_Parameters_To_Flash(void);
void Load_Parameters_From_Flash(void);
void Set_Default_Parameters(void);

// Prototipe fungsi untuk meminta data dari sensor
void Request_Sensor_Data(Sensor_HandleTypeDef *sensor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Mengirim string terformat melalui USB VCP.
 */
void VCP_printf(const char *format, ...)
{
	char buf[256];
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
	g_params.tinggi_A = 100.0f;
	g_params.ambang_bawah_A = 25.0f;
	g_params.target_penuh_A = 95.0f;
	g_params.tinggi_B = 150.0f;
	g_params.ambang_bawah_B = 25.0f;
	g_params.target_penuh_B = 95.0f;
	g_params.tinggi_C = 120.0f;
	g_params.ambang_bawah_C = 25.0f;
	g_params.target_penuh_C = 95.0f;
	g_params.sumber_kosong = 1.0f;
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

	if (strncmp(cmd, "$$", 2) == 0)
	{
		VCP_printf("--- Parameter Saat Ini ---\r\n");
		VCP_printf("Drum A: Tinggi=%.1f, Min=%.1f%%, Max=%.1f%%\r\n",
				   g_params.tinggi_A, g_params.ambang_bawah_A,
				   g_params.target_penuh_A);
		VCP_printf("Drum B: Tinggi=%.1f, Min=%.1f%%, Max=%.1f%%\r\n",
				   g_params.tinggi_B, g_params.ambang_bawah_B,
				   g_params.target_penuh_B);
		VCP_printf("Drum C: Tinggi=%.1f, Min=%.1f%%, Max=%.1f%%\r\n",
				   g_params.tinggi_C, g_params.ambang_bawah_C,
				   g_params.target_penuh_C);
		VCP_printf("Ambang Sumber Kosong: %.1f%%\r\n", g_params.sumber_kosong);
	}
	else if (strncmp(cmd, "$P", 2) == 0)
	{
		VCP_printf("--- Daftar Pin ---\r\n");
		VCP_printf("Sensor A (USART1): PA9 (TX), PA10 (RX)\r\n");
		VCP_printf("Sensor B (USART2): PA2 (TX), PA3 (RX)\r\n");
		VCP_printf("Sensor C (USART3): PB10 (TX), PB11 (RX)\r\n");
		VCP_printf("Pompa B->A: PB14\r\n");
		VCP_printf("Pompa C->B: PB15\r\n");
		VCP_printf("LED Merah (Alert): PB13\r\n");
		VCP_printf("LCD I2C (I2C1): PB6 (SCL), PB7 (SDA)\r\n");
	}
	else if (strncmp(cmd, "$H", 2) == 0)
	{
		VCP_printf("--- Bantuan ---\r\n");
		VCP_printf("$$          : Lihat semua parameter\r\n");
		VCP_printf("$P          : Lihat daftar pin\r\n");
		VCP_printf("$H          : Tampilkan pesan ini\r\n");
		VCP_printf(
			"$1=<min>,<max> : Set ambang batas Drum A (contoh: $1=20,90)\r\n");
		VCP_printf("$2=<min>,<max> : Set ambang batas Drum B\r\n");
		VCP_printf("$3=<min>,<max> : Set ambang batas Drum C\r\n");
		VCP_printf("$S          : Simpan parameter ke Flash\r\n");
		VCP_printf("$L          : Muat parameter dari Flash\r\n");
		VCP_printf("$D          : Kembalikan ke pengaturan default\r\n");
	}
	else if (sscanf(cmd, "$1=%f,%f", &min, &max) == 2)
	{
		g_params.ambang_bawah_A = min;
		g_params.target_penuh_A = max;
		VCP_printf("OK: Parameter Drum A diubah -> Min=%.1f, Max=%.1f\r\n", min,
				   max);
	}
	else if (sscanf(cmd, "$2=%f,%f", &min, &max) == 2)
	{
		g_params.ambang_bawah_B = min;
		g_params.target_penuh_B = max;
		VCP_printf("OK: Parameter Drum B diubah -> Min=%.1f, Max=%.1f\r\n", min,
				   max);
	}
	else if (sscanf(cmd, "$3=%f,%f", &min, &max) == 2)
	{
		g_params.ambang_bawah_C = min;
		g_params.target_penuh_C = max;
		VCP_printf("OK: Parameter Drum C diubah -> Min=%.1f, Max=%.1f\r\n", min,
				   max);
	}
	else if (strncmp(cmd, "$S", 2) == 0)
	{
		Save_Parameters_To_Flash();
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
	float LevelA_cm = g_params.tinggi_A - ((float)sensors[0].distance_mm / 10.0f);
	float LevelB_cm = g_params.tinggi_B - ((float)sensors[1].distance_mm / 10.0f);
	float LevelC_cm = g_params.tinggi_C - ((float)sensors[2].distance_mm / 10.0f);
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

	if (state_A == STATE_IDLE && LevelA_persen <= g_params.ambang_bawah_A)
	{
		if (LevelB_persen > g_params.sumber_kosong)
		{
			state_A = STATE_FILLING;
			HAL_GPIO_WritePin(POMPA_BA_GPIO_Port, POMPA_BA_Pin, GPIO_PIN_SET);
		}
	}
	else if (state_A == STATE_FILLING)
	{
		if (LevelA_persen >= g_params.target_penuh_A || LevelB_persen <= g_params.sumber_kosong)
		{
			state_A = STATE_IDLE;
			HAL_GPIO_WritePin(POMPA_BA_GPIO_Port, POMPA_BA_Pin, GPIO_PIN_RESET);
		}
	}

	if (state_A == STATE_IDLE)
	{
		if (state_B == STATE_IDLE && LevelB_persen <= g_params.ambang_bawah_B)
		{
			if (LevelC_persen > g_params.sumber_kosong)
			{
				state_B = STATE_FILLING;
				HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin,
								  GPIO_PIN_SET);
			}
		}
		else if (state_B == STATE_FILLING)
		{
			if (LevelB_persen >= g_params.target_penuh_B || LevelC_persen <= g_params.sumber_kosong)
			{
				state_B = STATE_IDLE;
				HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin,
								  GPIO_PIN_RESET);
			}
		}
	}
	else
	{
		state_B = STATE_IDLE;
		HAL_GPIO_WritePin(POMPA_CB_GPIO_Port, POMPA_CB_Pin, GPIO_PIN_RESET);
	}

	if (LevelC_persen <= g_params.ambang_bawah_C)
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
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
	//  sensors[0].de_port = MAX485_DE1_GPIO_Port;
	//  sensors[0].de_pin = MAX485_DE1_Pin;

	// Sensor 2 menggunakan USART2
	sensors[1].huart = &huart2;
	sensors[1].state = STATE_WAIT_FOR_START;
	sensors[1].new_data_available = 0;
	//  sensors[1].de_port = MAX485_DE2_GPIO_Port;
	//  sensors[1].de_pin = MAX485_DE2_Pin;

	// Sensor 3 menggunakan UART3
	sensors[2].huart = &huart3;
	sensors[2].state = STATE_WAIT_FOR_START;
	sensors[2].new_data_available = 0;
	//  sensors[2].de_port = MAX485_DE3_GPIO_Port;
	//  sensors[2].de_pin = MAX485_DE3_Pin;

	// Set semua MAX485 ke mode RX secara default saat inisialisasi
	// for (int i = 0; i < NUM_SENSORS; i++)
	// {
	//   HAL_GPIO_WritePin(sensors[i].de_port, sensors[i].de_pin, GPIO_PIN_RESET); // DE = LOW for RX
	// }
}

void Update_LCD_Display(void)
{
	lcd_set_cursor(0, 0);
	sprintf(lcd_buffer, "A:%3.0f%% B:%3.0f%%", LevelA_persen, LevelB_persen);
	lcd_send_string(lcd_buffer);
	lcd_set_cursor(1, 0);
	sprintf(lcd_buffer, "C:%3.0f%% S:%c%c", LevelC_persen,
			(state_A == STATE_FILLING) ? 'A' : ' ',
			(state_B == STATE_FILLING) ? 'B' : ' ');
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
	lcd_init(&hi2c1); // Inisialisasi LCD I2C
	lcd_clear();	  // Bersihkan LCD

	// pesan awal di LCD
	lcd_set_cursor(0, 0);
	sprintf(lcd_buffer, "hai dunia");
	lcd_send_string(lcd_buffer);

	// Muat parameter dari Flash saat startup
	Load_Parameters_From_Flash();

	// Inisialisasi handle sensor
	Initialize_Sensors();

	// Mulai menerima data dari semua sensor via interrupt
	for (int i = 0; i < NUM_SENSORS; i++)
		HAL_UART_Receive_IT(sensors[i].huart, &sensors[i].rx_buffer, 1);

	// Beri sedikit waktu agar USB siap
	HAL_Delay(2000);
	VCP_printf("\r\nSistem Kontrol Drum Siap. Ketik '$H' untuk bantuan.\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		// Minta data dari setiap sensor secara berurutan
		for (int i = 0; i < NUM_SENSORS; i++)
		{
			Request_Sensor_Data(&sensors[i]);
			// Pastikan interrupt receive aktif untuk sensor ini agar bisa menerima respons
			// HAL_UART_Receive_IT(sensors[i].huart, &sensors[i].rx_buffer, 1); sudah ada sebelum while loop
			HAL_Delay(50); // Beri sedikit waktu agar sensor merespons dan data masuk
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

		if (sensors[0].new_data_available && sensors[1].new_data_available && sensors[2].new_data_available)
		{
			for (int i = 0; i < NUM_SENSORS; i++)
				sensors[i].new_data_available = 0;
		}

		HAL_Delay(1000);
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
	HAL_GPIO_WritePin(GPIOA, MAX485_DE2_Pin | MAX485_DE1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin | MAX485_DE3_Pin | USER_LED_Pin | LED_GREEN_Pin | LED_RED_Pin | POMPA_BA_Pin | POMPA_CB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_BTN_Pin */
	GPIO_InitStruct.Pin = USER_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MAX485_DE2_Pin MAX485_DE1_Pin */
	GPIO_InitStruct.Pin = MAX485_DE2_Pin | MAX485_DE1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_CS_Pin MAX485_DE3_Pin USER_LED_Pin LED_GREEN_Pin
							 LED_RED_Pin POMPA_BA_Pin POMPA_CB_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin | MAX485_DE3_Pin | USER_LED_Pin | LED_GREEN_Pin | LED_RED_Pin | POMPA_BA_Pin | POMPA_CB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
				g_command_ready = 1;				// Set flag untuk diproses di main loop
			}
		}
		else
		{
			g_usb_rx_buffer[g_rx_index++] = Buf[i]; // Tambahkan karakter ke buffer
		}
	}
}

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
 * @brief  Meminta data dari sensor dengan mengirimkan byte 0xA0.
 * @param  sensor: pointer ke handle sensor yang akan diminta datanya.
 * @retval None
 */
void Request_Sensor_Data(Sensor_HandleTypeDef *sensor)
{
	uint8_t command_byte = 0xA0;

	// 1. Set MAX485 ke mode TX
	// HAL_GPIO_WritePin(sensor->de_port, sensor->de_pin, GPIO_PIN_SET); // DE = HIGH for TX
	// HAL_Delay(1);                                                     // Tunggu sebentar untuk stabilisasi (sesuaikan jika perlu)

	// 2. Kirim byte 0xA0
	HAL_UART_Transmit(sensor->huart, &command_byte, 1, 100); // Timeout 100ms
	HAL_Delay(1);											 // Tunggu sebentar setelah transmit selesai

	// 3. Set MAX485 kembali ke mode RX
	// HAL_GPIO_WritePin(sensor->de_port, sensor->de_pin, GPIO_PIN_RESET); // DE = LOW for RX
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
		// Byte pertama setelah request adalah H
		sensor->raw_data[0] = received_byte;
		sensor->state = STATE_READ_M_DATA;
		break;

	case STATE_READ_M_DATA: // Middle byte
		sensor->raw_data[1] = received_byte;
		sensor->state = STATE_READ_L_DATA;
		break;

	case STATE_READ_L_DATA: // Low byte
		sensor->raw_data[2] = received_byte;

		sensor->raw_distance_value = ((uint32_t)sensor->raw_data[0] << 16) | // H byte
									 ((uint32_t)sensor->raw_data[1] << 8) |	 // M byte
									 (uint32_t)sensor->raw_data[2];			 // L byte

		sensor->distance_mm = (float)sensor->raw_distance_value / 1000.0f;
		sensor->new_data_available = 1;

		sensor->state = STATE_WAIT_FOR_START;
		break;

	case STATE_READ_SUM: // Jika sensor masih mengirim checksum, pertahankan logika ini
		// Jika tidak, Anda bisa menghapus state ini dan melompat dari L_DATA langsung ke STATE_WAIT_FOR_START
		uint8_t checksum = (sensor->raw_data[0] + sensor->raw_data[1] + sensor->raw_data[2]) & 0xFF;
		if (checksum == received_byte) // Jika checksum ada dan valid
		{
			// Data sudah dihitung di L_DATA, ini hanya validasi checksum
		}
		sensor->state = STATE_WAIT_FOR_START; // Reset state untuk frame berikutnya
		break;

	default:
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
