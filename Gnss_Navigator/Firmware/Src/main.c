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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.h"
#include "minmea.h"
#include "stdio.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KM_PER_DEGREE_LAT 111000  // 1 degree of latitude ≈ 111 km (scaled by 1000)
#define EARTH_RADIUS_KM 6371      // Earth's radius in km
#define PI 3.14159265358979323846
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
FATFS FatFs;

char rx_buffer[MINMEA_MAX_SENTENCE_LENGTH];
volatile uint16_t rx_head = 0;
char output_str[100];
volatile bool rx_overflow = false;
uint8_t rx_byte;
int32_t bbox[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI1_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    int16_t lat;  // Latitude (scaled by 10)
    int16_t lon;  // Longitude (scaled by 10)
} TileCoord;



void calculate_bounding_box(int32_t lat, int32_t lon, int32_t distance_m, int32_t* bbox) {
    // Convert distance to degrees (scaled by 1000)
    int32_t lat_offset = (distance_m * 1000) / KM_PER_DEGREE_LAT;

    // Precompute cosine of latitude (scaled by 1000)
    double lat_rad = (lat / 1000.0) * (PI / 180.0);  // Convert to radians
    int32_t cos_lat = (int32_t)(cos(lat_rad) * 1000);  // Scaled by 1000

    // Calculate longitude offset (scaled by 1000)
    int32_t lon_offset = (distance_m * 1000) / (KM_PER_DEGREE_LAT * cos_lat / 1000);

    // Calculate bounding box (scaled by 1000)
    bbox[0] = lat - lat_offset;  // min_lat
    bbox[1] = lon - lon_offset;  // min_lon
    bbox[2] = lat + lat_offset;  // max_lat
    bbox[3] = lon + lon_offset;  // max_lon
}

void coord_to_pixel(int32_t lat, int32_t lon, int32_t* bbox, uint32_t color) {
    uint16_t pos_x = ((lat - bbox[0]) * 240) / (bbox[2] - bbox[0]);
    uint16_t pos_y = ((lon - bbox[1]) * 240) / (bbox[3] - bbox[1]);

    ST7789_DrawPixel_4px(pos_x,pos_y, color);
}

// Function to check if a tile overlaps with the bounding box
bool is_tile_in_bbox(TileCoord tile, int16_t min_lat, int16_t max_lat, int16_t min_lon, int16_t max_lon) {
    return (tile.lat >= min_lat && tile.lat <= max_lat &&
            tile.lon >= min_lon && tile.lon <= max_lon);
}

// Function to get all tiles within a bounding box
void get_tiles_in_bbox(int16_t min_lat, int16_t max_lat, int16_t min_lon, int16_t max_lon, TileCoord* tiles, size_t* count) {
    *count = 0;

    // Iterate over all possible tiles in the bounding box
    for (int16_t lat = min_lat; lat <= max_lat; lat += 1) {
        for (int16_t lon = min_lon; lon <= max_lon; lon += 1) {
            TileCoord tile = {lat, lon};
            if (is_tile_in_bbox(tile, min_lat, max_lat, min_lon, max_lon)) {
                tiles[(*count)++] = tile;
            }
        }
    }
}

int32_t round_by_d(int32_t value, uint8_t d) {
	uint16_t scaler = 1;
	for (uint8_t i = 0; i < d; i++) {
		scaler *= 10;
	}
    if (value <= 0) {
        return (int32_t)((value / scaler) - 1);  // Round negative numbers
    } else {
        return (int32_t)((value) / scaler);  // Round positive numbers
    }
}



void get_tile_directory(TileCoord tile, char* dir, size_t size) {
	/* ROOT/10 DEGREE DIR/1 DEGREE DIR/0.1 DEGREE BIN */
	int16_t lat_10d = round_by_d(tile.lat, 2)*10;
	int16_t lon_10d = round_by_d(tile.lon, 2)*10;

	int16_t lat_1d = round_by_d(tile.lat, 1);
	int16_t lon_1d = round_by_d(tile.lon, 1);

	snprintf(dir, size, "/(%d, %d)/(%d, %d)/(%d, %d).bin", lat_10d, lon_10d, lat_1d, lon_1d, tile.lat, tile.lon);
}


void nmea_parse(char* line) {

	if (minmea_sentence_id(line,false) == MINMEA_SENTENCE_RMC) {
        struct minmea_sentence_rmc frame;
        if (minmea_parse_rmc(&frame, line)) {
			sprintf(output_str, "$RMC: raw coordinates and speed: (%ld/%ld,%ld/%ld) %ld/%ld",
                    frame.latitude.value, frame.latitude.scale,
                    frame.longitude.value, frame.longitude.scale,
                    frame.speed.value, frame.speed.scale);
			//ST7789_WriteString(10,10,output_str,Font_11x18,GREEN,BLACK);

			int32_t lat_dd = (int32_t)(minmea_tocoord(&frame.latitude)*frame.latitude.scale);
			int32_t lon_dd = (int32_t)(minmea_tocoord(&frame.longitude)*frame.longitude.scale);

			int32_t lat_3d = round_by_d(lat_dd, 2);
			int32_t lon_3d = round_by_d(lon_dd, 2);

			coord_to_pixel(lat_3d, lon_3d, bbox, GREEN);
        }

	}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_head >= MINMEA_MAX_SENTENCE_LENGTH || rx_byte == '$'){
        	rx_head = 0;
        	rx_buffer[rx_head] = rx_byte;
        	rx_head++;
        } else if (rx_byte == '\r') {
        	rx_buffer[rx_head] = '\0';
        	nmea_parse(rx_buffer);
        } else {
        	rx_buffer[rx_head] = rx_byte;
        	rx_head++;
        }


        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
	FIL fil;
	size_t length = 0;
	int32_t coordbuf[1024] = {0};
	size_t coord_b_read = 0;
	FRESULT fr;
	FRESULT fr2;
	char outtext[64] = {0};

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
  MX_USART1_UART_Init();
  MX_SPI4_Init();
  MX_SPI1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  ST7789_Init();
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  int32_t lat = -43524;  // -43.550 -43.524327, 172.594754
  int32_t lon = 172594; // 172.550



  TileCoord tiles[200];
  size_t tile_count;

  calculate_bounding_box(lat, lon, 5000, bbox);

  get_tiles_in_bbox(round_by_d(bbox[0], 2), round_by_d(bbox[2], 2), round_by_d(bbox[1], 2), round_by_d(bbox[3], 2), tiles, &tile_count);

	fr = f_mount(&FatFs, "", 0);
	for (uint8_t i = 0; i < tile_count; i++) {
		bool first_read = true;
        char dir[60];

        get_tile_directory(tiles[i], dir, sizeof(dir));


		fr2 = f_open(&fil, dir, FA_READ);


		do {
			// read bulk coordinates into buffer
	        size_t coord_index = 0;



			if (first_read) {
				//read and discard length
				fr = f_read(&fil, &coordbuf, sizeof(coordbuf)-4, &coord_b_read);

				length = (int32_t)coordbuf[0];

				length = __builtin_bswap32(length); // swap byte endianness

				coord_index++;

				first_read = false;

			} else {
				fr = f_read(&fil, &coordbuf, sizeof(coordbuf), &coord_b_read);
			}
			for ( ; (coord_index * sizeof(int32_t)) < coord_b_read; coord_index+=2) {
				// cycle through read coordinates
				if (coord_index >= 1019) {
					coordbuf[0] = 1;

				}
				coordbuf[coord_index] = __builtin_bswap32(coordbuf[coord_index]);
				coordbuf[coord_index+1] = __builtin_bswap32(coordbuf[coord_index+1]);
				if (coordbuf[coord_index] > bbox[2]) {
					break;
				}
				if (coordbuf[coord_index] > bbox[0]) {
					if (coordbuf[coord_index+1] > bbox[1] && coordbuf[coord_index+1] < bbox[3]) {
						//do math
						coord_to_pixel(coordbuf[coord_index], coordbuf[coord_index+1], bbox, MAGENTA);
					}
				}
			}
		} while(coord_b_read >= sizeof(coordbuf)-4);

		f_close(&fil);
	}
	uint16_t index = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 20;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
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
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
