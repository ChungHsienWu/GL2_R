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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DSI_HandleTypeDef hdsi;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
#define MAX_IMAGE_SIZE 65536
//for mode
#define Max_pic_per_mode 5
#define Max_mode_num 11

uint32_t IMAGE_H = 120;
uint32_t IMAGE_W = 156;
uint8_t frame_rate_r = 0x1E;
uint8_t spi_flash_content_length = 70;
uint8_t auto_run_start_content = 0;
uint8_t auto_run_end_content = 69;
uint8_t content_size = 1; //0=16kb, 1=32kb, 2=64kb
uint8_t Horizontal_blanking_H = 0x02;
uint8_t Horizontal_blanking_L = 0x20;
uint8_t Vertical_blanking_H = 0x05;
uint8_t Vertical_blanking_L = 0x05;
uint16_t HSA = 184;
uint16_t HBP = 180;
uint16_t HFP = 180;
uint16_t VSA = 9;
uint16_t VBP = 8;
uint16_t VFP = 8;
uint8_t Pixel_Mapping_one_L = 0x05;
uint8_t Pixel_Mapping_one_H = 0x05;
uint8_t Power_Status = 0x55;

uint8_t BOARD_NUMBER = 2;
uint8_t frame_buf_tmp[MAX_IMAGE_SIZE] = {0};
uint8_t frame_buf_0[MAX_IMAGE_SIZE] = {0};
uint8_t frame_buf_1[MAX_IMAGE_SIZE] = {0};
uint8_t frame_buf_flash[MAX_IMAGE_SIZE] = {0};
uint8_t play_mode = 0;		  //0=Static display, 1=Dynamic display, 2=Dynamic display frame buffer (0) and frame buffer (1)
uint8_t play_mode_source = 0; //0=flash, 1=frame_buf_0, 2=frame_buf_1
uint8_t image_arr_rgb888[MAX_IMAGE_SIZE*3] = {0};
uint8_t display_image_number = 0;
uint8_t spi_rev_2byte[2] = {0};
uint8_t setting_changed = 0;

//for mode
uint8_t Playing_mode = 1;
uint8_t Mode_pic[Max_pic_per_mode]={0};
/*
uint8_t Mode_config[Max_pic_per_mode*2*Max_mode_num+1]=
{
		{0,1,0,1,0,1,0,1,0,1,//mode_config 0
		0,1,0,1,0,1,0,1,0,1, //mode_config 1
		0,1,0,1,0,1,0,1,0,1, //mode_config 2
		0,1,0,1,0,1,0,1,0,1, //mode_config 3
		0,1,0,1,0,1,0,1,0,1, //mode_config 4
		0,1,0,1,0,1,0,1,0,1, //mode_config 5
		0,1,0,1,0,1,0,1,0,1, //mode_config 6
		0,1,0,1,0,1,0,1,0,1, //mode_config 7
		0,1,0,1,0,1,0,1,0,1, //mode_config 8
		0,1,0,1,0,1,0,1,0,1, //mode_config 9
		0,1,0,1,0,1,0,1,0,1} //playing_mode record at last.
};
*/
uint8_t Mode_changed = 1;//initial or mode change
uint8_t Mode_config[Max_pic_per_mode*2*Max_mode_num]={0};
uint8_t Current_mode_config[10]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DSIHOST_DSI_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
static void mipi_config(void);
static void LCD_PowerOn(void);
void display_panel(uint8_t *frame_buf);
void write_flash_page(uint8_t *data, uint8_t image_id);
void read_flash_page(uint8_t *data, uint8_t image_id);
void erase_flash_sector(uint8_t image_id);
void reset_flash_software();
void delay_us(int time);
void delay_100ns(int time);
static void my_MX_DSIHOST_DSI_Init(void);
static void my_MX_LTDC_Init(void);
void mode_init();
void write_flash_config();
void read_flash_config();
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
  MX_DSIHOST_DSI_Init();
  MX_LTDC_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  mipi_config();
  HAL_UART_Transmit(&huart4, "start_s", 7, 1000);

  play_mode = 3;
  play_mode_source = 0;
  mode_init();
  write_flash_config();
  for(int i=0; i <= Max_pic_per_mode*Max_mode_num;i++){
	  Mode_config[i*2]=0;    //picture_id
	  Mode_config[i*2+1]=0;  //picture_delay_time
  }
  Mode_config[Max_pic_per_mode*2*Max_mode_num-1] = 0; //playing_mode
  read_flash_config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(bat_en_GPIO_Port, bat_en_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(en_GPIO_Port, en_Pin, GPIO_PIN_SET);

  HAL_SPI_Receive_IT(&hspi3, &spi_rev_2byte, 2);
  for (int i = 0; i < IMAGE_H*IMAGE_W*3; i++)
  {
	  image_arr_rgb888[i] = 0xFF;
  }

    while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (play_mode_source == 0)
	  {
		  if (play_mode == 0)
		  {
			  read_flash_page(&frame_buf_flash, display_image_number);
			  display_panel(&frame_buf_flash);
		  }
		  else if (play_mode == 1)
		  {
			  uint8_t should_break = 0;
			  while(1)
			  {
				  for (int i = auto_run_start_content; i <= auto_run_end_content; i++)
				  {
					  HAL_Delay(5);
					  while(HAL_GPIO_ReadPin(sync_GPIO_Port, sync_Pin) == GPIO_PIN_SET);
					  if (play_mode_source != 0 || play_mode != 1 || setting_changed == 1)
					  {
						  should_break = 1;
						  break;
					  }
					  read_flash_page(&frame_buf_flash, i);
					  display_panel(&frame_buf_flash);
					  display_image_number = i;
				  }
				  if (should_break == 0)
				  {
					  display_image_number = 0;
				  }
				  else if (should_break == 1)
				  {
				  	  break;
				  }
			  }
		  }
		  else if (play_mode == 3)
		  {
			  uint8_t should_break = 0;
			  uint8_t Picture_count = 0;
			  //initial or change mode
			  if( Mode_changed )
			  {
				  read_flash_config();
				  Playing_mode = Mode_config[100];//playing_mode_store = [Max_pic_per_mode*2*(Max_mode_num-1)]
				  //fill Current_mode_config from Mode_config by using Playing_mode
				  for(int i=Playing_mode*10,j=0; i<(Playing_mode+1)*10 ;i++){
					 Current_mode_config[j]=Mode_config[i];
					 j++;
				  }
				  Mode_changed = 0;
				  //check how many pics to display
				  //warning don't set Current_mode_config = [255 255 1 2 10 2 255 255 255 255]
				  Picture_count = 0;
				  for(int i = 0 ; i < 5 ; i++){
					 if(Current_mode_config[i*2] != 255)
						 Picture_count++;
				  }
			  }
			  while(1)//display
			  {
				  for (int i = 0; i < Picture_count*2; i = i+2)
				  {
					  while(HAL_GPIO_ReadPin(sync_GPIO_Port, sync_Pin) == GPIO_PIN_SET);// wait to sync
					  //int current_pic_delay=(Current_mode_config[i+1])*250;//ms
					  //HAL_Delay(current_pic_delay);
					  if (play_mode_source != 0 || play_mode != 3 || Mode_changed == 1)
					  {
						  should_break = 1;
						  break;
					  }
					  read_flash_page(&frame_buf_flash, Current_mode_config[i]);
					  display_panel(&frame_buf_flash);
				  }
				  if (should_break == 1)
				  {
					  break;
				  }
			  }
		  }
	  }
	  else if (play_mode_source == 1)
	  {
		  if (play_mode == 2)
		  {
			  while(1)
			  {
				  HAL_Delay(5);
				  while(HAL_GPIO_ReadPin(sync_GPIO_Port, sync_Pin) == GPIO_PIN_SET);
				  if (play_mode_source != 1 || play_mode != 2 || setting_changed == 1)
				  {
					  break;
				  }
				  display_panel(&frame_buf_0);

				  HAL_Delay(5);
				  while(HAL_GPIO_ReadPin(sync_GPIO_Port, sync_Pin) == GPIO_PIN_SET);
				  if (play_mode_source != 1 || play_mode != 2 || setting_changed == 1)
				  {
					  break;
				  }
				  display_panel(&frame_buf_1);
			  }
		  }
		  else
		  {
			  display_panel(&frame_buf_0);
		  }
	  }
	  else if (play_mode_source == 2)
	  {
		  display_panel(&frame_buf_1);
	  }

	  if(setting_changed == 1)
	  {
		  uint16_t Horizontal_blanking_total = Horizontal_blanking_H*256 + Horizontal_blanking_L;
		  HBP = Horizontal_blanking_total / 3;
		  HFP = Horizontal_blanking_total / 3;
		  HSA = Horizontal_blanking_total - (HBP + HFP);

		  uint16_t Vertical_blanking_total = Vertical_blanking_H*256 + Vertical_blanking_L;
		  VBP = Vertical_blanking_total / 3;
		  VFP = Vertical_blanking_total / 3;
		  VSA = Vertical_blanking_total - (VBP + VFP);

		  my_MX_DSIHOST_DSI_Init();
		  my_MX_LTDC_Init();
		  mipi_config();
		  HAL_Delay(10);

		  setting_changed = 0;
	  }

	  HAL_Delay(10);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
static void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_VidCfgTypeDef VidCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 2;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 50;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 19;
  PhyTimings.ClockLaneLP2HSTime = 15;
  PhyTimings.DataLaneHS2LPTime = 9;
  PhyTimings.DataLaneLP2HSTime = 10;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  VidCfg.VirtualChannelID = 0;
  VidCfg.ColorCoding = DSI_RGB888;
  VidCfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  VidCfg.Mode = DSI_VID_MODE_NB_EVENTS;
  VidCfg.PacketSize = 120;
  VidCfg.NumberOfChunks = 1;
  VidCfg.NullPacketSize = 0;
  VidCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  VidCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  VidCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  VidCfg.HorizontalSyncActive = 184;
  VidCfg.HorizontalBackPorch = 180;
  VidCfg.HorizontalLine = 664;
  VidCfg.VerticalSyncActive = 9;
  VidCfg.VerticalBackPorch = 8;
  VidCfg.VerticalFrontPorch = 8;
  VidCfg.VerticalActive = 156;
  VidCfg.LPCommandEnable = DSI_LP_COMMAND_ENABLE;
  VidCfg.LPLargestPacketSize = 28;
  VidCfg.LPVACTLargestPacketSize = 80;
  VidCfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
  VidCfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
  VidCfg.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
  VidCfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
  VidCfg.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
  VidCfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;
  VidCfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;
  if (HAL_DSI_ConfigVideoMode(&hdsi, &VidCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */
  LCD_PowerOn();
  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 183;
  hltdc.Init.VerticalSync = 8;
  hltdc.Init.AccumulatedHBP = 363;
  hltdc.Init.AccumulatedVBP = 16;
  hltdc.Init.AccumulatedActiveW = 483;
  hltdc.Init.AccumulatedActiveH = 172;
  hltdc.Init.TotalWidth = 663;
  hltdc.Init.TotalHeigh = 180;
  hltdc.Init.Backcolor.Blue = 255;
  hltdc.Init.Backcolor.Green = 255;
  hltdc.Init.Backcolor.Red = 255;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 120;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 156;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = (uint32_t *)image_arr_rgb888;
  pLayerCfg.ImageWidth = 120;
  pLayerCfg.ImageHeight = 156;
  pLayerCfg.Backcolor.Blue = 255;
  pLayerCfg.Backcolor.Green = 255;
  pLayerCfg.Backcolor.Red = 255;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESXP_Pin|flash_cs_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(bat_en_GPIO_Port, bat_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(en_GPIO_Port, en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_Pin RESXP_Pin flash_cs_Pin */
  GPIO_InitStruct.Pin = led_Pin|RESXP_Pin|flash_cs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : sync_Pin */
  GPIO_InitStruct.Pin = sync_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sync_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pic_sw_Pin */
  GPIO_InitStruct.Pin = pic_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(pic_sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : bat_en_Pin */
  GPIO_InitStruct.Pin = bat_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(bat_en_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : en_Pin */
  GPIO_InitStruct.Pin = en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(en_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void mipi_config()
{
	if (HAL_DSI_Start(&hdsi) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xF0, 0xC3);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xF0, 0x96);
	uint8_t cmd3[7] = {0x00, 0x77, 0x1F, 0x04, 0x2A, 0x80, 0x33};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 8, 0xE7, cmd3);
	uint8_t cmd4[3] = {0xC0, 0x68, 0xE0};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, 0xA4, cmd4);
	uint8_t cmd5[4] = {0x42, 0x05, 0x24, 0x03};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC3, cmd5);
	uint8_t cmd6[4] = {0x42, 0x05, 0x24, 0x03};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC4, cmd6);
	uint8_t cmd7[12] = {0x0F, 0xF5, 0x10, 0x13, 0x22, 0x25, 0x10, 0x55, 0x55, 0x55, 0x55, 0x55};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 13, 0xE5, cmd7);
	uint8_t cmd8[12] = {0x0F, 0xF5, 0x10, 0x13, 0x22, 0x25, 0x10, 0x55, 0x55, 0x55, 0x55, 0x55};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 13, 0xE6, cmd8);
	uint8_t cmd9[7] = {0x00, 0x55, 0x00, 0x00, 0x00, 0x49, 0x22};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 8, 0xEC, cmd9);
	uint8_t cmd10[4] = {0x88, 0x05, 0x0F, 0x18};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC1, cmd10);
	uint8_t cmd11[4] = {0x88, 0x05, 0x0F, 0x18};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xC2, cmd11);
	//HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x00);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x36, 0x08);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x3A, 0x07);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xC5, 0xBE);
	uint8_t cmd15[14] = {0xC0, 0x01, 0x04, 0x0B, 0x0B, 0x29, 0x41, 0x55, 0x55, 0x3D, 0x19, 0x18, 0x24, 0x27};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 15, 0xE0, cmd15);
	uint8_t cmd16[14] = {0xC0, 0x01, 0x05, 0x0B, 0x0C, 0x29, 0x42, 0x55, 0x56, 0x3E, 0x1A, 0x18, 0x24, 0x28};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 15, 0xE1, cmd16);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB2, 0x10);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB3, 0x01);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB4, 0x01);
	//uint8_t cmd20[2] = {0x27, 0x09};
	uint8_t cmd20[2] = {0x4D, 0x0E};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 3, 0xB6, cmd20);
	uint8_t cmd21[4] = {0x00, 0x54, 0x00, 0x54};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 5, 0xB5, cmd21);
	uint8_t cmd22[9] = {0x20, 0x12, 0x40, 0x00, 0x00, 0x2F, 0x2A, 0x0A, 0x00};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xA5, cmd22);
	uint8_t cmd23[9] = {0x20, 0x12, 0x40, 0x00, 0x00, 0x2F, 0x2A, 0x0A, 0x00};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xA6, cmd23);
	uint8_t cmd24[7] = {0x58, 0x0A, 0x21, 0x00, 0x20, 0x01, 0x00};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 8, 0xBA, cmd24);
	uint8_t cmd25[8] = {0x00, 0x45, 0x00, 0x1F, 0x15, 0x87, 0x07, 0x04};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 9, 0xBB, cmd25);
	uint8_t cmd26[8] = {0x00, 0x45, 0x00, 0x1F, 0x15, 0x87, 0x07, 0x04};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 9, 0xBC, cmd26);
	uint8_t cmd27[11] = {0x11, 0x77, 0xFF, 0xFF, 0x25, 0x34, 0x43, 0x52, 0xFF, 0xFF, 0xF9};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 12, 0xBD, cmd27);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xED, 0xC3);
	uint8_t cmd29[3] = {0x40, 0x0F, 0x00};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, 0xE4, cmd29);
	uint8_t cmd30[9] = {0x90, 0x00, 0x3F, 0x10, 0x3F, 0x35, 0x7F, 0x7F, 0x25};
	HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xCC, cmd30);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00);
	HAL_Delay(0);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x11, 0x00);
	HAL_Delay(120);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x29, 0x00);
	HAL_Delay(120);
	HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00);
}

static void LCD_PowerOn(void)
{
	/* Activate XRES active low */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	HAL_Delay(20); /* wait 20 ms */

	/* Desactivate XRES */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	/* Wait for 10ms after releasing XRES before sending commands */
	HAL_Delay(120);
}

void Write_Registers_data(uint8_t do_flag)
{
	uint8_t Register_Address[1] = {0};
	uint8_t data[1] = {0};

	HAL_SPI_Receive(&hspi3, (uint8_t *)Register_Address, 1, 1000);
	HAL_SPI_Receive(&hspi3, (uint8_t *)data, 1, 1000);
	if (do_flag == 1)
	{
		switch (Register_Address[0])
		{
		case 0: //Horizontal Resolution
			IMAGE_H = (uint32_t)data[0];
			setting_changed = 1;
			break;
		case 1: //Vertical Resolution
			IMAGE_W = (uint32_t)data[0];
			setting_changed = 1;
			break;
		case 2: //Horizontal blanking (High byte)
			Horizontal_blanking_H = data[0];
			setting_changed = 1;
			break;
		case 3: //Horizontal blanking (Low byte)
			Horizontal_blanking_L = data[0];
			setting_changed = 1;
			break;
		case 4: //Vertical blanking (High byte)
			Vertical_blanking_H = data[0];
			setting_changed = 1;
			break;
		case 5: //Vertical blanking (Low byte)
			Vertical_blanking_L = data[0];
			setting_changed = 1;
			break;
		case 6: //Frame rate x 2 (Hz)
			frame_rate_r = data[0];
			break;
		case 7: //Show SPI flash content length
			spi_flash_content_length = data[0];
			break;
		case 8: //Content number of each frame
			break;
		case 9: //Clock rate of SPI
			break;
		case 10: //Clock rate of I2C
			break;
		case 11: //Pixel Mapping one_L
			Pixel_Mapping_one_L = data[0];
			break;
		case 12: //Pixel Mapping one_H
			Pixel_Mapping_one_H = data[0];
			break;
		case 13: //Auto Run start content of SPI flash
			auto_run_start_content = data[0];
			break;
		case 14: //Show first content number of SPI flash
			display_image_number = data[0];
			break;
		case 15: //Auto Run end content of SPI flash
			auto_run_end_content = data[0];
			break;
		case 16: //Control A
			switch (data[0] & 0b00000011) //Display Mode
			{
			case 0b00000000: //Display content of frame buffer (0)
				play_mode_source = 1;
				break;
			case 0b00000001: //Display content of frame buffer (1)
				play_mode_source = 2;
				break;
			case 0b00000011: //Display Flash content
				play_mode_source = 0;
				break;
			}
			switch (data[0] & 0b00001100) //Static or Dynamic mode
			{
			case 0b00000000: //Static display
				play_mode = 0;
				break;
			case 0b00000100: //Dynamic display frame buffer (0) and frame buffer (1)
				play_mode = 2;
				break;
			case 0b00001000: //Dynamic display flash content
				play_mode = 1;
				break;
			}
			break;
		case 17: //Yesr of Version
			break;
		case 18: //Day of Version
			break;
		case 19: //Month of Version
			break;
		case 20: //Status
			break;
		case 21: //Detection
			break;
		case 22: //Power Status
			break;
		case 23: //Serial number
			break;
		case 24: //content size
			content_size = data[0];
			break;
		case 25:// force play buffer to write flash.
			play_mode_source = 1;
			play_mode = 2;
			break;
		case 33:// force play buffer to write flash.
			Mode_config[100] = data[0];
			Playing_mode = data[0];

			write_flash_config();
			Mode_changed = 1;

			break;
		}
	}
	//HAL_UART_Transmit(&huart4, &Register_Address, 1, 1000);
	//HAL_UART_Transmit(&huart4, &data, 1, 1000);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//HAL_UART_Transmit(&huart4, &spi_rev_2byte, 2, 1000);
	// USB command: Type and command
	if(hspi == SPI2){
		int a=2;
	}
	if(hspi == SPI3){
		int a=3;
	}
	switch (spi_rev_2byte[0] & 0b11000000)
	{
	case 0b00000000: //Chain SPI functions
		switch (spi_rev_2byte[0] & 0b00111000)
		{
		case 0b00000000: //Write content of full frame buffer to fram buffer (0)
			switch (spi_rev_2byte[0] & 0b00000111)
			{
			case 0b00000000: //Command for DIP switch ID = 00
				if (BOARD_NUMBER == 1)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_0, IMAGE_H*IMAGE_W, 1000);
				}
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				if (BOARD_NUMBER == 2)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_0, IMAGE_H*IMAGE_W, 1000);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_tmp, IMAGE_H*IMAGE_W, 1000);
				}
				break;
			case 0b00000100: //Broadcast to every board
				HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_0, IMAGE_H*IMAGE_W, 1000);
				break;
			default:
				break;
			}
			break;
		case 0b00001000: //Write content of full frame buffer to fram buffer (1)
			switch (spi_rev_2byte[0] & 0b00000111)
			{
			case 0b00000000: //Command for DIP switch ID = 00
				if (BOARD_NUMBER == 1)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_1, IMAGE_H*IMAGE_W, 1000);
				}
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				if (BOARD_NUMBER == 2)
				{
					HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_1, IMAGE_H*IMAGE_W, 1000);
				}
				else
				{
					HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_tmp, IMAGE_H*IMAGE_W, 1000);
				}
				break;
			case 0b00000100: //Broadcast to every board
				HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_1, IMAGE_H*IMAGE_W, 1000);
				break;
			default:
				break;
			}
			break;
		case 0b00010000: //Write Registers data
			switch (spi_rev_2byte[0] & 0b00000111)
			{
			case 0b00000000: //Command for DIP switch ID = 00
				if (BOARD_NUMBER == 1)
				{
					Write_Registers_data(1);
				}
				else
				{
					Write_Registers_data(0);
				}
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				if (BOARD_NUMBER == 2)
				{
					Write_Registers_data(1);
				}
				else
				{
					Write_Registers_data(0);
				}
				break;
			case 0b00000100: //Broadcast to every board
				Write_Registers_data(1);
				break;
			default:
				break;
			}
			break;
		case 0b00011000: //Write partial content of frame buffer
			break;
		case 0b00100000: //Read content of full frame buffer to fram buffer (0)
			break;
		case 0b00101000: //Read content of full frame buffer to fram buffer (1)
			break;
		case 0b00110000: //Read Registers data
			break;
		case 0b00111000: //Read partial content of frame buffer
			break;
		}
		break;
	case 0b01000000: //Master SPI functions
		switch (spi_rev_2byte[0] & 0b00111000)
		{
		case 0b00000000: //Start SPI write data
			break;
		case 0b00001000: //Continuous write SPI data
			break;
		case 0b00010000: //End SPI write data
			break;
		case 0b00011000: //Start SPI Read data
			break;
		case 0b00100000: //Continuous Read SPI data
			break;
		case 0b00101000: //End SPI Read data
			break;
		}
		break;
	case 0b10000000: //I2C command
		switch (spi_rev_2byte[0] & 0b00111000)
		{
		case 0b00000000: //I2C Write Data
			break;
		case 0b00100000: //I2C Read Data
			break;
		}
		break;
	case 0b11000000: //SPI flash function & Slave SPI
		switch (spi_rev_2byte[0] & 0b00111000)
		{
		case 0b00000000: //Write data to SPI flash
			switch (spi_rev_2byte[0] & 0b00000111)
			{
			case 0b00000000: //Command for DIP switch ID = 00
				HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_tmp, IMAGE_H*IMAGE_W, 1000);
				if (BOARD_NUMBER == 1)
				{
					erase_flash_sector(spi_rev_2byte[1] - 1);
					write_flash_page(&frame_buf_tmp, spi_rev_2byte[1] - 1);
				}
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_tmp, IMAGE_H*IMAGE_W, 1000);
				if (BOARD_NUMBER == 2)
				{
					erase_flash_sector(spi_rev_2byte[1] - 1);
					write_flash_page(&frame_buf_tmp, spi_rev_2byte[1] - 1);
				}
				break;
			case 0b00000100: //Broadcast to every board
				HAL_SPI_Receive(&hspi3, (uint8_t *)frame_buf_tmp, IMAGE_H*IMAGE_W, 1000);
				erase_flash_sector(spi_rev_2byte[1] - 1);
				write_flash_page(&frame_buf_tmp, spi_rev_2byte[1] - 1);
				break;
			default:
				break;
			}
			break;
		case 0b00001000: //Read data from SPI flash
			switch (spi_rev_2byte[0] & 0b00000111)
			{
			case 0b00000000: //Command for DIP switch ID = 00
				if (BOARD_NUMBER == 1)
				{
					reset_flash_software();
					read_flash_page(&frame_buf_tmp, spi_rev_2byte[1] - 1);
				}
				break;
			case 0b00000001: //Command for DIP switch ID = 01
				break;
			case 0b00000100: //Broadcast to every board
				reset_flash_software();
				read_flash_page(&frame_buf_tmp, spi_rev_2byte[1] - 1);
				break;
			default:
				break;
			}
			break;
		case 0b00100000: //Display Data by Slave SPI
			break;
		}
		break;
	}
	// USB command: ID
	switch (spi_rev_2byte[0] & 0b00000111)
	{
	case 0b00000000: //Command for DIP switch ID = 00
		break;
	case 0b00000001: //Command for DIP switch ID = 01
		break;
	default: //Broadcast to every board
		break;
	}

	HAL_SPI_Receive_IT(&hspi3, &spi_rev_2byte, 2);
}

void display_panel(uint8_t *frame_buf)
{
	int num_ones = 0;
	uint16_t Pixel_Mapping_one = Pixel_Mapping_one_L | Pixel_Mapping_one_H << 8;
	for (int i = 0; i < 12; i++)
	{
		if (Pixel_Mapping_one & (1 << i))
		{
			num_ones++;
		}
	}

	int frame_buf_count = 0;
	int Pixel_Mapping_one_count = 0;
	int c = 0;
	for (int i = 0; i < IMAGE_H*IMAGE_W; i += num_ones)
	{
		Pixel_Mapping_one_count = 0;
		c = 0;
		for (int j = 0; j < 12; j++)
		{
			if (Pixel_Mapping_one & (1 << j))
			{
				frame_buf_count += Pixel_Mapping_one_count;
				image_arr_rgb888[frame_buf_count] = frame_buf[i + c];
				c++;
				Pixel_Mapping_one_count = 0;
			}
			Pixel_Mapping_one_count++;
		}
		frame_buf_count += Pixel_Mapping_one_count;
	}
}

/*====================================flash function begin====================================*/
uint8_t read_flash_SR()
{
	uint8_t dat[1] = {0x00};
	HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x05}, 1, 1000);
	HAL_SPI_Receive(&hspi2, (uint8_t *)dat, 1, 1000);
	HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);

	return dat[0];
}

void flash_wait_nobusy(void)
{
    while(((read_flash_SR()) & 0x01)==0x01);
}

/*void write_flash_page(uint8_t *data, uint8_t image_id)
{
	uint8_t page_count = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(0x10 * i * 256 >= IMAGE_H*IMAGE_W)
		{
			page_count = i; //5
			break;
		}
	}

	uint8_t page_count_more = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(i * 256 >= IMAGE_H*IMAGE_W)
		{
			page_count_more = i; //74
			break;
		}
	}

	uint8_t divide_value = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(0x10 * page_count * i > 256)
		{
			divide_value = i - 1; //3
			break;
		}
	}

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;
	int count = 0;
	for (uint32_t i = image_id_L * 0x10 * page_count; i < image_id_L * 0x10 * page_count + page_count_more; i++)
	{
		// enable write
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x06}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		// write data to flash page
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x02}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){i}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, &data[count*256], 256, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		// disable write
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x04}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(1000);

		count++;
		if(count*256 >= IMAGE_H*IMAGE_W)
			break;
	}

	flash_wait_nobusy();
}

void read_flash_page(uint8_t *data, uint8_t image_id)
{
	uint8_t page_count = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(0x10 * i * 256 >= IMAGE_H*IMAGE_W)
		{
			page_count = i;
			break;
		}
	}

	uint8_t page_count_more = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(i * 256 >= IMAGE_H*IMAGE_W)
		{
			page_count_more = i;
			break;
		}
	}

	uint8_t divide_value = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(0x10 * page_count * i > 256)
		{
			divide_value = i - 1;
			break;
		}
	}

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;
	int count = 0;
	for (uint32_t i = image_id_L * 0x10 * page_count; i < image_id_L * 0x10 * page_count + page_count_more; i++)
	{
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x03}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){i}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_SPI_Receive(&hspi2, &data[count*256], 256, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);

		count++;
		if(count*256 >= IMAGE_H*IMAGE_W)
			break;
	}
}

void erase_flash_sector(uint8_t image_id)
{
	uint8_t page_count = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(0x10 * i * 256 >= IMAGE_H*IMAGE_W)
		{
			page_count = i;
			break;
		}
	}

	uint8_t divide_value = 0;
	for(uint8_t i = 1; i < 256; i++)
	{
		if(0x10 * page_count * i > 256)
		{
			divide_value = i - 1;
			break;
		}
	}

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;

	reset_flash_software();

	for(uint8_t i = 0; i < page_count; i++)
	{
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x06}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x20}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){(image_id_L*(0x10*page_count)) + (0x10*i)}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x04}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		flash_wait_nobusy();
	}
}*/
void mode_init(){
	for(int i=0; i <= Max_pic_per_mode*Max_mode_num*2;i++){
		Mode_config[i*2]=i;//picture_id
		Mode_config[i*2+1]=i;//picture_delay_time
	}
	Mode_config[Max_pic_per_mode*(Max_mode_num-1)*2]=0;
	//test mode 1
	Mode_config[0]=0;
	Mode_config[1]=1;
	Mode_config[2]=1;
	Mode_config[3]=1;
	Mode_config[4]=255;
	Mode_config[5]=1;
	Mode_config[6]=255;
	Mode_config[7]=1;
	Mode_config[8]=255;
	Mode_config[9]=1;
	//test mode 2
	Mode_config[10]=0;
	Mode_config[11]=4;
	Mode_config[12]=1;
	Mode_config[13]=4;
	Mode_config[14]=255;
	Mode_config[15]=1;
	Mode_config[16]=255;
	Mode_config[17]=1;
	Mode_config[18]=255;
	Mode_config[19]=1;
	//test mode 3
	Mode_config[20]=2;
	Mode_config[21]=1;
	Mode_config[22]=3;
	Mode_config[23]=1;
	Mode_config[24]=4;
	Mode_config[25]=1;
	Mode_config[26]=5;
	Mode_config[27]=1;
	Mode_config[28]=255;
	Mode_config[29]=1;
	//test mode 3
	Mode_config[30]=2;
	Mode_config[31]=4;
	Mode_config[32]=3;
	Mode_config[33]=4;
	Mode_config[34]=4;
	Mode_config[35]=4;
	Mode_config[36]=5;
	Mode_config[37]=4;
	Mode_config[38]=255;
	Mode_config[39]=1;
	//test mode 4
	Mode_config[40]=6;
	Mode_config[41]=1;
	Mode_config[42]=7;
	Mode_config[43]=1;
	Mode_config[44]=8;
	Mode_config[45]=1;
	Mode_config[46]=9;
	Mode_config[47]=1;
	Mode_config[48]=10;
	Mode_config[49]=1;
	//test mode 5
	Mode_config[50]=6;
	Mode_config[51]=4;
	Mode_config[52]=7;
	Mode_config[53]=4;
	Mode_config[54]=8;
	Mode_config[55]=4;
	Mode_config[56]=9;
	Mode_config[57]=4;
	Mode_config[58]=10;
	Mode_config[59]=4;
	//test mode 6
	Mode_config[60]=11;
	Mode_config[61]=1;
	Mode_config[62]=12;
	Mode_config[63]=1;
	Mode_config[64]=255;
	Mode_config[65]=1;
	Mode_config[66]=255;
	Mode_config[67]=1;
	Mode_config[68]=255;
	Mode_config[69]=1;
	//test mode 7
	Mode_config[70]=11;
	Mode_config[71]=4;
	Mode_config[72]=12;
	Mode_config[73]=4;
	Mode_config[74]=255;
	Mode_config[75]=1;
	Mode_config[76]=255;
	Mode_config[77]=1;
	Mode_config[78]=255;
	Mode_config[79]=1;
	//test mode 8
	Mode_config[80]=13;
	Mode_config[81]=1;
	Mode_config[82]=14;
	Mode_config[83]=1;
	Mode_config[84]=255;
	Mode_config[85]=1;
	Mode_config[86]=255;
	Mode_config[87]=1;
	Mode_config[88]=255;
	Mode_config[89]=1;
	//test mode 8
	Mode_config[90]=13;
	Mode_config[91]=1;
	Mode_config[92]=14;
	Mode_config[93]=1;
	Mode_config[94]=255;
	Mode_config[95]=1;
	Mode_config[96]=255;
	Mode_config[97]=1;
	Mode_config[98]=255;
	Mode_config[99]=1;
}
void write_flash_config()
{
	//content_size：0=16kb, 1=32kb, 2=32kb, 3=64kb
	int divide_value = 0;
	if(content_size==0) divide_value=256/64; //divide_value=4
	else if(content_size==1) divide_value=256/128; //divide_value=2
	else if(content_size==2) divide_value=256/256;//divide_value=1
	int image_id = 31;
	erase_flash_sector(image_id);

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;
	int count = 0;
	for (uint32_t i = image_id_L*(256/divide_value); i < (image_id_L+1)*(256/divide_value); i++)
	{
		// enable write
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x06}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		// write data to flash page
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x02}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){i}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, &Mode_config[0], Max_pic_per_mode*Max_mode_num*2, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		// disable write
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x04}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(1000);
		count++;
	}

	flash_wait_nobusy();
}
void read_flash_config()
{
	int divide_value = 0;
	if(content_size==0) divide_value=256/64;
	else if(content_size==1) divide_value=256/128;
	else if(content_size==2) divide_value=256/256;

	int image_id= 31;
	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;
	int count = 0;
	for (uint32_t i = image_id_L*(256/divide_value); i < (image_id_L+1)*(256/divide_value); i++)
	{
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x03}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){i}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_SPI_Receive(&hspi2, &Mode_config[0],  Max_pic_per_mode*Max_mode_num*2, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
	}
}
void write_flash_page(uint8_t *data, uint8_t image_id)
{
	int divide_value = 0;
	if(content_size==0) divide_value=256/64;
	else if(content_size==1) divide_value=256/128;
	else if(content_size==2) divide_value=256/256;

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;
	int count = 0;
	for (uint32_t i = image_id_L*(256/divide_value); i < (image_id_L+1)*(256/divide_value); i++)
	{
		// enable write
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x06}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		// write data to flash page
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x02}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){i}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, &data[count*256], 256, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		// disable write
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x04}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(1000);
		count++;
	}

	flash_wait_nobusy();
}

void read_flash_page(uint8_t *data, uint8_t image_id)
{
	int divide_value = 0;
	if(content_size==0) divide_value=256/64;
	else if(content_size==1) divide_value=256/128;
	else if(content_size==2) divide_value=256/256;

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;
	int count = 0;
	for (uint32_t i = image_id_L*(256/divide_value); i < (image_id_L+1)*(256/divide_value); i++)
	{
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x03}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){i}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_SPI_Receive(&hspi2, &data[count*256], 256, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		count++;
	}
}

void erase_flash_sector(uint8_t image_id)
{
	int divide_value = 0;
	if(content_size==0) divide_value=256/64;
	else if(content_size==1) divide_value=256/128;
	else if(content_size==2) divide_value=256/256;

	uint8_t page_count = 0;
	if(content_size==0) page_count=16/4;
	else if(content_size==1) page_count=32/4;
	else if(content_size==2) page_count=64/4;

	int image_id_H = image_id / divide_value;
	int image_id_L = image_id % divide_value;

	reset_flash_software();

	for(uint8_t i = 0; i < page_count; i++)
	{
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x06}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x20}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){image_id_H}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){(image_id_L*(0x10*page_count)) + (0x10*i)}, 1, 1000);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x00}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x04}, 1, 1000);
		HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
		delay_us(10);

		flash_wait_nobusy();
	}
}

void reset_flash_software()
{
	HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x66}, 1, 1000);
	HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
	delay_100ns(1);
	HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t[]){0x99}, 1, 1000);
	HAL_GPIO_WritePin(GPIOB, flash_cs_Pin, GPIO_PIN_SET);
	delay_100ns(1);
	delay_us(1000);
}
/*====================================flash function end====================================*/

/*====================================customized function start====================================*/
static void my_MX_DSIHOST_DSI_Init(void)
{
  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_VidCfgTypeDef VidCfg = {0};
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 2;
  hdsi.Init.NumberOfLanes = DSI_ONE_DATA_LANE;
  PLLInit.PLLNDIV = 50;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV1;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 19;
  PhyTimings.ClockLaneLP2HSTime = 15;
  PhyTimings.DataLaneHS2LPTime = 9;
  PhyTimings.DataLaneLP2HSTime = 10;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 0;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetLowPowerRXFilter(&hdsi, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  VidCfg.VirtualChannelID = 0;
  VidCfg.ColorCoding = DSI_RGB888;
  VidCfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  VidCfg.Mode = DSI_VID_MODE_NB_EVENTS;
  VidCfg.PacketSize = 120;
  VidCfg.NumberOfChunks = 1;
  VidCfg.NullPacketSize = 0;
  VidCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  VidCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  VidCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  VidCfg.HorizontalSyncActive = HSA;
  VidCfg.HorizontalBackPorch = HBP;
  VidCfg.HorizontalLine = HSA+HBP+IMAGE_H+HFP;
  VidCfg.VerticalSyncActive = VSA;
  VidCfg.VerticalBackPorch = VBP;
  VidCfg.VerticalFrontPorch = VFP;
  VidCfg.VerticalActive = IMAGE_W;
  VidCfg.LPCommandEnable = DSI_LP_COMMAND_ENABLE;
  VidCfg.LPLargestPacketSize = 28;
  VidCfg.LPVACTLargestPacketSize = 80;
  VidCfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
  VidCfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
  VidCfg.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
  VidCfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
  VidCfg.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
  VidCfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;
  VidCfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_DISABLE;
  if (HAL_DSI_ConfigVideoMode(&hdsi, &VidCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  LCD_PowerOn();
}
static void my_MX_LTDC_Init(void)
{
  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = HSA-1;
  hltdc.Init.VerticalSync = VSA-1;
  hltdc.Init.AccumulatedHBP = HSA+HBP-1;
  hltdc.Init.AccumulatedVBP = VSA+VBP-1;
  hltdc.Init.AccumulatedActiveW = HSA+HBP+IMAGE_H-1;
  hltdc.Init.AccumulatedActiveH = VSA+VBP+IMAGE_W-1;
  hltdc.Init.TotalWidth = HSA+HBP+IMAGE_H+HFP-1;
  hltdc.Init.TotalHeigh = VSA+VBP+IMAGE_W+VFP-1;
  hltdc.Init.Backcolor.Blue = 255;
  hltdc.Init.Backcolor.Green = 255;
  hltdc.Init.Backcolor.Red = 255;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = IMAGE_H;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = IMAGE_W;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = (uint32_t *)image_arr_rgb888;
  pLayerCfg.ImageWidth = IMAGE_H;
  pLayerCfg.ImageHeight = IMAGE_W;
  pLayerCfg.Backcolor.Blue = 255;
  pLayerCfg.Backcolor.Green = 255;
  pLayerCfg.Backcolor.Red = 255;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
}
/*====================================customized function end====================================*/

void delay_us(int time)
{
	int i = 0;
	while (time--)
	{
		i = 13;
		while (i--);
	}
}

void delay_100ns(int time)
{
	int i = 0;
	while (time--)
	{
		i = 1;
		while (i--);
	}
}

int button_count = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_12)
	{
		button_count++;
		delay_us(50000);
		for (int i = 0; i < 200; i++)
		{
			if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_RESET)
			{
				if (button_count < 80 && button_count > 5 && play_mode == 0)
				{
					display_image_number++;
					if (display_image_number >= spi_flash_content_length)
						display_image_number = 0;
				}
				button_count = 0;
				return;
			}
			button_count++;
			delay_us(10000);
		}
		/*if (play_mode == 1 || play_mode == 2)
		{
			play_mode = 0;
		}
		else if (play_mode == 0)
		{
			play_mode = 1;
		}*/
		button_count = 0;
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
