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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Bring relay port definitions to an array structure
struct RELAY {
	GPIO_TypeDef	*gpio;
	uint16_t		pin;
};

// NMEA2K network operational NAME parameters
struct NMEA2K_NETWORK {
	uint8_t		senderId;
	uint32_t	identityNumber;
	uint16_t	manufacturerCode;
	uint8_t		deviceInstance;
	uint8_t		deviceFunction;
	uint8_t		deviceClass;
	uint8_t		systemInstance;
	uint8_t		industryGroup;
	uint8_t		linkState;
	uint8_t		linkRetries;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define N2K_PRIO			3		/* Priority of the messages on the network */
#define N2K_ID				128		/* Initial NMEA2K sender id */
#define RELAY_BOARD_ID		128		/* Switch bank id (0-255), Only top 4 bits count, lower 4 are set with switches */
#define RELAY_CHANNELS		8		/* Number of relays on this board */

// Initial ISO NAME parameters
#define ISO_IDENT			0x1BEAF	/* AKA serial number used in address claim */
#define MFG_CODE			666		/* Manufacturer code (0-2048) 666 for homemade */
#define DEV_INST			0		/* Device instance */
#define DEV_FUNCT			130		/* Binary Event Monitor */
#define DEV_CLASS			30		/* Electrical Distribution */
#define SYS_INST			0		/* System instance */
#define IND_GROUP			4		/* Industry group = 4 (Marine Industry) */
#define ARB_ADDRESS			1		/* Arbitrary address capable */
#define ISO_MAX_RETRIES		8		/* Maximum number of address claim retries */

// NMEA2K Link States
#define LINK_STATE_IDLE		0
#define LINK_STATE_CLAIMED	1
#define LINK_STATE_ACTIVE	2
#define LINK_STATE_ABORT	99

// Compare macro
#define compare(a, b) (((a) < (b)) ? -1 : (((a) > (b)) ? 1 : 0))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint16_t relayStatus;
uint8_t relayId;
struct RELAY relays[RELAY_CHANNELS];
struct NMEA2K_NETWORK nmea2kNetwork;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Send_ISOAddressClaim();
int Compare_NameWeight(uint8_t *data);
void Set_Relays(uint8_t *data);
void Report_Status(void);
int Get_RelayBoardId(void);
void Error_Handler(int error);
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

  // Clear all relays
  relayStatus = 0;

  // Populate port definition array
  relays[0].gpio = relay0_GPIO_Port;
  relays[0].pin  = relay0_Pin;
  relays[1].gpio = relay1_GPIO_Port;
  relays[1].pin  = relay1_Pin;
  relays[2].gpio = relay2_GPIO_Port;
  relays[2].pin  = relay2_Pin;
  relays[3].gpio = relay3_GPIO_Port;
  relays[3].pin  = relay3_Pin;
  relays[4].gpio = relay4_GPIO_Port;
  relays[4].pin  = relay4_Pin;
  relays[5].gpio = relay5_GPIO_Port;
  relays[5].pin  = relay5_Pin;
  relays[6].gpio = relay6_GPIO_Port;
  relays[6].pin  = relay6_Pin;
  relays[7].gpio = relay7_GPIO_Port;
  relays[7].pin  = relay7_Pin;

  // Populate initial N2K network structure
  nmea2kNetwork.senderId = N2K_ID;
  nmea2kNetwork.identityNumber = ISO_IDENT;
  nmea2kNetwork.manufacturerCode = MFG_CODE;
  nmea2kNetwork.deviceInstance = DEV_INST;
  nmea2kNetwork.deviceFunction = DEV_FUNCT;
  nmea2kNetwork.deviceClass = DEV_CLASS;
  nmea2kNetwork.systemInstance = SYS_INST;
  nmea2kNetwork.industryGroup = IND_GROUP;
  nmea2kNetwork.linkState = LINK_STATE_IDLE;
  nmea2kNetwork.linkRetries = 0;

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
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Get the relay board channel id
  relayId = Get_RelayBoardId();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // If the NMEA2K link is idle, try to (re)activate it
	  if (nmea2kNetwork.linkState == LINK_STATE_IDLE) {
		  // Stop the seconds timer
		  HAL_TIM_Base_Stop_IT(&htim3);
		  // Check the retry counter against max retries
		  if (nmea2kNetwork.linkRetries > ISO_MAX_RETRIES) {
			  // ERROR, no valid address
			  nmea2kNetwork.linkState = LINK_STATE_ABORT;
			  Error_Handler(13);
		  }
		  // Try a PGN 60928 - ISO Address Claim
		  Send_ISOAddressClaim();
		  // Set link state to CLAIMED
		  nmea2kNetwork.linkState = LINK_STATE_CLAIMED;
		  // Wait for 250 ms for the claim
		  HAL_Delay(250);
		  // Check if the claim is still valid
		  if (nmea2kNetwork.linkState == LINK_STATE_CLAIMED) {
			  // Claim is valid, activate network
			  nmea2kNetwork.linkState = LINK_STATE_ACTIVE;
			  // Clear the retry counter
			  nmea2kNetwork.linkRetries = 0;
			  // (re)start the seconds timer
			  HAL_TIM_Base_Start_IT(&htim3);
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(1);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler(2);
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef  sFilterN2K127502, sFilterN2K060928;

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler(3);
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // Filter for Switch Bank Control messages
  uint32_t can_id = 127502 << 8;

  sFilterN2K127502.FilterBank = 1;
  sFilterN2K127502.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterN2K127502.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterN2K127502.FilterIdHigh = (can_id >> 13) & 0xFFFF;
  sFilterN2K127502.FilterIdLow = ((can_id << 3) & 0xFFF8) | 4;
  sFilterN2K127502.FilterMaskIdHigh = 0x0FFF;
  sFilterN2K127502.FilterMaskIdLow = 0xF804;
  sFilterN2K127502.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterN2K127502.FilterActivation = CAN_FILTER_ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &sFilterN2K127502);

  // Filter for ISO messages
  can_id = 0x0E800 << 8;

  sFilterN2K060928.FilterBank = 0;
  sFilterN2K060928.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterN2K060928.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterN2K060928.FilterIdHigh = (can_id >> 13) & 0xFFFF;
  sFilterN2K060928.FilterIdLow = ((can_id << 3) & 0xFFF8) | 4;
  sFilterN2K060928.FilterMaskIdHigh = 0x0FC0;
  sFilterN2K060928.FilterMaskIdLow = 0x0004;
  sFilterN2K060928.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterN2K060928.FilterActivation = CAN_FILTER_ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &sFilterN2K060928);

  // Start CAN bus
  HAL_CAN_Start(&hcan);
  // Enable RX interrupts
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
	  Error_Handler(4);
  }
  /* USER CODE END CAN_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler(5);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler(6);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(7);
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, relay0_Pin|relay1_Pin|relay2_Pin|relay3_Pin
                          |relay4_Pin|relay5_Pin|relay6_Pin|relay7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : relay0_Pin relay1_Pin relay2_Pin relay3_Pin
                           relay4_Pin relay5_Pin relay6_Pin relay7_Pin */
  GPIO_InitStruct.Pin = relay0_Pin|relay1_Pin|relay2_Pin|relay3_Pin
                          |relay4_Pin|relay5_Pin|relay6_Pin|relay7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : id0_Pin id1_Pin id2_Pin id3_Pin */
  GPIO_InitStruct.Pin = id0_Pin|id1_Pin|id2_Pin|id3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/******************************************************************************
 * HAL_TIM_PeriodElapsedCallback
 *
 * HAL function periodictly called by the TIM3 timer
 * Sends a status report every second
 *
 *****************************************************************************/
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
	Report_Status();
}

/******************************************************************************
 * HAL_CAN_RxFifo1MsgPendingCallback
 *
 * HAL function called when a CAN message is received through the filters
 * Determines which action must be taken as a reaction
 *
 *****************************************************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef	RxHeader;
	uint8_t				RxData[8];
	uint32_t			N2KPgn;
	uint8_t				senderId;

	// RX control LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	// Get the message from the FIFO
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
		Error_Handler(8);
	}

	// Get the NMEA2000 PGN and the sender id from the extended CAN id
	N2KPgn = (RxHeader.ExtId >> 8) & 0x01FFFF;
	senderId = RxHeader.ExtId & 0xFF;

	// Check for Switch Bank Control message addressed to our channelID
	if ((N2KPgn == 127502) && (RxData[0] == relayId)) {
		// Set the relays and send a new status message
		Set_Relays(RxData);
		Report_Status();
	}

	// Check for ISO Address Claim message with our senderId
	if (((N2KPgn & 0x1FE00) == 0x0EE00) && (senderId == nmea2kNetwork.senderId)) {
		// If we are waiting for the claimed address, address is in use, send a new claim
		if (nmea2kNetwork.linkState == LINK_STATE_CLAIMED) {
			nmea2kNetwork.linkState = LINK_STATE_IDLE;
			nmea2kNetwork.linkRetries += 1;
			nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
		}
		// If our address is already active, check a couple of things
		if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
			// If the other contender can't change his senderID or has a higher NAME score, we change our senderID
			if (!(RxData[7] & 0x01) || (Compare_NameWeight(RxData) == 1)) {
				nmea2kNetwork.linkState = LINK_STATE_IDLE;
				nmea2kNetwork.linkRetries += 1;
				nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
			}
			else {
				// If we have the same NAME score, update our deviceInstance value
				if (Compare_NameWeight(RxData) == 0)
					nmea2kNetwork.deviceInstance += 1;
				// We reclaim the address because we have a higher NAME score
				Send_ISOAddressClaim();
			}
		}
	}

	// Check for ISO Request to our senderId or broadcast id
	if (((N2KPgn & 0x1FE00) == 0x0EA00) && (((N2KPgn & 0xFF) == nmea2kNetwork.senderId) || ((N2KPgn & 0xFF) == 0xFF))) {
		uint32_t requestPGN = (RxData[0] << 16) | (RxData[1] << 8) | RxData[2];
		// Check for a ISO Address Claim request
		if (requestPGN == 0x00EE00) {
			Send_ISOAddressClaim();
		}
	}

	// RX control LED
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

/******************************************************************************
 * Send_ISOAddressClaim
 *
 * Send a ISO Address Claim message to claim our sender ID
 *
 *****************************************************************************/
void Send_ISOAddressClaim() {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x0EEFF << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialise the data array with the ISO NAME info
	uint8_t TxData[] = {
			(nmea2kNetwork.identityNumber >> 13) & 0xFF,
			(nmea2kNetwork.identityNumber >> 5) & 0xFF,
			((nmea2kNetwork.identityNumber << 3) & 0xF8) | ((nmea2kNetwork.manufacturerCode >> 8) & 0x07),
			nmea2kNetwork.manufacturerCode & 0xFF,
			nmea2kNetwork.deviceInstance,
			nmea2kNetwork.deviceFunction,
			nmea2kNetwork.deviceClass & 0x7F,
			((nmea2kNetwork.systemInstance << 4) & 0xF0) | ((nmea2kNetwork.industryGroup << 1) & 0x0E) | 0x01
	};

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler(9);
	}

}

/******************************************************************************
 * Compare_NameWeight
 *
 * Helper function to compare our ISO NAME values against a contender
 *
 * Parameters:
 * 	- *data		Pointer to a CAN RxData[8] structure
 *
 * Returns:
 * 	- (-1)		Our ISO NAME value is greater, reclaim our address
 * 	- (0)		The ISO NAME values are the same, increase deviceInstance and
 * 				reclaim our address (We were first! :-P )
 * 	- (+1)		Our contender has a greater ISO NAME value, we do a modified
 * 				address claim
 *
 *****************************************************************************/
int Compare_NameWeight(uint8_t *data) {
	int ret;

	// compare is defined as a macro
	if ((ret = compare(data[0], ((nmea2kNetwork.identityNumber >> 13) & 0xFF))) != 0)
		return ret;
	if ((ret = compare(data[1], ((nmea2kNetwork.identityNumber >> 5) & 0xFF))) != 0)
		return ret;
	if ((ret = compare(data[2], (((nmea2kNetwork.identityNumber << 3) & 0xF8) | ((nmea2kNetwork.manufacturerCode >> 8) & 0x07)))) != 0)
		return ret;
	if ((ret = compare(data[3], (nmea2kNetwork.manufacturerCode & 0xFF))) != 0)
		return ret;
	if ((ret = compare(data[4], nmea2kNetwork.deviceInstance)) != 0)
		return ret;
	if ((ret = compare(data[5], nmea2kNetwork.deviceFunction)) != 0)
		return ret;
	if ((ret = compare(data[6], (nmea2kNetwork.deviceClass & 0x7F))) != 0)
		return ret;
	return compare(data[7], (((nmea2kNetwork.systemInstance << 4) & 0xF0) | ((nmea2kNetwork.industryGroup << 1) & 0x0E) | 0x01));
}

/******************************************************************************
 * Set_Relays
 *
 * Set the relays according to the NMEA2000 Switch Bank Control message
 *
 * Parameters:
 * 	- *data		Pointer to a CAN RxData[8] structure
 *
 *****************************************************************************/
void Set_Relays(uint8_t *data)
{
	int offset, index;

	// Count for the number of relays on the controller
	for (int i = 0; i < RELAY_CHANNELS; i++) {
		// Determine index and offset for the parameters in the data structure
		offset = 6 - (2 * (i % 4));
		index = 1 + (i / 4);

		switch ((data[index] >> offset) & 0x03) {
			case 0:
				// Parameter is 00, release the relay
				relayStatus &= ~(1 << i);
				HAL_GPIO_WritePin(relays[i].gpio, relays[i].pin, GPIO_PIN_RESET);
				break;
			case 1:
				// Parameter is 01, set the relay
				relayStatus |= (1 << i);
				HAL_GPIO_WritePin(relays[i].gpio, relays[i].pin, GPIO_PIN_SET);
				break;
			// Parameter 11 = no change, 10 is reserved. No action is needed
		}
	}
}

/******************************************************************************
 * Report_Status
 *
 * Send a NMEA2000 Binary Switch Bank Status to report the actual relay
 * settings. This report will be send periodicly or after a NMEA2000 Switch
 * Bank Control message
 *
 *****************************************************************************/
void Report_Status(void)
{
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;
	int 				offset, index;

	// Stop the timer
	HAL_TIM_Base_Stop_IT(&htim3);

	// Initialize the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (3 << 26) | (127501 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialize the status array with the relay instance id
	uint8_t TxData[] = {relayId, 0, 0, 0, 0, 0, 0, 0};

	// Fill the status array according to PGN 127501
	for (int i = 0; i < RELAY_CHANNELS; i++) {
		offset = 6 - (2 * (i % 4));
		index = 1 + (i / 4);
		if ((relayStatus & (1 << i)) != 0) {
			TxData[index] |= (1 << offset);
		}
	}

	// Send the NMEA 127501 Binary Switch Bank Status message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler(10);
	}

	// Restart/reset the timer
	HAL_TIM_Base_Start_IT(&htim3);
}

/******************************************************************************
 * Get_RelayBoardId
 *
 * Read the switches on GPIOB[15:12] for the lower nibble of the relay board ID
 *
 * Returns
 * 	- relayId		Relay board id, combination of RELAY_BOARD_ID and the
 * 					switches
 *
 *****************************************************************************/
int Get_RelayBoardId(void) {
	// Initialize thereturn value with the high nibble of RELAY_BOARD_ID
	int id = RELAY_BOARD_ID & 0xF0;

	// Read the ID switches
	if (HAL_GPIO_ReadPin(id0_GPIO_Port, id0_Pin) == GPIO_PIN_SET)
		id |= (1 << 0);
	if (HAL_GPIO_ReadPin(id1_GPIO_Port, id1_Pin) == GPIO_PIN_SET)
		id |= (1 << 1);
	if (HAL_GPIO_ReadPin(id2_GPIO_Port, id2_Pin) == GPIO_PIN_SET)
		id |= (1 << 2);
	if (HAL_GPIO_ReadPin(id3_GPIO_Port, id3_Pin) == GPIO_PIN_SET)
		id |= (1 << 3);

	return id;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(int error)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  __disable_irq();
  while (1)
  {
	  for (int i = 0; i < error; i++) {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  HAL_Delay(250);
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		  HAL_Delay(250);
	  }
	  HAL_Delay(1000);
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
