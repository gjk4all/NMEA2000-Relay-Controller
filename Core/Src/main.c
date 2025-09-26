/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * NMEA2000 Relay Board Controller
  *
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

#include <stdio.h>
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
	uint8_t			senderId;
	uint8_t			linkState;
	uint16_t		heartbeatTime;
	uint8_t			*nameData;
	uint8_t			linkRetries;
	uint16_t		statusTime;
};

struct ISO_TRANSPORT_MESSAGE {
	uint8_t			senderId;
	uint8_t			isoCommand;
	uint8_t			packetsExpected;
	uint8_t			packetsReceived;
	uint16_t		messageSize;
	uint32_t		pgn;
	uint8_t			*data;
	struct ISO_TRANSPORT_MESSAGE *prev;
	struct ISO_TRANSPORT_MESSAGE *next;
};

struct CAN_MSG_T {
	uint32_t		pgn;
	uint8_t			senderId;
	uint8_t			data[8];
};

struct TIMER_T {
	int 			type;
	int				timeout;
	int 			countdown;
	void 			(*callback)(void *);
	void 			*payload;
	struct 			TIMER_T *prev;
	struct 			TIMER_T *next;
};

// Queue type for jobs to be executed
struct JOB_QUEUE_T {
	void 			(*callback)(void *);
	void 			*payload;
	struct JOB_QUEUE_T	*next;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NMEA2000_VERSION	3000	/* NMEA2000 Version 3.000 */

#define N2K_PRIO			3		/* Priority of the messages on the network */
#define N2K_ID				128		/* Initial NMEA2K sender id */
#define RELAY_BOARD_ID		128		/* Switch bank id (0-255), Only top 4 bits count, lower 4 are set with switches */
#define RELAY_CHANNELS		8		/* Number of relays on this board */

// Initial ISO NAME parameters
#define MFG_CODE			2046	/* Manufacturer code (0-2047) 2046 for homemade */
#define DEV_INST			0		/* Device instance */
#define DEV_FUNCT			130		/* Binary Event Monitor */
#define DEV_CLASS			30		/* Electrical Distribution */
#define SYS_INST			0		/* System instance */
#define IND_GROUP			4		/* Industry group = 4 (Marine Industry) */
#define ARB_ADDRESS			1		/* Arbitrary address capable */
#define ISO_MAX_RETRIES		8		/* Maximum number of address claim retries */
#define ISO_HEARTBEAT		2000	/* Heartbeat default interval = 20 sec (2000 * 10ms) */
#define STATUS_TIMER		200		/* Switch status default interval = 2 sec (200 * 10ms) */

// NMEA2K Link States
#define LINK_STATE_IDLE		0
#define LINK_STATE_CLAIMED	1
#define LINK_STATE_ACTIVE	2
#define LINK_STATE_ABORT	99

#define TIMER_ONESHOT		1
#define TIMER_RELOAD		2

#define MODEL_ID			"GJK NMEA2000 Relay Controller"
#define SOFTWARE_VERSION	"V 1.0"
#define HARDWARE_VERSION	"V 1.0"

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
struct RELAY relays[RELAY_CHANNELS] = {
		{relay0_GPIO_Port, relay0_Pin},
		{relay1_GPIO_Port, relay1_Pin},
		{relay2_GPIO_Port, relay2_Pin},
		{relay3_GPIO_Port, relay3_Pin},
		{relay4_GPIO_Port, relay4_Pin},
		{relay5_GPIO_Port, relay5_Pin},
		{relay6_GPIO_Port, relay6_Pin},
		{relay7_GPIO_Port, relay7_Pin}
};
struct NMEA2K_NETWORK nmea2kNetwork;
struct TIMER_T *timers = NULL, *statusTimer;
struct JOB_QUEUE_T *jobs = NULL;

// List of transmitted and received PGN's capable by this device
uint32_t txList[] = {59392, 60928, 126996, 127501, 126993, 126464, 0};
uint32_t rxList[] = {59904, 60928, 60160, 60416, 65240, 127502, 0};
uint32_t * pgnList[] = {txList, rxList};

// Static strings for PGN 126996
char productInformation[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Send_ISOAddressClaim(void);
void Send_ISOAcknowledgement(uint8_t dest, uint8_t control, uint8_t group, uint32_t pgn);
void Send_HeartBeat(void * payload);
void Send_Status(void * payload);
void Send_ProductInformation(void * payload);
void Send_PGNList(void * payload);
int Send_FFMessage(CAN_TxHeaderTypeDef *TxHeader, uint8_t *data, int seqNo, int len);
void Handle_ISOTransportMessages(void *payload);
void Handle_ISOCommandedAddress(uint8_t *data);
int Compare_NameWeight(uint8_t *data);
void Set_Relays(uint8_t *data);
struct TIMER_T * Add_Timer(int type, int timeout, void(*callback)(), void * payload);
void Delete_Timer(struct TIMER_T *timer);
struct JOB_QUEUE_T * Add_Job(void(*callback)(), void * payload);
void Delete_Job(struct JOB_QUEUE_T *job);
int Get_RelayBoardId(void);
void Error_Handler(void);
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

	// Initialize Product Information record
	for (int i = 0; i < 128; i++)
		productInformation[i] = 0xFF;

	sprintf(productInformation, "%s", MODEL_ID);
	sprintf(productInformation + 32, "%s", SOFTWARE_VERSION);
	sprintf(productInformation + 64, "%s", HARDWARE_VERSION);
	sprintf(productInformation + 96, "%lu", (HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2()));

	// Initialise the data array with the ISO NAME info
	uint8_t nameData[8];

	*(uint32_t *)nameData = (MFG_CODE << 21) | ((HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2()) & 0x1FFFFF);
	nameData[4] = DEV_INST;
	nameData[5] = DEV_FUNCT;
	nameData[6] = (DEV_CLASS << 1) | 1;
	nameData[7] = ((ARB_ADDRESS == 0)?0:0x80) | ((IND_GROUP << 4) & 0x70) | ((SYS_INST) & 0x0F);

	// Populate initial N2K network structure
	nmea2kNetwork.senderId = N2K_ID;
	nmea2kNetwork.linkState = LINK_STATE_IDLE;
	nmea2kNetwork.heartbeatTime = ISO_HEARTBEAT;
	nmea2kNetwork.nameData = nameData;
	nmea2kNetwork.linkRetries = 0;
	nmea2kNetwork.statusTime = STATUS_TIMER;

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

  // Initialize continues timers
  HAL_TIM_Base_Stop_IT(&htim3);

  // Heartbeat timer
  if (Add_Timer(TIMER_RELOAD, nmea2kNetwork.heartbeatTime, &Send_HeartBeat, NULL) == NULL) {
	  Error_Handler();
  }
  // Binary switch status timer
  if ((statusTimer = Add_Timer(TIMER_RELOAD, nmea2kNetwork.statusTime, &Send_Status, NULL)) == NULL) {
	  Error_Handler();
  }

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
			  Error_Handler();
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

		// If the link is up, do the switch polling
		if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
			// Run job queue
			while (jobs != NULL) {
				(jobs->callback)(jobs->payload);
				Delete_Job(jobs);
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
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
  CAN_FilterTypeDef   sFilterN2K127502, sFilterN2K060928;

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
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
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

  sFilterN2K060928.FilterBank = 2;
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
	  Error_Handler();
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
  htim3.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
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
 * HAL function periodicly called by the TIM3 timer
 * Check timers every 10 ms
 *
 *****************************************************************************/
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	struct TIMER_T *current, *temp;

	__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);

	current = timers;
	while (current != NULL) {
		if (current->countdown != 0) {
			current->countdown -= 1;
			current = current->next;
		}
		else {
			// Submit timer job to the job queue
 			Add_Job(current->callback, current->payload);

			if (current->type == TIMER_RELOAD) {
				current->countdown = current->timeout;
				current = current->next;
			}
			else if (current->type == TIMER_ONESHOT) {
				temp = current;
				current = current->next;
				Delete_Timer(temp);
			}
		}
	}
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
		Error_Handler();
	}

	// Get the NMEA2000 PGN and the sender id from the extended CAN id
	N2KPgn = (RxHeader.ExtId >> 8) & 0x01FFFF;
	senderId = RxHeader.ExtId & 0xFF;

	// Check for Switch Bank Control message addressed to our channelID
	if ((N2KPgn == 127502) && (RxData[0] == relayId)) {
		// Set the relays and send a new status message
		Set_Relays(RxData);
		statusTimer->countdown = 1;
	}

	// Check for ISO Address Claim message with our senderId
	if (((N2KPgn & 0x1FF00) == 0x0EE00) && (senderId == nmea2kNetwork.senderId)) {
		// If we are waiting for the claimed address, address is in use, send a new claim
		if (nmea2kNetwork.linkState == LINK_STATE_CLAIMED) {
			nmea2kNetwork.linkState = LINK_STATE_IDLE;
			nmea2kNetwork.linkRetries += 1;
			nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
		}
		// If our address is already active, check a couple of things
		if (nmea2kNetwork.linkState == LINK_STATE_ACTIVE) {
			// If the other contender can't change his senderID or has a higher NAME score, we change our senderID
			if (Compare_NameWeight(RxData)) {
				Send_ISOAddressClaim();
			}
			else {
				nmea2kNetwork.linkState = LINK_STATE_IDLE;
				nmea2kNetwork.linkRetries += 1;
				nmea2kNetwork.senderId += nmea2kNetwork.linkRetries;
			}
		}
	}

	struct CAN_MSG_T *msg;

	// Check for ISO Request to our senderId or broadcast id
	if (((N2KPgn & 0x1FF00) == 0x0EA00) && (((N2KPgn & 0xFF) == nmea2kNetwork.senderId) || ((N2KPgn & 0xFF) == 0xFF))) {
		if ((msg = malloc(sizeof(struct CAN_MSG_T))) == NULL) {
			Error_Handler();
		}

		msg->pgn = *(uint32_t *)RxData & 0x00FFFFFF;
		msg->senderId = senderId;

		// Check for a ISO Address Claim request
		if (msg->pgn == 0x00EE00) {
			Add_Job(&Send_ISOAddressClaim, msg);
		}

		// Check for a Product Information request
		if (msg->pgn == 0x01F014) {
			Add_Job(&Send_ProductInformation, msg);
		}

		// Check for a PGN List request
		if (msg->pgn == 0x01EE00) {
			Add_Job(&Send_PGNList, msg);
		}

		// Check for a Binary Switch Bank Status request
		if (msg->pgn == 0x01F20D) {
			Add_Job(&Send_Status, msg);
		}
	}

	// Check for ISO Transport Protocol messages
	if (((N2KPgn & 0x1FF00) == 0x0EC00) || ((N2KPgn & 0x1FF00) == 0x0EB00)) {
		if ((msg = malloc(sizeof(struct CAN_MSG_T))) == NULL) {
			Error_Handler();
		}

		msg->pgn = N2KPgn;
		msg->senderId = senderId;
		memcpy(msg->data, RxData, RxHeader.DLC);

		Add_Job(&Handle_ISOTransportMessages, msg);
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

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, nmea2kNetwork.nameData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

}


/******************************************************************************
 * Send_ISOAcknowledgement
 *
 * Send a ISO Acknowledgement message
 *
 * Parameters:
 * - dest		Destination address
 * - control	Acknowledgement type
 * - group		Group ID
 * - pgn		PGN to acknowledge
 *
 *****************************************************************************/
void Send_ISOAcknowledgement(uint8_t dest, uint8_t control, uint8_t group, uint32_t pgn) {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX0;
	uint32_t			N2KId;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x0E800 << 8) | (dest << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialise the data array with the ISO NAME info
	uint8_t TxData[] = {control, group, 0xFF, 0xFF, 0xFF, (pgn & 0xFF), ((pgn >> 8) & 0xFF), ((pgn >> 16) & 0xFF)};

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

}


/******************************************************************************
 * Send_HeartBeat
 *
 * Send a NMEA2000 Heartbeat message to the network
 *
 *****************************************************************************/
void Send_HeartBeat(void * payload) {
	static int			sequence = 0;
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX1;
	uint32_t			N2KId;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x1F011 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialise the data array with the ISO NAME info
	uint8_t TxData[8];

	*((uint16_t *)TxData) = nmea2kNetwork.heartbeatTime;
	TxData[2] = sequence++;
	TxData[3] = 0b01010011;
	*((uint32_t *)(TxData + 4)) = 0xFFFFFFFF;

	if (sequence > 252)
		sequence = 0;

	// Send the NMEA 60928 ISO Address Claim message
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

	free(payload);
}


/******************************************************************************
 * Send_Status
 *
 * Send a NMEA2000 Binary Switch Bank Status to report the actual relay
 * settings. This report will be send periodicly or after a NMEA2000 Switch
 * Bank Control message
 *
 *****************************************************************************/
void Send_Status(void * payload)
{
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			TxMailbox = CAN_TX_MAILBOX1;
	uint32_t			N2KId;
	int 				offset, index;

	// Initialize the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (3 << 26) | (127501 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialize the status array with the relay instance id
	uint8_t TxData[] = {relayId, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	// Fill the status array according to PGN 127501
	for (int i = 0; i < RELAY_CHANNELS; i++) {
		offset = 2 * (i % 4);
		index = 1 + (i / 4);
		if ((relayStatus & (1 << i)) != 0) {
			TxData[index] &= ~(0b10 << offset);
		}
		else {
			TxData[index] &= ~(0b11 << offset);
		}
	}

	// Send the NMEA 127501 Binary Switch Bank Status message
 	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

	free(payload);
}


void Send_ProductInformation(void * payload) {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			N2KId;
	uint8_t				*data;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x1F014 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Allocate data structure
	if ((data = malloc(134 * sizeof(uint8_t))) == NULL) {
		Error_Handler();
	}

	// Populate data structure
	*(uint16_t *)data = 3000;					// NMEA2000 v3.000
	*(uint16_t *)(data + 2) = 101;				// Model number 1.01
	memcpy(data + 4, productInformation, 128);	// 128 bytes of product information
	*(data + 132) = 0xFF;						// No official NMEA2000 certification
	*(data + 133) = 4;							// Guaranteed max load = 4 * 50mA = 200mA

	// Send as Fast Frame message
	Send_FFMessage(&TxHeader, data, 0, 134);

	free(data);
	free(payload);
}


void Send_PGNList(void * payload) {
	CAN_TxHeaderTypeDef	TxHeader;
	uint32_t			N2KId;
	uint8_t				*data;
	int 				length;

	// Initialise the CAN header with the NMEA2K info
	// Can ID = PRIO(3) - 0 - PGN(17) - Sender ID(8)
	N2KId = (6 << 26) | (0x1EE00 << 8) | nmea2kNetwork.senderId;
	TxHeader.ExtId = N2KId;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	for (int i = 0; i < 2; i++) {
		for (length = 0; pgnList[i][length] != 0; length++);

		// Allocate data structure
		if ((data = malloc(1 + (length * 3))) == NULL) {
			Error_Handler();
		}

		// Populate data structure
		data[0] = (uint8_t)i;
		for (int j = 0; j < length; j++) {
			memcpy(data + 1 + (j * 3), &pgnList[i][j], 3);
		}

		Send_FFMessage(&TxHeader, data, 0, 1 + (length * 3));
		free(data);
	}

	free(payload);
}


int Send_FFMessage(CAN_TxHeaderTypeDef *TxHeader, uint8_t *data, int seqNo, int len) {
	uint32_t TxMailbox = CAN_TX_MAILBOX0;
	int offset = 0, frameCounter = 0;
	uint8_t TxData[8];

	// Check validity of len parameter
	if ((len > 223) || (len < 9))
		return frameCounter;

	// Frame length is always 8 data bytes
	TxHeader->DLC = 8;

	// Fill TxData for first package
	TxData[0] = (seqNo << 5) | (frameCounter++ & 0x1F);
	TxData[1] = len & 0xFF;
	memcpy(&TxData[2], data, ((len - offset) < 6)?(len - offset):6);

	// Pad till 8 bytes
	for (int i = (len - offset) + 1; i < 8; i++) {
		TxData[i] = 0xFF;
	}

	offset += ((len - offset) < 6)?(len - offset):6;

	// Send the NMEA2000 FF Frame
	if (HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, &TxMailbox) != HAL_OK) {
		Error_Handler();
	}

	while (offset < len) {
		// Delay for 1 ms to prevent buffer flooding (non critical)
		HAL_Delay(10);

		// Fill TxData for additional packages
		TxData[0] = (seqNo << 5) | (frameCounter++ & 0x1F);
		memcpy(&TxData[1], data + offset, ((len - offset) < 7)?(len - offset):7);

		// Pad till 8 bytes
		for (int i = (len - offset) + 1; i < 8; i++) {
			TxData[i] = 0xFF;
		}

		offset += ((len - offset) < 7)?(len - offset):7;

		// Send the NMEA2000 FF Frame
		if (HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, &TxMailbox) != HAL_OK) {
			Error_Handler();
		}
	}

	// Return number of frames send
	return frameCounter;
}


void Handle_ISOTransportMessages(void *payload) {
	struct CAN_MSG_T *msg = payload;
	static struct ISO_TRANSPORT_MESSAGE *msgList = NULL;
	struct ISO_TRANSPORT_MESSAGE *currentMsg, *newMsg;
	int offset, byteCount;

	// BAM announcement
	if (((msg->pgn & 0x1FF00) == 0x0EC00) && (msg->data[0] == 0x20)) {
		// Find the last message to insert after
		if ((currentMsg = msgList) != NULL) {
			while (currentMsg->next != NULL)
				currentMsg = currentMsg->next;
		}

		// Create the new message structure
		if ((newMsg = malloc(sizeof(struct ISO_TRANSPORT_MESSAGE))) == NULL) {
			Error_Handler();
		}

		// Fill the structure
		newMsg->senderId = msg->senderId;
		newMsg->isoCommand = msg->data[0];
		newMsg->packetsExpected = msg->data[3];
		newMsg->packetsReceived = 0;
		newMsg->messageSize = (uint16_t)msg->data[1];
		newMsg->pgn = (((uint32_t)msg->data[4]) >> 8);
		newMsg->prev = currentMsg;
		newMsg->next = NULL;

		// Create the data structure
		if ((newMsg->data = malloc(newMsg->messageSize * sizeof(uint8_t))) == NULL) {
			Error_Handler();
		}

		// Link newMsg in the list
		if (currentMsg == NULL) {
			msgList = newMsg;
		}
		else {
			currentMsg->prev = newMsg;
		}
	}

	// Data package
	if ((msg->pgn & 0x1FF00) == 0x0EB00) {
		currentMsg = msgList;
		// Find the right message
		while(currentMsg != NULL) {
			if (currentMsg->senderId == msg->senderId) {
				offset = (msg->data[0] - 1) * 7;
				if ((byteCount = currentMsg->messageSize - (msg->data[0] * 7)) > 7)
					byteCount = 7;
				memcpy(currentMsg->data + offset, msg->data + 1, byteCount);
				currentMsg->packetsReceived += 1;

				// Check if message is complete
				if (currentMsg->packetsReceived == currentMsg->packetsExpected) {
					// Check for ISO Commanded Address
					if (currentMsg->pgn == 65240)
						Handle_ISOCommandedAddress(currentMsg->data);

					// Cleanup memory and restore linked list
					if (currentMsg->prev == NULL)
						msgList = currentMsg->next;
					else
						currentMsg->prev->next = currentMsg->next;

					if (currentMsg->next != NULL)
						currentMsg->next->prev = currentMsg->prev;

					free(currentMsg->data);
					free(currentMsg);
				}
				break;
			}
		}
	}

	free(payload);
}


void Handle_ISOCommandedAddress(uint8_t *data) {
	if (*(uint64_t *)nmea2kNetwork.nameData == *(uint64_t *)data) {
		nmea2kNetwork.senderId = data[8];
		nmea2kNetwork.nameData[7] &= ~(1 << 7);
		nmea2kNetwork.linkState = LINK_STATE_IDLE;
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
	return (*(uint64_t *)nmea2kNetwork.nameData <= *(uint64_t *)data);
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
		offset = 2 * (i % 4);
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


struct TIMER_T * Add_Timer(int type, int timeout, void(*callback)(), void * payload) {
	struct TIMER_T *current, *new;

	// Start atomic function
	__disable_irq();
	if ((new = malloc(sizeof(struct TIMER_T))) == NULL)
		return NULL;

	new->type = type;
	new->timeout = timeout;
	new->countdown = timeout;
	new->callback = callback;
	new->payload = payload;

	if (timers == NULL) {
		new->prev = NULL;
		new->next = NULL;
		timers = new;
	}
	else {
		current = timers;
		while (current->next != NULL)
			current = current->next;
		new->prev = current;
		new->next = NULL;
		current->next = new;
	}
	__enable_irq();
	// End atomic function

	return new;;
}


void Delete_Timer(struct TIMER_T *timer) {
	// Start atomic function
	__disable_irq();
	if (timer->prev == NULL)
		timers = timer->next;
	else
		timer->prev->next = timer->next;

	if (timer->next != NULL)
		timer->next->prev = timer->prev;

	free(timer);
	__enable_irq();
	// End atomic function
}


struct JOB_QUEUE_T * Add_Job(void(*callback)(), void * payload) {
	struct JOB_QUEUE_T *current, *new;

	// Start atomic function
	__disable_irq();
	if ((new = malloc(sizeof(struct JOB_QUEUE_T))) == NULL)
		return NULL;

	new->callback = callback;
	new->payload = payload;
	new->next = NULL;

	if (jobs == NULL)
		jobs = new;
	else {
		current = jobs;
		while (current->next != NULL)
			current = current->next;
		current->next = new;
	}
	__enable_irq();
	// End atomic function

	return new;
}

void Delete_Job(struct JOB_QUEUE_T *job) {
	struct JOB_QUEUE_T *current;

	// Start atomic function
	__disable_irq();
	if (jobs == NULL) {}	// Do nothing
	else if (jobs == job)
		jobs = job->next;
	else {
		current = jobs;
		while (current->next != job)
			current = current->next;
		current->next = job->next;
	}

	free(job);
	__enable_irq();
	// End atomic function
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
	// Initialize the return value with the high nibble of RELAY_BOARD_ID
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
void Error_Handler()
{
  volatile int a = 1, b = 7;
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  for (int i = 0; i < 2000000; i++) {
		  a = a * b;
	  }
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
