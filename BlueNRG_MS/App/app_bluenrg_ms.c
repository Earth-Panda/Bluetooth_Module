/**
  ******************************************************************************
  * @file    app_bluenrg_ms.c
  * @author  SRA Application Team
  * @brief   BlueNRG-M0 initialization and applicative code
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

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_ms.h"

#include "hci_tl.h"
#include "sample_service.h"
#include "role_type.h"
#include "bluenrg_utils.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/**
 * Define the role here only if it is not already defined in the project options
 * For the CLIENT_ROLE comment the line below
 * For the SERVER_ROLE uncomment the line below
 */
#define SERVER_ROLE

#define BDADDR_SIZE 6

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

#ifdef SERVER_ROLE
  BLE_RoleTypeDef BLE_Role = SERVER;
#else
  BLE_RoleTypeDef BLE_Role = CLIENT;
#endif

extern volatile uint8_t set_connectable;
extern volatile int     connected;
extern volatile uint8_t notification_enabled;

extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Process(float channel_data[64]);
static void User_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (DK GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02ld:%02ld:%02ld.%03ld", (long)(ms/(60*60*1000)%24), (long)(ms/(60*1000)%60), (long)((ms/1000)%60), (long)(ms%1000));
}
#endif

void MX_BlueNRG_MS_Init(void)
{
	/* USER CODE BEGIN SV */

	/* USER CODE END SV */

	/* USER CODE BEGIN BlueNRG_MS_Init_PreTreatment */

	/* USER CODE END BlueNRG_MS_Init_PreTreatment */

	/* Initialize the peripherals and the BLE Stack */
	uint8_t SERVER_BDADDR[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
	uint8_t bdaddr[BDADDR_SIZE];
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

	uint8_t  hwVersion;
	uint16_t fwVersion;
	int ret;

	User_Init();

	/* Get the User Button initial state */
	user_button_init_state = BSP_PB_GetState(BUTTON_KEY);

	hci_init(user_notify, NULL);

	/* get the BlueNRG HW and FW versions */
	getBlueNRGVersion(&hwVersion, &fwVersion);

	/*
	* Reset BlueNRG again otherwise we won't
	* be able to change its MAC address.
	* aci_hal_write_config_data() must be the first
	* command after reset otherwise it will fail.
	*/
	hci_reset();

	HAL_Delay(100);

	printf("HWver %d, FWver %d\n", hwVersion, fwVersion);

	if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
		bnrg_expansion_board = IDB05A1;
	}

	BLUENRG_memcpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));

	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
								  CONFIG_DATA_PUBADDR_LEN,
								  bdaddr);
	if (ret) {
		printf("Setting BD_ADDR failed 0x%02x.\n", ret);
	}

	ret = aci_gatt_init();
	if (ret) {
		printf("GATT_Init failed.\n");
	}

	if (bnrg_expansion_board == IDB05A1) {
		ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}
	else {
		ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}

	if (ret != BLE_STATUS_SUCCESS) {
		printf("GAP_Init failed.\n");
	}

	ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
									 OOB_AUTH_DATA_ABSENT,
									 NULL,
									 7,
									 16,
									 USE_FIXED_PIN_FOR_PAIRING,
									 123456,
									 BONDING);
	if (ret == BLE_STATUS_SUCCESS) {
		printf("BLE Stack Initialized.\n");
	}

	printf("SERVER: BLE Stack Initialized\n");
	ret = Add_Sample_Service();

	if (ret == BLE_STATUS_SUCCESS){
		printf("Service added successfully.\n");
	} else {
		printf("Error while adding service.\n");
	}

	/* Set output power level */
	ret = aci_hal_set_tx_power_level(1,4);

	/* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */

	/* USER CODE END BlueNRG_MS_Init_PostTreatment */
}

/*
 * BlueNRG-MS background task
 */
void MX_BlueNRG_MS_Process(float channel_data[64])
{
  /* USER CODE BEGIN BlueNRG_MS_Process_PreTreatment */

  /* USER CODE END BlueNRG_MS_Process_PreTreatment */

  User_Process(channel_data);
  hci_user_evt_proc();

  /* USER CODE BEGIN BlueNRG_MS_Process_PostTreatment */

  /* USER CODE END BlueNRG_MS_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  BSP_LED_Init(LED2);

  BSP_COM_Init(COM1);
}

/**
 * @brief  Configure the device as Client or Server and manage the communication
 *         between a client and a server.
 *
 * @param  None
 * @retval None
 */
static void User_Process(float channel_data[64])
{
  if (set_connectable)
  {
    /* Establish connection with remote device */
    Make_Connection();
    set_connectable = FALSE;
    user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
  }

  /* Check if the User Button has been pushed */
    /* Debouncing */
    HAL_Delay(50);

    /* Wait until the User Button is released */
    while (BSP_PB_GetState(BUTTON_KEY) == !user_button_init_state);

    /* Debouncing */
    HAL_Delay(50);

    if (connected)
    {
    	/* Handshake */
    	uint8_t start[1] = {'F'};
    	uint8_t end[1] = {'L'};
    	uint8_t ble_buf[500];
		uint8_t ble_buf_len = 0;

		sendData(start, sizeof(start));

    	for (int i = 0; i < 64; i++){
    		ble_buf_len = sprintf(ble_buf, "%f", channel_data[i]);
    		sendData(ble_buf, ble_buf_len);
    		HAL_Delay(50);
    	}

//		for (int i = 0; i < 64; i++){
//			ble_buf_len += sprintf (ble_buf + ble_buf_len, "%d,", channel_data[i]);
//		}
//		sendData(ble_buf, ble_buf_len);
//
    	sendData(end, sizeof(end));
      //BSP_LED_Toggle(LED2);  /* Toggle the LED2 locally. */
                               /* If uncommented be sure the BSP_LED_Init(LED2)
                                * is called in main().
                                * E.g. it can be enabled for debugging. */
    }
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Set the User Button flag */
  user_button_pressed = 1;
}
