/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_it.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define REC_BUFF_SIZE 128
#define SPI_BUFF_SIZE 16
#define USART_TIME_OUT_VALUE 100

/*enum for flag status */
typedef enum {
	false, true
} flag_state_t;

/*enum for commands */
typedef enum {
	READ, WRITE
} cmd_t;

/*enum for type of sensor */
typedef enum {
	LED, TEMP
} sensor_t;

/*enum for frame structure  */
typedef enum {
	SYNC_BYTE_1, SYNC_BYTE_2, CMD_BYTE, SENSOR_BYTE, DATA_BYTE,DATA_BYTE_2, CRC_HI, CRC_LO
} frame_struct_t;

/*enum for frame structure  */
typedef enum {
	REST,PROC,RESPONSE,SPI_TX,SPI_RX
} app_cmd_state_t;


/*enum for frame structure  */
typedef enum {
	OK,VAL,TEMPR
} spi_response_t;

/*Flags for application   */
typedef struct __attribute__((__packed__)) {
	uint8_t process_usart;
} flags_ctx_t;

/*struc for serial comms   */
typedef struct __attribute__((__packed__)) {
	uint8_t rec_buff[REC_BUFF_SIZE];
	uint8_t tx_buff[REC_BUFF_SIZE];
	flags_ctx_t flags;
	uint8_t rec_index;
	uint8_t cmd_fom_serial;
	uint8_t sensor_form_serial;
	uint8_t data;
	uint8_t lock;
	uint16_t rx_timer;
} serial_comms_t;

/*Structure for application */
typedef struct __attribute__((__packed__)){

	serial_comms_t uart;
	serial_comms_t spi;
	uint16_t led_timer;
	uint8_t switch_state;
	uint16_t lock_timer;

} app_ctx_t;

extern app_ctx_t app;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define GREEN_LED_OFF     HAL_GPIO_WritePin (LED3_GPIO_Port, LED3_Pin,GPIO_PIN_SET);
#define GREEN_LED_ON      HAL_GPIO_WritePin (LED3_GPIO_Port, LED3_Pin,GPIO_PIN_RESET);
#define GREEN_LED_TOGGLE  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
#define READ_GREEN_LED    HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
