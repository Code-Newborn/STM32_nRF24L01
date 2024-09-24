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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "nRF24L01P.h"

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

/* USER CODE BEGIN PV */

nRF24L01P     myNRF;
const uint8_t RXAddr[ 5 ] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
const uint8_t TXAddr[ 5 ] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

uint8_t RXBuffer[ 32 ], TXBuffer[ 32 ], regStatus, uartSent;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void ) {

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
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

    /* ---- myNRF24L01+ Definitions 对象管理 ---- */
    myNRF.hspi       = &hspi1;  // You should make this definition at CubeMX
    myNRF.CRC_Width  = nRF_CRC_WIDTH_BYTE;
    myNRF.ADDR_Width = nRF_ADDR_WIDTH_5;
    myNRF.Data_Rate  = nRF_DATA_RATE_2MBPS;
    myNRF.TX_Power   = nRF_TX_PWR_0dBm;
    myNRF.State      = nRF_STATE_RX;

    myNRF.RF_Channel      = 2;
    myNRF.PayloadWidth    = nRF_RXPW_32BYTES;
    myNRF.RetransmitCount = nRF_RETX_DISABLED;
    myNRF.RetransmitDelay = nRF_RETX_DELAY_1000uS;

    myNRF.RX_Address = ( uint8_t* )RXAddr;
    myNRF.TX_Address = ( uint8_t* )TXAddr;

    myNRF.RX_Buffer = RXBuffer;
    myNRF.TX_Buffer = TXBuffer;

    myNRF.nRF_nSS_GPIO_PORT = nRF_nSS_GPIO_Port;  // You should make this definition at CubeMX
    myNRF.nRF_nSS_GPIO_PIN  = nRF_nSS_Pin;        // You should make this definition at CubeMX

    myNRF.nRF_CE_GPIO_PORT = nRF_CE_GPIO_Port;  // You should make this definition at CubeMX
    myNRF.nRF_CE_GPIO_PIN  = nRF_CE_Pin;        // You should make this definition at CubeMX

    /* ---- myNRF24L01+ Definitions ---- */

    if ( HAL_nRF24L01P_Init( &myNRF ) != HAL_OK ) {
        HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_SET );
        HAL_UART_Transmit( &huart1, ( uint8_t* )"nRF24 Init fucked UP!\r\n", 23, 250 );
        Error_Handler();
    }
    else {
        HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET );
        HAL_UART_Transmit( &huart1, ( uint8_t* )"nRF24 Init is OK!\r\n", 19, 250 );
    }
    uartSent = 1;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while ( 1 ) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if ( ( myNRF.RX_Buffer[ 0 ] != 0x00 ) && !myNRF.Busy && uartSent ) {
            HAL_UART_Transmit( &huart1, myNRF.RX_Buffer, 32, 250 );
            uartSent = 0;
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void ) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 168;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK ) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_5 ) != HAL_OK ) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {
    if ( GPIO_Pin == GPIO_PIN_13 ) {
        if ( HAL_nRF24L01P_IRQ_Handler( &myNRF ) != HAL_OK ) {
            Error_Handler();
        }
        HAL_nRF24L01P_ReadRegister( &myNRF, nRF_STATUS, &regStatus );
        HAL_UART_Transmit( &huart1, ( uint8_t* )"Status >", 8, 250 );
        HAL_UART_Transmit( &huart1, &regStatus, 1, 250 );
        uartSent = 1;
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void ) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while ( 1 ) {
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
void assert_failed( uint8_t* file, uint32_t line ) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
