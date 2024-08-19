
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"

void SystemClock_Config(void);
#define FRAME_LENGTH 25
uint8_t RxData[FRAME_LENGTH];
//保证至少有一个完整的帧
uint8_t InitializationData[FRAME_LENGTH * 2] = {0};
uint8_t Initialization = 0;
uint16_t channels[16];

/* sbus frame Initializing state */
typedef enum {
  NotInitialized = 0u, // not started
  Initializing , // started 
  Initialized //ended
} FrameInitializationState;

FrameInitializationState InitializationState = NotInitialized;
uint8_t InitializationBytesToDiscard = 0;
uint8_t FrameStart = 0x0F;
uint8_t FrameEnd = 0x00;

int main(void)
{
  HAL_Init();


  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  
  //先接收到临时数组进行解析
  HAL_UART_Receive_DMA (&huart1, InitializationData, FRAME_LENGTH * 2);

  while (1)
  {

  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1){
    //未初始化
     if(InitializationState == NotInitialized){
       //帧头和尾匹配
      if(InitializationData[0] == FrameStart && InitializationData[FRAME_LENGTH - 1] == FrameEnd){
        //协议匹配完成，开始读取下一帧
        InitializationState = Initialized;
        Initialization = 1;
        HAL_UART_Transmit_DMA(&huart3, &Initialization, 1);
        HAL_UART_Receive_DMA (&huart1, RxData, FRAME_LENGTH);
      }else{
        //查找帧头
        for(int i = 1; i < FRAME_LENGTH * 2; i++){
          //可能是帧头
          
          if(InitializationData[i] == FrameStart){
            int endIndex = i + FRAME_LENGTH - 1;
            if(endIndex < FRAME_LENGTH * 2 && InitializationData[endIndex] == FrameEnd){
              //协议匹配完成，开始读取下一帧
              InitializationState = Initializing;
              //需要丢弃的字节数
              InitializationBytesToDiscard = FRAME_LENGTH - i;
              Initialization = 2;
              HAL_UART_Transmit_DMA(&huart3, &Initialization, 1);
              HAL_UART_Receive_DMA (&huart1, RxData, InitializationBytesToDiscard);
              break;
            }
          }
        }
      }
     }else if(InitializationState == Initializing){
      if(InitializationData[InitializationBytesToDiscard - 1] == 0x00){
        //协议匹配完成，开始读取下一帧
        InitializationState = Initialized;
        Initialization = 3;
        HAL_UART_Transmit_DMA(&huart3, &Initialization, 1);
        HAL_UART_Receive_DMA (&huart1, RxData, FRAME_LENGTH);
      }else{
        InitializationState = NotInitialized;
        //先接收到临时数组进行解析
        Initialization = 4;
        HAL_UART_Transmit_DMA(&huart3, &Initialization, 1);
        HAL_UART_Receive_DMA (&huart1, InitializationData, FRAME_LENGTH * 2);
      }
     }else{
       Initialization = 5;
        channels[0]  = (((RxData[2]  << 8) | (RxData[1])) & (0x07FF));
        channels[1]  = (((RxData[2]  >> 3) | (RxData[3]  << 5)) & (0x07FF));
        channels[2]  = ((((RxData[3]  >> 6) | (RxData[4]  << 2))  | (RxData[5] << 10)) & (0x07FF));
        channels[3]  = (((RxData[5]  >> 1) | (RxData[6]  << 7)) & (0x07FF));
        channels[4]  = (uint16_t)((RxData[6]  >> 4) | (RxData[7]  << 4) & (0x07FF));
        channels[5]  = (uint16_t)((RxData[7]  >> 7) | (RxData[8]  << 1)  | (RxData[9] << 9) & (0x07FF));
        channels[6]  = (uint16_t)((RxData[9]  >> 2) | (RxData[10] << 6) & (0x07FF));
        channels[7]  = (uint16_t)((RxData[10] >> 5) | (RxData[11] << 3) & (0x07FF));
        channels[8]  = (uint16_t)((RxData[12])      | (RxData[13] << 8) & (0x07FF));
        channels[9]  = (uint16_t)((RxData[13] >> 3) | (RxData[14] << 5) & (0x07FF));
        channels[10] = (uint16_t)((RxData[14] >> 6) | (RxData[15] << 2)  | (RxData[16] << 10) & (0x07FF));
        channels[11] = (uint16_t)((RxData[16] >> 1) | (RxData[17] << 7) & (0x07FF));
        channels[12] = (uint16_t)((RxData[17] >> 4) | (RxData[18] << 4) & (0x07FF));
        channels[13] = (uint16_t)((RxData[18] >> 7) | (RxData[19] << 1)  | (RxData[20] << 9) & (0x07FF));
        channels[14] = (uint16_t)((RxData[20] >> 2) | (RxData[21] << 6) & (0x07FF));
        channels[15] = (uint16_t)((RxData[21] >> 5) | (RxData[22] << 3) & (0x07FF));
       HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&channels, 2);
       HAL_UART_Receive_DMA (&huart1, RxData, FRAME_LENGTH);
     }
  }
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
