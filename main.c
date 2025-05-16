
#include "main.h"
#include "stdio.h"
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

#define ALPHA 0.8f
void Set_duty_cycle(uint16_t d_CY){

	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
	//uint32_t arr =65535;
	uint32_t ccr = (d_CY * arr) / 100;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
}

void marche_MCC_CCW(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void marche_MCC_CW(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void MCC_dem(uint16_t d_Cy){
	marche_MCC_CW();
	Set_duty_cycle(d_Cy);
}

void stop_MCC(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

}

void update_vitesse(float speed,int32_t last_pos,int32_t new_pos,int16_t Te){
	volatile int16_t delta;
	    delta = new_pos - last_pos;
	    new_pos = (__HAL_TIM_GET_COUNTER(&htim2) /1170);
	        delta = new_pos - last_pos;
	        speed = delta /(Te/1000); // en tr/min si delay = 100ms
	        last_pos = new_pos;
}


/*uint16_t pwm = 24;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	MCC_dem(pwm);
	//HAL_Delay(1000);
}*/



/*int32_t calcul_vitesse(volatile int16_t new_pos,volatile int16_t last_pos,int16_t Te){
	volatile int16_t delta;
	int16_t speed;
    delta = new_pos - last_pos;
   // new_pos = (__HAL_TIM_GET_COUNTER(&htim2) / 1170);
        delta = new_pos - last_pos;
        speed = delta /(Te/1000); // en tr/min si delay = 100ms
        last_pos = new_pos;
       // HAL_Delay(Te);
        return speed;
}*/

/*int32_t calcul_vitesse(int32_t new_pos, int32_t *last_pos, int32_t Te_ms) {
    int32_t delta = new_pos - *last_pos;
    *last_pos = new_pos;
    float Te_s = Te_ms / 1000.0; // conversion en secondes
    float speed = delta / Te_s;  // ticks/sec
    float tr_min = (speed / 4680.0) * 60.0; // 1170 ticks/tr
    return (int32_t)tr_min;
}*/

typedef struct{
	int16_t velocity;
	int16_t position;
	uint32_t last_counter_value;
}encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
 {
uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
static uint8_t first_time = 0;
if(!first_time)
{
   encoder_value ->velocity = 0;
   first_time = 1;
}
else
{
  if(temp_counter == encoder_value ->last_counter_value)
  {
    encoder_value ->velocity = 0;
  }
  else if(temp_counter > encoder_value ->last_counter_value)
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
      encoder_value ->velocity = -encoder_value ->last_counter_value -
	(__HAL_TIM_GET_AUTORELOAD(htim)-temp_counter);
    }
    else
    {
      encoder_value ->velocity = temp_counter -
           encoder_value ->last_counter_value;
    }
  }
  else
  {
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
	encoder_value ->velocity = temp_counter -
            encoder_value ->last_counter_value;
    }
    else
    {
	encoder_value ->velocity = temp_counter +
	(__HAL_TIM_GET_AUTORELOAD(htim) -
              encoder_value ->last_counter_value);
    }
   }
}
encoder_value ->position += encoder_value ->velocity;
encoder_value ->last_counter_value = temp_counter;
 }


/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
int16_t timer_counter,encoder_position,encoder_velocity;
 encoder_instance enc_instance_mot ;

  timer_counter = __HAL_TIM_GET_COUNTER(&htim3);
  // measure velocity, position
  update_encoder(&enc_instance_mot, &htim3);
  encoder_position = enc_instance_mot.position;
  encoder_velocity = enc_instance_mot.velocity;
}*/








void send_uart(uint32_t *data) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, 4, 10);
}


int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  //HAL_TIM_PeriodElapsedCallback(&htim3);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

//volatile int16_t DCY=2;
   volatile float speed=0;
  // volatile int16_t new_pos=0;
   //volatile int16_t last_pos=0;
   int16_t u_cmd, Te_ms = 100;
float Te = (float)(Te_ms) / 1000;

   int16_t speed_cmd = 45;   //  tr/min

  // float Kp = 0.0533343f, Ki = 0.0711743f,  Kd = 0.0f; //--> old

   float Kp = 0.0503343f, Ki = 0.0811743f,  Kd = 0.013f;
   //float Kp =0.6f, Ki = 0.0751f,  Kd = 0.0f;
float Err=0,Err_preced=0,Som_Err=0;
uint32_t temp_counter;
static uint8_t first_time = 0;
encoder_instance encodeur = {0};
encoder_instance *encoder_value = &encodeur;
float u;
static float speed_filtered = 0;
  while (1)
  {


 temp_counter = __HAL_TIM_GET_COUNTER(&htim2);
	  if(!first_time)
	  {
	     encoder_value ->velocity = 0;
	     first_time = 1;
	  }
	  else
	  {
	    if(temp_counter == encoder_value ->last_counter_value)
	    {
	      encoder_value ->velocity = 0;
	    }
	    else if(temp_counter > encoder_value ->last_counter_value)
	    {
	      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
	      {
	        encoder_value ->velocity = -encoder_value ->last_counter_value -
	  	(__HAL_TIM_GET_AUTORELOAD(&htim2)-temp_counter);
	      }
	      else
	      {
	        encoder_value ->velocity = temp_counter -
	             encoder_value ->last_counter_value;
	      }
	    }
	    else
	    {
	      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
	      {
	  	encoder_value ->velocity = temp_counter -
	              encoder_value ->last_counter_value;
	      }
	      else
	      {
	  	encoder_value ->velocity = temp_counter +
	  	(__HAL_TIM_GET_AUTORELOAD(&htim2) -
	                encoder_value ->last_counter_value);
	      }
	     }
	  }
	  encoder_value ->position += encoder_value ->velocity;
	  encoder_value ->last_counter_value = temp_counter;

	  /////////////////////////////////////////////--> CALCUL VITESSE

///////////////////////////////////////////////77


	  speed_filtered = ALPHA * speed_filtered + (1 - ALPHA) * encoder_value ->velocity;
	  speed = ( speed_filtered* 60) / (1170 * 2 * Te);
	//  speed = (encoder_value ->velocity * 60) / (1170 * 2 * Te);

	   Err= speed_cmd - speed;
		 Som_Err += Err* Te;
		 float delta_erreur = (Err - Err_preced)/ Te;
	     Err_preced = Err;
	  u = Kp * Err + Ki * Som_Err + Kd * delta_erreur;
		if (u >30) u_cmd = 30;
		else if (u < 0) u_cmd = 0;

		else u_cmd = (uint16_t)u;

		MCC_dem(u_cmd);
	//  if (Err >0) u_cmd = 30;
	  //	else if (Err < 0) u_cmd = 0;
	 // MCC_dem(u_cmd);

        HAL_Delay(Te_ms);
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  // htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
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
