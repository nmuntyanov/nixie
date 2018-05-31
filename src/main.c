
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define REV_1

#ifdef REV_0
	#define DC_PINS ((uint16_t) (GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6))
#endif

#ifdef REV_1
	#define DC_PINS ((uint16_t) (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15))
#endif

QueueHandle_t xEncoderQueue;
QueueHandle_t xDisplayQueue;

void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void GeneratorTask(void * pvParameters); // Display output task
void PushButtonTask(void * pvParameters); // checks for button pushing and menu context
void LedBlinkerTask(void * pvParameters); // blinks with display leds
void ClockEditTask(void * pvParameters); // clock edit task
void LedPwmTask(void * pvParameters);
void parseTime(uint32_t * RTC_time, uint8_t *parsedTime);

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_USART3, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	USART_InitTypeDef USART_InitStruct;

	#ifdef REV_0
		uint16_t CCR1_Val = 500;
	#endif
	#ifdef REV_1
		uint16_t CCR1_Val = 50;
	#endif
	uint16_t CCR2_Val = 80;
	uint16_t PrescalerValue = 0;

	// PWM pins configuration
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Lamps power supply pins configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Push button pin configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Led pin configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// DC pins configuration
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = DC_PINS;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	#ifdef REV_0
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	#endif

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 				// Pin 9 (TX)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		// the pins are configured as alternate function so the USART peripheral has access to them, Push pull output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 800;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

	GPIO_ResetBits(GPIOA, GPIO_Pin_0| GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	GPIO_SetBits(GPIOB, (uint16_t) DC_PINS);

	USART_InitStruct.USART_BaudRate = 9600;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx; // we want to enable the transmitter and the receiver
	USART_Init(USART3, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART3, ENABLE);

	// ------------------------- NVIC configuration

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY + 11;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY + 10;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// ----------------------- RTC configuration

	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	RCC_LSEConfig(RCC_LSE_ON);
	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);
	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/* Set RTC prescaler: set RTC period to 1sec */
	RTC_SetPrescaler(32767);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* To output second signal on Tamper pin, the tamper functionality
	   must be disabled (by default this functionality is disabled) */
	BKP_TamperPinCmd(DISABLE);
	/* Enable the RTC Second Output on Tamper Pin */
	BKP_RTCOutputConfig(BKP_RTCOutputSource_Second);

	// ---------- ENCODER INIT

	//Channels TIM1_CH1, TIM1_CH2 as pull up inputs
	GPIO_InitTypeDef GPIO_InitDef;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitDef.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitDef);

	/* TIM1 tunning*/
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	// set TIM_Period - encoder rotate count reset
	TIMER_InitStructure.TIM_Period = 4;
	// backward and forward count
	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up | TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM1, &TIMER_InitStructure);

	/* Tuning Encoder Interface */
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

	TIM_ICInitTypeDef TIM_ICI;
	TIM_ICStructInit(&TIM_ICI);
	TIM_ICI.TIM_Channel = TIM_Channel_1;
	TIM_ICI.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICI.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICI.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICI.TIM_ICFilter = 0x0F;
	TIM_ICInit(TIM1, &TIM_ICI);
	TIM_ICI.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM1, &TIM_ICI);

    TIM_Cmd(TIM1, ENABLE);

	xEncoderQueue = xQueueCreate( 10, sizeof( short int ) );
	xDisplayQueue = xQueueCreate( 3, sizeof( uint8_t ) * 4 );

	USART_puts(USART3, "INITED!");
	xTaskCreate(GeneratorTask, "generator", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(PushButtonTask, "pushButton", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	NVIC_EnableIRQ(RTC_IRQn);
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	RTC_WaitForLastTask();

	vTaskStartScheduler();

	for(;;);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s)
{
	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s++);
	}
}

void RTC_IRQHandler(void)
{
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
		uint32_t time = RTC_GetCounter();
		uint8_t display[4] = { 0x00 };
		parseTime(&time, &display);

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSendFromISR( xDisplayQueue, &display, &xHigherPriorityTaskWoken );

		/* Clear Interrupt pending bit */
		RTC->CRL &= (uint16_t)~RTC_FLAG_SEC;
	}
}

void parseTime(uint32_t *RTC_time, uint8_t *parsedTime)
{
	uint32_t t1;
	uint32_t time = * RTC_time;
	uint8_t hours, minutes = 0;
	t1 = time/60;
	time = t1;
	t1 = time/60;
	minutes = time - t1 * 60;
	time = t1;
	t1 = time / 24;
	hours = time - t1 * 24;

	parsedTime[0] = hours / 10;
	parsedTime[1] = hours % 10;
	parsedTime[2] = minutes / 10;
	parsedTime[3] = minutes % 10;
}

void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		short count = (TIM1->CR1 & TIM_CR1_DIR) ? 1 : 2;
		xQueueSendFromISR( xEncoderQueue, &count, &xHigherPriorityTaskWoken );

		USART_SendData(USART3, count);
		TIM1->SR = (uint16_t)~TIM_IT_Update;
	}
}

void GeneratorTask(void * pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint16_t ODRA = 0x0000;
	uint16_t ODRB = 0x0000;
	uint8_t digit = 0x00;
	uint8_t PINS = 0x00;
	uint8_t display[4] = {0x00};

	while (1) {
		PINS = 0x01;
		xQueueReceive( xDisplayQueue, &display, 0 );
		for (digit = 0x04; digit > 0x00; digit--) {
			ODRA = GPIOA->ODR & 0xFFF0;
			GPIOA->ODR = ODRA;
			vTaskDelayUntil( &xLastWakeTime, (TickType_t) 1 );
			ODRB = GPIOB->ODR & ~DC_PINS; // xxxx xxxx x000 0xxx
			GPIOB->ODR |= DC_PINS;

			ODRA = ODRA | (uint16_t) PINS;
			#ifdef REV_0
				ODRB = ODRB | (uint16_t) (display[(digit - 1)] << 3);
			#endif
			#ifdef REV_1
				ODRB = ODRB | (uint16_t) (display[(digit - 1)] << 12);
			#endif

			GPIOB->ODR = ODRB;
			vTaskDelayUntil( &xLastWakeTime, (TickType_t) 1 );
			GPIOA->ODR = ODRA;
			PINS = PINS << 1;
			vTaskDelayUntil( &xLastWakeTime, (TickType_t) 3 );
		}
	}
}

void PushButtonTask(void * pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t xFrequency = 150;
	TaskHandle_t xMenuHandle = NULL;
	uint8_t menu = 0x00;

	while (1) {
		/**
		 * Check if button is pressed first time
		 * if so - need to wait 1 tick to filter dummy impulses
		 */
		if ( (GPIOA->IDR & GPIO_Pin_10) == Bit_RESET ) {
			vTaskDelay(1);
		}

		/**
		 * It it is still pushed go to handle code
		 * or sleep for another time slot
		 */
		if ( (GPIOA->IDR & GPIO_Pin_10) != Bit_RESET ) {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			continue;
		}

		if (xMenuHandle != NULL) {
			short count = 0x03;
			xQueueSend( xEncoderQueue, &count, portMAX_DELAY);
			xMenuHandle = NULL;
		}
		switch (++menu) {
			case 0x01:
				xTaskCreate(ClockEditTask, "MinutesEdit", configMINIMAL_STACK_SIZE, (void *) 0, 2, &xMenuHandle);
				break;
			case 0x02:
				xTaskCreate(ClockEditTask, "HourEdit", configMINIMAL_STACK_SIZE, (void *) 1, 2, &xMenuHandle);
				break;
			case 0x03:
				xTaskCreate(LedPwmTask, "LedPwmTask", configMINIMAL_STACK_SIZE, NULL, 2, &xMenuHandle);
				break;
			default:
				menu = 0x00;
		}
		while ((GPIOA->IDR & GPIO_Pin_10) == Bit_RESET);
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}

}

void ClockEditTask(void * pvParameters)
{
	uint8_t timerChannel = (uint8_t) pvParameters;
	TaskHandle_t xLedsHandle = NULL;
	short int count = 0x00;
	uint32_t RTC_time = RTC_GetCounter();
	RTC_time = RTC_time - (RTC_time % 60);

	RTC_ITConfig(RTC_IT_SEC, DISABLE);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	if (timerChannel == 0x00) {
		xTaskCreate(LedBlinkerTask, "MinuteLedBlinker", configMINIMAL_STACK_SIZE, (void *) 3, 2, &xLedsHandle);
		while(1) {
			if( xQueueReceive( xEncoderQueue, &count, portMAX_DELAY ) == pdFALSE) {
				continue;
			}
			if (count == 0x01) {
				RTC_time += 60;
			} else if (count == 0x02) {
				RTC_time -= 60;
			} else if (count == 0x03) {
				break;
			}
//			if (RTC_time >= 0x0001517F) {
//				RTC_time = 0x00;
//			}
			uint8_t display[4] = { 0x00 };
			parseTime(&RTC_time, &display);

			xQueueSend( xDisplayQueue, &display, portMAX_DELAY );
			USART_SendData(USART3, count);
		}
	} else {
		xTaskCreate(LedBlinkerTask, "HourLedBlinker", configMINIMAL_STACK_SIZE, (void *) 2, 2, &xLedsHandle);
		while(1) {
			if( xQueueReceive( xEncoderQueue, &count, portMAX_DELAY ) == pdFALSE ) {
				continue;
			}
			if (count == 0x01) {
				RTC_time += 3600;
			} else if (count == 0x02) {
				RTC_time -= 3600;
			} else if (count == 0x03) {
				break;
			}
//			if (RTC_time >= 0x0001517F) {
//				RTC_time = 0x00;
//			}
			uint8_t display[4] = { 0x00 };
			parseTime(&RTC_time, &display);

			xQueueSend( xDisplayQueue, &display, portMAX_DELAY );
			USART_SendData(USART3, count);
		}
	}
	RTC_SetCounter(RTC_time);
	RTC_WaitForLastTask();
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	USART_SendData(USART3, 'E');
	xTaskNotifyGive( xLedsHandle );
	vTaskDelete(NULL);
}

void LedBlinkerTask(void * pvParameters)
{
	uint8_t timerChannel = (uint8_t) pvParameters;
	uint16_t TIM_Channel = 0;
	if (timerChannel == 0x02) {
		TIM_Channel = TIM_Channel_2;
	} else if (timerChannel == 0x03) {
		TIM_Channel = TIM_Channel_3;
	}
	while (1) {
		TIM_CCxCmd(TIM3, TIM_Channel, TIM_CCx_Disable);
		if (ulTaskNotifyTake( pdTRUE, (TickType_t) 200 ) > 0x00) {
			break;
		}
		TIM_CCxCmd(TIM3, TIM_Channel, TIM_CCx_Enable);
		if (ulTaskNotifyTake( pdTRUE, (TickType_t) 200 ) > 0x00) {
			break;
		}
	}

	TIM_CCxCmd(TIM3, TIM_Channel, TIM_CCx_Enable);
	vTaskDelete(NULL);
}

void LedPwmTask(void * pvParameters)
{
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	short int count = 0x00;
	while (1) {
		if( xQueueReceive( xEncoderQueue, &count, portMAX_DELAY ) == pdFALSE ) {
			continue;
		}
		if (50 >= TIM3->CCR2) {
			TIM3->CCR2 = 700;
			TIM3->CCR3 = 700;
		} else if (700 <= TIM3->CCR2) {
			TIM3->CCR2 = 50;
			TIM3->CCR3 = 50;
		}

		if (count == 0x01) {
			TIM3->CCR2 += 50;
			TIM3->CCR3 += 50;
		} else if (count == 0x02) {
			TIM3->CCR2 -= 50;
			TIM3->CCR3 -= 50;
		} else if (count == 0x03) {
			break;
		}
	}

    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	vTaskDelete(NULL);
}
