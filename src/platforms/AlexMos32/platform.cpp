#include "platform.h"



#define PWM_period  3600





void Platform::toggle_led()
{
    GPIOB->ODR ^= GPIO_Pin_12;
}

void Platform::systick_timer_init()
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);// Прерывание от SysTick 1000 Гц
    NVIC_SetPriority (SysTick_IRQn, 0);
}


void Platform::led_init()
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  // Инициализация светодиода (PC13)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}



void Platform::init()
{
    led_init();
    systick_timer_init();

}

void Platform::control_loop_timer_init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef timer_init;
	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period = 3600-1;/// ---> 100 microsec/ 10kHz
	timer_init.TIM_Prescaler = 1-1; /// ---> 36 MHz
	TIM_TimeBaseInit(TIM3, &timer_init);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);



	TIM_Cmd(TIM3, ENABLE);


	NVIC_InitTypeDef NVIC_InitStructure1;

    NVIC_InitStructure1.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure1.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure1.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure1);

    NVIC_EnableIRQ(TIM3_IRQn);

}



//Platform::Platform(){};

//Platform::~Platform(){}
