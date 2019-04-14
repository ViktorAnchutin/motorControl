#include "platform.h"

#define CS3_ON() GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define CS3_OFF() GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define PWM_period  3600


void Platform::spi_init()
{
      /**
    PB3 - CLCK
	PB4 - MISO
	PB5 - MOSI
	PA8 - CS */

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI3, ENABLE); /// 36MHz

	GPIO_InitTypeDef SPI3_pins;
	SPI3_pins.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	SPI3_pins.GPIO_Mode = GPIO_Mode_AF;
	SPI3_pins.GPIO_Speed = GPIO_Speed_2MHz;
	SPI3_pins.GPIO_OType = GPIO_OType_PP;
	SPI3_pins.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB, &SPI3_pins);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_6);

	GPIO_InitTypeDef SPI3_CS;
	SPI3_CS.GPIO_Pin = GPIO_Pin_8;
	SPI3_CS.GPIO_Mode = GPIO_Mode_OUT;
	SPI3_CS.GPIO_Speed = GPIO_Speed_2MHz;
	SPI3_CS.GPIO_OType = GPIO_OType_PP;
	SPI3_CS.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &SPI3_CS);

	CS3_OFF();


	SPI_InitTypeDef SPI3_encoder;
	SPI3_encoder.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;
	SPI3_encoder.SPI_Mode = SPI_Mode_Master;
	SPI3_encoder.SPI_DataSize = SPI_DataSize_16b;
	SPI3_encoder.SPI_CPOL = SPI_CPOL_Low;
	SPI3_encoder.SPI_CPHA = SPI_CPHA_2Edge;
	SPI3_encoder.SPI_NSS = SPI_NSS_Soft;
	SPI3_encoder.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8 ;/// 4.5 MHz
	SPI3_encoder.SPI_FirstBit =  SPI_FirstBit_MSB;
	SPI3_encoder.SPI_CRCPolynomial = 7;

	SPI_Init(SPI3, &SPI3_encoder);
	SPI_Cmd(SPI3, ENABLE);

	SPI_NSSInternalSoftwareConfig(SPI3, SPI_NSSInternalSoft_Set);
}



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




//Platform::Platform(){};

//Platform::~Platform(){}
