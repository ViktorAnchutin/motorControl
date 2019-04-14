#include "encoder.h"

void Encoder::init_interface()
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


float Encoder::get_angle()
{
   /* if(!pr)
	{
		CS3_ON();
		SPI_I2S_SendData16(SPI3, 0xFFFF );
		while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) {}
		while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {}
		ResData1 = SPI_I2S_ReceiveData16(SPI3);
		CS3_OFF();
		pr=1;
    } */

	CS3_ON();
	///myDelay_microsec(1);
	SPI_I2S_SendData16(SPI3, 0xFFFF );
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) {}
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {}
	uint16_t ResData = SPI_I2S_ReceiveData16(SPI3);
	///myDelay_microsec(1);
	CS3_OFF();

	float angle_return = (float)(ResData & 0x3FFF);
	angle_return = angle_return*0.021973997;
	return 	angle_return;//(float)(ResData2 & 0x3FFF)*0.021973997;

}
