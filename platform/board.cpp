#include "board.h"
#include "system.h"

#define CS3_ON() GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define CS3_OFF() GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define PWM_period  3600




void LED_init()
{

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  // Инициализация светодиода (PC13)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}



void SPI_init()
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





uint16_t ResData1, ResData2, ResData_err;

	uint32_t counter_;
 	uint16_t MODF;
	uint8_t pr;
	float angle_return;


float get_angle(void) // with SPI3
{





		if(!pr)
		{
			CS3_ON();
			///myDelay_microsec(1);
			SPI_I2S_SendData16(SPI3, 0xFFFF );
			while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) {}
			while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {}
			ResData1 = SPI_I2S_ReceiveData16(SPI3);
			///myDelay_microsec(1);
			CS3_OFF();
			//myDelay_microsec(1);
			pr=1;
		}

	CS3_ON();
	///myDelay_microsec(1);
	SPI_I2S_SendData16(SPI3, 0xFFFF );
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) {}
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {}
	ResData2 = SPI_I2S_ReceiveData16(SPI3);
	///myDelay_microsec(1);
	CS3_OFF();



			// if error occurs
	/*		if(ResData2 & 0x4000)
			{
				myDelay_microsec(1);
				CS3_ON();
				myDelay_microsec(1);
				SPI_I2S_SendData(SPI3, 0x4001 );
				while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) {}
				while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {}
				ResData1 = SPI_I2S_ReceiveData(SPI3);
				myDelay_microsec(1);
				CS3_OFF();

				myDelay_microsec(1);
				CS3_ON();
				myDelay_microsec(5);
				SPI_I2S_SendData(SPI3, 0xFFFF );
				while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET) {}
				while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) {}
				ResData_err = SPI_I2S_ReceiveData(SPI3);
				myDelay_microsec(5);
				CS3_OFF();
			}
			*/


	//angle = (float)(ResData2 & 0x3FFF)*0.021973997;
		angle_return = (float)(ResData2 & 0x3FFF);
		angle_return = angle_return*0.021973997;
	return 	angle_return;//(float)(ResData2 & 0x3FFF)*0.021973997;


}



// PWM out for 12th pin

void PWM_INx_init(void) // INx, IN - PWMpins
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


	//IN1 --> PB13--------------------------
	GPIO_InitTypeDef init_AF;//
	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF;//
	init_AF.GPIO_Pin = GPIO_Pin_13;//
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_6);//
	GPIO_Init(GPIOB, &init_AF);//

	//IN2 --> PB14--------------------------

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF;//
	init_AF.GPIO_Pin = GPIO_Pin_14;//
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_6);//
	GPIO_Init(GPIOB, &init_AF);//


	//IN3 --> PB15--------------------------

	GPIO_StructInit(&init_AF);//
	init_AF.GPIO_Mode = GPIO_Mode_AF;//
	init_AF.GPIO_Pin = GPIO_Pin_15;//
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_4);//
	GPIO_Init(GPIOB, &init_AF);//

}


void ENx_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef ENx_init_struct;
	GPIO_StructInit(&ENx_init_struct);
	ENx_init_struct.GPIO_Pin = GPIO_Pin_10;
	ENx_init_struct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOB,&ENx_init_struct);

}



void Set_ENx(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_10); // EN1,2,3 to 1 enable all half-bridges
}





void PWM_Timer(void)/// PWM
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef timer_init;
	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Period = PWM_period-1;// ---> 20kHz
	timer_init.TIM_Prescaler = 1-1; // ---> 72 MHz
	TIM_TimeBaseInit(TIM1, &timer_init);

	//channel 1;
	TIM_OCInitTypeDef TIM_OCStruct;
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//TIM_OCPolarity_High;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCStruct.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCStruct.TIM_OCNPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//TIM_OCNPolarity_High;
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &TIM_OCStruct);


	//channel 2;
	TIM_OCInitTypeDef TIM_OCStruct2;
    TIM_OCStruct2.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct2.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//TIM_OCPolarity_High;
    TIM_OCStruct2.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCStruct2.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCStruct2.TIM_OCNPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//TIM_OCNPolarity_High;
    TIM_OCStruct2.TIM_Pulse = 0;
    TIM_OC2Init(TIM1, &TIM_OCStruct2);

	//channel 3;
	TIM_OCInitTypeDef TIM_OCStruct3;
    TIM_OCStruct3.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct3.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//TIM_OCPolarity_High;
    TIM_OCStruct3.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCStruct3.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCStruct3.TIM_OCNPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;//TIM_OCNPolarity_High;
    TIM_OCStruct3.TIM_Pulse = 0;
    TIM_OC3Init(TIM1, &TIM_OCStruct3);



 /*   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
     TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
 TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
 TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
 TIM_BDTRInitStructure.TIM_DeadTime = 0; // 1 microsec(168)
 TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
 TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
 TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
 TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

*/





 TIM_CtrlPWMOutputs(TIM1, ENABLE);




	TIM_Cmd(TIM1, ENABLE);



}




#include "arm_math.h"
#include "math.h"
#define window 100

////Mean of circular quantities
	float sine_sum, cos_sum, arr_CQ[window], sine_arr[window], cos_arr[window], sine_av, cos_av, X_i_CQ;
	float a_i_CQ, sine_i, cos_i, raw_value;
	uint8_t filled_CQ;
	uint16_t k_CQ;


	float CQ_average_angle(void)
	{
		if(!filled_CQ)
		{
			sine_sum = 0;
			cos_sum = 0;
			for (int i=0; i < window; i++ )
			 {
				arr_CQ[i] = get_angle()*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians;
				sine_arr[i] = arm_sin_f32(arr_CQ[i]);
				cos_arr[i] = arm_cos_f32(arr_CQ[i]);
				sine_sum = sine_sum + sine_arr[i];
				cos_sum = cos_sum + cos_arr[i];
			 }

			 sine_av = sine_sum/window;
			 cos_av = cos_sum/window;
			 X_i_CQ  = atan2f(sine_av, cos_av)*57.295779513082320876798154814105 ; // out of the filter

			 filled_CQ = 1;
			 k_CQ=0;
			 return X_i_CQ;

		}

		// 2) start filtering

		else
		{
			raw_value = get_angle();
			a_i_CQ = raw_value*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians;
			sine_i = arm_sin_f32(a_i_CQ);
			cos_i = arm_cos_f32(a_i_CQ);
			sine_sum = sine_sum - sine_arr[k_CQ] + sine_i;
			cos_sum = cos_sum - cos_arr[k_CQ] + cos_i;
			sine_av = sine_sum/window;
			cos_av = cos_sum/window;
			X_i_CQ  = atan2f(sine_av, cos_av)*57.295779513082320876798154814105 ;

			sine_arr[k_CQ] = sine_i;
			cos_arr[k_CQ] = cos_i;
			 // substitute thrown out value with new value for cycling
			k_CQ++;
			if(k_CQ >= window) k_CQ=0; // array loop
			return X_i_CQ;

		}

	}






#define CALIBRATION 1
#define High			0x01
#define Low				0x00
#define average		 100
#define Pole_Pairs 11
#define Vdc 				12
#define Pi 3.1415926535897932384
float Vq;
float	Vd;
float Va, Vb, Vc, Va_1, Vb_1, Vc_1;
float theta, theta_elec_degrees;
float Vinv1,Vinv2, Vinv3;
float angle_init , error_angle_last, sine_init, cos_init;
 extern float angle ;
//extern float Speed;
//extern float alpha;

//CCR1 - A, CCR2 - B, CCR3 - C

void FOC_InitPosition(void) // establishing zero position, d-axis directed to A winding, theta = 90
{

	if(CALIBRATION)
	{
	Vq=3;

	Va_1 = arm_cos_f32(0);//cos(theta         );
	Vb_1 = arm_cos_f32(0 - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 ); //2*Pi/3
	Vc_1 = arm_cos_f32(0 + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);

	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase

	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM)
	Vinv3 = Vc + 6;

	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ;
    TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
    TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;

	sys_Delay_ms(1000);

	angle_init = CQ_average_angle();

	}
	else

	{

	// init angle was calculated once. Now it is used like starting point for electrical angles and engine does not need position initialization

        angle_init = 238.066;

	}


}








 float dif;
float error_in_proc, er_mem, angle_mem,integral ;

void FOC(float angle, float error_angle, float K_p, float K_d, float K_I)
{
	theta_elec_degrees = ((angle - angle_init)*Pole_Pairs + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90
	theta = theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians


	Vd = 0; // !

	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}


	error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100))
	{
		er_mem = error_angle;
		angle_mem = angle;
	}


	//integral = dt*0.000001*error_angle_last+integral;
	dif = (error_angle - error_angle_last)*K_d;
	Vq = K_p*error_angle;// (dif) ;//+ integral*K_I; //Speed; ////arm_pid_f32(&pid, error_angle);
	error_angle_last = error_angle;




	if(Vq < -6) Vq = -6; // 6V = Vdc/2 , voltage limitation
	if(Vq > 6) Vq = 6;



	Va_1 = arm_cos_f32(theta);//cos(theta         );
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);


	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase

	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM)
	Vinv3 = Vc + 6;

	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ;
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;


}
















