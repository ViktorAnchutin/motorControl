#include "MotorDriver.h"

MotorDriver::MotorDriver(int index)
{
    _driverNum = index;
}

void MotorDriver::init()
{
    if(_driverNum==1)
    {
      	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        GPIO_InitTypeDef ENx_init_struct;
        GPIO_StructInit(&ENx_init_struct);
        ENx_init_struct.GPIO_Pin = GPIO_Pin_10;
        ENx_init_struct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_Init(GPIOB,&ENx_init_struct);

        GPIO_SetBits(GPIOB, GPIO_Pin_10); /// EN1,2,3 to 1 enable all half-bridges

        ///init PWM pins
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


        ///init PWM timer

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


        TIM_CtrlPWMOutputs(TIM1, ENABLE);


        TIM_Cmd(TIM1, ENABLE);


    }
}
