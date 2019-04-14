#pragma once

#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include  "stm32f30x_spi.h"

void LED_init();

void SPI_init();

float get_angle(void) ;
void PWM_INx_init(void); // INx, IN - PWMpins
void ENx_init(void);
void Set_ENx(void);
void PWM_Timer(void);/// PWM
float CQ_average_angle(void);
void FOC_InitPosition(void);
void FOC(float angle, float error_angle, float K_p, float K_d, float K_I);


