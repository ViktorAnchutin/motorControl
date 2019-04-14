#ifndef ENCODER
#define ENCODER

#include "encoder_base.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"


#define CS3_ON() GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define CS3_OFF() GPIO_SetBits(GPIOA, GPIO_Pin_8)


class Encoder : public EncoderBase
{
    public:
        Encoder()=default;
        ~Encoder()=default;
        void init_interface() override;
        float get_angle() override;

        float get_initial_angle();
        void set_initial_angle();

    private:
        float initial_angle;
};

#endif
