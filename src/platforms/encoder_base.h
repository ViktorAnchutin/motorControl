#ifndef ENCODER_BASE
#define ENCODER_BASE

class EncoderBase
{
public:
    EncoderBase() = default;
    ~EncoderBase() = default;
    virtual void init_interface()=0;
    virtual float get_angle()=0;
};


#endif // ENCODER_BASE
