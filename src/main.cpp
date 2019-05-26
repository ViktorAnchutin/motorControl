
#include "stm32f30x_conf.h"
#include "system.h"




int main(void)
{
  System::init();

  System::setDesiredRotorPosition(30);

  System::start_control_loop(); /// control loop is implemented in system.cpp / System::control_loop();
                                /// and is called from an ISR

  while(1)
  {
    System::toggle_led_periodically(500);
  }

}






