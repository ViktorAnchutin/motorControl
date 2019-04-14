/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f30x_conf.h"
#include "./system/system.h"




int main(void)
{
    System::init();

  while(1)
  {
    System::toggle_led_periodically(500);
  }

}






