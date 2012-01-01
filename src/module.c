#include "module.h" 
#include "semphr.h"
#include "FreeRTOS.h"
#include <stdlib.h>
#include <stdint.h>
#include "task.h"

void PHMeter_Module_Init(PHMeter_Module *module) {
    if (module) {
        module->state = MODULE_STANDBY;
        module->mutex = xSemaphoreCreateMutex();
    }
}

void UDS_Module_Init( UDS_Module *module) { 
    if (module) {
        module->state = MODULE_STANDBY;
        module->mutex = xSemaphoreCreateMutex();
    }
}

int UDS_Module_distance(UDS_Module *module) {

  __IO uint8_t flag=0;
  __IO uint32_t disTime=0;

    GPIO_ResetBits(module->port_in, module->pin_in);

    GPIO_ResetBits(module->port_out, module->pin_out);
    delay_us(2);
    GPIO_SetBits(module->port_out, module->pin_out);
    delay_us(11);
    GPIO_ResetBits(module->port_out, module->pin_out);


  while(flag == 0) {

    while(GPIO_ReadInputDataBit(module->port_in, module->pin_in) == SET) {
        disTime++;
        flag = 1;
        delay_us(1);
        if (disTime == module->timeout*1000) {
            return -1;
        }
    }
  }

    return disTime/CM_CONVERSION*2; //converts to centimeters
}

void PPump_Module_Init(PPump_Module *module, UDS_Module *sensor) {
    if (module) {
        //use sensor to calculate distance Make this a task.  One parent task for getting tank size and one for ultrasonic distance finding
        sensor->value = 8;
        //module->tank = 100*(sensor->value/module->tank_height); //finds the percentage filled
        module->state = MODULE_STANDBY;
        module->mutex = xSemaphoreCreateMutex();
    }
}

void PPump_Module_dispense(PPump_Module *module, int amount_ml) {
    if (module) {
        GPIO_SetBits(module->port_out, module->pin_out);
        delay_ms((module->ml_per_m/60*amount_ml)*1000);
        GPIO_ResetBits(module->port_out, module->pin_out);
    }
}

void Fan_Module_Init(Fan_Module *module) {
    if (module){
        module->mutex = xSemaphoreCreateMutex();
        module->state = MODULE_STANDBY;
    }


}


