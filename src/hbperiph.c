#include "hbperiph.h"
#include "stm32f4xx.h"
#include "delay.h"



void ultrasonic_init( Ultrasonic_Typedef *sensor, 
                      GPIO_TypeDef *port_in,
                      GPIO_TypeDef *port_out,
                      uint16_t pin_in,
                      uint16_t pin_out,
                      uint16_t timeout)
{
    sensor->port_in = port_in;
    sensor->pin_in = pin_in;
    sensor->port_out = port_out;
    sensor->pin_out = pin_out;

    sensor->timeout = timeout;
}

uint32_t ultrasonic_read_distance(Ultrasonic_Typedef *sensor) {
  __IO uint8_t flag=0;
  __IO uint32_t disTime=0;

    GPIO_ResetBits(sensor->port_in, sensor->pin_in);

    GPIO_ResetBits(sensor->port_out, sensor->pin_out);
    delay_us(2);
    GPIO_SetBits(sensor->port_out, sensor->pin_out);
    delay_us(11);
    GPIO_ResetBits(sensor->port_out, sensor->pin_out);

  while(flag == 0) {
    while(GPIO_ReadInputDataBit(sensor->port_in, sensor->pin_in) == SET) {
        disTime++;
        flag = 1;
        delay_us(1);
        //if (disTime == timeout) return 0;
    }

  }
    return disTime/58*2; //converts to centimeters
}

void servo_init(Servo_Typedef *servo, volatile uint32_t *CCR, uint16_t offset, uint16_t range) {
    servo->CCR = CCR;
    servo->offset = offset;
    servo->ratio = range/180;
    servo->last_pos = 0;
    servo->current_pos = 0;

    *(servo->CCR) = 0;
}

int servo_set_degrees(Servo_Typedef *servo, uint8_t degrees) {
    if (degrees==-1) *(servo->CCR) = -1;
    else {
        *(servo->CCR) = servo->offset+degrees*servo->ratio;
        //int8_t offset = (10*(degrees - servo->current_pos));
        //if (offset<0) offset*-1;

        //*(servo->CCR) = 0;

        servo->last_pos = servo->current_pos;
        servo->current_pos = degrees;
        return servo->offset+degrees*servo->ratio;


    }
}

