#include "stm32f4xx.h"


#ifndef HBPERIPH_H_INCLUDED


#define HBPERIPH_H_INCLUDED

typedef struct{
    volatile uint16_t *CCR;
    uint16_t offset;
    uint8_t ratio;
    uint8_t last_pos;
    uint8_t current_pos;
    


} Servo_Typedef;

typedef struct {
    GPIO_TypeDef *port_in;
    uint16_t pin_in;
    GPIO_TypeDef *port_out;
    uint16_t pin_out;

    uint16_t timeout;
} Ultrasonic_Typedef;


void ultrasonic_init( Ultrasonic_Typedef *sensor, 
                      GPIO_TypeDef *port_in,
                      GPIO_TypeDef *port_out,
                      uint16_t pin_in,
                      uint16_t pin_out,
                      uint16_t timeout);

uint32_t ultrasonic_read_distance(Ultrasonic_Typedef *sensor);

void servo_init(Servo_Typedef *servo, volatile uint32_t *CCR, uint16_t offset, uint16_t range); 
int servo_set_degrees(Servo_Typedef *servo, uint8_t degrees); 
void servo_increment_degrees(Servo_Typedef *servo, uint8_t degrees);


#endif
