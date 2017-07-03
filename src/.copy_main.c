/**
 * Copyright (C) 2013 Chetan Patil, http://chetanpatil.info
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * 
 * @author Chetan Patil | http://chetanpatil.info
 */

//Example code to loop back the data sent to USART2 on STM32F4DISCOVERY

//Inlcude header files
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "hbperiph.h"
#include "clock.h"

//Task For Sending Data Via USART


#define MAX_STRLEN 300 // this is the maximum string length of our string in characters
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string

uint8_t write_pos = 0;
uint8_t read_pos = 0;

volatile uint8_t input_lock = 0;


void setSysTick(void){
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1){};
    }
}

void USART_put(USART_TypeDef* USARTx, volatile char c) {
    // wait until data register is empty
    while(!(USARTx->SR & 0x00000040) );
    USART_SendData(USARTx, c);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){
	while(*s){
        USART_put(USARTx, *s);
		*s++;
	}
}

void USART_put_int(USART_TypeDef* USARTx, int number) {
    uint8_t neg = 0;

    if (number < 0) {
        number*=-1;
        neg = 1;
    }
    char value[10];
    int i=0;
    do {
        value[i++] = (char)(number % 10) + '0';
        number /= 10;
    } while (number);

    if (neg) USART_put(USARTx, '-');    
    while(i) {
        USART_put(USARTx, value[--i]);
    }
}

void USART_getline(void) {
    if (!input_lock) {
        input_lock=1;
        while(input_lock){
            delay_ms(250);
            GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
            delay_ms(250);
            GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
        }
    }
}

char USART_getkey(void) {
    static char prev_key;
    prev_key = received_string[write_pos-1];
    while (prev_key == received_string[write_pos-1]);
    return received_string[write_pos-1]; 
}

int USART_nextline_length(void) {
    int i;
    for(i=read_pos; received_string[i]!='\r'; i++);

    return (i-read_pos)+1;
}
/*
int char_array_compare(char *str, char end, char *str2 char end2) {
    while (*str != end || *str2 != end2) {
        if (str != str2) return 0;
    }

    return 1;
}
*/
void handle_uart_command(char *command) {

    USART_puts(USART2, "Command: ");

    char *pos = command;
/*
    while(*command != '\r') {
        if (*command == ' ') {
            while (pos != command) {
                if (pos == )
                //something like this in python command[pos1:pos2] == "command1"
                pos++;
            }
        }
        USART_put(USART2, *command);
        command++;
    }
    */

    USART_puts(USART2, "\r\n");
    vTaskDelete(NULL);
}

void idle_blinky (void *pvParameters) {
    while(1) {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        delay_ms(500);

        GPIO_SetBits(GPIOD, GPIO_Pin_13);
        delay_ms(500);

        GPIO_SetBits(GPIOD, GPIO_Pin_14);
        delay_ms(500);

        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        delay_ms(500);

        GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
        delay_ms(500);


    }

}


void USART_readline(char *buffer, uint8_t length) {
    int i;
 //   static uint8_t first=1;

    buffer[length-1] = '\0';

    //for(i=0; received_string[read_pos]!='\r' && buffer[i]!='\0'; i++) {
    for(i=0; i<length-1;i++){
        if (read_pos == MAX_STRLEN-1) read_pos=0;
        buffer[i] = received_string[read_pos];
        read_pos++;
    }
   // if (!first) read_pos++; //This skips the previous Carriage return added to the end of data added to buffer
    //else first = 0;
    read_pos++;

}

void USART_readline_int(int *num) {
    *(num) = 0;
    int i=0;
    uint8_t neg = 0;

    int len = USART_nextline_length();
    char buffer[len];
    USART_readline(buffer, len);

    if (buffer[i] == '-') {
        i++;
        neg = 1;

    }

    for(i; buffer[i]!='\0'; i++) {
        if (buffer[i] >= '0' && buffer[i]<='9'){
            *(num) *= 10; 
            *(num) += buffer[i]-'0';
        }

        else {
            break;
        }    
    }

    if (neg) {
        *(num) *= -1;
    }

}

void USART2_IRQHandler(void){
	
	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE)){
        static uint8_t count=0;

		
		//static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART2 data register is saved in t

        if (write_pos == MAX_STRLEN-1) write_pos=0;

        if (t == 8 || t== 127) {
            write_pos--;
            received_string[write_pos] = '\r';
            USART_put(USART2, '\b');
            USART_put(USART2, ' ');
            USART_put(USART2, '\b');
            return;

        }

		// check if the received character is not the LF character (used to determine end of string) 
		// or the if the maximum string length has been been reached 
		else if/*(*/ (t != '\r')/* && (cnt< MAX_STRLEN-1) )*/{ 
            count++;
			received_string[/*cnt*/write_pos] = t;
            USART_put(USART2, t);

		}

		else{ // otherwise reset the character counter and print newline 
            if (count != 0) {
                
                received_string[/*cnt*/write_pos] = '\r';
                USART_puts(USART2, "\r\n");

                char *command = &received_string[write_pos-count];

                xTaskCreate(handle_uart_command, (signed char*)"handle_uart_command", 128, command, tskIDLE_PRIORITY+1, NULL);
//                xTaskCreate(idle_blinky, (signed char*)"idle_blinky", 128, NULL, tskIDLE_PRIORITY, NULL);


                input_lock = 0;
                count = 0;
            }
            else USART_puts(USART2, "\r\n");
		}
        write_pos++;
	}
}



//Initialize GPIO and USART2
void initx(void){

	//Enable GPIO Clocks For USART2
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//Enable Clocks for USART2
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	

	//Structure With Data For GPIO Configuration
	GPIO_InitTypeDef GPIO_InitStructure;

	//Structure With Data For USART Configuration
	USART_InitTypeDef USART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

	//GPIO Configuration
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	//Connect USART pins to AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	//Initialize LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//USART Parameters
	USART_InitStructure.USART_BaudRate = 38400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;

	//Configuring And Enabling USART2
	USART_Init(USART2, &USART_InitStructure);


    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2, ENABLE);

}


void TIM_PWM_init() {

    //TIMER SETUP
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    TIM_TimeBaseInitStruct.TIM_Period = 19999;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 84;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);


    //pwm setup
    
    TIM_OCInitTypeDef TIM_OCInitStruct;
    
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    


    TIM_OC1Init(TIM4, &TIM_OCInitStruct); //channel 1
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM4, &TIM_OCInitStruct); // channel 2
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);


    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}


typedef struct {
    int  port;
    int pin;
    int amount;

} vPPump_parameters;

typedef struct {
    portTickType delay ;
    int target_ph;
} vPH_parameters;

typedef struct {
    int status;
    int delay;
} vLight_parameters;


typedef struct {
    int value;
    xSemaphoreHandle mutex;
} pHMeter_t;



void vLight_task(vLight_parameters *params) {
    USART_puts(USART2, "Executed Light_task\r\n");
    /*
    Light_set(params->status);
    int height = Plant_get_height();
    Light_distance_set(distance+height);

    //maybe look for something like recurring task
    //you need to find a way to change variable while its waiting;
    */
    vTaskDelete(NULL);
} 

int TEST_PH_VALUE = 50;

int PH_STATE = 0;
void vPH_read_value(pHMeter_t *meter) {
    PH_STATE = 1;
    if (xSemaphoreTake(meter->mutex, (portTickType) 10) == pdTRUE) {
        USART_puts(USART2, "Read pH Meter...\r\n");
        meter->value = TEST_PH_VALUE;
        xSemaphoreGive(meter->mutex);

    }

    else {
        USART_puts(USART2, "COULDNT TAKE SEMAPHORE");

    }

    PH_STATE = 2;
    vTaskDelete(NULL);

}

void vPPump_dispense(vPPump_parameters *params) {
    const int ppump_speed = 1;
    //GPIO_WriteBit(params->port, params->pin, Bit_SET);
    USART_puts(USART2, "Starting to dispense...\r\n");
    if (params->pin == 1) TEST_PH_VALUE+=5;
    else if(params->pin == 2) TEST_PH_VALUE-=5; 

    portTickType xDelay = (1000 * ppump_speed) / portTICK_RATE_MS;
    vTaskDelay(xDelay);
    USART_puts(USART2, "Finished dispensing\r\n");

    vTaskDelete(NULL);
}


void vPH_task(vPH_parameters *params) {
    USART_puts(USART2, "Executing PH_task...\r\n");

    pHMeter_t ph_meter;
    ph_meter.mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(ph_meter.mutex);

    vPPump_parameters ppump_p; 

    if (ph_meter.mutex == NULL) USART_puts(USART2, "PH_METER->mutex == NULL\r\n");

    //PH_CHANGING = 1; use this so the web console knows when changes are happening
    
   //add if sepamphore is not null error check
   //---------------------------------------
  
    int read_ph=1; 
    int TARGET_PH_REACHED = 0;
    /*
     *
     *
     *
     * MAKE SUB TASKS CREATED INCREMENT ITS PRIORITY BY ONE RELATIVE TO PARENT TASK
     */

    while (1) {

        if (read_ph){ 
            read_ph=0;
            xTaskCreate(vPH_read_value, (signed char*)"PH Read", 128, &ph_meter, tskIDLE_PRIORITY+4, NULL);
        }
        vTaskDelay((portTickType) 500 / portTICK_RATE_MS);

        if (PH_STATE == 2) { 
            if (xSemaphoreTake(ph_meter.mutex, (portTickType) 10) == pdTRUE) {
                USART_puts(USART2, "METER READ SUCCESSFULLY. PH=");
                USART_put_int(USART2, ph_meter.value);
                USART_puts(USART2, "\r\n");
     
                if (ph_meter.value < params->target_ph){
                    USART_puts(USART2, "ADDING PH UP\r\n");
                    ppump_p.pin = 1;
                    xTaskCreate(vPPump_dispense, (signed char*)"PP", 128, &ppump_p, tskIDLE_PRIORITY+3, NULL); 

                }
                else if (ph_meter.value > params->target_ph) {
                    ppump_p.pin = 2;
                    USART_puts(USART2, "ADDING PH DOWN\r\n");

                    xTaskCreate(vPPump_dispense, (signed char*)"PP", 128, &ppump_p, tskIDLE_PRIORITY+3, NULL); 

                    
                }
                else {
                    USART_puts(USART2, "STABILIZED PH LEVELS @");
                    USART_put_int(USART2, ph_meter.value);
                    USART_puts(USART2, "\r\n");
                    break;
                };

                read_ph=1;
                //vTaskDelay(params->delay);        
                
                xSemaphoreGive(ph_meter.mutex);

            }
            else {
                USART_puts(USART2, "COULDNT ACCESS SAFELY IN PH_TASK\r\n");
            }
            PH_STATE = 0;
        }
    }
    //PH_CHANGING = 0;  
    vTaskDelete(NULL);
}


//Main Function
int main(void)
{

	//Call initx(); To Initialize USART & GPIO

	initx();
    init_us_timer();
    //
    //CLOCK_SetClockTo168MHz();
    //setSysTick();


    int i;

    USART_puts(USART2, "Booting up...\r\n");
    for(i=0; i<2; i++) { 
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
        GPIO_SetBits(GPIOD, GPIO_Pin_13);
        GPIO_SetBits(GPIOD, GPIO_Pin_14);
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
        delay_ms(500);

        GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
        delay_ms(500);

    }


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitTypeDef gpio_ultrasonic;

    gpio_ultrasonic.GPIO_Pin = GPIO_Pin_0;
    gpio_ultrasonic.GPIO_OType = GPIO_OType_PP; //PP
    gpio_ultrasonic.GPIO_Mode = GPIO_Mode_OUT;
    //gpio_ultrasonic.GPIO_PuPd = GPIO_PuPd_NOPULL;

    gpio_ultrasonic.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio_ultrasonic);


    gpio_ultrasonic.GPIO_Pin = GPIO_Pin_1;
    gpio_ultrasonic.GPIO_OType = GPIO_OType_PP;
    //gpio_ultrasonic.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio_ultrasonic.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(GPIOE, &gpio_ultrasonic);

    Ultrasonic_Typedef sensor;
    ultrasonic_init(&sensor, GPIOE, GPIOE, GPIO_Pin_1, GPIO_Pin_0, 300); 
    

    /*
    while (1) {
        USART_puts(USART2, "Type 'Enter'");
        USART_getline();
        int d = ultrasonic_read_distance(&sensor);
        USART_puts(USART2, "Distance: ");
        USART_put_int(USART2, d);
        USART_puts(USART2, "\r\n");
    }
    */
    //SERVO SETUP
    TIM_PWM_init();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);




    GPIO_InitTypeDef GPIO_InitStructServo;

    GPIO_InitStructServo.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructServo.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructServo.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructServo.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructServo.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructServo);


    Servo_Typedef servo1, servo2;
    servo_init(&servo1, &(TIM4->CCR1), 500, 1850/*61*/); //up and down servo
    servo_init(&servo2, &(TIM4->CCR2), 500, 1000);
    servo_set_degrees(&servo1, 90);

    char hello[] = "hello";

    xTaskCreate(idle_blinky, (signed char*)"idle_blinky", 128, NULL, tskIDLE_PRIORITY, NULL);

    USART_puts(USART2, "pH Level: ");
    USART_getline();
    int target_ph = 0;
    USART_readline_int(&target_ph);
    USART_puts(USART2, "\r\nYOU SAID: ");
    USART_put_int(USART2, target_ph);
    USART_puts(USART2, "\r\n");

    vLight_parameters light_p;
    vPH_parameters ph_p;
    ph_p.target_ph = target_ph;

    xTaskCreate(vLight_task, (signed char*)"vLight", 128, &light_p, tskIDLE_PRIORITY+1, NULL);

    xTaskCreate(vPH_task, (signed char*)"vPH", 128, &ph_p, tskIDLE_PRIORITY+2, NULL);


 //   xTaskCreate(handle_uart_command, (signed char*)"handle_uart_command", 128, hello, tskIDLE_PRIORITY+1, NULL);


	//xTaskCreate(UsartTask, (signed char*)"UsartTask", 128, NULL, tskIDLE_PRIORITY+1, NULL);

	//Call Scheduler
	vTaskStartScheduler();
    //
    
   

}
