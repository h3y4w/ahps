#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include "clock.h"
#include "module.h"
#include "command_parser.h"
#include "hbconfig.h"

//Task For Sending Data Via USART

//MAKE THIS A SEPERATE CONFIG FILE LATER//


//-------------------------------//
//FIND A WAY TO GET MUTEX FROM OTHER PRIOTIY.  IF THERE IS NO WAY SET UP A TMP BUFFER WHERE IT CAN STORE VALUES TO WAIT TO COPY// 
//FUNCTIONS TO COPY STRINGS TO SEPERATE BUFFER FOR COMMAND_HANDLER
// USART_IRQ():
//  create task copy_buffer() with high priority and pass pointer to buffer 
//  copy_buffer() will then have a callback function which will be command_handler which will pass newly copied pointer
//USE sent_mutex to check if usart has sent message

void vUSART_command_handler (char *pos) {
    char term = '\003';
    replace_char(pos, ' ', term);
    command_routing(pos, term);
    vTaskDelete(NULL);
}



USART_rtos USART1_rtos;
USART_rtos USART2_rtos;

//xSemaphoreHandle USART2_mutex;
//xSemaphoreHandle USART1_mutex;

//xSemaphoreHandle USART1_mutex_input;
//xSemaphoreHandle USART2_mutex_input;



//volatile uint8_t USART1_input_lock = 0;

//volatile uint8_t USART2_input_lock = 0;


void setSysTick(void){
    if (SysTick_Config(SystemCoreClock / 1000)) {
        while (1){};
    }
}


void idle_blinky (void *pvParameters) {

    GPIO_SetBits(GPIOD, GPIO_Pin_4);
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


void USART1_IRQHandler(void){
	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE)){
        static unsigned int count=0;

		
		//static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART2 data register is saved in t


        if (t == 8 || t== 127) {
            USART1_rtos.write_pos--;
            USART1_rtos.buffer[USART1_rtos.write_pos] = '\r';
            USART_put(USART1, '\b');
            USART_put(USART1, ' ');
            USART_put(USART1, '\b');
            return;

        }

		// check if the received character is not the LF character (used to determine end of string) 
		// or the if the maximum string length has been been reached 
		else if/*(*/ (t != '\r')/* && (cnt< MAX_STRLEN-1) )*/{ 
            count++;
        if (USART1_rtos.write_pos == USART_BUFFER_LEN-1) USART1_rtos.write_pos=0;

			USART1_rtos.buffer[USART1_rtos.write_pos] = t;
            USART_put(USART1, t);
            USART1_rtos.write_pos++;

		}

		else{ // otherwise reset the character counter and print newline 
            if (count != 0) {
                
                USART1_rtos.buffer[USART1_rtos.write_pos] = '\r';

                USART_puts(USART1, "\r\n");

                //char *pos = &USART1_received_string[USART1_write_pos-count];
                //char command[len];
//                USART_readline(USART1, &command, len);
                char *pos = &USART1_rtos.buffer[USART1_rtos.write_pos-count];


                xTaskCreate(vUSART_command_handler, (signed char*)"vUSART_command_handler", 468, pos, tskIDLE_PRIORITY+9, NULL);
//                xTaskCreate(idle_blinky, (signed char*)"idle_blinky", 128, NULL, tskIDLE_PRIORITY, NULL);


                xSemaphoreGive(USART1_rtos.transmit_event_mutex);

                count = 0;
                USART1_rtos.write_pos++;

            }
            else USART_puts(USART1, "\r\n");
		}
	}
}

/*
void USART2_IRQHandler(void){
	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE)){
        static uint8_t count=0;

		
		//static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART2->DR; // the character from the USART2 data register is saved in t

        if (USART2_write_pos == MAX_STRLEN-1) USART2_write_pos=0;

        if (t == 8 || t== 127) {
            USART2_write_pos--;
            USART2_received_string[USART2_write_pos] = '\r';
            USART_put(USART2, '\b');
            USART_put(USART2, ' ');
            USART_put(USART2, '\b');
            return;

        }

		// check if the received character is not the LF character (used to determine end of string) 
		// or the if the maximum string length has been been reached 
		else if(t != '\r') && (cnt< MAX_STRLEN-1) ){ 
            count++;
			USART2_received_string[USART2_write_pos] = t;
            USART_put(USART2, t);

		}

		else{ // otherwise reset the character counter and print newline 
            if (count != 0) {
                
                USART2_received_string[USART2_write_pos] = '\r';
                USART_puts(USART2, "\r\n");

                char *command = &USART2_received_string[USART2_write_pos-count];
                //change this to copy because it is volatile

                xTaskCreate(handle_usart_command, (signed char*)"handle_uart_command", 128, command, tskIDLE_PRIORITY+1, NULL);
//                xTaskCreate(idle_blinky, (signed char*)"idle_blinky", 128, NULL, tskIDLE_PRIORITY, NULL);


                USART_puts(USART2, "HAVENT IMPLENETED WAITING FOR TRANSMIT\r\n");
                count = 0;
            }
            else USART_puts(USART2, "\r\n");
		}
        USART2_write_pos++;
	}
}

*/

//Initialize GPIO and USART2
//

void USART1_Init(void) {

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	

	//Structure With Data For GPIO Configuration
	GPIO_InitTypeDef GPIO_InitStructure;

	//Structure With Data For USART Configuration
	USART_InitTypeDef USART_InitStructure;

    NVIC_InitTypeDef NVIC_InitStructure;

	//GPIO Configuration
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	//Connect USART pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);


	//USART Parameters
	USART_InitStructure.USART_BaudRate = 38400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;

	//Configuring And Enabling USART2
	USART_Init(USART1, &USART_InitStructure);


    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //allow usart interrupt

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);


}
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
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //USART2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //USART1


    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    TIM_TimeBaseInitStruct.TIM_Period = 19999;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 84;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);


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




enum System_State {SYSTEM_WRITING, SYSTEM_WRITTEN, SYSTEM_READ, SYSTEM_ERROR};


typedef struct {
    PPump_Module *ppump;
    int amount;

} vPPump_parameters;



typedef struct {
    int status;
    int delay;
} vLight_parameters;


typedef struct {
    enum System_State state;
    PPump_Module PPump_PHUp;
    UDS_Module UDS_PHUp;
    PPump_Module PPump_PHDown;
    UDS_Module UDS_PHDown;
    PPump_Module PPump_Nutrient;
    UDS_Module UDS_Nutrient;
    PHMeter_Module PHMeter_Resvoir;
    int target_ph;
    int ph_delay;
} HydroponicSystem;


typedef struct {
    Light_Module Lighting; //change to Lighting_Module
    enum System_State state;
    int distance;
    int hours_on;
} LightingSystem;


typedef struct {
    enum System_State state;
    Fan_Module Fan_Reservoir; //fans the water
    Fan_Module Fan_Plant; //fans the plants
    Fan_Module Fan_In; //brings in co2
    Fan_Module Fan_Out; //puts out oxygen through carbon filter
    Temperature_Module Temperature_Resevoir; // straightforward, above comments apply here
    Temperature_Module Temperature_Plant;
} ACSystem;

/*
void System_update_member(NULL *system_member, NULL* member_value ) {
    if (system_member && member_value) {
        //do an if stsatement to check if system member and member value are of the same type
        return;
    }
    //error
}
*/
HydroponicSystem hydroponics;
LightingSystem lighting;
ACSystem AC;


void vACSystem_Init(void) {
    USART_rtos_packet packet, packet1;
    packet.USARTx_rtos = &USART1_rtos;
    USART_rtos_puts(&packet, "Init AC System...");


    AC.state = SYSTEM_WRITING;

    AC.state = SYSTEM_WRITTEN;
//    USART_puts(USART2, "AC successfully initialized...\r\n");

    packet1.USARTx_rtos = &USART1_rtos;
    USART_rtos_puts(&packet1, " OK\r\n");


    USART_rtos_wait_send(&packet);
    USART_rtos_wait_send(&packet1);

    vTaskDelete(NULL);
}

void vLightingSystem_Init(void) {
    //USART_rtos_puts(USART2, "Init Lighting System...");

    lighting.state = SYSTEM_WRITING;

    lighting.state = SYSTEM_WRITTEN;
    //USART_puts(USART2, "Lighting successfully initialized...\r\n");
    
    //USART_rtos_puts(USART2, " OK\r\n");

    vTaskDelete(NULL);
}


void HydroponicUDS_Init(void) {

    hydroponics.UDS_PHUp.state = MODULE_RUNNING;
    hydroponics.UDS_PHDown.state = MODULE_RUNNING;
    hydroponics.UDS_Nutrient.state = MODULE_RUNNING;


	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpio_ultrasonic;

    gpio_ultrasonic.GPIO_Pin = GPIO_Pin_3; //echo
    gpio_ultrasonic.GPIO_OType = GPIO_OType_PP; //PP
    gpio_ultrasonic.GPIO_Mode = GPIO_Mode_IN;
    gpio_ultrasonic.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOD, &gpio_ultrasonic);

    gpio_ultrasonic.GPIO_Pin = GPIO_Pin_4; //trigger
	gpio_ultrasonic.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_ultrasonic.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOD, &gpio_ultrasonic);

    hydroponics.UDS_PHUp.pin_in = UDS_PHUP_PIN_IN;
    hydroponics.UDS_PHUp.port_in = UDS_PHUP_PORT_IN;
    hydroponics.UDS_PHUp.pin_out = UDS_PHUP_PIN_OUT;
    hydroponics.UDS_PHUp.port_out = UDS_PHUP_PORT_OUT;


    UDS_Module_Init(&hydroponics.UDS_PHUp);
    UDS_Module_Init(&hydroponics.UDS_PHDown);
    UDS_Module_Init(&hydroponics.UDS_Nutrient);



}
void HydroponicPPumps_Init(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef gpio_ppump;

    gpio_ppump.GPIO_Pin = GPIO_Pin_6; //echo
	gpio_ppump.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_ppump.GPIO_Mode = GPIO_Mode_OUT;
    gpio_ppump.GPIO_OType = GPIO_OType_PP; //PP
    gpio_ppump.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio_ppump);

/*
    gpio_ppump.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &gpio_ppump);

    gpio_ppump.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &gpio_ppump);
*/

    hydroponics.PPump_PHDown.port_out = GPIOC;
    hydroponics.PPump_PHDown.pin_out = GPIO_Pin_6;

    hydroponics.PPump_PHUp.state = MODULE_RUNNING;
    hydroponics.PPump_PHDown.state = MODULE_RUNNING;
    hydroponics.PPump_Nutrient.state = MODULE_RUNNING;

    PPump_Module_Init(&hydroponics.PPump_PHUp, &hydroponics.UDS_PHUp);
    PPump_Module_Init(&hydroponics.PPump_PHDown, &hydroponics.UDS_PHDown);
    PPump_Module_Init(&hydroponics.PPump_Nutrient, &hydroponics.UDS_Nutrient);

/*
    GPIO_Init(GPIOD, &gpio_ppump);

    GPIO_ResetBits(GPIOD, GPIO_Pin_3);
    delay_ms(1000);
    GPIO_SetBits(GPIOD, GPIO_Pin_3);
*/

}

void vHydroponicSystem_Init(void) {

    USART_rtos_packet packet;
    packet.USARTx_rtos = &USART1_rtos;
    USART_rtos_puts(&packet, " Init Hydroponic System...");

    hydroponics.state = SYSTEM_WRITING;

    HydroponicPPumps_Init();
    HydroponicUDS_Init();



    hydroponics.PHMeter_Resvoir.state = MODULE_RUNNING;
    PHMeter_Module_Init(&hydroponics.PHMeter_Resvoir);



    if (hydroponics.PPump_PHUp.state == MODULE_STANDBY &&
        hydroponics.PPump_PHDown.state == MODULE_STANDBY &&
        hydroponics.PPump_Nutrient.state == MODULE_STANDBY &&  
        hydroponics.UDS_PHUp.state == MODULE_STANDBY &&
        hydroponics.UDS_PHDown.state == MODULE_STANDBY &&
        hydroponics.UDS_Nutrient.state == MODULE_STANDBY &&
        hydroponics.PHMeter_Resvoir.state == MODULE_STANDBY) {

        hydroponics.state = SYSTEM_WRITTEN;
        //USART_puts(USART2, "Hydroponics successfully initialized...\r\n");
    }

    else {
        //USART2_rtos_puts("Hydroponics could not initialize...\r\n");

        hydroponics.state = SYSTEM_ERROR;
        //xTaskCreate(vHydroponicSystem_Init, (signed char*)"vHydroponicSystem_Init", 64, NULL, tskIDLE_PRIORITY+10, NULL);

    }
    //vTaskDelay() Delay for a little than do the task again
    //
    //USART_rtos_puts(USART2, " OK\r\n");

    USART_rtos_packet packet1;
    packet1.USARTx_rtos = &USART1_rtos;
    USART_rtos_puts(&packet1, " Ok\r\n");

    USART_rtos_wait_send(&packet);
    USART_rtos_wait_send(&packet1);

    vTaskDelete(NULL);

}

void vLight_task(vLight_parameters *params) {
    /*
    Light_set(params->status);
    int height = Plant_get_height();
    Light_distance_set(distance+height);

    //maybe look for something like recurring task
    //you need to find a way to change variable while its waiting;
    */

    static int counter = 5;
    int i;
    while (1) {
        for(i=0; i<counter; i++) {
            vTaskDelay((portTickType)(1000*3600) / portTICK_RATE_MS);
        }
    }

    vTaskDelete(NULL);
} 

int TEST_PH_VALUE = 50;

void vUDS_read_distance(UDS_Module *module) {

   USART_rtos_packet packet1;
   packet1.USARTx_rtos = &USART1_rtos;

   USART_rtos_puts(&packet1, "Reading Ultrasonic Distance Sensor...\r\n");

   taskENTER_CRITICAL();
   int v = UDS_read_distance(module); 
   taskEXIT_CRITICAL();



   USART_puts(USART1, "\r\nDistance: ");
   USART_put_int(USART1, v);
   USART_puts(USART1, "\r\n");
   //USART_rtos_sputs(&packet, "Distance: %d", v);

   //USART_rtos_wait_send(&packet);
   //USART_rtos_wait_send(&packet1);
   
   USART_puts(USART1, "FUCK"); //ERROR OCCURS WHEN I UNCOMMENT THE CODE CODE ABOVE

   vTaskDelete(NULL);
}



void vPH_read_value(PHMeter_Module *meter) {
    if (meter) {
        meter->state = MODULE_RUNNING;
     //   USART_rtos_puts(USART2, "Reading PHMeter Reservoir...\r\n");
        //USART2_rtos_puts("Read pH Meter...\r\n");
        meter->value = TEST_PH_VALUE;
        meter->state = MODULE_EXECUTED;
        xSemaphoreGive(meter->mutex);

        }

    vTaskDelete(NULL);
}

void vPPump_dispense(vPPump_parameters *params) {
    params->ppump->state = MODULE_RUNNING; 
    //GPIO_WriteBit(params->port, params->pin, Bit_SET);
    //USART2_rtos_puts("Starting to dispense...\r\n");
    
    taskENTER_CRITICAL();
    PPump_Module_dispense(params->ppump, params->amount);

    taskEXIT_CRITICAL();
    //    TEST_PH_VALUE+=5;

    params->ppump->state = MODULE_EXECUTED; 

    vTaskDelete(NULL);
}

void vPH_task(void) {
    static int16_t counter = 24 - 1; //24 hours

    USART_rtos_packet packet, packet1, packet2, packet3;

    packet.USARTx_rtos = packet1.USARTx_rtos = packet2.USARTx_rtos = packet3.USARTx_rtos = &USART1_rtos;
    USART_rtos_puts(&packet, "PH Task Launched...\r\n");

    unsigned portBASE_TYPE parent_priority = uxTaskPriorityGet(NULL);

    while (1) {

        if (hydroponics.PHMeter_Resvoir.mutex == NULL) {
            hydroponics.PHMeter_Resvoir.mutex = xSemaphoreCreateMutex();
            continue;
        }

       //add if sepamphore is not null error check
        int TARGET_PH_REACHED = 0;
         //MAKE SUB TASKS CREATED INCREMENT ITS PRIORITY BY ONE RELATIVE TO PARENT TASK
        
        if (xSemaphoreTake(hydroponics.PHMeter_Resvoir.mutex, portMAX_DELAY) == pdTRUE) {
            USART_rtos_puts(&packet1, "Acquired mutex for PHMeter...\r\n");

            hydroponics.PHMeter_Resvoir.state = MODULE_STANDBY;
            while (!TARGET_PH_REACHED) {
                vPPump_parameters ppump_params;
                portTickType start_time;
                portTickType end_time;

                xTaskCreate(vPH_read_value, (signed char*)"PH Read", 128, &hydroponics.PHMeter_Resvoir, parent_priority+1, NULL);

                while(hydroponics.PHMeter_Resvoir.state != MODULE_EXECUTED);

                int value = hydroponics.PHMeter_Resvoir.value; 
                char msg[20];
                packet3.msg = msg;
                USART_rtos_sputs(&packet3, "PH VALUE: %d\r\n", value);

                hydroponics.PHMeter_Resvoir.state = MODULE_STANDBY;

                int amount;
                PPump_Module *ppump;

                if (value < hydroponics.target_ph){
                    USART_rtos_puts(&packet2, "Adding PH Up...\r\n");
                    TEST_PH_VALUE+=5;
                 ///   USART_rtos_puts(USART2, "ADDING PH UP\r\n");
                    amount = hydroponics.target_ph - value;
                    ppump = &hydroponics.PPump_PHUp;

                }
                else if (value > hydroponics.target_ph) {
                    USART_rtos_puts(&packet2, "Adding PH DOwn...\r\n");
                    TEST_PH_VALUE-=5;
                    amount = value - hydroponics.target_ph;
                    ppump = &hydroponics.PPump_PHDown;

                }
                else {
                    TARGET_PH_REACHED = 1;
                    USART_rtos_puts(&packet2, "Stabilized PH...\r\n");
                    continue;
                };

                start_time = xTaskGetTickCount();
                end_time = (portTickType)(1000*hydroponics.ph_delay) / portTICK_RATE_MS;

                if (xSemaphoreTake(hydroponics.PHMeter_Resvoir.mutex, portMAX_DELAY) == pdTRUE) {
                    PPump_Module_dispense(ppump, amount);
                }
                else {
                    continue;
                }

                vTaskDelayUntil(&start_time, end_time);

                switch (ppump_params.ppump->state) {
                    case MODULE_EXECUTED: 
                      ///  USART_rtos_puts(USART2, "VPPUMP EXITED SUCCESSFULLY!\r\n");
                        break;
                    case MODULE_ERROR:
                       /// USART_rtos_puts(USART2, "VPPUMP EXITED WITH AN ERROR!\r\n");
                        break;
                }
            }
        }
        int i;
        for(i=0; i<counter; i++) {
            vTaskDelay((portTickType)(1000*3600) / portTICK_RATE_MS);
        }
    }
    vTaskDelete(NULL);
}


typedef struct {
    char ucMessageID;
    //char u

} command_data;

#define HEX_2_DEC(val) (((val)/16)*10+((val)%16))

void vRTC_Init(void) {
    RTC_InitTypeDef RTC_InitStructure;
    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_DateTypeDef RTC_DateStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    RCC_LSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();

    if (RTC_ReadBackupRegister(RTC_BKP_DR0)!=0x9527) {
        RTC_WriteProtectionCmd(DISABLE); 
        RTC_EnterInitMode();

        RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
        RTC_InitStructure.RTC_AsynchPrediv = 0x7D-1;
        RTC_InitStructure.RTC_SynchPrediv = 0xFF-1;
        RTC_Init(&RTC_InitStructure);

        RTC_TimeStructure.RTC_Seconds = 0x00;
        RTC_TimeStructure.RTC_Minutes = 0x01;
        RTC_TimeStructure.RTC_Hours = 0x01;
        RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
        RTC_SetTime(RTC_Format_BCD,&RTC_TimeStructure);

        RTC_DateStructure.RTC_Date = 30;
        RTC_DateStructure.RTC_Month = 5;
        RTC_DateStructure.RTC_WeekDay= RTC_Weekday_Thursday;
        RTC_DateStructure.RTC_Year = 12;
        RTC_SetDate(RTC_Format_BCD,&RTC_DateStructure);

        RTC_ExitInitMode();
        RTC_WriteBackupRegister(RTC_BKP_DR0,0X9527);
        RTC_WriteProtectionCmd(ENABLE);
        RTC_WriteBackupRegister(RTC_BKP_DR0,0x9527);  //Initialization is complete, set the flag
    }

      PWR_BackupAccessCmd(DISABLE);
}
//Main Function

void vIncrementDay(void) {
    RTC_TimeTypeDef t;
    portTickType xLastWake = xTaskGetTickCount();


    while(1) {
        int i;
        for(i=0; i<23; i++){
            /*
            USART_rtos_packet packet;
            packet.USARTx_rtos = &USART1_rtos; 
*/
            vTaskDelayUntil(&xLastWake, ((1000*3600) / portTICK_RATE_MS));
            xLastWake = xTaskGetTickCount();
            RTC_GetTime(RTC_Format_BIN, &t);

            USART_put_int(USART1, t.RTC_Hours);
            USART_put(USART1, ':');
            USART_put_int(USART1, t.RTC_Minutes);
            USART_puts(USART1, " (");
            USART_put_int(USART1, i);
            USART_puts(USART1, ")\r\n");


           /* 
            char buffer[60]; 
            packet.msg = buffer;
            USART_rtos_sputs(&packet, "HELLO%d\r\n", 69);
*/

            //USART_rtos_sputs(&packet, "%d:%d (%d)\r\n", t.RTC_Hours, t.RTC_Minutes, i);
//            USART_rtos_wait_send(&packet);


           
            //USART_puts(USART1, "5\r\n");

        }
        USART_puts(USART1, "=----DONE----=\r\n");
    }
}

int main(void){
	//Call initx(); To Initialize USART & GPIO

	initx();
    USART1_Init();
    init_us_timer();
    vRTC_Init();


    USART_rtos_init(&USART1_rtos, USART1);
    setup_output(&USART1_rtos);

    //CLOCK_SetClockTo168MHz();
   // setSysTick();

    int i;

    USART_puts(USART1, "\nBooting up...\r\n");
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


    GPIO_InitTypeDef gpio_ppump;

    gpio_ppump.GPIO_Pin = GPIO_Pin_5; 
    gpio_ppump.GPIO_OType = GPIO_OType_PP; 
    gpio_ppump.GPIO_Mode = GPIO_Mode_OUT; 
    gpio_ppump.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio_ppump);

    GPIO_ResetBits(GPIOE, GPIO_Pin_5);


/*
    RTC_TimeTypeDef t;
    for(i=0; i<10; i++) {
        RTC_GetTime(RTC_Format_BIN, &t);
        USART_put_int(USART1, t.RTC_Seconds);
        USART_puts(USART1, "\r\n");
        delay_ms(1000);
    }

    USART_puts(USART1, "======\r\n");
*/
/*
    GPIO_InitTypeDef gpio_ppump;

    gpio_ppump.GPIO_Pin = GPIO_Pin_6; 
    gpio_ppump.GPIO_OType = GPIO_OType_PP; 
    gpio_ppump.GPIO_Mode = GPIO_Mode_OUT; 
    gpio_ppump.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio_ppump);


    PPump_Typedef ppump;
    ppump.pin_out = GPIO_Pin_6;
    ppump.port_out = GPIOE;
    ppump.ml_per_m = 60;

        USART_puts(USART1, "DISPENSING...\r\n");
        GPIO_SetBits(GPIOE, GPIO_Pin_6);

*/


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpio_ultrasonic;

    gpio_ultrasonic.GPIO_Pin = GPIO_Pin_7; //echo
    gpio_ultrasonic.GPIO_OType = GPIO_OType_PP;
    //gpio_ultrasonic.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio_ultrasonic.GPIO_Mode = GPIO_Mode_IN;
    gpio_ultrasonic.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &gpio_ultrasonic);

    gpio_ultrasonic.GPIO_Pin = GPIO_Pin_8; //trigger
    gpio_ultrasonic.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOE, &gpio_ultrasonic);



    UDS_Module uds;

    uds.port_in = GPIOE;
    uds.pin_in = GPIO_Pin_7;
    uds.port_out = GPIOE;
    uds.pin_out = GPIO_Pin_8;
    uds.timeout = 500;

    UDS_Module_Init(&uds);


    USART_puts(USART1, "Starting UDS...\r\n");
    while(1) {
        int d = UDS_Module_distance(&uds);
        USART_puts(USART1, "Distance: ");
        USART_put_int(USART1, d);
        USART_puts(USART1, "\r\n");
        delay_ms(1000);
    }
    /*

    
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
    servo_init(&servo1, &(TIM4->CCR1), 500, 1850); //up and down servo
    servo_init(&servo2, &(TIM4->CCR2), 500, 1000);
    servo_set_degrees(&servo1, 90);
*/

    xTaskCreate(idle_blinky, (signed char*)"idle_blinky", 128, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vIncrementDay, (signed char*)"vIncrementDay", 128, NULL, tskIDLE_PRIORITY+10, NULL);


    vLight_parameters light_p;
    //ph_p.target_ph = 200;

    USART_puts(USART1, "LOOK AT LINE 18 FOR TODO!\r\n");

    //xTaskCreate(vLight_task, (signed char*)"vLight", 128, &light_p, tskIDLE_PRIORITY+1, NULL);
    //

    xTaskCreate(vHydroponicSystem_Init, (signed char*)"vHydroponicSystem_Init", 256, NULL, tskIDLE_PRIORITY+10, NULL);
    //xTaskCreate(vLightingSystem_Init, (signed char*)"vLightingSystem_Init", 64, NULL, tskIDLE_PRIORITY+10, NULL);
    //xTaskCreate(vACSystem_Init, (signed char*)"vACSystem_Init", 64, NULL, tskIDLE_PRIORITY+10, NULL);


    xTaskCreate(vPH_task, (signed char*)"vPH", 256, NULL, tskIDLE_PRIORITY+2, NULL);
   




	//xTaskCreate(UsartTask, (signed char*)"UsartTask", 128, NULL, tskIDLE_PRIORITY+1, NULL);

	//Call Scheduler
	vTaskStartScheduler();
    //
    
}
