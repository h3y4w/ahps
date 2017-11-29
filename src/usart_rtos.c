#include "usart_rtos.h"
#include <stdlib.h>
#include <stdarg.h>


int USART_rtos_wait_send(USART_rtos_packet *packet) {
    
    return (xSemaphoreTake(packet->sent_mutex, packet->max_block) == pdTRUE);
}

void vUSART_puts(USART_rtos_packet *packet) {

    if (xSemaphoreTake(packet->USARTx_rtos->transmit_mutex, portMAX_DELAY) == pdTRUE) {
        taskENTER_CRITICAL();
        USART_puts(packet->USARTx_rtos->id, packet->msg);

        xSemaphoreGive(packet->USARTx_rtos->transmit_event_mutex);
        xSemaphoreGive(packet->USARTx_rtos->transmit_mutex);

        xSemaphoreGive(packet->sent_mutex); 
        taskEXIT_CRITICAL();

    }
    vTaskDelete(NULL);
}

void USART_rtos_puts(USART_rtos_packet *packet, char *msg) {

    packet->msg = msg;
    packet->sent_mutex = xSemaphoreCreateMutex();


    if (xSemaphoreTake(packet->sent_mutex, portMAX_DELAY) == pdTRUE) { //change this from max_delay

        //if (packet->task_handle == NULL) //allow to assign different tasks wont work now tho
        //if (packet->max_block == NULL) packet->max_block = 
        packet->max_block = portMAX_DELAY;

        xTaskCreate(vUSART_puts, (signed char*)"vUSART_puts", 128, packet, tskIDLE_PRIORITY+8, NULL);

    }
    else {
        USART_puts(USART1, "ERROR483\r\n");
        USART_puts(USART2, "ERROR483\r\n");

    }

};

int USART_rtos_sputs(USART_rtos_packet *packet, const char *format, ...) {
    if (packet->msg){


        va_list ap;
        int rv;

        va_start(ap, format);
        rv = vsprintf(packet->msg, format, ap);
        va_end(ap);

        USART_rtos_puts(packet, packet->msg);

        return rv;
    }
    return -1;
}


void USART_rtos_init(USART_rtos *USARTx_rtos,USART_TypeDef *USARTx) { //pass usart_Rtos and usartx
    USARTx_rtos->id = USARTx;
    USARTx_rtos->write_pos = 0;
    USARTx_rtos->read_pos = 0;

    USARTx_rtos->transmit_mutex = xSemaphoreCreateMutex(); 
    USARTx_rtos->receive_mutex = xSemaphoreCreateMutex(); 

    USARTx_rtos->transmit_event_mutex = xSemaphoreCreateMutex(); 
    USARTx_rtos->receive_event_mutex = xSemaphoreCreateMutex(); 
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

void USART_put_unsigned_int(USART_TypeDef* USARTx, unsigned int number) {
    char value[10];
    int i=0;
    do {
        value[i++] = (char)(number % 10) + '0';
        number /= 10;
    } while (number);

    while(i) {
        USART_put(USARTx, value[--i]);
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

void USART_getline(USART_rtos *USARTx_rtos, int timeout) {

    if (xSemaphoreTake(USARTx_rtos->receive_event_mutex, (portTickType)(timeout) / portTICK_RATE_MS == pdTRUE)) { 
        //Takes the mutex.
        //Next time USART_IRQ executes it releases mutex 
        if (xSemaphoreTake(USARTx_rtos->receive_event_mutex, (portTickType)(timeout) / portTICK_RATE_MS) == pdTRUE) {
            //USART_IRQ exexcuted and relased mutex
        }
        xSemaphoreGive(USARTx_rtos->receive_event_mutex);
    }

}

int USART_nextline_length(USART_rtos *USARTx_rtos) {
    if (!USARTx_rtos) return -1;

    //add some mutex on read_pos;
    USART_put_int(USARTx_rtos->id, USARTx_rtos->read_pos);
    USART_puts(USARTx_rtos->id, "PRINTING NEXT LINE: ");

    int pos = USARTx_rtos->read_pos;
    for(pos; USARTx_rtos->buffer[pos]!='\r'; pos++) {
        USART_put(USARTx_rtos->id, USARTx_rtos->buffer[pos]);

    }
    USART_puts(USARTx_rtos->id, "\r\n");
    /*
    USART_puts(USART1, buffer[*read_pos-1]);
    USART_puts(USART1, buffer[*read_pos+1]);


    USART_puts(USART1, "-\r\n");
    */

    return (pos - USARTx_rtos->read_pos)+1;
}

void handle_usart_command(char *command) {
    USART_puts(USART2, "Command: ");
    if (str_compare(command, "overview\r", '\r')) {}

    //if (str_compare)

    //char *pos = command;
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

void USART_readline(USART_rtos *USARTx_rtos, char *buffer, uint8_t length) {
 //   static uint8_t first=1;

    buffer[length-1] = '\0';

    for(USARTx_rtos->read_pos; USARTx_rtos->read_pos<length-2; USARTx_rtos->read_pos++){
        uint32_t prim;

        if (xSemaphoreTake(USARTx_rtos->receive_mutex, portMAX_DELAY) == pdTRUE) {

            prim = __get_PRIMASK(); //returns if irq were disable already
            __disable_irq(); 
            if (!prim) __enable_irq(); //only enables irq if it recieved it in a disabled state

            if (USARTx_rtos->read_pos == USART_BUFFER_LEN-1) USARTx_rtos->read_pos=0;
            buffer[USARTx_rtos->read_pos] = USARTx_rtos->buffer[USARTx_rtos->read_pos];
            xSemaphoreGive(USARTx_rtos->receive_mutex);
        }
    }
   // if (!first) read_pos++; //This skips the previous Carriage return added to the end of data added to buffer
    //else first = 0;
    USARTx_rtos->read_pos++;

}

void USART_readline_int(USART_rtos *USARTx_rtos, int *num) {
    *(num) = 0;
    int i=0;
    uint8_t neg = 0;

    int len = USART_nextline_length(USARTx_rtos)+1; //add for str terminator
    char buffer[len];
    USART_readline(USARTx_rtos->id, buffer, len);

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

