#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define USART_BUFFER_LEN 300

typedef struct {
    int write_pos;
    int read_pos;
    USART_TypeDef *id;
    xSemaphoreHandle transmit_mutex;
    xSemaphoreHandle receive_mutex;
    xSemaphoreHandle receive_event_mutex;
    xSemaphoreHandle transmit_event_mutex;
    char buffer[USART_BUFFER_LEN];
} USART_rtos;

typedef struct {
    USART_rtos *USARTx_rtos;
    char *msg;
    xSemaphoreHandle sent_mutex;
    portTickType max_block;
} USART_rtos_packet;


void USART_rtos_init(USART_rtos *USARTx_rtos,USART_TypeDef *USARTx);


void USART_put(USART_TypeDef* USARTx, volatile char c);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USART_put_int(USART_TypeDef* USARTx, int number);

void USART_getline(USART_rtos *USARTx_rtos, int timeout);
int USART_nextline_length(USART_rtos *USARTx_rtos);

void handle_usart_command(char *command);

void USART_readline(USART_rtos *USARTx_rtos, char *buffer, uint8_t length);

void USART_readline_int(USART_rtos *USARTx_rtos, int *num);

int USART_rtos_wait_send(USART_rtos_packet *packet);
void vUSART_puts(USART_rtos_packet *packet);
void USART_rtos_puts(USART_rtos_packet *packet, char *msg);
int USART_rtos_sputs(USART_rtos_packet *packet, const char *format, ...);
void USART_rtos_init(USART_rtos *USARTx_rtos,USART_TypeDef *USARTx);












