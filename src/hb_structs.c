#include "hb_structs.h"
/*
USART_rtos *USARTOutput_rtos; 

void setup_debug_struct_output(USART_rtos *USARTx_rtos) {
    USARTOutput_rtos = USARTx_rtos;
}
*/

static unsigned int SYSTEM_DAY_COUNT; //private file var

unsigned getSystemDay() {
    return SYSTEM_DAY_COUNT;
}

void setSystemDay(unsigned int day) {
    SYSTEM_DAY_COUNT = day;
}

void correctTimeChangeOffset(unsigned int prev_epoch, Timer_node* timer_list_head) {
    RTC_TimeTypeDef t;


    Timer_node* cursor = timer_list_head;
    while(cursor) {
        RTC_GetTime(RTC_Format_BIN, &t);
        unsigned int current_epoch = getEpoch(&t, getSystemDay());
        unsigned int offset = 0;

        //gets the absolute offset
        int missed = 0;
        offset = cursor->epoch - prev_epoch; 
        
        //cursor->debug2 = cursor->epoch; //prev timer epoch
        cursor->epoch = current_epoch + offset; //current timer epoch
        cursor->debug =  prev_epoch; // prev epoch
        cursor->debug1 = current_epoch; //current epoch
        cursor = cursor->next;
    }
}

Timer_node* FindTimerInterrupt(int id, Timer_node *timer_list_head) {
    Timer_node* cursor = timer_list_head;
    while (cursor) {
        if (cursor->id == id) return cursor;
        cursor = cursor->next;
    }
    return NULL;
}

int addTimerInterrupt(Timer_node *timer, Timer_node **timer_list_head) {
    if (!timer->timer_mutex) timer->timer_mutex = xSemaphoreCreateMutex();
    if (xSemaphoreTake(timer->timer_mutex, portMAX_DELAY) == pdTRUE) {
        Timer_node* cursor = *timer_list_head;
        timer->next = NULL;//safety

        if (cursor == NULL) {
            *timer_list_head = timer; 
            return 1;
        }

        while(cursor->next) cursor = cursor->next;
        cursor->next = timer;

        int r = (xSemaphoreTake(timer->timer_mutex, portMAX_DELAY) == pdTRUE);
        if (r) xSemaphoreGive(timer->timer_mutex);
        return r;

    }

    return 0;
}
