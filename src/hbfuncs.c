#include "FreeRTOS.h"
#include "task.h"
#include "hb_structs.h"

extern Timer_node* timer_list_head;

void setTime(int hours, int minutes, int seconds) {
    RTC_TimeTypeDef RTC_TimeStructure;

    RTC_EnterInitMode();

    //debug
    if (timer_list_head)
        timer_list_head->debug3 = (unsigned) RTC_TimeStructure.RTC_Hours;

    RTC_TimeStructure.RTC_Seconds = DEC_2_HEX(seconds);
    RTC_TimeStructure.RTC_Minutes = DEC_2_HEX(minutes);//0x01;
    RTC_TimeStructure.RTC_Hours = DEC_2_HEX(hours);//0x01;
    RTC_TimeStructure.RTC_H12 = RTC_H12_AM;
    RTC_SetTime(RTC_Format_BCD,&RTC_TimeStructure);


    RTC_ExitInitMode();
}

//for future: dont pass the previous epoch  but rather create it within function
//Also prevent other tasks interrupting
void setTimeSafely(int hours, int minutes, int seconds, unsigned prev_epoch) { //sets time safely for timers and other tasks
    RTC_TimeTypeDef t;
    taskENTER_CRITICAL();

    getTime(&t);
    unsigned int c = getEpoch(&t, getSystemDay());

    setTime(hours, minutes, seconds);
    if (timer_list_head) correctTimeChangeOffset(c, timer_list_head);
    taskEXIT_CRITICAL();

} 


void setDate(int month, int day, int year) {
    RTC_DateTypeDef RTC_DateStructure;

    RTC_EnterInitMode();

    RTC_DateStructure.RTC_Date = day;
    RTC_DateStructure.RTC_Month = month;
    RTC_DateStructure.RTC_WeekDay= RTC_Weekday_Thursday;
    RTC_DateStructure.RTC_Year = year;
    RTC_SetDate(RTC_Format_BCD,&RTC_DateStructure);

    RTC_ExitInitMode();

}

void getTime(RTC_TimeTypeDef *time) {
    RTC_GetTime(RTC_Format_BIN, time);
/*
    time->RTC_Hours = time->RTC_Hours;
    time->RTC_Minutes = time->RTC_Minutes;
    time->RTC_Seconds = time->RTC_Seconds;
    */
}


unsigned getEpoch(RTC_TimeTypeDef *time, int days) {

    unsigned int epoch = days * 86400;
    epoch += time->RTC_Hours * 3600;
    epoch += time->RTC_Minutes * 60;
    epoch += time->RTC_Seconds;
    return epoch;
}

