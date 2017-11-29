#include "module.h"
#include "hbfuncs.h"
//#include "usart_rtos.h"

#ifndef HB_STRUCTS
#define HB_STRUCTS
enum System_State {SYSTEM_WRITING, SYSTEM_WRITTEN, SYSTEM_READ, SYSTEM_ERROR};


unsigned getSystemDay();

void setSystemDay(unsigned int day);

//void setup_debug_struct_output(USART_rtos *USARTx_rtos);


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

typedef struct Timer_node{
    xSemaphoreHandle* timer_mutex;
    struct Timer_node* next;

    unsigned  epoch;
    uint8_t id;
    unsigned debug;
    unsigned debug0;
    unsigned debug1;
    unsigned debug2;
    unsigned debug3;


} Timer_node;

extern Timer_node* timer_list_head;

int addTimerInterrupt(Timer_node *timer, Timer_node **timer_list_head);
void correctTimeChangeOffset(unsigned int prev_epoch, Timer_node *timer_list_head);
Timer_node* FindTimerInterrupt(int id, Timer_node *timer_list_head);

#endif
