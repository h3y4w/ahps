#include "FreeRTOS.h"
#include "semphr.h"

enum Module_State {MODULE_RUNNING, MODULE_STANDBY, MODULE_EXECUTED, MODULE_ERROR};
#define CM_CONVERSION ((float)58.2) 


typedef struct{
    volatile uint16_t *CCR;
    uint16_t offset;
    uint8_t ratio;
    uint8_t last_pos;
    uint8_t current_pos;
    enum Module_State state;
} Servo_Module;

typedef struct {
    int value;
    xSemaphoreHandle mutex;
    enum Module_State state;
} PHMeter_Module;

typedef struct {
    int distance;
    int lighton;
    enum Module_State state;
    Servo_Module *servo;
}Light_Module;

typedef struct {
    int value;
    xSemaphoreHandle mutex;
    enum Module_State state;
} Temperature_Module;

typedef struct {
    GPIO_TypeDef *port_out;
    uint16_t pin_out; 
    uint16_t ml_per_m;
    enum Module_State state;
    xSemaphoreHandle mutex;
} PPump_Module;

typedef struct {
    GPIO_TypeDef *port_in;
    GPIO_TypeDef *port_out;
    uint16_t pin_in;
    uint16_t pin_out;
    uint16_t timeout;
    xSemaphoreHandle mutex;
    enum Module_State state;
    int value;
} UDS_Module; //(U)ltrasonic (D)istance (S)ensor

typedef struct {

    //GPIO_TypeDef port_out;
    uint16_t pin_out;
    uint16_t speed;
    xSemaphoreHandle mutex;
    enum Module_State state;
} Fan_Module;

void PHMeter_Module_Init(PHMeter_Module *module);

void UDS_Module_Init( UDS_Module *module); 
int UDS_read_distance(UDS_Module *module);


void PPump_Module_Init(PPump_Module *module, UDS_Module *sensor);
void PPump_Module_dispense(PPump_Module *module, int amount_ml);


void Fan_Module_Init(Fan_Module *module);


