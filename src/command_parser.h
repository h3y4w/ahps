#include "usart_rtos.h"

#ifndef COMMAND_PARSER_H__
#define COMMAND_PARSER_H__

#define MAX_STAGES 10
#define MAX_REGIME_PH MAX_STAGES * 1
#define MAX_REGIME_LIGHTING MAX_STAGES * 1

void setup_output(USART_rtos *USARTx_rtos);

void replace_char(char *msg, char find, char replace);
int str_compare(char *str1, char *str2, const char terminator);
int find_next_char(char *msg, int offset, char find); 
void message_handler(char *msg);
int str_to_int(char *buffer, char terminator);

typedef struct {
    int id;
    int length;
    int order;
    int regime_ph_id;
    int regime_lighting_id;
    int using;
} Stage;

void command_set_regime_ph(char *pos, char term);
void command_set_stage(char *pos, char term);
void command_set(char *pos, char term);

void command_del_regime_ph (char *pos, char term);
void command_del_stage (char *pos, char term);
void command_del(char *pos, char term);


void command_get(char *pos, char term);

int command_routing(char *pos, char term);


void stage_print(Stage *stage);
void regime_ph_print(int *regime_ph, int regime_id);

#endif
