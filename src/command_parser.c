#include "command_parser.h"
#include "hbfuncs.h" 
#include "hb_structs.h"

USART_rtos *USARTOutput_rtos; 

void setup_output(USART_rtos *USARTx_rtos) {
    USARTOutput_rtos = USARTx_rtos;
}

void replace_char(char *msg, char find, char replace) {
    while(*msg!='\r') {
        if (*msg == find) *msg = replace;
        msg++;
    }
}

int str_compare(char *str1, char *str2, const char terminator) {
    int i;
    for(i=0;; i++) {

        if (str1[i] == str2[i]) {
            if (str1[i] == terminator) return 1; // strs match
        }

        else {
            return 0;
        }
    }
}


int find_next_char(char *msg, int offset, char find) {
    int i=0;
    while (1) {
        if (msg[i] == find) break;
        else if (msg[i] == '\0') {
            return -1;
            break;
        }
        i++;
    }

    return i;
}


int str_to_int(char *buffer, char terminator) {
    int num = 0;
    int i=0;
    int neg=0;

    if (buffer[i] == '-') {
        i++;
        neg = 1;
    }

    for(i; buffer[i]!=terminator; i++) {
        if (buffer[i] >= '0' && buffer[i]<='9'){
            num *= 10; 
            num += buffer[i]-'0';
        }

        else {
            break;
        }    
    }

    if (neg) {
        num *= -1;
    }
    return num;

}




Stage stages[MAX_STAGES];
int REGIME_PH[MAX_REGIME_PH][2] = {
                                    {-1, -1}, 
                                    {-1, -1}, 
                                    {-1, -1}, 
                                    {-1, -1}, 
                                    {-1, -1}, 
                                    {-1, -1},
                                    {-1, -1}, 
                                    {-1, -1}, 
                                    {-1, -1},
                                    {-1, -1}
                                  }; //ph, delay

int REGIME_LIGHTING[MAX_REGIME_LIGHTING][2] ={{-1, -1}}; //distance, hours_on

void command_set_regime_ph(char *pos, char term) {
    USART_rtos_packet packet;
    char buffer[80];
    char msg[] = "{'object_type': 'REGIME_PH', 'method': 'set', 'error':%d, 'object_id':%d}\r\n";
    packet.msg = buffer;

    packet.USARTx_rtos = USARTOutput_rtos; 


    int NEW_ITEM = 0;

    int idx = find_next_char(pos, 0, term);
    pos+=idx+1;

    int regime_id = str_to_int(pos, '\003')-1;
    if (!(regime_id >= -1 && regime_id <MAX_REGIME_PH)){
        USART_rtos_sputs(&packet, msg, 100, -1); // error 100: incorrect id
        USART_rtos_wait_send(&packet);
        return;
    }

    else if (regime_id == -1) {
        int i;
        for(i=0; i<MAX_REGIME_PH; i++) if (REGIME_PH[i][1]  == -1 && REGIME_PH[i][0] == -1) break;

        if (i == MAX_REGIME_PH) {

            USART_rtos_sputs(&packet, msg, 101, -1); //error 101: need to overwrite existing rows
            USART_rtos_wait_send(&packet);
            return;
        }

        regime_id = i;
        NEW_ITEM = 1;
    }


    idx = find_next_char(pos, 0, term);
    pos+=idx+1;

    //break this up into smaller functions for each task

    int i;
    for(i=0; i<2 && idx!=0; i++) {
        switch (*pos) {
            int v;
            char buffer[30];

            case 'p':
                pos++;
                v = str_to_int(pos, '\003');
                //printf("p=%d\n", v);
                REGIME_PH[regime_id][0] = v;
                break;

            case 'd': 
               pos++;
               v = str_to_int(pos, '\003'); 
               //printf("d=%d\n", v);
               REGIME_PH[regime_id][1] = v;
               break;

            default:
               USART_rtos_sputs(&packet, msg, 102, -1); //error 102: incorrect variable
               USART_rtos_wait_send(&packet);
               if (NEW_ITEM) {
                   REGIME_PH[regime_id][0] = -1; //PH setting
                   REGIME_PH[regime_id][1] = -1; // delay
                   return;
               }

        }
        idx = find_next_char(pos, 0, term);
        if (idx == -1) break;
        pos+=idx+1;
    }
    USART_rtos_sputs(&packet, msg, 0, regime_id+1);


}

void command_set_stage(char *pos, char term) {
    USART_rtos_packet packet;
    char buffer[80];
    char msg[] = "{'object_type': 'STAGE', 'method': 'set', 'error':%d, 'object_id':%d}\r\n";
    packet.msg = buffer;
    packet.USARTx_rtos = USARTOutput_rtos;

    int NEW_ITEM = 0;

    int idx = find_next_char(pos, 0, term);
    pos+=idx+1;

    int stage_id = str_to_int(pos, '\003')-1;

    if (!(stage_id >= -1 && stage_id <MAX_STAGES)) {
        USART_rtos_sputs(&packet, msg, 100, -1); // error 100: incorrect id
        USART_rtos_wait_send(&packet);
        return;
    }

    else if (stage_id == -1) {
        int i;
        for(i=0; i<MAX_STAGES; i++) {
            if (stages[i].using == 0) break;
        }

        if (i == MAX_STAGES) {
            USART_rtos_sputs(&packet, msg, 101, -1); //error 101: need to overwrite existing rows
            USART_rtos_wait_send(&packet);
            return;
        }

        stage_id = i;
        stages[stage_id].using = 1;
        NEW_ITEM = 1;
    }

    idx = find_next_char(pos, 0, term);
    pos+=idx+1;

    int i;

    stages[stage_id].id = stage_id+1;

    for(i=0; i<5 && idx!=0; i++) {
        int v;
        //pos++;
        switch (/**(pos-1)*/*pos) {
            case 'l': //length in days
               pos++;
               v = str_to_int(pos, '\003'); 
               stages[stage_id].length = v;
               break;

            case 'p': //regime_ph_id 
               pos++;
               v = str_to_int(pos, '\003'); 
  //             printf("p=%d\n", v);
               stages[stage_id].regime_ph_id = v;
               break;

            case 'i': //regime_lighting_id 
               pos++;
               v = str_to_int(pos, '\003'); 
//               printf("g=%d\n", v);
               stages[stage_id].regime_lighting_id = v;
               break;

            case 'o': //order 
               pos++;
               v = str_to_int(pos, '\003'); 
//               printf("g=%d\n", v);
               stages[stage_id].order = v;
               break;
            

            default:
               USART_rtos_sputs(&packet, msg, 102, stage_id+1); // error 102: incorrect variable
               if (NEW_ITEM) {
                   stages[stage_id].using = 0;
                   stages[stage_id].order = 0;
                   stages[stage_id].regime_lighting_id = 0;
                   stages[stage_id].regime_ph_id = 0;
                   stages[stage_id].length = 0;

               }
               USART_rtos_wait_send(&packet);
               return;
        }
        idx = find_next_char(pos, 0, term);
        if (idx == -1) break;
        pos+=idx+1;
    }
    USART_rtos_sputs(&packet, msg, 0, stage_id+1);
    USART_rtos_wait_send(&packet);
}

void command_set(char *pos, char term) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;
    pos+=4;

    if (str_compare(pos, "REGIME_PH\003", term)) {
        command_set_regime_ph(pos, term);
    }

    else if (str_compare(pos, "STAGE\003", term)) {
        command_set_stage(pos, term);
    }

    else if (str_compare(pos, "REGIME_LIGHTING\003", term)) {
    }

    else {
        USART_rtos_puts(&packet, "{'object_type': '', method: 'set', error: 10, 'object_id': -1}\r\n"); // error 10: unknown object
        USART_rtos_wait_send(&packet);
        return;
    }

}

void command_del_regime_ph (char *pos, char term) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;

    int idx = find_next_char(pos, 0, term);
    pos+=idx+1;
    int regime_id = str_to_int(pos, '\003')-1;

    if (!(regime_id>=0 && regime_id < MAX_REGIME_PH)) {
        USART_rtos_puts(&packet, "Error: Incorrect REGIME_PH id\r\n");
        return;
    }

    USART_rtos_sputs(&packet, "DEBUG: DELETING REGIME_ID %d\r\n", regime_id);
    REGIME_PH[regime_id][0] = -1;
    REGIME_PH[regime_id][1] = -1;
}

void command_del_stage (char *pos, char term) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;

    int idx = find_next_char(pos, 0, term);
    pos+=idx+1;

    int stage_id = str_to_int(pos, '\003')-1;

    if (!(stage_id >= 0 && stage_id <MAX_STAGES)) {

        USART_rtos_puts(&packet, "Error: Incorrect stage id\r\n");
        return;
    }

    stages[stage_id].using = 0;
    stages[stage_id].order = 0;
    stages[stage_id].regime_lighting_id = 0;
    stages[stage_id].regime_ph_id = 0;
    stages[stage_id].length = 0;

    USART_rtos_sputs(&packet, "Succesfully deleted stage id: %d\r\n", stage_id+1);
}

void command_del(char *pos, char term) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;
    pos+=4;
    USART_puts(USARTOutput_rtos->id, "COOL\r\n");

    if (str_compare(pos, "REGIME_PH\003", term)) {
        command_del_regime_ph(pos, term);
    }

    else if (str_compare(pos, "STAGE\003", term)) {
        command_del_stage(pos, term);
    }

    else if (str_compare(pos, "REGIME_LIGHTING\003", term)) {
//        printf("OK LIGHTING SHIT\n");
    }

    else {
        USART_rtos_sputs(&packet, "Error: Unknown Object '%s'\r\n", pos);
        USART_rtos_wait_send(&packet);
    }
}

void stage_print(Stage *stage) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;
    char buffer[100];
    packet.msg = buffer;

    USART_rtos_sputs(
            &packet,
            "{'id': %d, 'length': %d, 'order':%d, 'ph_id': %d, 'lighting_id':%d, 'using':%d}\r\n", 
            stage->id,
            stage->length,
            stage->order,
            stage->regime_ph_id,
            stage->regime_lighting_id,
            stage->using
    );
    USART_rtos_wait_send(&packet);

}

void regime_ph_print(int *regime_ph, int regime_id) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;
    char buffer[50];
    packet.msg = buffer;

    USART_rtos_sputs(&packet, "{'id': %d, 'ph': %d, 'delay': %d}\r\n", regime_id+1, regime_ph[0], regime_ph[1]);
    USART_rtos_wait_send(&packet);

}

void command_get(char *pos, char term) {

    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;

    pos+=4; //moves cursor 4 chars to right to access next word. ex( |get ...  --> get |...) 

    if (str_compare(pos, "REGIME_PH\003", term)) {

        int idx = find_next_char(pos, 0, term);
        pos+=idx+1;
        int regime_id = str_to_int(pos, '\003')-1;

        if (regime_id >= 0 && regime_id <MAX_REGIME_PH) {
            regime_ph_print(REGIME_PH[regime_id], regime_id);

        }
        else if(regime_id == -1) {
            int i;
            for(i=0; i<MAX_REGIME_PH; i++) {
                regime_ph_print(REGIME_PH[i], i);
            }
        }

        else {

            USART_rtos_puts(&packet, "Error: Invalid regime_ph_id\r\n");
            USART_rtos_wait_send(&packet);
            return;
        }

    }

    if (str_compare(pos, "STAGE\003", term)) {

        int idx = find_next_char(pos, 0, term);
        pos+=idx+1;
        int stage_id = str_to_int(pos, '\003')-1;
        if (stage_id == -1) {
            int i;
            for(i=0; i<MAX_STAGES; i++) {
                stage_print(&stages[i]);
            }
        }
        else if (stage_id >= 0 && stage_id <MAX_STAGES) {
            stage_print(stages+stage_id);

        }
        else {
            USART_rtos_puts(&packet, "Error: Invalid stage_id\r\n");
            USART_rtos_wait_send(&packet);
            return;
        }

    }

    else {
        //USART_rtos_sputs(&packet, "Error: Unknown Object '%s'\r\n", pos);
        //USART_rtos_wait_send(&packet);

    }
}

void command_config_clock(char *pos, char term) {
    unsigned error_code = 99; //unknown error
    USART_rtos_packet packet;
    char buffer[80];
    char msg[] = "{'object_type': 'CLOCK', 'method': 'config', 'error':%u, h=%d, m=%d, s=%d}\r\n";
    packet.msg = buffer;
    packet.USARTx_rtos = USARTOutput_rtos;

    int NEW_ITEM = 0;

    int idx = find_next_char(pos, 0, term);
    pos+=idx+1;
   
    int i;

    int hours = 0, minutes = 0, seconds = 0;
    for(i=0; i<4 && idx!=0; i++) {
        //pos++;
        switch (*pos) {
            case 'h': //hours
               pos++;
               hours = str_to_int(pos, '\003'); 
               if (!(hours >= 0 && hours <=23)) {
                   hours = 0;
                   error_code = 400;
               }
               break;

            case 'm': //minutes 
               pos++;
               minutes = str_to_int(pos, '\003'); 
               if (!(minutes >= 0 && minutes <=59)) {
                   minutes = 0;
                   error_code = 400;
               }
               break;

            case 's': //seconds 
               pos++;
               seconds = str_to_int(pos, '\003'); 
               if (!(seconds >= 0 && seconds <=59)) {
                   seconds = 0;
                   error_code = 400;
               }
               break;

            default:
               error_code = 401;
               break;
        }

        if (error_code != 99) break;

        idx = find_next_char(pos, 0, term);
        if (idx == -1) break;
        pos+=idx+1;
    }

    RTC_TimeTypeDef t;
    if (error_code == 99) {
        getTime(&t);
        unsigned int c = getEpoch(&t, getSystemDay());
        setTimeSafely(hours, minutes, seconds, c);
        //RTC_GetTime(RTC_Format_BIN, &t);

        error_code = 0;
    }
        getTime(&t);






    /* FOR DEBUGGING 
    USART_puts(USARTOutput_rtos->id, "debug3: ");
    USART_put_unsigned_int(USARTOutput_rtos->id, timer_list_head->debug3);
    USART_puts(USARTOutput_rtos->id, "\r\n");

    
    USART_puts(USARTOutput_rtos->id, "prev: ");
    USART_put_unsigned_int(USARTOutput_rtos->id, timer_list_head->debug);
    USART_puts(USARTOutput_rtos->id, "\r\n");


    USART_puts(USARTOutput_rtos->id, "prev timer: ");
    USART_put_unsigned_int(USARTOutput_rtos->id, timer_list_head->debug2);
    USART_puts(USARTOutput_rtos->id, "\r\n");

    USART_puts(USARTOutput_rtos->id, "prev set timer: ");
    USART_put_unsigned_int(USARTOutput_rtos->id, timer_list_head->debug0);
    USART_puts(USARTOutput_rtos->id, "\r\n");



    USART_puts(USARTOutput_rtos->id, "current: ");
    USART_put_unsigned_int(USARTOutput_rtos->id, timer_list_head->debug1);

    USART_puts(USARTOutput_rtos->id, "\r\n");

    USART_puts(USARTOutput_rtos->id, "current timer: ");
    USART_put_unsigned_int(USARTOutput_rtos->id, timer_list_head->epoch);
    USART_puts(USARTOutput_rtos->id, "\r\n");


    */


    USART_rtos_sputs(&packet, msg, error_code, hours, minutes, seconds);
    USART_rtos_wait_send(&packet);
}

void command_config(char *pos, char term) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;

    pos+=7; //move cursor to next word
    if (str_compare(pos, "CLOCK\003", term)) {
        command_config_clock(pos, term);
//        USART_rtos_puts(&packet, "OK!\r\n");
 //       USART_rtos_wait_send(&packet);
        return;
    }
}



int command_routing(char *pos, char term) {
    USART_rtos_packet packet;
    packet.USARTx_rtos = USARTOutput_rtos;
    int r = 0;
    
    if (str_compare(pos, "get\003", term)) {
        command_get(pos, term);
    }

    else if (str_compare(pos, "set\003", term)) {
        command_set(pos, term);
    }

    else if (str_compare(pos, "del\003", term)) {
        command_del(pos, term);
    }

    else if(str_compare(pos, "config\003", term)) {
        command_config(pos, term);
    }

    else if (*pos != term){
        USART_rtos_puts(&packet, "{'method': '', 'error': 9}\r\n"); //error 9: Unknown start command
        USART_rtos_wait_send(&packet);
        r = 1;
    }
    return r; //1 = Error
}


/*
int main(void) {
    int i=0;
    //message_handler("Hello World man fuck you\n");
    while (1) {
        //char msg[]="set REGIME_PH $cycle1.stage1\0";

        char term = '\003';
        char msg[55] = {term};
        printf("> ");
        fgets(msg, 55, stdin);
        replace_char(msg, (char)10, term);//remove when moved to embbedd, casues be fgets which passed LF

        char *pos = msg;
        replace_char(msg, ' ', term);
        printf("(DEBUG)-%s-\n", pos);
        
        command_routing(pos, term);

    }

    return 0;
}

*/
