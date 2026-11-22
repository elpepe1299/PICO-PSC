/* Gution coms parser and library, lacks some cleaning but working 
   Coms protocol:
    - AABB<DATA>AABB : 
        - AABB : Start and end token
        - <DATA> : Phase angle in degrees, float
    - AACC<DATA>AACC : 
        - AACC : Start and end token
        - <DATA> : PWM enable/disable, 1 or 0
*/
#ifndef GUITION_H
#define GUITION_H

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"

#define TOKEN_MAX           10U
#define GUITION_DATA_MAX    32U

typedef enum
{
    GUI_EVENT_NONE = 0,
    GUI_EVENT_UP,
    GUI_EVENT_DOWN,
    GUI_EVENT_MAN,
    GUI_EVENT_AUTO,
    GUI_EVENT_DATA,
    GUI_EVENT_PWM_ENABLE,
    GUI_EVENT_PWM_DISABLE

} gui_event_t;

gui_event_t guition_parse_char(char c);

const char *guition_get_data(void);
void guition_send_text(uart_inst_t *uart_port,const char *text);
void guition_change_fill(uart_inst_t *uart_port, const char *name, uint8_t value);
void guition_change_txt(uart_inst_t *uart_port, const char *name, const char *txt);
void guition_change_screen_pack(uart_inst_t *uart_port, float temp_c, float angle_deg);

#endif
