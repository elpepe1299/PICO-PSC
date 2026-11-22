/* Gution coms parser and library, lacks some cleaning but working 
   Coms protocol:
    - AABB<DATA>AABB : 
        - AABB : Start and end token
        - <DATA> : Phase angle in degrees, float
    - AACC<DATA>AACC : 
        - AACC : Start and end token
        - <DATA> : PWM enable/disable, 1 or 0
*/
#include "guition.h"
#include "control.h"

#include <stdio.h>
#include <string.h>

#define AABB_TOKEN "AABB"
#define AABB_LEN   4

#define AACC_TOKEN "AACC"
#define AACC_LEN   4

static char token_buffer[TOKEN_MAX + 1];
static uint8_t token_len = 0;

static char data_buffer[GUITION_DATA_MAX + 1];
static uint8_t data_len = 0;

static char last_data[GUITION_DATA_MAX + 1];

static uint8_t aabb_start_match = 0;
static uint8_t aabb_end_match = 0;

static bool aabb_capturing = false;

static uint8_t aacc_start_match = 0;
static uint8_t aacc_end_match = 0;

static bool aacc_capturing = false;

static char aacc_value = '\0';

static bool is_digit_char(char c)
{
    return (c >= '0' && c <= '9');
}


static void token_reset(void)
{
    token_len = 0;

    token_buffer[0] = '\0';
}


static void token_push_char(char c)
{
    if (token_len < TOKEN_MAX)
    {
        token_buffer[token_len++] = c;

        token_buffer[token_len] = '\0';
    }
    else
    {
        for (int i = 0; i < TOKEN_MAX - 1; i++)
        {
            token_buffer[i] =
                token_buffer[i + 1];
        }

        token_buffer[TOKEN_MAX - 1] = c;

        token_buffer[TOKEN_MAX] = '\0';
    }
}

static bool guition_token_has_valid_format(
    const char *token
)
{
    if (token == NULL)
    {
        return false;
    }

    if (strlen(token) != TOKEN_MAX)
    {
        return false;
    }

    for (int i = 0; i < TOKEN_MAX; i++)
    {
        if (!is_digit_char(token[i]))
        {
            return false;
        }
    }

    if (strncmp(token, "10101", 5) != 0)
    {
        return false;
    }

    if (strcmp(&token[6], "1010") != 0)
    {
        return false;
    }

    return true;
}


static gui_event_t guition_check_token_string(
    const char *token
)
{
    if (!guition_token_has_valid_format(token))
    {
        return GUI_EVENT_NONE;
    }

    switch (token[5])
    {
        case '1':
            return GUI_EVENT_UP;

        case '2':
            return GUI_EVENT_DOWN;

        case '3':
            return GUI_EVENT_MAN;

        case '4':
            return GUI_EVENT_AUTO;

        default:
            return GUI_EVENT_NONE;
    }
}

static void aabb_reset(void)
{
    aabb_start_match = 0;
    aabb_end_match = 0;

    aabb_capturing = false;

    data_len = 0;

    data_buffer[0] = '\0';
}


static void data_push_char(char c)
{
    if (data_len < GUITION_DATA_MAX)
    {
        data_buffer[data_len++] = c;

        data_buffer[data_len] = '\0';
    }
}


static void data_push_pending_aabb_chars(void)
{
    for (
        uint8_t i = 0;
        i < aabb_end_match;
        i++
    )
    {
        data_push_char(
            AABB_TOKEN[i]
        );
    }

    aabb_end_match = 0;
}


static gui_event_t guition_parse_aabb_char(
    char c
)
{
    if (!aabb_capturing)
    {
        if (
            c ==
            AABB_TOKEN[aabb_start_match]
        )
        {
            aabb_start_match++;

            if (
                aabb_start_match >=
                AABB_LEN
            )
            {
                aabb_capturing = true;

                aabb_start_match = 0;
                aabb_end_match = 0;

                data_len = 0;

                data_buffer[0] = '\0';

                token_reset();
            }
        }
        else
        {
            if (c == AABB_TOKEN[0])
            {
                aabb_start_match = 1;
            }
            else
            {
                aabb_start_match = 0;
            }
        }

        return GUI_EVENT_NONE;
    }

    if (
        c ==
        AABB_TOKEN[aabb_end_match]
    )
    {
        aabb_end_match++;

        if (
            aabb_end_match >=
            AABB_LEN
        )
        {
            data_buffer[data_len] = '\0';

            strncpy(
                last_data,
                data_buffer,
                GUITION_DATA_MAX
            );

            last_data[GUITION_DATA_MAX] =
                '\0';

            aabb_reset();

            return GUI_EVENT_DATA;
        }

        return GUI_EVENT_NONE;
    }

    if (aabb_end_match > 0)
    {
        data_push_pending_aabb_chars();
    }

    if (c == AABB_TOKEN[0])
    {
        aabb_end_match = 1;
    }
    else
    {
        data_push_char(c);
    }

    return GUI_EVENT_NONE;
}

static void aacc_reset(void)
{
    aacc_start_match = 0;
    aacc_end_match = 0;

    aacc_capturing = false;

    aacc_value = '\0';
}


static gui_event_t guition_parse_aacc_char(
    char c
)
{
    if (!aacc_capturing)
    {
        if (
            c ==
            AACC_TOKEN[aacc_start_match]
        )
        {
            aacc_start_match++;

            if (
                aacc_start_match >=
                AACC_LEN
            )
            {
                aacc_capturing = true;

                aacc_start_match = 0;
                aacc_end_match = 0;

                aacc_value = '\0';

                token_reset();
            }
        }
        else
        {
            if (c == AACC_TOKEN[0])
            {
                aacc_start_match = 1;
            }
            else
            {
                aacc_start_match = 0;
            }
        }

        return GUI_EVENT_NONE;
    }

    if (aacc_value == '\0')
    {
        if (c == '0' || c == '1')
        {
            aacc_value = c;

            aacc_end_match = 0;

            return GUI_EVENT_NONE;
        }

        aacc_reset();

        return GUI_EVENT_NONE;
    }

    if (
        c ==
        AACC_TOKEN[aacc_end_match]
    )
    {
        aacc_end_match++;

        if (
            aacc_end_match >=
            AACC_LEN
        )
        {
            char value =
                aacc_value;

            aacc_reset();

            if (value == '1')
            {
                return
                    GUI_EVENT_PWM_ENABLE;
            }

            if (value == '0')
            {
                return
                    GUI_EVENT_PWM_DISABLE;
            }

            return GUI_EVENT_NONE;
        }

        return GUI_EVENT_NONE;
    }

    aacc_reset();

    return GUI_EVENT_NONE;
}

gui_event_t guition_parse_char(char c)
{
    gui_event_t event;

    if (aabb_capturing)
    {
        return guition_parse_aabb_char(c);
    }

    if (aacc_capturing)
    {
        return guition_parse_aacc_char(c);
    }

    event =
        guition_parse_aabb_char(c);

    if (event != GUI_EVENT_NONE)
    {
        return event;
    }

    if (aabb_capturing)
    {
        return GUI_EVENT_NONE;
    }

    event =
        guition_parse_aacc_char(c);

    if (event != GUI_EVENT_NONE)
    {
        return event;
    }

    if (aacc_capturing)
    {
        return GUI_EVENT_NONE;
    }

    if (is_digit_char(c))
    {
        token_push_char(c);

        if (token_len == TOKEN_MAX)
        {
            event =
                guition_check_token_string(
                    token_buffer
                );

            if (event != GUI_EVENT_NONE)
            {
                token_reset();

                return event;
            }
        }
    }
    else
    {
        token_reset();
    }

    return GUI_EVENT_NONE;
}

const char *guition_get_data(void)
{
    return last_data;
}

void guition_send_text(
    uart_inst_t *uart_port,
    const char *text
)
{
    uart_puts(
        uart_port,
        text
    );

    uart_puts(
        uart_port,
        "\r\n"
    );
}


void guition_change_fill(
    uart_inst_t *uart_port,
    const char *name,
    uint8_t value
)
{
    char buffer[128];

    snprintf(
        buffer,
        sizeof(buffer),
        "[{\"name\":\"%s\",\"prop\":{\"val\":%u}}]\f",
        name,
        value
    );

    uart_puts(
        uart_port,
        buffer
    );
}


void guition_change_txt(
    uart_inst_t *uart_port,
    const char *name,
    const char *txt
)
{
    char buffer[128];

    snprintf(
        buffer,
        sizeof(buffer),
        "{\"name\":\"%s\",\"prop\":{\"txt\":\"%s\"}}\f",
        name,
        txt
    );

    uart_puts(
        uart_port,
        buffer
    );
}

void guition_change_screen_pack(
    uart_inst_t *uart_port,
    float temp_c,
    float angle_deg
)
{
    if (uart_port == NULL)
    {
        return;
    }

    char buffer[384];

    char temp_txt[24];
    char angle_txt[24];
    char perc_txt[24];

    float ndeg =
        saturate(
            angle_deg,
            -90.0f,
            90.0f
        );

    float deg_fill =
        ((ndeg + 90.0f) * 100.0f) /
        180.0f;

    uint8_t fill_value =
        (uint8_t)(
            deg_fill + 0.5f
        );

    snprintf(
        temp_txt,
        sizeof(temp_txt),
        "%.2f C",
        (double)temp_c
    );

    snprintf(
        angle_txt,
        sizeof(angle_txt),
        "%.2f deg",
        (double)ndeg
    );

    snprintf(
        perc_txt,
        sizeof(perc_txt),
        "%.2f %%",
        (double)deg_fill
    );

    snprintf(
        buffer,
        sizeof(buffer),

        "["
        "{\"name\":\"Temp_sensor\",\"prop\":{\"txt\":\"%s\"}},"
        "{\"name\":\"Text_ang\",\"prop\":{\"txt\":\"%s\"}},"
        "{\"name\":\"ang_perc\",\"prop\":{\"txt\":\"%s\"}},"
        "{\"name\":\"SP_fill\",\"prop\":{\"val\":%u}}"
        "]\f",

        temp_txt,
        angle_txt,
        perc_txt,
        fill_value
    );

    uart_puts(
        uart_port,
        buffer
    );
}