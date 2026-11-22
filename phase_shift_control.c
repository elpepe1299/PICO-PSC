#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

// Custom headers for control, GUI communication, and PIO programs
#include "control.h"
#include "guition.h"
#include "dab_pwm.pio.h"

// Constants for system clock, PWM frequency, UART configuration, GPIO pins, state machines, and phase control
#define OC_SYS_CLK_KHZ     340000U
#define PWM_FREQ_HZ        200000.0f
#define UART_SCR           uart0
#define BAUDS              115200
#define DATA_BITS          8
#define STOP_BITS          1
#define PARITY             UART_PARITY_NONE
#define TX0                0
#define RX0                1
#define PRI_BASE_PIN       13U
#define SEC_BASE_PIN       18U
#define SM_MASTER_PRI      0U
#define SM_MASTER_SEC      1U
#define SM_BRIDGE_PRI      2U
#define SM_BRIDGE_SEC      3U
#define IRQ_COMMIT_PRI     4U
#define IRQ_COMMIT_SEC     5U
#define PHASE_STEP_DEG     0.5f
#define PHASE_UPDATE_US    1000U

static PIO dab_pio = pio0;

static uint32_t period_ticks;
static uint32_t half_period_ticks;
static uint32_t master_count;

static bool pwm_initialized = false;
static bool pwm_enabled = false;
static float current_phase_deg = 0.0f;

volatile float deg = 0.0f;
volatile bool shift_ready = false;
volatile bool pwm_enable_request = false;
volatile bool pwm_disable_request = false;

critical_section_t phase_cs;

repeating_timer_t angle_timer;

volatile uint16_t temp_raw = 0;
volatile uint32_t temp_mv = 0;
volatile float temp_c = 0.0f;
volatile bool temp_ready = false;
repeating_timer_t temp_timer;

// Callback function for the repeating timer to read the temperature sensor, averaging 10 readings for stability
bool temp_sensing_callback(repeating_timer_t *rt)
{
    uint32_t raw_sum = 0;
    uint16_t raw_avg;

    adc_select_input(4);

    for (uint8_t i = 0; i < 10; i++)
    {
        raw_sum += adc_read();
    }

    raw_avg = raw_sum / 10U;
    temp_raw = raw_avg;

    temp_mv = ((uint32_t)raw_avg * 3300UL) / 4095UL;
    temp_c = 27.0f - (((float)temp_mv - 706.0f) / 1.721f);
    temp_ready = true;
    return true;
}

// RT task to read the temperature sensor every 200 ms, averaging 10 readings for stability
void update_temp(void)
{
    adc_init();

    adc_set_temp_sensor_enabled(true);
    add_repeating_timer_ms(
        -200,
        temp_sensing_callback,
        NULL,
        &temp_timer
    );
}

// Updates the GUI with the latest temperature reading, angle, and percentage fill
void scr_update_temp(void)
{
    if (!temp_ready)
    {
        return;
    }

    temp_ready = false;

    float temp_c1 = temp_c;
    char txt[24];
    char ang[24];
    char perc[24];

    float ndeg = saturate(deg, -90.0f, 90.0f);
    float deg_fill = (ndeg + 90.0f) * 0.555555555f;

    snprintf(ang, sizeof(ang), "%.2f deg ", (double)ndeg);
    snprintf(txt, sizeof(txt), "%.2f C", (double)temp_c1);
    snprintf(perc, sizeof(perc), "%.2f %%", (double)deg_fill);

    guition_change_txt(UART_SCR, "Temp_sensor", txt);
    guition_change_txt(UART_SCR, "Text_ang", ang);
    guition_change_txt(UART_SCR, "ang_perc", perc);
}

// Initializes the UART for communication with the GUI, setting the baud rate, data format, and GPIO pins
void init_uart0(void)
{
    uart_init(UART_SCR, BAUDS);

    gpio_set_function(TX0, UART_FUNCSEL_NUM(uart0, TX0));
    gpio_set_function(RX0, UART_FUNCSEL_NUM(uart0, RX0));

    uart_set_hw_flow(UART_SCR, false, false);

    uart_set_format(UART_SCR, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_SCR, true);
    uart_set_baudrate(UART_SCR, BAUDS);
}

static uint32_t phase_to_ticks(float phase_deg)
{
    float phase = fabsf(phase_deg);
    return (uint32_t)lroundf(phase * ((float)period_ticks / 360.0f));
}


static void init_bridge_pins(PIO pio, uint sm, uint pin)
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_set_pins_with_mask64(pio, sm, 0, 1ULL << pin);
}

static uint32_t dab_pwm_get_sm_mask(void)
{
    return
        (1u << SM_MASTER_PRI) |
        (1u << SM_MASTER_SEC) |
        (1u << SM_BRIDGE_PRI) |
        (1u << SM_BRIDGE_SEC);
}

bool dab_pwm_is_enabled(void)
{
    return pwm_enabled;
}

// Starts the PWM output if it was already initialized, otherwise it does nothing
void dab_pwm_start(void)
{
    if (!pwm_initialized)
    {
        return;
    }

    if (pwm_enabled)
    {
        return;
    }

    uint32_t sm_mask = dab_pwm_get_sm_mask();

    pio_set_sm_mask_enabled(dab_pio, sm_mask, false);

    pio_sm_clear_fifos(dab_pio, SM_MASTER_PRI);
    pio_sm_clear_fifos(dab_pio, SM_MASTER_SEC);
    pio_sm_clear_fifos(dab_pio, SM_BRIDGE_PRI);
    pio_sm_clear_fifos(dab_pio, SM_BRIDGE_SEC);

    pio_sm_restart(dab_pio, SM_MASTER_PRI);
    pio_sm_restart(dab_pio, SM_MASTER_SEC);
    pio_sm_restart(dab_pio, SM_BRIDGE_PRI);
    pio_sm_restart(dab_pio, SM_BRIDGE_SEC);

    pio_interrupt_clear(dab_pio, 0);
    pio_interrupt_clear(dab_pio, 1);
    pio_interrupt_clear(dab_pio, IRQ_COMMIT_PRI);
    pio_interrupt_clear(dab_pio, IRQ_COMMIT_SEC);

    pio_sm_put_blocking(dab_pio, SM_MASTER_PRI, master_count);
    pio_sm_put_blocking(dab_pio, SM_MASTER_SEC, master_count);
    pio_sm_put_blocking(dab_pio, SM_BRIDGE_PRI, 0);
    pio_sm_put_blocking(dab_pio, SM_BRIDGE_SEC, 0);

    current_phase_deg = 0.0f;
    pio_enable_sm_mask_in_sync(dab_pio, sm_mask);

    pwm_enabled = true;
}

// Stop the PWM and reset the state machines, clearing the output pins
void dab_pwm_stop(void)
{
    if (!pwm_initialized)
    {
        return;
    }

    if (!pwm_enabled)
    {
        return;
    }

    uint32_t sm_mask = dab_pwm_get_sm_mask();

    pio_set_sm_mask_enabled(dab_pio, sm_mask, false);

    pio_sm_clear_fifos(dab_pio, SM_BRIDGE_PRI);
    pio_sm_clear_fifos(dab_pio, SM_BRIDGE_SEC);

    pio_sm_set_pins_with_mask64(dab_pio, SM_BRIDGE_PRI, 0, 1ULL << PRI_BASE_PIN);
    pio_sm_set_pins_with_mask64(dab_pio, SM_BRIDGE_SEC, 0, 1ULL << SEC_BASE_PIN);

    pwm_enabled = false;
}

// PIOs and state machines are initialized only once, and the PWM is started immediately if it was already initialized
void dab_pwm_init(void)
{
    if (pwm_initialized)
    {
        dab_pwm_start();
        return;
    }

    PIO pio = dab_pio;

    uint32_t sys_clk = clock_get_hz(clk_sys);
    period_ticks = sys_clk / PWM_FREQ_HZ;
    half_period_ticks = period_ticks / 2U;
    master_count = half_period_ticks - 3U;

    pio_sm_claim(pio, SM_MASTER_PRI);
    pio_sm_claim(pio, SM_MASTER_SEC);
    pio_sm_claim(pio, SM_BRIDGE_PRI);
    pio_sm_claim(pio, SM_BRIDGE_SEC);

    uint tick_offset = pio_add_program(pio, &dab_tick_program);
    uint bridge_offset = pio_add_program(pio, &dab_bridge_program);

    pio_sm_config ct = dab_tick_program_get_default_config(tick_offset);

    sm_config_set_clkdiv(&ct, 1.0f);

    pio_sm_init(pio, SM_MASTER_PRI, tick_offset, &ct);
    pio_sm_init(pio, SM_MASTER_SEC, tick_offset, &ct);

    init_bridge_pins(pio, SM_BRIDGE_PRI, PRI_BASE_PIN);
    init_bridge_pins(pio, SM_BRIDGE_SEC, SEC_BASE_PIN);

    pio_sm_config cp = dab_bridge_program_get_default_config(bridge_offset);

    sm_config_set_clkdiv(&cp, 1.0f);
    sm_config_set_set_pins(&cp, PRI_BASE_PIN, 1);
    sm_config_set_mov_status(&cp, STATUS_IRQ_SET, IRQ_COMMIT_PRI);

    pio_sm_config cs = dab_bridge_program_get_default_config(bridge_offset);

    sm_config_set_clkdiv(&cs, 1.0f);
    sm_config_set_set_pins(&cs, SEC_BASE_PIN, 1);
    sm_config_set_mov_status(&cs, STATUS_IRQ_SET, IRQ_COMMIT_SEC);

    pio_sm_init(pio, SM_BRIDGE_PRI, bridge_offset, &cp);
    pio_sm_init(pio, SM_BRIDGE_SEC, bridge_offset, &cs);

    pio_interrupt_clear(pio, 0);
    pio_interrupt_clear(pio, 1);
    pio_interrupt_clear(pio, IRQ_COMMIT_PRI);

    pio_interrupt_clear(pio, IRQ_COMMIT_SEC);

    pio_sm_put_blocking(pio, SM_MASTER_PRI, master_count);
    pio_sm_put_blocking(pio, SM_MASTER_SEC, master_count);
    pio_sm_put_blocking(pio, SM_BRIDGE_PRI, 0);
    pio_sm_put_blocking(pio, SM_BRIDGE_SEC, 0);

    pwm_initialized = true;
    current_phase_deg = 0.0f;

    uint32_t sm_mask = dab_pwm_get_sm_mask();
    pio_enable_sm_mask_in_sync(pio, sm_mask);

    pwm_enabled = true;
}

// Gradual phase shifting to avoid abrupt changes in the PWM output
bool dab_pwm_set_phase(float target_phase_deg)
{
    if (!pwm_initialized)
    {
        return false;
    }

    if (!pwm_enabled)
    {
        return false;
    }

    // Saturation for security purposes, although the GUI should already handle this
    target_phase_deg = saturate(target_phase_deg, -30.0f, 30.0f);

    float error = target_phase_deg - current_phase_deg;

    if (fabsf(error) < 0.001f)
    {
        current_phase_deg = target_phase_deg;
        return true;
    }

    float next_phase;

    if (fabsf(error) <= PHASE_STEP_DEG)
    {
        next_phase = target_phase_deg;
    }
    else if (error > 0.0f)
    {
        next_phase = current_phase_deg + PHASE_STEP_DEG;
    }
    else
    {
        next_phase = current_phase_deg - PHASE_STEP_DEG;
    }

    if (!pio_sm_is_tx_fifo_empty(dab_pio, SM_BRIDGE_PRI))
    {
        return false;
    }

    if (!pio_sm_is_tx_fifo_empty(dab_pio, SM_BRIDGE_SEC))
    {
        return false;
    }

    uint32_t ticks = phase_to_ticks(next_phase);

    uint32_t pri_ticks;
    uint32_t sec_ticks;

    if (next_phase >= 0.0f)
    {
        pri_ticks = 0;
        sec_ticks = ticks;
    }
    else
    {
        pri_ticks = ticks;
        sec_ticks = 0;
    }

    pio_sm_put(dab_pio, SM_BRIDGE_PRI, pri_ticks);
    pio_sm_put(dab_pio, SM_BRIDGE_SEC, sec_ticks);

    dab_pio->irq_force =
        (1u << IRQ_COMMIT_PRI) |
        (1u << IRQ_COMMIT_SEC);

    current_phase_deg = next_phase;

    if (fabsf(target_phase_deg - current_phase_deg) < 0.001f)
    {
        current_phase_deg = target_phase_deg;
        return true;
    }

    return false;
}

// Overclocking the RP2350 to *340 MHz* for better pwm resolution and performance
void overclock(void)
{
    bool success;

    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);

    // Don't touch this, otherwise USB will not work :)
    clock_configure(clk_usb, 0, CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * MHZ, 48 * MHZ);

    success = set_sys_clock_khz(OC_SYS_CLK_KHZ, true);

    if (!success)
    {
        while (true)
        {
            tight_loop_contents();
        }
    }

    // Just in case 
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,48 * MHZ, 48 * MHZ);
}

// Core 1 in charge of UART coms (screen), temp sensing and miscs
void core1_func(void)
{
    update_temp();

    while (true)
    {
        scr_update_temp();

        while (uart_is_readable(UART_SCR))
        {
            char c = uart_getc(UART_SCR);

            gui_event_t event = guition_parse_char(c);

            switch (event)
            {
                case GUI_EVENT_DATA:
                {
                    const char *data = guition_get_data();

                    char *endptr;

                    float new_deg = strtof(data, &endptr);

                    if (endptr != data)
                    {
                        new_deg = saturate(new_deg, -30.0f, 30.0f);

                        critical_section_enter_blocking(&phase_cs);
                        deg = new_deg;
                        shift_ready = true;
                        critical_section_exit(&phase_cs);
                    }

                    break;
                }

                case GUI_EVENT_PWM_ENABLE:
                {
                    critical_section_enter_blocking(&phase_cs);
                    pwm_enable_request = true;
                    pwm_disable_request = false;
                    critical_section_exit(&phase_cs);

                    break;
                }

                case GUI_EVENT_PWM_DISABLE:
                {
                    critical_section_enter_blocking(&phase_cs);
                    pwm_disable_request = true;
                    pwm_enable_request = false;
                    critical_section_exit(&phase_cs);

                    break;
                }

                case GUI_EVENT_UP:
                case GUI_EVENT_DOWN:
                case GUI_EVENT_MAN:
                case GUI_EVENT_AUTO:
                case GUI_EVENT_NONE:
                default:
                {
                    break;
                }
            }
        }

        tight_loop_contents();
    }
}

// Main core in charge of PWM control and phase shifting
int main(void)
{
    overclock();

    stdio_init_all();

    sleep_ms(10);

    init_uart0();

    critical_section_init(&phase_cs);

    // Core 1 Initializacion 
    multicore_launch_core1(core1_func);

    // Degrees buffer for pending phase shift requests, to be applied gradually
    float pending_deg = 0.0f;

    bool pending = false;

    uint64_t phase_update_time = time_us_64();

    while (true)
    {
        bool enable_request = false;
        bool disable_request = false;

        critical_section_enter_blocking(&phase_cs);

        if (pwm_enable_request)
        {
            pwm_enable_request = false;
            enable_request = true;
        }

        if (pwm_disable_request)
        {
            pwm_disable_request = false;
            disable_request = true;
        }

        critical_section_exit(&phase_cs);

        if (disable_request)
        {
            if (dab_pwm_is_enabled())
            {
                dab_pwm_stop();
                pending = false;
            }
        }

        if (enable_request)
        {
            if (!dab_pwm_is_enabled())
            {
                dab_pwm_init();

                critical_section_enter_blocking(&phase_cs);
                pending_deg = deg;
                shift_ready = false;
                critical_section_exit(&phase_cs);

                pending = true;
                phase_update_time = time_us_64();
            }
        }

        if (dab_pwm_is_enabled())
        {
            critical_section_enter_blocking(&phase_cs);

            if (shift_ready)
            {
                pending_deg = deg;
                shift_ready = false;
                pending = true;
                phase_update_time = time_us_64();
            }

            critical_section_exit(&phase_cs);
        }

        if (dab_pwm_is_enabled() && pending)
        {
            uint64_t now =
                time_us_64();

            if ((now - phase_update_time) >= PHASE_UPDATE_US)
            {
                phase_update_time = now;

                if (dab_pwm_set_phase(pending_deg))
                {
                    pending = false;
                }
            }
        }

        tight_loop_contents();
    }
}