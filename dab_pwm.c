/*PWM utilities library*/
#include "dab_pwm.h"

#include <math.h>

#include "hardware/clocks.h"
#include "hardware/pwm.h"

static bool pwm_initialized = false;
static bool pwm_enabled = false;

static uint primary_slice = 0;
static uint secondary_slice = 0;

static uint primary_channel = 0;
static uint secondary_channel = 0;

static uint32_t pwm_period_ticks = 0;
static uint16_t pwm_wrap = 0;

static float pwm_actual_frequency = 0.0f;

static float current_phase_deg = 0.0f;

static float dab_pwm_clampf(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static float dab_pwm_absf(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static int32_t dab_pwm_phase_to_ticks(
    float phase_deg
)
{
    float ticks =
        phase_deg *
        ((float)pwm_period_ticks / 360.0f);

    return (int32_t)lroundf(ticks);
}

static uint32_t dab_pwm_mod_period(
    int32_t value
)
{
    int32_t period =
        (int32_t)pwm_period_ticks;

    while (value < 0)
    {
        value += period;
    }

    while (value >= period)
    {
        value -= period;
    }

    return (uint32_t)value;
}

static void dab_pwm_apply_phase(
    float phase_deg
)
{
    uint16_t primary_counter =
        pwm_get_counter(
            primary_slice
        );

    int32_t phase_ticks =
        dab_pwm_phase_to_ticks(
            phase_deg
        );

    int32_t secondary_counter =
        (int32_t)primary_counter -
        phase_ticks;

    uint32_t normalized_counter =
        dab_pwm_mod_period(
            secondary_counter
        );

    pwm_set_counter(
        secondary_slice,
        (uint16_t)normalized_counter
    );
}

static void dab_pwm_configure_slice(
    uint pin,
    uint slice,
    uint channel
)
{
    gpio_set_function(
        pin,
        GPIO_FUNC_PWM
    );

    pwm_config config =
        pwm_get_default_config();

    pwm_config_set_clkdiv(
        &config,
        1.0f
    );

    pwm_config_set_wrap(
        &config,
        pwm_wrap
    );

    pwm_init(
        slice,
        &config,
        false
    );

    uint16_t level =
        (uint16_t)(
            pwm_period_ticks / 2U
        );

    pwm_set_chan_level(
        slice,
        channel,
        level
    );

    pwm_set_counter(
        slice,
        0
    );
}

bool dab_pwm_init(void)
{
    if (pwm_initialized)
    {
        dab_pwm_start();

        return true;
    }

    uint32_t sys_clk =
        clock_get_hz(
            clk_sys
        );

    if (sys_clk == 0U)
    {
        return false;
    }

    float ideal_period =
        (float)sys_clk /
        DAB_PWM_FREQ_HZ;

    pwm_period_ticks =
        (uint32_t)(
            ideal_period + 0.5f
        );

    if (
        pwm_period_ticks < 2U ||
        pwm_period_ticks > 65536U
    )
    {
        return false;
    }

    pwm_wrap =
        (uint16_t)(
            pwm_period_ticks - 1U
        );

    pwm_actual_frequency =
        (float)sys_clk /
        (float)pwm_period_ticks;

    primary_slice =
        pwm_gpio_to_slice_num(
            DAB_PWM_PRIMARY_PIN
        );

    secondary_slice =
        pwm_gpio_to_slice_num(
            DAB_PWM_SECONDARY_PIN
        );

    primary_channel =
        pwm_gpio_to_channel(
            DAB_PWM_PRIMARY_PIN
        );

    secondary_channel =
        pwm_gpio_to_channel(
            DAB_PWM_SECONDARY_PIN
        );

    if (
        primary_slice ==
        secondary_slice
    )
    {
        return false;
    }

    dab_pwm_configure_slice(
        DAB_PWM_PRIMARY_PIN,
        primary_slice,
        primary_channel
    );

    dab_pwm_configure_slice(
        DAB_PWM_SECONDARY_PIN,
        secondary_slice,
        secondary_channel
    );

    current_phase_deg = 0.0f;

    pwm_initialized = true;

    dab_pwm_start();

    return true;
}

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

    pwm_set_enabled(
        primary_slice,
        false
    );

    pwm_set_enabled(
        secondary_slice,
        false
    );

    gpio_set_function(
        DAB_PWM_PRIMARY_PIN,
        GPIO_FUNC_PWM
    );

    gpio_set_function(
        DAB_PWM_SECONDARY_PIN,
        GPIO_FUNC_PWM
    );

    pwm_set_counter(
        primary_slice,
        0
    );

    pwm_set_counter(
        secondary_slice,
        0
    );

    uint32_t mask =
        (1u << primary_slice) |
        (1u << secondary_slice);

    pwm_set_mask_enabled(
        mask
    );

    current_phase_deg = 0.0f;

    pwm_enabled = true;
}

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

    pwm_set_enabled(
        primary_slice,
        false
    );

    pwm_set_enabled(
        secondary_slice,
        false
    );

    gpio_set_function(
        DAB_PWM_PRIMARY_PIN,
        GPIO_FUNC_SIO
    );

    gpio_set_function(
        DAB_PWM_SECONDARY_PIN,
        GPIO_FUNC_SIO
    );

    gpio_set_dir(
        DAB_PWM_PRIMARY_PIN,
        GPIO_OUT
    );

    gpio_set_dir(
        DAB_PWM_SECONDARY_PIN,
        GPIO_OUT
    );

    gpio_put(
        DAB_PWM_PRIMARY_PIN,
        0
    );

    gpio_put(
        DAB_PWM_SECONDARY_PIN,
        0
    );

    pwm_enabled = false;
}

bool dab_pwm_set_phase(
    float target_phase_deg
)
{
    if (!pwm_initialized)
    {
        return false;
    }

    if (!pwm_enabled)
    {
        return false;
    }

    target_phase_deg =
        dab_pwm_clampf(
            target_phase_deg,
            DAB_PWM_PHASE_MIN_DEG,
            DAB_PWM_PHASE_MAX_DEG
        );

    float error =
        target_phase_deg -
        current_phase_deg;

    if (
        dab_pwm_absf(error) <
        0.001f
    )
    {
        current_phase_deg =
            target_phase_deg;

        return true;
    }

    float next_phase;

    if (
        dab_pwm_absf(error) <=
        DAB_PWM_PHASE_STEP_DEG
    )
    {
        next_phase =
            target_phase_deg;
    }
    else if (error > 0.0f)
    {
        next_phase =
            current_phase_deg +
            DAB_PWM_PHASE_STEP_DEG;
    }
    else
    {
        next_phase =
            current_phase_deg -
            DAB_PWM_PHASE_STEP_DEG;
    }

    dab_pwm_apply_phase(
        next_phase
    );

    current_phase_deg =
        next_phase;

    if (
        dab_pwm_absf(
            target_phase_deg -
            current_phase_deg
        )
        < 0.001f
    )
    {
        current_phase_deg =
            target_phase_deg;

        return true;
    }

    return false;
}

bool dab_pwm_is_initialized(void)
{
    return pwm_initialized;
}

bool dab_pwm_is_enabled(void)
{
    return pwm_enabled;
}

float dab_pwm_get_phase(void)
{
    return current_phase_deg;
}

float dab_pwm_get_actual_frequency(void)
{
    return pwm_actual_frequency;
}

float dab_pwm_get_phase_resolution_deg(void)
{
    if (pwm_period_ticks == 0U)
    {
        return 0.0f;
    }

    return
        360.0f /
        (float)pwm_period_ticks;
}
