/*PWM utilities library*/
#ifndef DAB_PWM_H
#define DAB_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"

#define DAB_PWM_PRIMARY_PIN          10U
#define DAB_PWM_SECONDARY_PIN        18U

#define DAB_PWM_FREQ_HZ              200000.0f

#define DAB_PWM_PHASE_STEP_DEG       0.5f

#define DAB_PWM_PHASE_MIN_DEG       -30.0f
#define DAB_PWM_PHASE_MAX_DEG        30.0f

bool dab_pwm_init(void);

void dab_pwm_start(void);

void dab_pwm_stop(void);

bool dab_pwm_set_phase(float target_phase_deg);

bool dab_pwm_is_initialized(void);

bool dab_pwm_is_enabled(void);

float dab_pwm_get_phase(void);

float dab_pwm_get_actual_frequency(void);

float dab_pwm_get_phase_resolution_deg(void);

#endif
