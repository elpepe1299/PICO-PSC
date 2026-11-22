/* Control algorithms library along with some plant simulation functions and saturation function*/
#include "control.h"

float saturate(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float PI_action(PI_controller *pi, float e, float min, float max)
{
    const float Ts = pi->Ts;
    const float A  = pi->Kp + 0.5 * pi->Ki * Ts;
    const float B  = -pi->Kp + 0.5 * pi->Ki * Ts;

    pi->u = pi->u + A * e + B * pi->e_ant;
    pi->u = saturate(pi->u, min, max);

    pi->e_ant = e;
    return pi->u;
}

float simulated_plant(float u)
{
    float y = 0.1033 * u;
    return y;
}