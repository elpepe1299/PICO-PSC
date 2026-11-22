/* Control algorithms library along with some plant simulation functions and saturation function*/
#ifndef CONTROL_H

typedef struct
{
    float Kp;
    float Ki;
    float Ts;
    float e_ant;
    float u;

} PI_controller;

float saturate(float x, float lo, float hi);
float PI_action(PI_controller *pi, float e, float min, float max);
float simulated_plant(float u);

#define CONTROL_H
#endif