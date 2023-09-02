#include <stdio.h> //Basic Lib
#include <stdlib.h>
#include <string.h>
#include <math.h>
// Peripheral Lib(GPIO,UART,PCNT,PWM,TIM)
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_system.h" //SysLog Lib
#include "esp_log.h"
#include "esp_err.h"

// PWM
#define motor_ctrl_pwm_max 4095
#define motor_ctrl_pwm_min 0
#define motor_ctrl_pwm_max_limit 3000
#define motor_ctrl_pwm_min_limit -3000
#define R_param 16.1 // 轮距
// #define ODOMETER_EST_PULSE_PER_METER_LEFT   0.000013795058
// #define ODOMETER_EST_PULSE_PER_METER_RIGHT  0.000013882939 // Calibration Needed.
#define ODOMETER_EST_PULSE_PER_METER_LEFT   0.0013795058*2
#define ODOMETER_EST_PULSE_PER_METER_RIGHT  0.0013882939*2
#define PI 3.14159265359

typedef struct
{
    int Encoder_Left;
    int Encoder_Right;
    long long int Encoder_Left_Sum;
    long long int Encoder_Right_Sum;
    double Velocity;
    double Angle;
    float X;
    float Y;
} Location_t;
extern int PWM_Left;
extern int PWM_Right;
// extern pcnt_unit_handle_t pcnt_unit_right;
// extern pcnt_unit_handle_t pcnt_unit_left;
// extern pcnt_channel_handle_t pcnt_chan_left;
// extern pcnt_channel_handle_t pcnt_chan_right;
// extern pcnt_unit_config_t unit_config;
// extern pcnt_chan_config_t chan_config_left;
// extern pcnt_chan_config_t chan_config_right;

extern gptimer_handle_t gptimer;
extern gptimer_config_t timer_config;

extern float delta_time;
void PWM_Init(ledc_timer_config_t ledc_timer, ledc_channel_config_t* ledc_channel);
// void PCNT_Init(pcnt_unit_config_t unit_config,
//                pcnt_unit_handle_t pcnt_unit_left,
//                pcnt_unit_handle_t pcnt_unit_right,
//                pcnt_chan_config_t chan_config_left,
//                pcnt_chan_config_t chan_config_right,
//                pcnt_channel_handle_t pcnt_chan_left,
//                pcnt_channel_handle_t pcnt_chan_right);
void PCNT_GetCounter(pcnt_unit_handle_t pcnt_left, int* value_left, pcnt_unit_handle_t pcnt_right, int* value_right);
void PCNT_ClearCounter(pcnt_unit_handle_t pcnt_left, pcnt_unit_handle_t pcnt_right);
int Limit(int IN, int max, int min);
void Motor_PWM_Set(int motor_left, int motor_right);
void Runtime_Init();
void Runtime_Enable();
void Runtime_Disable();
double Runtime_GetDeltaTime_ms();
void Get_Velocity();
void Get_Angle();
