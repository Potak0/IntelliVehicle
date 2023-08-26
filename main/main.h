// Libs
#include <stdio.h> //Basic Lib
#include <stdlib.h>
#include <string.h>
#include <driver/gpio.h> //Peripheral Lib(GPIO,UART,PCNT,PWM,TIM)
#include <driver/uart.h>
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h" //FreeRTOS Lib
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h" //SysLog Lib
#include "esp_log.h"
#include "esp_err.h"
#include "pid.h" //PID Lib

// Param Settings
#define buffer_size_unit 1024
// PWM
#define motor_ctrl_pwm_max 4095
#define motor_ctrl_pwm_min 0
#define motor_ctrl_pwm_max_limit 3000
#define motor_ctrl_pwm_min_limit -3000






