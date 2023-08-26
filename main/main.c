#include "main.h"
// PWM
ledc_channel_config_t ledc_channel[4] = {
    {.channel = LEDC_CHANNEL_0, // Left_Pos
     .duty = 0,
     .gpio_num = 36,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER_0,
     .flags.output_invert = 0},
    {.channel = LEDC_CHANNEL_1, // Left_Neg
     .duty = 0,
     .gpio_num = 35,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER_0,
     .flags.output_invert = 0},
    {.channel = LEDC_CHANNEL_2, // Right_Pos
     .duty = 0,
     .gpio_num = 38,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER_0,
     .flags.output_invert = 0},
    {.channel = LEDC_CHANNEL_3, // Right_Neg
     .duty = 0,
     .gpio_num = 37,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER_0,
     .flags.output_invert = 0}};

ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
    .freq_hz = 1000,                      // frequency of PWM signal
    .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
    .timer_num = LEDC_TIMER_0,            // timer index
    .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
};

// UART
uart_config_t uart_config_1 = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
uart_config_t uart_config_2 = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

// PCNT
pcnt_unit_handle_t pcnt_unit_right = NULL;
pcnt_unit_handle_t pcnt_unit_left = NULL;
pcnt_channel_handle_t pcnt_chan_left = NULL;
pcnt_channel_handle_t pcnt_chan_right = NULL;
pcnt_unit_config_t unit_config = {
    .high_limit = 10000,
    .low_limit = -10000,
};
pcnt_chan_config_t chan_config_left = {
    .edge_gpio_num = 4,
    .level_gpio_num = 5,
};
pcnt_chan_config_t chan_config_right = {
    .edge_gpio_num = 6,
    .level_gpio_num = 7,
};

// PID
PidInputData_t Pid_Angle;
PidInputData_t Pid_Velocity;

// motor control
void Task_motor_control(void)
{
    Pid_Angle.deltaTimeSampling = 0.5;
    Pid_Angle.integralFilter = 0;
    Pid_Angle.derivativeFilter = 0;
    Pid_Angle.feedback = 0;
    Pid_Angle.outputPID = 0;

    Pid_Angle.coefficient_t.proportional = -30000; // P
    Pid_Angle.coefficient_t.integral = -700000;    // I
    Pid_Angle.coefficient_t.derivative = -4000;    // D
    Pid_Angle.saturation_t.lowThershold = -5500;
    Pid_Angle.saturation_t.highThershold = 5500;

    Pid_Velocity.coefficient_t.proportional = 80; // P
    Pid_Velocity.coefficient_t.integral = 10;     // I
    Pid_Velocity.coefficient_t.derivative = 1700; // D
    Pid_Velocity.saturation_t.lowThershold = -7000;
    Pid_Velocity.saturation_t.highThershold = 7000;
}
void app_main(void)
{
    // xTaskCreate();
}
