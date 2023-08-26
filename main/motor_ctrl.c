#include "motor_ctrl.h"
#include "main.h"
void PCNT_Init(pcnt_unit_config_t unit_config,
                pcnt_unit_handle_t pcnt_unit_left,
                pcnt_unit_handle_t pcnt_unit_right,
                pcnt_chan_config_t chan_config_left,
                pcnt_chan_config_t chan_config_right,
                pcnt_channel_handle_t pcnt_chan_left,
                pcnt_channel_handle_t pcnt_chan_right)
{
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_config_left, &pcnt_chan_left));
    // decrease the counter on rising edge, increase the counter on falling edge
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    // keep the counting mode when the control signal is high level, and reverse the counting mode when the control signal is low level
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    pcnt_unit_enable(pcnt_unit_left);
    pcnt_unit_start(pcnt_unit_left);

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_config_right, &pcnt_chan_right));
    // decrease the counter on rising edge, increase the counter on falling edge
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    // keep the counting mode when the control signal is high level, and reverse the counting mode when the control signal is low level
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    pcnt_unit_enable(pcnt_unit_right);
    pcnt_unit_start(pcnt_unit_right);

}
void PCNT_GetCounter(pcnt_unit_handle_t pcnt_left,int value_left,pcnt_unit_handle_t pcnt_right,int value_right)
{
    pcnt_unit_get_count(pcnt_left,&value_left);
    pcnt_unit_get_count(pcnt_right,&value_right);
}
void PCNT_ClearCounter(pcnt_unit_handle_t pcnt_left,pcnt_unit_handle_t pcnt_right)

{
    pcnt_unit_clear_count(pcnt_left);
    pcnt_unit_clear_count(pcnt_right);
}
int Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
void Motor_PWM_Set(int motor_left,int motor_right)
{
    motor_left =Limit(motor_left,motor_ctrl_pwm_max_limit,motor_ctrl_pwm_min_limit);
    motor_right =Limit(motor_right,motor_ctrl_pwm_max_limit,motor_ctrl_pwm_min_limit);
	if (motor_left >0){
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,(uint32_t)((double)motor_left));
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,0);
	}else{
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,(uint32_t)(-(double)motor_left));
	}

	if (motor_right >0){
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,(uint32_t)((double)motor_right));
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3,0);
	}else{
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,0);
		ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3,(uint32_t)(-(double)motor_right));
	}
	ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
	ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1);
	ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2);
	ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_3);
}
void Runtime_Init()
{
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
}
void Runtime_Disable()
{
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
}
int Runtime_GetDeltaTime()
{
    int deltatime=0;
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
    ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &deltatime));
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    return deltatime;
}