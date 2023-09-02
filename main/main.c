#include "main.h"

// gptimer_handle_t gptimer;
// gptimer_config_t timer_config;

// RTOS Handles/Queues
TaskHandle_t Task_motor_handle;
TaskHandle_t Task_uart_1_recv_handle;
QueueHandle_t Task_uart_1_recv_queue = NULL;

float delta_time;
// PWM

extern Location_t Location;

// config a gpio for output
gpio_config_t motor_en_config =
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_1),
        .pull_down_en = 0,
        .pull_up_en = 1,
};
pcnt_unit_handle_t pcnt_unit_right;
pcnt_unit_handle_t pcnt_unit_left;
pcnt_channel_handle_t pcnt_chan_left;
pcnt_channel_handle_t pcnt_chan_right;
pcnt_unit_config_t unit_config = {
    .high_limit = 10000,
    .low_limit = -10000,
};
pcnt_chan_config_t chan_config_left = {
    .edge_gpio_num = 5,
    .level_gpio_num = 4,
};
pcnt_chan_config_t chan_config_right = {
    .edge_gpio_num = 7,
    .level_gpio_num = 6,
};
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
     .gpio_num = 37,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER_0,
     .flags.output_invert = 0},
    {.channel = LEDC_CHANNEL_3, // Right_Neg
     .duty = 0,
     .gpio_num = 38,
     .speed_mode = LEDC_LOW_SPEED_MODE,
     .hpoint = 0,
     .timer_sel = LEDC_TIMER_0,
     .flags.output_invert = 0}};
ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
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

// PID
PidInputData_t Pid_Angle;
PidInputData_t Pid_Velocity;

// main tasks
void Task_motor_control(void *arg)
{
    Location.Encoder_Left_Sum = 0;
    Location.Encoder_Right_Sum = 0;
    Location.Velocity = 0;
    Location.Angle = 0;
    Location.X = 0;
    Location.Y = 0;
    Location.Encoder_Left = 0;
    Location.Encoder_Right = 0;
    int PWM_Left = 0;
    int PWM_Right = 0;
    Pid_Angle.deltaTimeSampling = 0.5;
    Pid_Angle.integralFilter = 0;
    Pid_Angle.derivativeFilter = 0;
    Pid_Angle.feedback = 0;
    Pid_Angle.outputPID = 0;

    Pid_Angle.coefficient_t.proportional = 8000; // P
    Pid_Angle.coefficient_t.integral = 1200;      // I
    Pid_Angle.coefficient_t.derivative = 70000;   // D
    Pid_Angle.saturation_t.lowThershold = -3000;
    Pid_Angle.saturation_t.highThershold = 3000;

    Pid_Velocity.deltaTimeSampling = 0.5;
    Pid_Velocity.integralFilter = 0;
    Pid_Velocity.derivativeFilter = 0;
    Pid_Velocity.feedback = 0;
    Pid_Velocity.outputPID = 0;
    Pid_Velocity.coefficient_t.proportional = 100; // P
    Pid_Velocity.coefficient_t.integral = 10;       // I
    Pid_Velocity.coefficient_t.derivative = 1700;   // D
    Pid_Velocity.saturation_t.lowThershold = -3000;
    Pid_Velocity.saturation_t.highThershold = 3000;

    // Debug

    Pid_Angle.reference = 0;
    Pid_Velocity.reference = 0; // LOWEST:20  HIGHEST:200
    int i = 0;
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    while (1)
    {
        PCNT_GetCounter(pcnt_unit_left, &Location.Encoder_Left, pcnt_unit_right, &Location.Encoder_Right);
        PCNT_ClearCounter(pcnt_unit_left, pcnt_unit_right);
        Location.Encoder_Left_Sum += Location.Encoder_Left;
        Location.Encoder_Right_Sum += Location.Encoder_Right;
        Get_Velocity();
        Get_Angle();
        // PID Computing
        Pid_Velocity.feedback = Location.Velocity;
        Pid_Angle.feedback = 2*PI-Location.Angle;
        PWM_Left = PidCompute(Pid_Velocity) + PidComputeAngle(Pid_Angle);
        PWM_Right = PidCompute(Pid_Velocity) - PidComputeAngle(Pid_Angle); // Normal

        // PWM_Left = -PidComputeAngle(Pid_Angle);
        // PWM_Right = +PidComputeAngle(Pid_Angle);//Angle Loop Only

        // PWM_Left = PidCompute(Pid_Velocity);
        // PWM_Right = PidCompute(Pid_Velocity);//Velocity loop Only

        Motor_PWM_Set(PWM_Left, PWM_Right);

        if (i > 20)
        {
            printf("Sum:PCNT_Left: %lld  PCNT_Right: %lld\r\n", Location.Encoder_Left_Sum, Location.Encoder_Right_Sum);
            // printf("PCNT_Left: %d  PCN_Right: %d \r\n", Location.Encoder_Left, Location.Encoder_Right);
            printf("Elapsedtime=%.3f,Velocity=%.6f,Angle=%.6f\r\n", delta_time, Location.Velocity, Location.Angle/(float)(2*PI)*360.0);
            printf("Pid:V= %.3f A= %.3f", PidCompute(Pid_Velocity), PidComputeAngle(Pid_Angle));
            printf("PWM_Left: %d  PWM_Right: %d\r\n", PWM_Left, PWM_Right);
            printf("Location:X= %.3f  Y= %.3f\r\n",Location.X,Location.Y);
            i = 0;
        }
        else
        {
            i++;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
uint8_t Data_Recv[10] = {0};
void Task_uart_recv(void *arg)
{
    uart_event_t event;

    unsigned int sum = 0;
    unsigned int sum_Recv = 0;
    for (;;)
    {

        // Waiting for UART event.
        if (xQueueReceiveFromISR(Task_uart_1_recv_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            // printf("I Entered!!!\r\n");
            switch (event.type)
            {
            // Event of UART receiving data
            case UART_DATA:
                uart_read_bytes(UART_NUM_1, Data_Recv, event.size, portMAX_DELAY);
                uart_write_bytes(UART_NUM_1, (const char *)("received!"), sizeof("received!"));
                sum = Data_Recv[0] ^ Data_Recv[1] ^ Data_Recv[2] ^ Data_Recv[3] ^ Data_Recv[4] ^ Data_Recv[5];
                sum_Recv = Data_Recv[6];
                printf("sum=%x,sum_Recv=%x\r\n", sum, sum_Recv);
                // if (sum_Recv == sum)
                {
                    // memcpy(&Pid_Angle.reference, &Data_Recv[0], 2);
                    // memcpy(&Pid_Velocity.reference, &Data_Recv[2], 2);
                    
                    Pid_Velocity.reference = (Data_Recv[1] << 8) + Data_Recv[0];
                    Pid_Angle.reference = (Data_Recv[3] << 8) + Data_Recv[2];
                    Pid_Angle.reference =  Pid_Angle.reference/360.0*2*PI;//Debug
                    printf("RecvSuccess.%x %x %x %x %x\r\n", Data_Recv[0], Data_Recv[1], Data_Recv[2], Data_Recv[3], Data_Recv[4]);
                    printf("Aim: Angle: %.0f  Velocity: %.0f\r\n", Pid_Angle.reference, Pid_Velocity.reference);
                    // processing
                }
                // else
                // {
                //     printf("Checksum Error!\r\n");
                // }
                // sum = 0;
                sum_Recv = 0;
                uart_flush_input(UART_NUM_1);
                memset(Data_Recv, 0, 6);

                break;
            default:
                // 不处理其他类型的事件
                uart_flush_input(UART_NUM_2);
                memset(Data_Recv, 0, 6);
                break;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // 清空接收缓冲区
    }
}

void app_main(void)
{
    // INIT SEQUENCE
    //  No fking init by f(x)
    //   Runtime_Init();

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // PWM Init
    ledc_timer_config(&ledc_timer);

    ledc_channel_config(&ledc_channel[0]);
    ledc_channel_config(&ledc_channel[1]);
    ledc_channel_config(&ledc_channel[2]);
    ledc_channel_config(&ledc_channel[3]);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_config(&motor_en_config); // motor en pin
    // motor en
    gpio_set_level(GPIO_NUM_1, 1);

    // PCNT Init
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

    // UART Init
    Task_uart_1_recv_queue = xQueueCreate(10, sizeof(uint32_t));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, buffer_size_unit * 4, buffer_size_unit * 4, 10, &Task_uart_1_recv_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config_1));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_15, GPIO_NUM_16, -1, -1)); // TX=15  RX=16

    // Task Allocation
    xTaskCreatePinnedToCore(Task_motor_control, "Task_motor_control", buffer_size_unit * 8, NULL, 7, &Task_motor_handle, 0);
    xTaskCreatePinnedToCore(Task_uart_recv, "Task_uart_recv", buffer_size_unit * 8, NULL, 5, &Task_uart_1_recv_handle, 1);
    vTaskDelay(5 / portTICK_PERIOD_MS);
}
