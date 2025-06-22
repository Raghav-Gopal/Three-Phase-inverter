extern "C" {
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_periph.h"
// #include <inttypes.h>
}

#define Sin_LUT_size 4096
#define PI 3.14159625
#define DUTY_QUEUE_LENGTH 64
#define UPDATE_FREQ 60000 //PWM Frequency -- Update freq should always be greater than desired_freq
#define RESOLUTION 24000000
#define FreqPot_GPIO GPIO_NUM_32 //Change ADC channel in adc_oneshot
// #define deadTime 1 //microseconds
#define MODULATION_INDEX 0.8f
#define MCPWM_GPIO_H1 15 //Change comparator gpio output in mcpwm
#define MCPWM_GPIO_L1 19
#define MCPWM_GPIO_H2 2
#define MCPWM_GPIO_L2 18
#define MCPWM_GPIO_H3 5
#define MCPWM_GPIO_L3 4
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
QueueHandle_t duty_queue;
TaskHandle_t duty_filler_handle = NULL;
// QueueHandle_t debug_queue;
const uint32_t period_ticks = RESOLUTION / UPDATE_FREQ;
uint32_t dt;
float deadTime_us = 0.3f;


uint16_t SLUT[Sin_LUT_size];
volatile float phase =0.0f;
volatile float phase_step =0.0f;
typedef struct {
    uint32_t cmp_u;
    uint32_t cmp_v;
    uint32_t cmp_w;
} cmp_ticks_triplet_t;


// Generates the sinusoidal lookup table
void generate_SinLUT() {
    printf("Generating SLUT\n");
    for (int i = 0; i < Sin_LUT_size; i++) {
        float angle = ((float)i / Sin_LUT_size) * 2.0f * PI;
        float sine_val = ((sinf(angle)*MODULATION_INDEX) + 1.0f) * 32767.5;  // scale to [0, 255]
        SLUT[i] = (uint16_t)(sine_val + 0.5f); // rounding
    }
}

//Reads the lookup table at a decided speed and writes the final comparator values to a queue.
//This allows the ISR to do minimal work and also spread out CPU load.
void duty_filler(void *pvParameters) {
    uint32_t notify_value;
    cmp_ticks_triplet_t cmp_val;

    while (1) {
        notify_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (uint32_t i = 0; i < notify_value; ++i) {
            if((i%16) ==0) {
                vTaskDelay(1);
            }
            float phase_u = phase;
            int index_u = (int)phase_u;
            int next_index_u = (index_u + 1) % Sin_LUT_size;
            float frac_u = phase_u - (float)index_u;
            // printf("phase_u: %f, index_u: %d, next_index_u: %d\n", phase_u, index_u, next_index_u);

            float val_u = (1 - frac_u) * SLUT[index_u] + frac_u * SLUT[next_index_u];

            float phase_v = phase + Sin_LUT_size *(1.0f/3.0f); // Divide by 3.0f when going to 3 phase
            if (phase_v >= Sin_LUT_size) phase_v -= Sin_LUT_size;
            int index_v = (int)phase_v;
            int next_index_v = (index_v + 1) % Sin_LUT_size;
            float frac_v = phase_v - (float)index_v;
            float val_v = (1 - frac_v) * SLUT[index_v] + frac_v * SLUT[next_index_v];

            float phase_w = phase + Sin_LUT_size *(2.0f/3.0f); // Divide by 3.0f when going to 3 phase
            if (phase_w >= Sin_LUT_size) phase_w -= Sin_LUT_size;
            int index_w = (int)phase_w;
            int next_index_w = (index_w + 1) % Sin_LUT_size;
            float frac_w = phase_w - (float)index_w;
            float val_w = (1 - frac_w) * SLUT[index_w] + frac_w * SLUT[next_index_w];

            cmp_val.cmp_u = (uint32_t)((val_u * (float)(period_ticks) / 65535.0f) + 0.5f);
            cmp_val.cmp_v = (uint32_t)((val_v * (float)(period_ticks) / 65535.0f) + 0.5f);
            cmp_val.cmp_w = (uint32_t)((val_w * (float)(period_ticks) / 65535.0f) + 0.5f);
            cmp_val.cmp_u = MIN(MAX(cmp_val.cmp_u, dt), period_ticks - dt);
            cmp_val.cmp_u = MIN(MAX(cmp_val.cmp_u, dt), period_ticks - dt);
            cmp_val.cmp_u = MIN(MAX(cmp_val.cmp_u, dt), period_ticks - dt);




            if (xQueueSend(duty_queue, &cmp_val, 0) != pdTRUE) {
                break;
            }

            phase = phase + phase_step;
            if (phase >= Sin_LUT_size) {
                phase = phase - Sin_LUT_size;
            }
        }
    }
}

//Uses adc_onshot driver to read the value of potentiometer
void input_check(void *pvParameters) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &chan_cfg);
    
    int adc_op;
    float desired_freq;
    
    while(1) {
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_op);
        desired_freq = 10.0f + ((float)adc_op/4095.0f)*(500.0f);
        phase_step = (Sin_LUT_size*desired_freq)/(float)UPDATE_FREQ;
        // printf("ADC: %d, desired_freq: %f, phase_step: %f\n", adc_op, desired_freq, phase_step);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Comparator handles defined globally for use by pwm_ISR
mcpwm_cmpr_handle_t comparator0 = NULL;
mcpwm_cmpr_handle_t comparator1 = NULL;
mcpwm_cmpr_handle_t comparator2 = NULL;

// This task pops the comparator value from the queue and updates the hardware comparators
// It also notifies duty_filler task when the number of items in the queue are less than 1/3rd of its designed value.
static bool pwm_ISR(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx){
    cmp_ticks_triplet_t cmp;
    
    if(xQueueReceiveFromISR(duty_queue, &cmp, NULL) == pdTRUE) {
        // xQueueSendFromISR(debug_queue, &cmp, NULL);



        uint32_t cmp_ticks_u = cmp.cmp_u;
        uint32_t cmp_ticks_v = cmp.cmp_v;
        uint32_t cmp_ticks_w = cmp.cmp_w;

        
        mcpwm_comparator_set_compare_value(comparator0, cmp_ticks_u);
        mcpwm_comparator_set_compare_value(comparator1, cmp_ticks_v);
        mcpwm_comparator_set_compare_value(comparator2, cmp_ticks_w);
    }

    UBaseType_t queue_status = uxQueueMessagesWaitingFromISR(duty_queue);

    if(queue_status < DUTY_QUEUE_LENGTH /3) {
        if(duty_filler_handle != NULL) {
        vTaskNotifyGiveFromISR(duty_filler_handle, NULL);
        }
    }

    return pdTRUE;
}

// Motor control pulse width modulation hardware peripheral intialization
void mcpwm_init_pwm() {
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id =0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = RESOLUTION,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = period_ticks
    }; // If you change resolution here, update deadtime below. Change the ticks value.
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t mcpwm_operator0 = NULL;
    mcpwm_operator_config_t op_config ={
        .group_id =0
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &mcpwm_operator0));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_operator0, timer));

    mcpwm_oper_handle_t mcpwm_operator1 = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &mcpwm_operator1));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_operator1, timer));

    mcpwm_oper_handle_t mcpwm_operator2 = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&op_config, &mcpwm_operator2));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_operator2, timer));

    mcpwm_gen_handle_t gen_H1 = NULL;
    mcpwm_gen_handle_t gen_L1 = NULL;
    mcpwm_gen_handle_t gen_H2 = NULL;
    mcpwm_gen_handle_t gen_L2 = NULL;
    mcpwm_gen_handle_t gen_H3 = NULL;
    mcpwm_gen_handle_t gen_L3 = NULL;
    mcpwm_generator_config_t gen_H1_config = {
        .gen_gpio_num = MCPWM_GPIO_H1
    };
    mcpwm_generator_config_t gen_L1_config = {
        .gen_gpio_num = MCPWM_GPIO_L1
    };
    mcpwm_generator_config_t gen_H2_config = {
        .gen_gpio_num = MCPWM_GPIO_H2
    };
    mcpwm_generator_config_t gen_L2_config = {
        .gen_gpio_num = MCPWM_GPIO_L2
    };
    mcpwm_generator_config_t gen_H3_config = {
        .gen_gpio_num = MCPWM_GPIO_H3
    };
    mcpwm_generator_config_t gen_L3_config = {
        .gen_gpio_num = MCPWM_GPIO_L3
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator0, &gen_H1_config, &gen_H1));
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator0, &gen_L1_config, &gen_L1));
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator1, &gen_H2_config, &gen_H2));
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator1, &gen_L2_config, &gen_L2));
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator2, &gen_H3_config, &gen_H3));
    ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operator2, &gen_L3_config, &gen_L3));

    mcpwm_dead_time_config_t dt_config_H = {
        .posedge_delay_ticks = dt,
        .negedge_delay_ticks = 0
    };
    dt_config_H.flags.invert_output = false;

    mcpwm_dead_time_config_t dt_config_L = {
        .posedge_delay_ticks = 0,
        .negedge_delay_ticks = dt
    };
    dt_config_L.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_H1, gen_H1, &dt_config_H));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_H1, gen_L1, &dt_config_L));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_H2, gen_H2, &dt_config_H));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_H2, gen_L2, &dt_config_L));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_H3, gen_H3, &dt_config_H));
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_H3, gen_L3, &dt_config_L));

    mcpwm_comparator_config_t cmpr_config ={};
    cmpr_config.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_operator0, &cmpr_config, &comparator0));
    ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_operator1, &cmpr_config, &comparator1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_operator2, &cmpr_config, &comparator2));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_H1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_H1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,comparator0,MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_H2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_H2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,comparator1,MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_H3, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_H3, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,comparator2,MCPWM_GEN_ACTION_HIGH)));

    mcpwm_timer_event_callbacks_t cbs = {
        .on_empty = pwm_ISR
    };
    mcpwm_timer_register_event_callbacks(timer, &cbs, NULL);
    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}


// void debug_task(void *arg) {
//     cmp_ticks_doublet_t debug_duty;
//     while (1) {
//         if (xQueueReceive(debug_queue, &debug_duty, portMAX_DELAY)) {
//             printf("duty_u: %"PRIu32", duty_v: %"PRIu32"\n", debug_duty.cmp_u, debug_duty.cmp_v);
            
//         }
//         vTaskDelay(1);
//     }
// }

extern "C" void app_main(void) {
    generate_SinLUT();
    duty_queue = xQueueCreate(DUTY_QUEUE_LENGTH, sizeof(cmp_ticks_triplet_t));
    // debug_queue = xQueueCreate(10, sizeof(cmp_ticks_doublet_t));
    if (duty_queue == NULL) {
        printf("Duty queue creation failed!\n");
        return;
    }
    
    dt = (uint32_t)(RESOLUTION * (deadTime_us / 1e6f) + 0.5f);  // Rounded correctly

    mcpwm_init_pwm();

    xTaskCreatePinnedToCore(duty_filler, "duty_filler", 4096, NULL, 4, &duty_filler_handle, 1);
    xTaskCreatePinnedToCore(input_check, "input_check", 4096, NULL, 3, NULL, 1);
    // xTaskCreatePinnedToCore(debug_task,"debug_task", 2048, NULL,5, NULL,1);

    while(true) {
        vTaskDelay(portMAX_DELAY);
    }
}