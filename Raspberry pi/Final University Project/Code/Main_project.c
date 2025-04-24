#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/flash.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/timer.h"

// Constants definition
#define BUTTON_ADC_SEL0_PIN 2
#define BUTTON_ADC_SEL1_PIN 3
#define BUTTON_ADC_SEL2_PIN 4
#define ADC_SEL0 26
#define ADC_SEL1 27
#define ADC_SEL2 28
#define ADC_CONV_CONST (3.3/(1<<12))

#define ADC_SAMPLING_RATE 80    //Defined in uSeconds
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - 38456 - 1)

volatile bool timer_fired = false;
volatile bool adc_is_reading = false;

typedef struct { 
    double* buffer;
    size_t length;
} ADC_Result;

static void call_flash_range_erase(void*param){
    uint32_t offset = (const uint32_t)((uintptr_t*)param)[0];
    size_t count = (const size_t)((uintptr_t*)param)[1] * sizeof(double);    //in bytes
    flash_range_erase(offset, count);
}

static void call_flash_range_program(void *param){
    uint32_t offset = ((uintptr_t*)param)[0];
    const uint8_t *data = (const uint8_t *)((uintptr_t*)param)[1];
    size_t count = (const size_t)((uintptr_t*)param)[2] * sizeof(double);   //in bytes
    flash_range_program(offset, data, count);
}

void adc_sel_butt_callback(uint gpio, uint32_t event_mask) {
    switch (gpio) {
        case BUTTON_ADC_SEL0_PIN:
            printf("\nADC 0 has been selected\n");
            adc_select_input(0);
            break;
        case BUTTON_ADC_SEL1_PIN:
            printf("\nADC 1 has been selected\n");
            adc_select_input(1);
            break;
        case BUTTON_ADC_SEL2_PIN:
            printf("\nADC 2 has been selected\n");
            adc_select_input(2);
            break;
        default:
            printf("\nUnknown GPIO event\n");
            break;
    }
}

void adc_sel_butt_init() {
    gpio_init(BUTTON_ADC_SEL0_PIN);
    gpio_init(BUTTON_ADC_SEL1_PIN);
    gpio_init(BUTTON_ADC_SEL2_PIN);

    adc_gpio_init(ADC_SEL0);
    adc_gpio_init(ADC_SEL1);
    adc_gpio_init(ADC_SEL2);

    gpio_set_dir(BUTTON_ADC_SEL0_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_ADC_SEL1_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_ADC_SEL2_PIN, GPIO_IN);

    gpio_pull_up(BUTTON_ADC_SEL0_PIN);
    gpio_pull_up(BUTTON_ADC_SEL1_PIN);
    gpio_pull_up(BUTTON_ADC_SEL2_PIN);

    gpio_set_irq_enabled_with_callback(BUTTON_ADC_SEL0_PIN, GPIO_IRQ_EDGE_FALL, true, adc_sel_butt_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_ADC_SEL1_PIN, GPIO_IRQ_EDGE_FALL, true, adc_sel_butt_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_ADC_SEL2_PIN, GPIO_IRQ_EDGE_FALL, true, adc_sel_butt_callback);
}

// Timer callback for ADC reading
int64_t alarm_callback(alarm_id_t id, __unused void *user_data) {
    printf("\nAlarm <%d> has fired\n", (int)id);
    timer_fired = true;
    return 0;
}

ADC_Result adc_read_values_in_us(float u_seconds) {
    if(adc_is_reading == 1){
        ADC_Result adc_result = {0, 0};
        return adc_result;
    }else{
        size_t nr_samples = (((u_seconds) / 80));
        //Debug info of buffer size for the ADC read values; Remember to free buffer
        printf("Size_t of the buffer: %f\n", (float)nr_samples);
        double* buff = malloc(nr_samples * sizeof(double));
        if (!buff) {
            printf("Memory allocation failed!\n");
            return (ADC_Result){NULL, 0};
        }

        int i = 0;
        ADC_Result result = {buff, 0}; 
        alarm_id_t read_adc_alarm_id = add_alarm_in_us(u_seconds, alarm_callback, NULL, false);
        absolute_time_t next_sample_time = get_absolute_time();

        printf("Values read from ADC input:\n");
        while (!timer_fired) {
            result.buffer[i] = ADC_CONV_CONST * adc_read();
            printf("%.05f ", result.buffer[i]);
            i++;

            next_sample_time = delayed_by_us(next_sample_time, ADC_SAMPLING_RATE);
            sleep_until(next_sample_time);
        }
        printf("\n");
        printf("Total ADC values read: %d\n", (i));
        timer_fired = false;
        result.length = i;
        printf("Size of ADC_Result struct: %d\n", (int)result.length);
        
        return result;
    }
}
    
void print_buf(const uint8_t *buf, size_t len){
        for(size_t i=0; i< len; i++){
            printf("%02x", buf[i]);
            if(i % 16 == 15)
            printf("\n");
            else
            printf(" ");
    }
}

size_t get_nr_samples(float u_seconds){
    //Make this more modular by using the time divider for the ADC sampling rate
    return (u_seconds) / ADC_SAMPLING_RATE;
}

size_t get_data_size_bytes(float u_seconds){ 
    return get_nr_samples(u_seconds)*8;
}

//Function that saves data from adc readings to the flash memory. 
//Only ADC_Result struct is needed because everything else will be calculated depending on the read time
//Last page of the last sector sould be used for saving data from the ADC
void flash_save_data(ADC_Result adc_result, uint32_t flash_offs){
    printf("Erasing target region...\n");
    uintptr_t params_erase[] = {flash_offs, adc_result.length};
    int rc = flash_safe_execute(call_flash_range_erase, (void*)params_erase, UINT32_MAX);
    hard_assert(rc == PICO_OK);
    printf("Done\n");

    printf("\nProgramming target region... \n");
    uintptr_t params_program[] = {flash_offs, (uintptr_t)adc_result.buffer, adc_result.length};
    rc = flash_safe_execute(call_flash_range_program, params_program, UINT32_MAX);
    hard_assert(rc==PICO_OK);
    printf("Done\n");
}

ADC_Result flash_read_to_double_array(uint32_t flash_offs, size_t count){
    const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
    double * buffer =malloc((int)count*sizeof(double));
    if(!buffer){
        printf("Failed reading from memory - memory allocation for local buffer has failed");
        return (ADC_Result){NULL, 0};
    }

    memcpy(buffer, flash_target_contents, count*sizeof(double));
    return (ADC_Result){buffer, count};
}

int main() {
    stdio_init_all();

    //For reading debug info, DELETE later
    gpio_init(0);
    gpio_set_dir(0, GPIO_IN);
    gpio_init(1);
    gpio_set_dir(1, GPIO_IN);
    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    
    adc_init();
    adc_sel_butt_init();

    const float u_seconds = 80 * 10;
    while (true) {
        //Debugging
        printf("\nGPIO IS: %d\n", (int) gpio_get(0));
        printf("Selected ADC: %d\n", adc_get_selected_input());
        if(gpio_get(0)){
            adc_read_values_in_us(80);
            printf("\n");
            sleep_ms(500);
            adc_read_values_in_us(80 * 3);
            printf("\n");
            sleep_ms(500);
            adc_read_values_in_us(80 * 10);
            printf("\n");
            sleep_ms(500);
        }
        if (gpio_get(1)) {
            ADC_Result adc_result = adc_read_values_in_us(u_seconds);
            flash_save_data(adc_result, FLASH_TARGET_OFFSET);

            printf("Values read from flash: \n");
            ADC_Result flash_result = flash_read_to_double_array(FLASH_TARGET_OFFSET, adc_result.length);
            for(size_t i=0;i< flash_result.length;i++){
                printf("%.5f ", flash_result.buffer[i]);
            }
            printf("\n");
        }
        if(gpio_get(5)){
            printf("Values read from flash: \n");
            ADC_Result flash_result = flash_read_to_double_array(FLASH_TARGET_OFFSET, (size_t)u_seconds/80);
            for(size_t i=0;i< flash_result.length;i++){
                printf("%.5f ", flash_result.buffer[i]);
            }
            printf("\n");
        }

        sleep_ms(500);
    }
}
