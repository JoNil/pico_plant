#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

const float CONVERSION_FACTOR = 3.3f / (1 << 12);

const uint LED_PIN = 25;
const uint LED_PIN_R1 = 16;
const uint ADC_PIN = 31;
const uint MOTOR_PWM_PIN = 15;

int main(void)
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(LED_PIN_R1);
    gpio_set_dir(LED_PIN_R1, GPIO_OUT);

    adc_init();
    adc_gpio_init(ADC_PIN);

    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);

    const uint motor_pwm = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    const uint motor_chn = pwm_gpio_to_channel(MOTOR_PWM_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 999);
    pwm_config_set_clkdiv(&cfg, 125);
    pwm_init(motor_pwm, &cfg, false);

    pwm_set_chan_level(motor_pwm, motor_chn, 500);
    pwm_set_enabled(motor_pwm, true);

    for (;;)
    {
        adc_select_input(0);
        const float adc0 = CONVERSION_FACTOR * (float)adc_read();

        gpio_put(LED_PIN, 1);
        gpio_put(LED_PIN_R1, adc0 > 0.95f ? 1 : 0);
        sleep_ms(500);

        gpio_put(LED_PIN, 0);
        gpio_put(LED_PIN_R1, 0);
        sleep_ms(500);

        printf("adc0: %.7f\n", adc0);
    }

    return 0;
}
