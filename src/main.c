#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

int main(void)
{
    stdio_init_all();

    // on board led
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    const uint LED_PIN_R1 = 16;
    gpio_init(LED_PIN_R1);
    gpio_set_dir(LED_PIN_R1, GPIO_OUT);

    const uint LED_PIN_R2 = 17;
    gpio_init(LED_PIN_R2);
    gpio_set_dir(LED_PIN_R2, GPIO_OUT);

    const float conversion_factor = 3.3f / (1 << 12);

    adc_init();

    adc_gpio_init(31);
    adc_gpio_init(32);

    for (;;)
    {
        adc_select_input(0);
        const float adc0 = conversion_factor * (float)adc_read();

        adc_select_input(1);
        const float adc1 = conversion_factor * (float)adc_read();

        gpio_put(LED_PIN, 1);
        gpio_put(LED_PIN_R1, adc0 > 0.94f ? 1 : 0);
        gpio_put(LED_PIN_R2, adc1 > 0.94f ? 1 : 0);
        sleep_ms(500);

        gpio_put(LED_PIN, 0);
        gpio_put(LED_PIN_R1, 0);
        gpio_put(LED_PIN_R2, 0);
        sleep_ms(500);

        printf("adc0: %.5f, adc1: %.5f\n", adc0, adc1);
    }

    return 0;
}
