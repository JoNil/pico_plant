#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

int main(void)
{
    stdio_init_all();

    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    adc_init();

    adc_set_temp_sensor_enabled(true);

    //adc_gpio_init(26);

    adc_select_input(4);

    for (;;)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);

        float conversion_factor = 3.3f / (1 << 12);
        float reading = conversion_factor * (float)adc_read();

        float temperature = 27.0f - (reading - 0.706f) / 0.001721f;

        printf("Temp is %.1f\n", temperature);
    }

    return 0;
}
