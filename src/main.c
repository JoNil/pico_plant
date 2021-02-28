#include <stdio.h>

#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();

    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (;;)
    {
        gpio_put(LED_PIN, 1);
        printf("Set to 1\n");
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        printf("Set to 0\n");
        sleep_ms(500);
    }

    return 0;
}
