#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    
    // Initialize GPIO 20 and 21 as outputs
    gpio_init(20);
    gpio_set_dir(20, GPIO_OUT);
    gpio_init(21);
    gpio_set_dir(21, GPIO_OUT);
    
    // Blink forever
    while (true) {
        gpio_put(20, 1);
        gpio_put(21, 1);
        sleep_ms(500);
        gpio_put(20, 0);
        gpio_put(21, 0);
        sleep_ms(500);
    }
    
    return 0;
}
