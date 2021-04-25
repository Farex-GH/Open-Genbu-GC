#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "board_io.h"
#include "debounce.h"
#include "utils.h"

int64_t debounce_timer_elapsed(alarm_id_t id, void *arg)
{
    io_map_container *ioc = (io_map_container *)arg;
    size_t i;

    DB_PRINT_L(0, "\n");

    for (i = 0; i < ioc->btn_size; ++i) {
        gpio_set_irq_enabled(ioc->btn_map[i].gpio,
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    for (i = 0; i < ioc->dpad_size; ++i) {
        gpio_set_irq_enabled(ioc->dpad_map[i].gpio,
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    return 0;
}
