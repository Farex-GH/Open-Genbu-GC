#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "proc_queue.h"
#include "usb_driver.h"
#include "usb_gamepad.h"
#include "utils.h"
#include "arm_utils.h"
#include "board_io.h"
#include "ws2812.h"

void control_loop(void)
{
    proc_info *proc;

    while (1) {
        proc = proc_next();
        if (proc) {
            proc->proc_fn(proc->pfn_args);
        } else {
            __DSB;
            __WFI;
        }
    }
}

void dbg_gpio_init(void)
{
    gpio_init(PIN_DBG_0);
    gpio_init(PIN_DBG_1);
    gpio_init(PIN_DBG_2);
    gpio_set_dir(PIN_DBG_0, true);
    gpio_set_dir(PIN_DBG_1, true);
    gpio_set_dir(PIN_DBG_2, true);
}

int main(void) {
    stdio_init_all();
    usb_ep_add_callback(0x81, ep1_in_cb);
    usb_ep_add_callback(0x02, ep2_out_cb);
    usb_device_init();
    proc_init();
    board_io_init();
    dbg_gpio_init();

    /*
     * In theory we could handle the WS2812s by using our priority queue, but
     * we have an entire core sitting idle, so put it to work.
     */
    multicore_launch_core1(handle_ring_led);

    /* Spin until USB is ready */
    while(!usb_is_configured());

    control_loop();

    return 0;
}
