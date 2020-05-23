/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 * 2018-11-19     flybreak     add stm32f407-atk-explorer bsp
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#include <board.h>

/* defined the LED0 pin: PD3 */
#define LED0_PIN    GET_PIN(D, 3)

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);

        if (count < 30)
            rt_kprintf("count %d\n", count);
        // if (count < 60)
        //     rt_kprintf("cd %d wp %d\n", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2), HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8));

        // if (count == 30) {
        //     if (dfs_mount("sd0", "/sd", "elm", 0, 0))
        //         rt_kprintf("mount failed\n");
        //     else
        //         rt_kprintf("mount ok\n");
        // }
    }

    return RT_EOK;
}
