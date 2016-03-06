/***************************************************************************//**
 * @file    drv_led.h
 * @brief   LED driver of RT-Thread RTOS for TinySTM32
 *  COPYRIGHT (C) 2016, RT-Thread Development Team
 * @author  onelife
 * @version 0.4 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2009-01-05   Bernard     the first version
 * 2016-01-23   onelife     Convert to a driver
 ******************************************************************************/
#ifndef __DRV_LED_H__
#define __DRV_LED_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
/* LED command options */
#define LED_COMMAND_OFF            	(0x000001)
#define LED_COMMAND_ON              (0x000002)
#define LED_COMMAND_TOGGLE         	(0x000004)

/* Exported types ------------------------------------------------------------*/
struct board_led_device
{
	rt_uint8_t              number;
	GPIO_TypeDef          	*port;
    rt_uint16_t             pin;
};

struct board_led_unit_struct
{
    struct rt_device        device;
    struct board_led_device led;
};

struct board_led_unit_init
{
    rt_uint8_t              number;
    const rt_uint8_t        *name;
    struct board_led_unit_struct *unit;
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t board_hw_led_init(void);

#endif /* __DRV_LED_H__ */
