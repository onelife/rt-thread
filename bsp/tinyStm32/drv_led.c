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

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_led.h"
#if (defined(BSP_USING_LED1) || defined(BSP_USING_LED2))
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if (defined(BSP_USING_LED1))
static struct board_led_unit_struct led1;
#endif

#if (defined(BSP_USING_LED2))
static struct board_led_unit_struct led2;
#endif

/* Private macro -------------------------------------------------------------*/
#ifdef BOARD_LED_DEBUG
#define led_debug(format,args...)           rt_kprintf(format, ##args)
#else
#define led_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *   Initialize LED device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t board_led_init (rt_device_t dev)
{
	led_debug("LED: Init OK\n");
	return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Open LED device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] oflag
 *   Device open flag
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t board_led_open(rt_device_t dev, rt_uint16_t oflag)
{
	led_debug("LED: Open OK\n");
	return RT_EOK;
}

/***************************************************************************//**
* @brief
*   Configure LED device
*
* @details
*
* @note
*
* @param[in] dev
*   Pointer to device descriptor
*
* @param[in] cmd
*   LED control command
*
* @param[in] args
*   Arguments
*
* @return
*   Error code
******************************************************************************/
static rt_err_t board_led_control(
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    RT_ASSERT(dev != RT_NULL);

    struct board_led_device *led;

    led = (struct board_led_device *)(dev->user_data);

    switch (cmd)
    {
	case LED_COMMAND_ON:
		GPIO_ResetBits(led->port, led->pin);
		break;

	case LED_COMMAND_OFF:
		GPIO_SetBits(led->port, led->pin);
		break;

	case LED_COMMAND_TOGGLE:
		{
			rt_uint8_t status = GPIO_ReadOutputDataBit(led->port, led->pin);
			if (status == 0x00)
			{
				GPIO_SetBits(led->port, led->pin);
			}
			else
			{
				GPIO_ResetBits(led->port, led->pin);
			}
		}
		break;
    default:
        break;
	}

    led_debug("LED: Control %d -> %d\n", led->number, cmd);

	return RT_EOK;
}

/***************************************************************************//**
* @brief
*   Initialize the specified LED unit
*
* @details
*
* @note
*
* @param[in] init
*   Unit init structure
*
* @return
*   Pointer to LED device
******************************************************************************/
static rt_err_t board_led_unit_init(struct board_led_unit_init *init)
{
    rt_device_t         device;
    struct board_led_device *led;
    rt_uint32_t         flag;
	GPIO_InitTypeDef    gpio_init;

    device = &(init->unit)->device;
    led = &(init->unit)->led;
    flag = RT_DEVICE_FLAG_RDWR;

    do
    {
        /* Unit number specified setting */
		gpio_init.GPIO_Speed	= GPIO_Speed_50MHz;
		gpio_init.GPIO_Mode		= GPIO_Mode_Out_OD;

        switch (init->number)
        {
        case 1:
            {
            	RCC_APB2PeriphClockCmd(LED1_CLOCK, ENABLE);
            	gpio_init.GPIO_Pin = LED1_PIN;
				GPIO_Init(LED1_PORT, &gpio_init);
				GPIO_SetBits(LED1_PORT, LED1_PIN);

				led->number = 1;
				led->port = LED1_PORT;
				led->pin = LED1_PIN;
            }
            break;
#if defined(BSP_USING_LED2)
        case 2:
            {
            	RCC_APB2PeriphClockCmd(LED2_CLOCK, ENABLE);
				gpio_init.GPIO_Pin = LED2_PIN;
				GPIO_Init(LED2_PORT, &gpio_init);
				GPIO_SetBits(LED2_PORT, LED2_PIN);

				led->number = 2;
				led->port = LED2_PORT;
				led->pin = LED2_PIN;
            }
            break;
#endif
        default:
            break;
        }

        /* Config and register device */
		device->type        = RT_Device_Class_Miscellaneous;
		device->rx_indicate = RT_NULL;
		device->tx_complete = RT_NULL;
		device->init        = board_led_init;
		device->open        = board_led_open;
		device->close       = RT_NULL;
		device->read        = RT_NULL;
		device->write       = RT_NULL;
		device->control     = board_led_control;
		device->user_data   = (void *)led;

		return rt_device_register(device, init->name, flag);
    } while(0);

    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize all LED related hardware and register LED device to kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t board_hw_led_init(void)
{
	struct board_led_unit_init init;

	do
	{
#if (defined(BSP_USING_LED1))
		const rt_uint8_t name[] = LED1_NAME;

		init.number         = 1;
		init.name           = &name[0];
		init.unit           = &led1;
		if (board_led_unit_init(&init) != RT_EOK)
		{
			break;
		}
#endif

#if (defined(BSP_USING_LED2))
		const rt_uint8_t name[] = LED2_NAME;

		init.number         = 2;
		init.name           = &name[0];
		init.unit           = &led2;
		if (board_led_unit_init(&init) != RT_EOK)
		{
			break;
		}
#endif

		led_debug("LED%d: H/W init OK!\n", init.number);
		return RT_EOK;
	} while (0);

	rt_kprintf("LED%d err: H/W init failed!\n", init.number);
	return -RT_ERROR;
}
INIT_BOARD_EXPORT(board_hw_led_init);


/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
#ifdef RT_USING_FINSH
#include <finsh.h>

void led(rt_uint32_t led, rt_int32_t value)
{
	rt_uint8_t cmd;

	switch (value)
	{
	case 0:
		cmd = LED_COMMAND_OFF;
		break;
	case 1:
		cmd = LED_COMMAND_ON;
		break;
	case -1:
		cmd = LED_COMMAND_TOGGLE;
		break;
	default:
		return;
	}

    switch (led)
    {
    case 1:
    	led1.device.control(&led1.device, cmd, RT_NULL);
        break;
#if defined(BSP_USING_LED2)
    case 2:
    	led2.device.control(&led1.device, cmd, RT_NULL);
        break;
#endif
    default:
        break;
    }
}
FINSH_FUNCTION_EXPORT(led, set led[1 - x] on[1] or off[0] or toggle[-1].)
#endif

#endif /* (defined(BSP_USING_LED1) || defined(BSP_USING_LED2)) */
/***************************************************************************//**
 * @}
 ******************************************************************************/

