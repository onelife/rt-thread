/***************************************************************************//**
 * @file 	dev_lcd.c
 * @brief 	LCD driver of RT-Thread RTOS for EFM32
 * 	COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author 	onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date			Author		Notes
 * 2012-01-24	onelife		Initial creation for EFM32 (Modified from
 *  "ssd1289.c")
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "drv_dou.h"

#if defined(MINISTM32_USING_DOU)
#include <rtgui/rtgui.h>
#include <rtgui/driver.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_DOU_DEBUG
#define dou_debug(format,args...)       rt_kprintf(format, ##args)
#else
#define dou_debug(format,args...)
#endif

/* Private function prototypes -----------------------------------------------*/
static void dou_setPixel(rtgui_color_t *c, int x, int y);
static void dou_getPixel(rtgui_color_t *c, int x, int y);
static void dou_drawHLine(rtgui_color_t *c, int x1, int x2, int y);
static void dou_drawVLine(rtgui_color_t *c, int x , int y1, int y2);
static void dou_drawRawHLine(rt_uint8_t *pixels, int x1, int x2, int y);

/* Private variables ---------------------------------------------------------*/
static rt_device_t dou_serial = RT_NULL;
static struct rt_device dou_device;
struct rt_semaphore dou_lock;
static struct rt_device_graphic_info dou_info;
static const struct rtgui_graphic_driver_ops dou_ops =
    {
        dou_setPixel,
        dou_getPixel,
        dou_drawHLine,
        dou_drawVLine,
        dou_drawRawHLine
    };

/* Private functions ---------------------------------------------------------*/
static void dou_clear(rt_uint32_t color)
{
    rt_err_t ret;
    rt_uint8_t cmd[] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_COMMAND,
        DOU_COMMAND_CLEAR
        };
    rt_uint8_t data[6] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_DATA
        };

    rt_memcpy(&data[2], &color, 4);

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    do
    {
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            sizeof(cmd)) != sizeof(cmd))
        {
            dou_debug("DOU err: clear\n");
            break;
        }
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data,
            sizeof(data)) != sizeof(cmd))
        {
            dou_debug("DOU: clear\n");
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);
}

/***************************************************************************//**
 * @brief
 *   Draw a pixel with specified color
 *
 * @details
 *
 * @note
 *
 * @param[in] c
 *  Pointer to color
 *
 * @param[in] x
 *  Horizontal position
 *
 * @param[in] y
 *  Vertical position
 ******************************************************************************/
static void dou_setPixel(rtgui_color_t *c, int x, int y)
{
    rt_err_t ret;
    rt_uint8_t cmd[] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_COMMAND,
        DOU_COMMAND_DRAW_POINT,
        '\n'
        };
    rt_uint8_t data[15] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_DATA
        };

    rt_memcpy(&data[2], &x, 4);
    rt_memcpy(&data[6], &y, 4);
    rt_memcpy(&data[10], c, 4);
    data[sizeof(data) - 1] = '\n';

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    do
    {
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            sizeof(cmd)) != sizeof(cmd))
        {
            dou_debug("DOU err: setPixel\n");
            break;
        }
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data,
            sizeof(data)) != sizeof(data))
        {
            dou_debug("DOU err: setPixel\n");
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);
    dou_debug("DOU: setPixel\n");
}

/***************************************************************************//**
 * @brief
 *   Get the color of a pixel
 *
 * @details
 *
 * @note
 *
 * @param[out] c
 *  Pointer to color
 *
 * @param[in] x
 *  Horizontal position
 *
 * @param[in] y
 *  Vertical position
 ******************************************************************************/
static void dou_getPixel(rtgui_color_t *c, int x, int y)
{
    rt_kprintf("[[[dou_getPixel]]]\n");
}

/***************************************************************************//**
 * @brief
 *   Draw a horizontal line with raw color
 *
 * @details
 *
 * @note
 *
 * @param[in] pixels
 *  Pointer to raw color
 *
 * @param[in] x1
 *  Horizontal start position
 *
 * @param[in] x2
 *  Horizontal end position
 *
 * @param[in] y
 *  Vertical position
 ******************************************************************************/
static void dou_drawRawHLine(rt_uint8_t *pixels, int x1, int x2, int y)
{
    rt_err_t ret;
    rt_uint8_t cmd[] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_COMMAND,
        DOU_COMMAND_DRAW_COLOR_HLINE,
        '\n'
        };
    rt_uint8_t *data = RT_NULL;

    data = rt_malloc(15 + (x2 - x1 + 1) * 4);
    if (data == RT_NULL)
    {
        return;
    }
    data[0] = DOU_COMMUNICATION_ID;
    data[1] = DOU_HEADER_DATA;
    rt_memcpy(&data[2], &x1, 4);
    rt_memcpy(&data[6], &x2, 4);
    rt_memcpy(&data[10], &y, 4);
    rt_memcpy(&data[14], pixels, (x2 - x1 + 1) * 4);
    data[sizeof(data) - 1] = '\n';

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    do
    {
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            sizeof(cmd)) != sizeof(cmd))
        {
            dou_debug("DOU err: drawRawHLine\n");
            break;
        }
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data,
            sizeof(data)) != sizeof(data))
        {
            dou_debug("DOU err: drawRawHLine\n");
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);
    dou_debug("DOU: drawRawHLine\n");
}

/***************************************************************************//**
 * @brief
 *   Draw a horizontal line with specified color
 *
 * @details
 *
 * @note
 *
 * @param[in] c
 *  Pointer to color
 *
 * @param[in] x1
 *  Horizontal start position
 *
 * @param[in] x2
 *  Horizontal end position
 *
 * @param[in] y
 *  Vertical position
 ******************************************************************************/
static void dou_drawHLine(rtgui_color_t *c, int x1, int x2, int y)
{
    rt_err_t ret;
    rt_uint8_t cmd[] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_COMMAND,
        DOU_COMMAND_DRAW_HLINE,
        '\n'
        };
    rt_uint8_t data[19] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_DATA
        };

    rt_memcpy(&data[2], &x1, 4);
    rt_memcpy(&data[6], &x2, 4);
    rt_memcpy(&data[10], &y, 4);
    rt_memcpy(&data[14], c, 4);
    data[sizeof(data) - 1] = '\n';

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    do
    {
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            sizeof(cmd)) != sizeof(cmd))
        {
            dou_debug("DOU err: drawHLine\n");
            break;
        }
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data,
            sizeof(data)) != sizeof(data))
        {
            dou_debug("DOU err: drawHLine\n");
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);
    dou_debug("DOU: drawHLine\n");
}

/***************************************************************************//**
 * @brief
 *   Draw a vertical line with specified color
 *
 * @details
 *
 * @note
 *
 * @param[in] c
 *  Pointer to color
 *
 * @param[in] x
 *  Horizontal position
 *
 * @param[in] y1
 *  Vertical start position
 *
 * @param[in] y2
 *  Vertical end position
 ******************************************************************************/
static void dou_drawVLine(rtgui_color_t *c, int x , int y1, int y2)
{
    rt_err_t ret;
    rt_uint8_t cmd[] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_COMMAND,
        DOU_COMMAND_DRAW_VLINE,
        '\n'
        };
    rt_uint8_t data[19] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_DATA
        };

    rt_memcpy(&data[2], &x, 4);
    rt_memcpy(&data[6], &y1, 4);
    rt_memcpy(&data[10], &y2, 4);
    rt_memcpy(&data[14], c, 4);
    data[sizeof(data) - 1] = '\n';

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    do
    {
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            sizeof(cmd)) != sizeof(cmd))
        {
            dou_debug("DOU err: drawVLine\n");
            break;
        }
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data,
            sizeof(data)) != sizeof(data))
        {
            dou_debug("DOU err: drawVLine\n");
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);
    dou_debug("DOU: drawVLine\n");
}

/***************************************************************************//**
 * @brief
 *   Open LCD device
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
static rt_err_t miniStm32_dou_init(rt_device_t dev, rt_uint16_t oflag)
{
    /* Find serial device */
    dou_serial = rt_device_find(DOU_USING_DEVICE_NAME);
    if (dou_serial == RT_NULL)
    {
        dou_debug("DOU err: Can't find device %s!\n", DOU_USING_DEVICE_NAME);
        return RT_ERROR;
    }

    dou_debug("DOU: Init OK\n", DOU_USING_DEVICE_NAME);
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Open LCD device
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
static rt_err_t miniStm32_dou_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_err_t ret;
    rt_uint8_t cmd[] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_COMMAND,
        DOU_COMMAND_INIT,
        '\n'
        };
    rt_uint8_t data[10] = {
        DOU_COMMUNICATION_ID,
        DOU_HEADER_DATA,
        };

    data[2] = DOU_VERSION;
    data[3] = DOU_SUBVERSION;
    data[4] = (DOU_SCREEN_WIDTH >> 8) & 0xff;
    data[5] = DOU_SCREEN_WIDTH & 0xff;
    data[6] = (DOU_SCREEN_HEIGHT >> 8) & 0xff;
    data[7] = DOU_SCREEN_HEIGHT & 0xff;
    data[8] = DOU_COLOR_DEPTH;
    data[sizeof(data) - 1] = '\n';

    /* Open serial device */
    if (dou_serial->open(dou_serial, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        return RT_ERROR;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&dou_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    do
    {
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            sizeof(cmd)) != sizeof(cmd))
        {
            dou_debug("DOU err: init\n");
            break;
        }
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data,
            sizeof(data)) != sizeof(data))
        {
            dou_debug("DOU err: init\n");
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);

    dou_debug("DOU: open\n");
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Close LCD device
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
static rt_err_t miniStm32_dou_close(rt_device_t dev)
{
    /* Close serial device */
    if (dou_serial->close(dou_serial) != RT_EOK)
    {
        return RT_ERROR;
    }

    dou_debug("DOU: close\n");
    return RT_EOK;
}

/***************************************************************************//**
* @brief
*   Configure LCD device
*
* @details
*
* @note
*
* @param[in] dev
*   Pointer to device descriptor
*
* @param[in] cmd
*   IIC control command
*
* @param[in] args
*   Arguments
*
* @return
*   Error code
******************************************************************************/
static rt_err_t miniStm32_dou_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch (cmd)
	{
	case RTGRAPHIC_CTRL_RECT_UPDATE:
		break;
	case RTGRAPHIC_CTRL_POWERON:
		break;
	case RTGRAPHIC_CTRL_POWEROFF:
		break;
	case RTGRAPHIC_CTRL_GET_INFO:
		rt_memcpy(args, &dou_info, sizeof(dou_info));
		break;
	case RTGRAPHIC_CTRL_SET_MODE:
		break;
	}

	return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *	Register LCD device
 *
 * @details
 *
 * @note
 *
 * @param[in] device
 *	Pointer to device descriptor
 *
 * @param[in] name
 *	Device name
 *
 * @param[in] flag
 *	Configuration flags
 *
 * @param[in] iic
 *	Pointer to IIC device descriptor
 *
 * @return
 *	Error code
 ******************************************************************************/
static rt_err_t miniStm32_dou_register(
	rt_device_t	        device,
	const char          *name,
	rt_uint32_t         flag,
	void                *data)
{
	RT_ASSERT(device != RT_NULL);

	device->type 		= RT_Device_Class_Graphic;
	device->rx_indicate = RT_NULL;
	device->tx_complete = RT_NULL;
	device->init 		= miniStm32_dou_init;
	device->open		= miniStm32_dou_open;
	device->close		= miniStm32_dou_close;
	device->read 		= RT_NULL;
	device->write 		= RT_NULL;
	device->control 	= miniStm32_dou_control;
	device->user_data	= data;

	/* register a character device */
	return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}

/***************************************************************************//**
 * @brief
 *   Initialize LCD device
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void miniStm32_hw_dou_init(void)
{
    rt_uint32_t flag;
    static rt_device_t usart_dev;

    do
    {
        /* Init LCD info */
        flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE; //RT_DEVICE_FLAG_DMA_TX
        dou_info.pixel_format       = RTGRAPHIC_PIXEL_FORMAT_RGB888;
        dou_info.bits_per_pixel     = 32;
        dou_info.width              = DOU_SCREEN_WIDTH;
        dou_info.height             = DOU_SCREEN_HEIGHT;
        dou_info.framebuffer        = RT_NULL;
        if (miniStm32_dou_register(&dou_device, DOU_DEVICE_NAME, flag, (void *)&dou_ops) != RT_EOK)
        {
            break;
        }

        /* Init LCD lock */
        if (rt_sem_init(&dou_lock, DOU_DEVICE_NAME, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }

        /* Set as rtgui graphic driver */
        if (rtgui_graphic_set_device(&dou_device) != RT_EOK)
        {
            break;
        }

        dou_debug("DOU: H/W init OK!\n");
        return;
    } while (0);

    dou_debug("DOU err: H/W init failed!\n");
}

#endif /* defined(MINISTM32_USING_DOU) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
