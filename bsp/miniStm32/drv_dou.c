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
 #define DOU_COMMAND_MESSAGE_LENGTH             (6 + 1)
 #define DOU_DATA_MESSAGE_INIT_LENGTH           (6 + 7)
 #define DOU_DATA_MESSAGE_CLEAR_LENGTH          (6 + sizeof(dou_color_t))
 #define DOU_DATA_MESSAGE_READ_POINT_LENGTH     (0)
 #define DOU_DATA_MESSAGE_DRAW_POINT_LENGTH     (6 + 8 + sizeof(dou_color_t))
 #define DOU_DATA_MESSAGE_DRAW_CHLINE_LENGTH    (6 + 12)
 #define DOU_DATA_MESSAGE_DRAW_HLINE_LENGTH     (6 + 12 + sizeof(dou_color_t))
 #define DOU_DATA_MESSAGE_DRAW_VLINE_LENGTH     (6 + 12 + sizeof(dou_color_t))

static rt_err_t dou_sendCmd(rt_uint8_t id, int p1, int p2, int p3, dou_color_t *c)
{
    rt_err_t ret = RT_EOK;
    rt_uint8_t cmd[DOU_COMMAND_MESSAGE_LENGTH] = {
        DOU_COMMAND_START & 0x00FF,
        (DOU_COMMAND_START >> 8) & 0x00FF,
        DOU_HEADER_COMMAND,
        0x00,
        DOU_COMMAND_END & 0x00FF,
        (DOU_COMMAND_END >> 8) & 0x00FF,
        '\n'
        };
    rt_uint8_t *data;
    rt_uint32_t i = 0, len = 0;

    cmd[3] = id;

    do
    {
        /* Allocate buffer for data message */
        switch (id)
        {
            case DOU_COMMAND_INIT:
                len = DOU_DATA_MESSAGE_INIT_LENGTH;
                break;

            case DOU_COMMAND_CLEAR:
                len = DOU_DATA_MESSAGE_CLEAR_LENGTH;
                break;

            case DOU_COMMAND_READ_POINT:
                len = 0;
                break;

            case DOU_COMMAND_DRAW_POINT:
                len = DOU_DATA_MESSAGE_DRAW_POINT_LENGTH;
                break;

            case DOU_COMMAND_DRAW_CHLINE:
                len = DOU_DATA_MESSAGE_DRAW_CHLINE_LENGTH + (p2 - p1 + 1) * sizeof(dou_color_t);
                break;

            case DOU_COMMAND_DRAW_HLINE:
                len = DOU_DATA_MESSAGE_DRAW_HLINE_LENGTH;
                break;

            case DOU_COMMAND_DRAW_VLINE:
                len = DOU_DATA_MESSAGE_DRAW_VLINE_LENGTH;
                break;
        }

        data = rt_malloc(len);
        if (data == RT_NULL)
        {
            dou_debug("DOU err: no mem\n");
            ret = RT_ENOMEM;
            break;
        }
        data[i++] = DOU_COMMAND_START & 0x00FF;
        data[i++] = (DOU_COMMAND_START >> 8) & 0x00FF;
        data[i++] = DOU_HEADER_DATA;
        data[len - 3] = DOU_COMMAND_END & 0x00FF;
        data[len - 2] = (DOU_COMMAND_END >> 8) & 0x00FF;
        data[len - 1] = '\n';

        /* Prepare data message */
        switch (id)
        {
            case DOU_COMMAND_INIT:
                data[i++] = DOU_VERSION;
                data[i++] = DOU_SUBVERSION;
                data[i++] = (DOU_SCREEN_WIDTH >> 8) & 0xff;
                data[i++] = DOU_SCREEN_WIDTH & 0xff;
                data[i++] = (DOU_SCREEN_HEIGHT >> 8) & 0xff;
                data[i++] = DOU_SCREEN_HEIGHT & 0xff;
                data[i++] = DOU_COLOR_DEPTH;
                break;

            case DOU_COMMAND_CLEAR:
                rt_memcpy(&data[i], c, sizeof(dou_color_t));
                break;

            case DOU_COMMAND_READ_POINT:
                break;

            case DOU_COMMAND_DRAW_POINT:
                rt_memcpy(&data[i], &p1, 4);                        // x
                rt_memcpy(&data[i + 4], &p2, 4);                    // y
                rt_memcpy(&data[i + 8], c, sizeof(dou_color_t));    // c
                break;

            case DOU_COMMAND_DRAW_CHLINE:
                rt_memcpy(&data[i], &p1, 4);                        // x1
                rt_memcpy(&data[i + 4], &p2, 4);                    // x2
                rt_memcpy(&data[i + 8], &p3, 4);                    // y
                rt_memcpy(&data[i + 12], c, (p2 - p1 + 1) * sizeof(dou_color_t));
                break;

            case DOU_COMMAND_DRAW_HLINE:
                rt_memcpy(&data[i], &p1, 4);                        // x1
                rt_memcpy(&data[i + 4], &p2, 4);                    // x2
                rt_memcpy(&data[i + 8], &p3, 4);                    // y
                rt_memcpy(&data[i + 12], c, sizeof(dou_color_t));
                break;

            case DOU_COMMAND_DRAW_VLINE:
                rt_memcpy(&data[i], &p1, 4);                        // x
                rt_memcpy(&data[i + 4], &p2, 4);                    // y1
                rt_memcpy(&data[i + 8], &p3, 4);                    // y2
                rt_memcpy(&data[i + 12], c, sizeof(dou_color_t));
                break;
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
            break;
        }

        /* Send command message */
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, cmd,
            DOU_COMMAND_MESSAGE_LENGTH) != DOU_COMMAND_MESSAGE_LENGTH)
        {
            dou_debug("DOU err: send cmd msg\n");
            ret = RT_ERROR;
            break;
        }
        /* Send data message */
        if (dou_serial->write(dou_serial, MINISTM32_NO_DATA, data, len) != \
            len)
        {
            dou_debug("DOU err: send data msg\n");
            ret = RT_ERROR;
            break;
        }
    } while (0);

    rt_sem_release(&dou_lock);
    rt_free(data);
    return ret;
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
    dou_sendCmd(DOU_COMMAND_DRAW_POINT, x, y, 0, (dou_color_t *)c);
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
    dou_debug("[[[dou_getPixel]]]\n");
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
    dou_sendCmd(DOU_COMMAND_DRAW_CHLINE, x1, x2, y, (dou_color_t *)pixels);
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
    dou_sendCmd(DOU_COMMAND_DRAW_HLINE, x1, x2, y, (dou_color_t *)c);
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
    dou_sendCmd(DOU_COMMAND_DRAW_VLINE, x, y1, y2, (dou_color_t *)c);
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
static rt_err_t miniStm32_dou_init(rt_device_t dev)
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
    dou_color_t color = DOU_COLOR_BLUE;

    ret = dou_sendCmd(DOU_COMMAND_INIT, 0, 0, 0, RT_NULL);
    if (ret != RT_EOK)
    {
        return ret;
    }
   return dou_sendCmd(DOU_COMMAND_CLEAR, 0, 0, 0, &color);
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
/*    if (dou_serial->close(dou_serial) != RT_EOK)
    {
        return RT_ERROR;
    }
*/
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
        dou_info.pixel_format       = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
        dou_info.bits_per_pixel     = DOU_COLOR_DEPTH;
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
