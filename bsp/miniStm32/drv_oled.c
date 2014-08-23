/***************************************************************************//**
 * @file 	dev_oled.c
 * @brief 	OLED driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-06-15	onelife		Initial creation for MiniSTM32
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "drv_oled.h"

#if defined(MINISTM32_USING_OLED)
#include <rtgui/rtgui.h>
#include <rtgui/driver.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_OLED_DEBUG
#define oled_debug(format,args...)      rt_kprintf(format, ##args)
#else
#define oled_debug(format,args...)
#endif

/* Private function prototypes -----------------------------------------------*/
static void oled_setPixel(rtgui_color_t *c, int x, int y);
static void oled_getPixel(rtgui_color_t *c, int x, int y);
static void oled_drawHLine(rtgui_color_t *c, int x1, int x2, int y);
static void oled_drawVLine(rtgui_color_t *c, int x , int y1, int y2);
static void oled_drawRawHLine(rt_uint8_t *pixels, int x1, int x2, int y);

/* Private variables ---------------------------------------------------------*/
static struct rt_device oled_device;
struct rt_semaphore oled_lock;
static struct rt_device_graphic_info oled_info;
//static rt_uint8_t frame_buffer[MINISTM32_OLED_HEIGHT/8][MINISTM32_OLED_WIDTH];
static const struct rtgui_graphic_driver_ops oled_ops =
    {
        oled_setPixel,
        oled_getPixel,
        oled_drawHLine,
        oled_drawVLine,
        oled_drawRawHLine
    };

/* Private functions ---------------------------------------------------------*/
rt_inline void oled_delayUs(rt_uint32_t us)
{
    /* This function is not that accurate */
    rt_uint32_t i = SystemCoreClock / 1000000 * us / 3;

    for(; i > 0; i--);
}

rt_inline void ssd1306_writeByte(rt_uint8_t data)
{
    MINISTM32_OLED_DATA_OUT(data);
    MINISTM32_OLED_WR_RESET;
    MINISTM32_OLED_WR_SET;
}

rt_inline void ssd1306_readByte(rt_uint8_t *data)
{
    GPIOB->CRL = 0x88888888;
    GPIOB->ODR = (GPIOB->ODR & 0x0000FF00) | 0x000000FF;

    MINISTM32_OLED_RD_RESET;
    MINISTM32_OLED_DATA_IN(data);
    MINISTM32_OLED_RD_SET;

	GPIOB->CRL = 0x33333333;
    GPIOB->ODR = (GPIOB->ODR & 0x0000FF00) | 0x000000FF;
}

rt_inline void ssd1306_readBuffer(rt_uint8_t *data, rt_uint8_t size)
{
    rt_uint8_t i;

    GPIOB->CRL = 0x88888888;
    GPIOB->ODR = (GPIOB->ODR & 0x0000FF00) | 0x000000FF;

    for (i = 0; i < size; i++)
    {
        MINISTM32_OLED_RD_RESET;
        MINISTM32_OLED_DATA_IN(data++);
        MINISTM32_OLED_RD_SET;
    }

	GPIOB->CRL = 0x33333333;
    GPIOB->ODR = (GPIOB->ODR & 0x0000FF00) | 0x000000FF;
}

static void oled_writeCmd(rt_uint8_t data)
{
    MINISTM32_OLED_CS_RESET;
    MINISTM32_OLED_DC_RESET;
    ssd1306_writeByte(data);
    MINISTM32_OLED_DC_SET;
    MINISTM32_OLED_CS_SET;
}

static void oled_writeLongCmd(rt_uint8_t *data, rt_uint8_t size)
{
    rt_uint8_t i;

    MINISTM32_OLED_CS_RESET;
    MINISTM32_OLED_DC_RESET;
    for (i = 0; i < size; i++)
    {
        ssd1306_writeByte(*data++);
    }
    MINISTM32_OLED_DC_SET;
    MINISTM32_OLED_CS_SET;
}

static void oled_writeData(rt_uint8_t *data, rt_uint8_t size)
{
    rt_uint8_t i;

    MINISTM32_OLED_CS_RESET;
    for (i = 0; i < size; i++)
    {
        ssd1306_writeByte(*data++);
    }
    MINISTM32_OLED_CS_SET;
}

static void oled_readStatus(rt_uint8_t *data)
{
    MINISTM32_OLED_CS_RESET;
    MINISTM32_OLED_DC_RESET;
    ssd1306_readByte(data);
    MINISTM32_OLED_DC_SET;
    MINISTM32_OLED_CS_SET;
}

static void oled_readData(rt_uint8_t *data, rt_uint8_t size)
{
    rt_uint8_t dummy;

    MINISTM32_OLED_CS_RESET;
    ssd1306_readByte(&dummy);
    ssd1306_readBuffer(data, size);
    MINISTM32_OLED_CS_SET;
}

void oled_clear(void)
{
	rt_uint32_t i;
    rt_uint8_t data[3];

    // Set column address
    data[0] = 0x21;
    data[1] = 0x00;
    data[2] = MINISTM32_OLED_WIDTH - 1;
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = 0x00;
    data[2] = (MINISTM32_OLED_HEIGHT - 1) / 8;
    oled_writeLongCmd(data, 3);

    MINISTM32_OLED_CS_RESET;
	for (i = 0; i < (MINISTM32_OLED_WIDTH * (MINISTM32_OLED_HEIGHT / 8)); i++)
    {
        ssd1306_writeByte(0x00);
//        oled_delayUs(100);
	}
    MINISTM32_OLED_CS_SET;
}

static void ssd1306_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Config GPIO */
    RCC_APB2PeriphClockCmd(MINISTM32_OLED_DATA_CLOCK | \
        MINISTM32_OLED_CTRL_CLOCK | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = \
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
        GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(MINISTM32_OLED_DATA_PORT, &GPIO_InitStructure);
    GPIO_Write(MINISTM32_OLED_DATA_PORT, GPIO_InitStructure.GPIO_Pin);

    GPIO_InitStructure.GPIO_Pin   = \
        MINISTM32_OLED_CS_PIN | MINISTM32_OLED_DC_PIN | \
        MINISTM32_OLED_WR_PIN | MINISTM32_OLED_RD_PIN;
    GPIO_Init(MINISTM32_OLED_CTRL_PORT, &GPIO_InitStructure);
    GPIO_Write(MINISTM32_OLED_CTRL_PORT, GPIO_InitStructure.GPIO_Pin);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

static rt_err_t ssd1306_init(void)
{
    rt_uint8_t data[2];

    ssd1306_gpio_init();

    // Turn off panel
    oled_writeCmd(0xAE);

    // Set display clock
    data[0] = 0xD5;
    data[1] = 0x80;         // default
    oled_writeLongCmd(data, 2);
    // Set charge pump
    data[0] = 0x8D;
    data[1] = 0x14;         // enable
    oled_writeLongCmd(data, 2);
    // Set pre-charge period
    data[0] = 0xD9;
    data[1] = 0xF1;
    oled_writeLongCmd(data, 2);
    // Set Vcomh deselect level
    data[0] = 0xDB;
    data[1] = 0x30;         // 0x83 x Vcc
    oled_writeLongCmd(data, 2);
    // Set contrast
    data[0] = 0x81;
    data[1] = 0xEF;
    oled_writeLongCmd(data, 2);

    // Set memory addressing mode
    data[0] = 0x20;
    data[1] = 0x00;         // horizontal mode
    oled_writeLongCmd(data, 2);
    // Set segment remap
    oled_writeCmd(0xA1);    // colume 127 -> SEG0
    // Set normal display
    oled_writeCmd(0xA6);
    // Set multiplex ratio
    data[0] = 0xA8;
    data[1] = 0x3f;         // N = 64, default
    oled_writeLongCmd(data, 2);
    // Set COM output scan direction
    oled_writeCmd(0xC8);    // from COM[N-1] to COM0
    // Set COM pin
    data[0] = 0xDA;
    data[1] = 0x12;         // alternative, disable left/right remap, default
    oled_writeLongCmd(data, 2);
    // Set display offset
    data[0] = 0xD3;
    data[1] = 0x00;         // default
    oled_writeLongCmd(data, 2);
    // Set low column address
//    oled_writeCmd(0x00);    // default
    // Set high column address
//    oled_writeCmd(0x10);    // default
    // Set display start line
    oled_writeCmd(0x40);    // default

    // Turn on display
    oled_writeCmd(0xA4);
    // Turn on panel
    oled_writeCmd(0xAF);

    oled_clear();

    return RT_EOK;
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
static void oled_getPixel(rtgui_color_t *c, int x, int y)
{
    rt_err_t ret;
    rt_uint8_t color, data[3];

    if ((x >= MINISTM32_OLED_WIDTH) || (y >= MINISTM32_OLED_HEIGHT))
    {
        return;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    // Set column address
    data[0] = 0x21;
    data[1] = x;
    data[2] = x;
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = y / 8;
    data[2] = y / 8;
    oled_writeLongCmd(data, 3);

    oled_readData(&color, 1);
    if (color & (1 << (y % 8)))
    {
        *(rt_uint8_t *)c = 0x01;
    }
    else
    {
        *(rt_uint8_t *)c = 0x00;
    }

    rt_sem_release(&oled_lock);
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
static void oled_setPixel(rtgui_color_t *c, int x, int y)
{
    rt_err_t ret;
    rt_uint8_t color, data[3];

    if ((x >= MINISTM32_OLED_WIDTH) || (y >= MINISTM32_OLED_HEIGHT))
    {
        return;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    // Set column address
    data[0] = 0x21;
    data[1] = x;
    data[2] = x;
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = y / 8;
    data[2] = y / 8;
    oled_writeLongCmd(data, 3);

    oled_readData(&color, 1);
    color &= ~(1 << (y % 8));
    if (*(rt_uint8_t *)c)
    {
        color |= 1 << (y % 8);
    }
    oled_writeData(&color, 1);

    rt_sem_release(&oled_lock);
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
static void oled_drawRawHLine(rt_uint8_t *pixels, int x1, int x2, int y)
{
    rt_err_t ret;
    rt_uint8_t color[MINISTM32_OLED_WIDTH], data[3];
    rt_uint32_t i;

    if ((x1 >= MINISTM32_OLED_WIDTH) || (y >= MINISTM32_OLED_HEIGHT))
    {
        return;
    }
    if (x2 >= MINISTM32_OLED_WIDTH)
    {
        x2 = MINISTM32_OLED_WIDTH - 1;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    // Set column address
    data[0] = 0x21;
    data[1] = x1;
    data[2] = x2;
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = y / 8;
    data[2] = y / 8;
    oled_writeLongCmd(data, 3);

    oled_readData(color, x2 - x1 + 1);
    for (i = 0; i < x2 - x1; i++)
    {
        color[i] &= ~(1 << (y % 8));
        if (*pixels++)
        {
            color[i] |= 1 << (y % 8);
        }
    }
    oled_writeData(color, x2 - x1 + 1);

    rt_sem_release(&oled_lock);
    oled_debug("rawH (%d-%d, %d) %x\n", x1, x2, y, *pixels);
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
static void oled_drawHLine(rtgui_color_t *c, int x1, int x2, int y)
{
    rt_err_t ret;
    rt_uint8_t color[MINISTM32_OLED_WIDTH], data[3];
    rt_uint32_t i;

    if ((x1 >= MINISTM32_OLED_WIDTH) || (y >= MINISTM32_OLED_HEIGHT))
    {
        return;
    }
    if (x2 >= MINISTM32_OLED_WIDTH)
    {
        x2 = MINISTM32_OLED_WIDTH - 1;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    // Set column address
    data[0] = 0x21;
    data[1] = x1;
    data[2] = x2;
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = y / 8;
    data[2] = y / 8;
    oled_writeLongCmd(data, 3);

    oled_readData(color, x2 - x1 + 1);
    for (i = 0; i < x2 - x1; i++)
    {
        color[i] &= ~(1 << (y % 8));
    }
    if (*(rt_uint8_t *)c)
    {
        for (i = 0; i < x2 - x1; i++)
        {
            color[i] |= 1 << (y % 8);
        }
    }
    oled_writeData(color, x2 - x1 + 1);

    rt_sem_release(&oled_lock);
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
static void oled_drawVLine(rtgui_color_t *c, int x , int y1, int y2)
{
    rt_err_t ret;
    rt_uint8_t color[MINISTM32_OLED_HEIGHT], data[3];
    rt_uint32_t i;

    if ((x >= MINISTM32_OLED_WIDTH) || (y1 >= MINISTM32_OLED_HEIGHT))
    {
        return;
    }
    if (y2 >= MINISTM32_OLED_HEIGHT)
    {
        y2 = MINISTM32_OLED_HEIGHT - 1;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&oled_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    // Set memory addressing mode
    data[0] = 0x20;
    data[1] = 0x01;         // vertical mode
    oled_writeLongCmd(data, 2);

    // Set column address
    data[0] = 0x21;
    data[1] = x;
    data[2] = x;
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = y1 / 8;
    data[2] = y2 / 8;
    oled_writeLongCmd(data, 3);

    oled_readData(color, (y2 - y1 + 1 + 7) / 8);
    if (*(rt_uint8_t *)c)
    {
        for (i = y1; i <= y2; i++)
        {
            color[i / 8] |= 1 << (i % 8);
        }
    }
    else
    {
        for (i = y1; i <= y2; i++)
        {
            color[i / 8] &= ~(1 << (i % 8));
        }
    }
    oled_writeData(color, (y2 - y1 + 1 + 7) / 8);

    // Set memory addressing mode
    data[0] = 0x20;
    data[1] = 0x00;         // horizontal mode
    oled_writeLongCmd(data, 2);

    rt_sem_release(&oled_lock);
    oled_debug(" VLine (%d, %d-%d) %x\n", x, y1, y2, *(rt_uint8_t *)c);
}


/***************************************************************************//**
 * @brief
 *   Open OLED device
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
static rt_err_t miniStm32_oled_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Close OLED device
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
static rt_err_t miniStm32_oled_close(rt_device_t dev)
{
    return RT_EOK;
}

/*static void miniStm32_oled_update(struct rt_device_rect_info *rect)
{
    rt_uint8_t i, data[3];

    if ((rect->x >= MINISTM32_OLED_WIDTH) || (rect->y >= MINISTM32_OLED_HEIGHT))
    {
        return;
    }

    // Set column address
    data[0] = 0x21;
    data[1] = rect->x;
    if ((rect->x + rect->width) >= MINISTM32_OLED_WIDTH)
    {
        data[2] = MINISTM32_OLED_WIDTH - 1;
    }
    else
    {
        data[2] = rect->x + rect->width;
    }
    oled_writeLongCmd(data, 3);

    // Set page address
    data[0] = 0x22;
    data[1] = rect->y / 8;
    if ((rect->y + rect->height) >= MINISTM32_OLED_HEIGHT)
    {
        data[2] = (MINISTM32_OLED_HEIGHT - 1) / 8;
    }
    else
    {
        data[2] = (rect->y + rect->height) / 8;
    }
    oled_writeLongCmd(data, 3);

    for (i = data[1]; i <= data[2]; i++)
    {
        oled_writeData(&frame_buffer[i][rect->x], rect->width);
    }
}*/

/***************************************************************************//**
* @brief
*   Configure OLED device
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
static rt_err_t miniStm32_oled_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
	switch (cmd)
	{
	case RTGRAPHIC_CTRL_RECT_UPDATE:
//        miniStm32_oled_update((struct rt_device_rect_info *)args);
        oled_debug("OLED: update\n");
		break;
	case RTGRAPHIC_CTRL_POWERON:
		break;
	case RTGRAPHIC_CTRL_POWEROFF:
		break;
	case RTGRAPHIC_CTRL_GET_INFO:
		rt_memcpy(args, &oled_info, sizeof(oled_info));
		break;
	case RTGRAPHIC_CTRL_SET_MODE:
		break;
	}

	return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *	Register OLED device
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
static rt_err_t miniStm32_oled_register(
	rt_device_t	        device,
	const char          *name,
	rt_uint32_t         flag,
	void                *data)
{
	RT_ASSERT(device != RT_NULL);

	device->type 		= RT_Device_Class_Graphic;
	device->rx_indicate = RT_NULL;
	device->tx_complete = RT_NULL;
	device->init 		= RT_NULL;
	device->open		= miniStm32_oled_open;
	device->close		= miniStm32_oled_close;
	device->read 		= RT_NULL;
	device->write 		= RT_NULL;
	device->control 	= miniStm32_oled_control;
	device->user_data	= data;

	/* register a character device */
	return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR | flag);
}

/***************************************************************************//**
 * @brief
 *   Initialize OLED device
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void miniStm32_hw_oled_init(void)
{
    rt_uint32_t flag;
    rt_uint8_t status;

    do
    {
        /* Init OLED info */
        flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE; //RT_DEVICE_FLAG_DMA_TX
        oled_info.pixel_format      = RTGRAPHIC_PIXEL_FORMAT_MONO;
        oled_info.bits_per_pixel    = 1;
        oled_info.width             = MINISTM32_OLED_WIDTH;
        oled_info.height            = MINISTM32_OLED_HEIGHT;
        oled_info.framebuffer       = RT_NULL; //(rt_uint8_t *)frame_buffer;
//        if (miniStm32_oled_register(&oled_device, OLED_DEVICE_NAME, flag, RT_NULL) != RT_EOK)
        if (miniStm32_oled_register(&oled_device, OLED_DEVICE_NAME, flag,
            (void *)&oled_ops) != RT_EOK)
        {
            break;
        }

        /* Init OLED lock */
        if (rt_sem_init(&oled_lock, OLED_DEVICE_NAME, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }

        /* Init ssd1306 */
        if (ssd1306_init() != RT_EOK)
        {
            break;
        }

        oled_readStatus(&status);
        oled_debug("OLED: status %x\n", status);

        /* Set as rtgui graphic driver */
        if (rtgui_graphic_set_device(&oled_device) != RT_EOK)
        {
            break;
        }

        oled_debug("OLED: H/W init OK!\n");
        return;
    } while(0);

    oled_debug("OLED err: H/W init failed!\n");
}

#endif /* defined(MINISTM32_USING_OLED) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
