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
#include "drv_lcd.h"

#if defined(MINISTM32_USING_LCD)
#include <rtgui/rtgui.h>
#include <rtgui/driver.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_LCD_DEBUG
#define lcd_debug(format,args...)       rt_kprintf(format, ##args)
#else
#define lcd_debug(format,args...)
#endif

/* Private function prototypes -----------------------------------------------*/
static void lcd_setPixel(rtgui_color_t *c, int x, int y);
static void lcd_getPixel(rtgui_color_t *c, int x, int y);
static void lcd_drawHLine(rtgui_color_t *c, int x1, int x2, int y);
static void lcd_drawVLine(rtgui_color_t *c, int x , int y1, int y2);
static void lcd_drawRawHLine(rt_uint8_t *pixels, int x1, int x2, int y);

/* Private variables ---------------------------------------------------------*/
static struct rt_device lcd_device;
struct rt_semaphore lcd_lock;
static struct rt_device_graphic_info lcd_info;
static const struct rtgui_graphic_driver_ops lcd_ops =
    {
        lcd_setPixel,
        lcd_getPixel,
        lcd_drawHLine,
        lcd_drawVLine,
        lcd_drawRawHLine
    };

/* Private functions ---------------------------------------------------------*/
rt_inline void lcd_delayUs(rt_uint32_t us)
{
    /* This function is not that accurate */
    rt_uint32_t i = SystemCoreClock / 1000000 * us / 3;

    for(; i > 0; i--);
}

/*  SSD1289 8080 Interface Timing Characteristics
    - Write cycle: 100ns
    - Read cycle: 1000ns
    - CS pulse width for write: 50ns
    - CS pulse width for read: 500ns
 */
rt_inline void ssd1289_writeIndex(rt_uint16_t index)
{
#if defined(MINISTM32_LCD_SPEEDUP)
    MINISTM32_LCD_CS_RESET;
    MINISTM32_LCD_RS_RESET;
    MINISTM32_LCD_DATA_OUT(index);
    MINISTM32_LCD_WR_RESET;
    lcd_delayUs(1);
    MINISTM32_LCD_WR_SET;
    MINISTM32_LCD_RS_SET;
#else
    GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_CS_PIN);
    GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RS_PIN);
    GPIO_Write(MINISTM32_LCD_DATA_PORT, index);
    GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
    lcd_delayUs(1);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RS_PIN);
#endif
}

rt_inline void ssd1289_writeData(rt_uint16_t data)
{
#if defined(MINISTM32_LCD_SPEEDUP)
    MINISTM32_LCD_DATA_OUT(data);
    MINISTM32_LCD_WR_RESET;
    lcd_delayUs(1);
    MINISTM32_LCD_WR_SET;
    MINISTM32_LCD_CS_SET;
#else
    GPIO_Write(MINISTM32_LCD_DATA_PORT, data);
    GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
    lcd_delayUs(1);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_CS_PIN);
#endif
}

rt_inline void ssd1289_writeBuffer(rt_uint16_t color, rt_uint16_t length)
{
    rt_uint16_t i;

#if defined(MINISTM32_LCD_SPEEDUP)
    MINISTM32_LCD_DATA_OUT(color);
    for (i = 0; i < length; i++)
    {
        MINISTM32_LCD_WR_RESET;
        lcd_delayUs(1);
        MINISTM32_LCD_WR_SET;
    }
    MINISTM32_LCD_CS_SET;
#else
    GPIO_Write(MINISTM32_LCD_DATA_PORT, color);
    for (i = 0; i < length; i++)
    {
        GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
        lcd_delayUs(1);
        GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
    }
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_CS_PIN);
#endif
}

rt_inline void ssd1289_writePixels(rt_uint16_t *pixels, rt_uint16_t length)
{
    rt_uint16_t i;

#if defined(MINISTM32_LCD_SPEEDUP)
    for (i = 0; i < length; i++)
    {
        MINISTM32_LCD_DATA_OUT(*(pixels++));
        MINISTM32_LCD_WR_RESET;
        lcd_delayUs(1);
        MINISTM32_LCD_WR_SET;
    }
    MINISTM32_LCD_CS_SET;
#else
    for (i = 0; i < length; i++)
    {
        GPIO_Write(MINISTM32_LCD_DATA_PORT, *(pixels++));
        GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
        lcd_delayUs(1);
        GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_WR_PIN);
    }
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_CS_PIN);
#endif
}

rt_inline void ssd1289_readData(rt_uint16_t *data)
{
#if defined(MINISTM32_LCD_SPEEDUP)
    GPIOB->CRL = 0x88888888;
    GPIOB->CRH = 0x88888888;
    GPIOB->ODR = 0x0000FFFF;

    MINISTM32_LCD_RD_RESET;
    lcd_delayUs(1);
    MINISTM32_LCD_DATA_IN(data);
    MINISTM32_LCD_RD_SET;
    MINISTM32_LCD_CS_SET;

	GPIOB->CRL = 0x33333333;
	GPIOB->CRH = 0x33333333;
	GPIOB->ODR = 0x0000FFFF;
#else
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Set data port to input */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_All;
    GPIO_Init(MINISTM32_LCD_DATA_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RD_PIN);
    lcd_delayUs(1);
    *data = GPIO_ReadInputData(MINISTM32_LCD_DATA_PORT);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RD_PIN);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_CS_PIN);

    /* Set data port back to output */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(MINISTM32_LCD_DATA_PORT, &GPIO_InitStructure);
    GPIO_Write(MINISTM32_LCD_DATA_PORT, GPIO_InitStructure.GPIO_Pin);
#endif
}

rt_inline void ssd1289_readBuffer(rt_uint16_t *color, rt_uint16_t length)
{
    rt_uint16_t i;
#if defined(MINISTM32_LCD_SPEEDUP)
    GPIOB->CRL = 0x88888888;
    GPIOB->CRH = 0x88888888;
    GPIOB->ODR = 0x0000FFFF;

    MINISTM32_LCD_RD_RESET;
    lcd_delayUs(3);
    MINISTM32_LCD_RD_SET;

    for (i = 0; i < length; i++)
    {
        MINISTM32_LCD_RD_RESET;
        lcd_delayUs(3);
        MINISTM32_LCD_DATA_IN(color++);
        MINISTM32_LCD_RD_SET;
    }
    MINISTM32_LCD_CS_SET;
#else
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Set data port to input */
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_All;
    GPIO_Init(MINISTM32_LCD_DATA_PORT, &GPIO_InitStructure);

    /* Dummy read */
    GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RD_PIN);
    lcd_delayUs(3);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RD_PIN);

    for (i = 0; i < length; i++)
    {
        GPIO_ResetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RD_PIN);
        lcd_delayUs(3);
        *(color++) = GPIO_ReadInputData(MINISTM32_LCD_DATA_PORT);
        GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_RD_PIN);
    }
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, MINISTM32_LCD_CS_PIN);

    /* Set data port back to output */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(MINISTM32_LCD_DATA_PORT, &GPIO_InitStructure);
    GPIO_Write(MINISTM32_LCD_DATA_PORT, GPIO_InitStructure.GPIO_Pin);
#endif
}

static void lcd_writeReg(rt_uint16_t index, rt_uint16_t data)
{
    ssd1289_writeIndex(index);
    ssd1289_writeData(data);
}

static void lcd_writeData(rt_uint16_t color, rt_uint16_t length)
{
    ssd1289_writeIndex(0x0022);
    ssd1289_writeBuffer(color, length);
}

static void lcd_writePixels(rt_uint16_t *pixels, rt_uint16_t length)
{
    ssd1289_writeIndex(0x0022);
    ssd1289_writePixels(pixels, length);
}

static void lcd_readReg(rt_uint16_t index, rt_uint16_t *data)
{
    ssd1289_writeIndex(index);
    ssd1289_readData(data);
}

static void lcd_readData(rt_uint16_t *color, rt_uint16_t length)
{
    ssd1289_writeIndex(0x0022);
    ssd1289_readBuffer(color, length);
}

static void lcd_setCursor(rt_uint16_t x, rt_uint16_t y)
{
#if defined(MINISTM32_LCD_LANDSCAPE)
    lcd_writeReg(0x004E, y);
    lcd_writeReg(0x004F, 319 - x);
#else
    lcd_writeReg(0x004E, x);
    lcd_writeReg(0x004F, y);
#endif
}

static void lcd_clear(rt_uint16_t color)
{
	rt_uint16_t i;

    for (i = 0; i < MINISTM32_LCD_HEIGHT; i++)
    {
    	lcd_drawHLine((rtgui_color_t *)&color, 0, MINISTM32_LCD_WIDTH - 1, i);
    }

//    for (i = 0; i < LCD_WIDTH; i++)
//    {
//    	lcd_drawVLine((rtgui_color_t *)&color, i, 0, LCD_HEIGHT - 1);
//    }
}

static rt_err_t ssd1289_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    rt_uint16_t DeviceCode = 0;
    rt_uint32_t i;

    /* Config GPIO */
    RCC_APB2PeriphClockCmd(MINISTM32_LCD_DATA_CLOCK | \
        MINISTM32_LCD_CTRL_CLOCK | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_All;
    GPIO_Init(MINISTM32_LCD_DATA_PORT, &GPIO_InitStructure);
    GPIO_SetBits(MINISTM32_LCD_DATA_PORT, GPIO_InitStructure.GPIO_Pin);

    GPIO_InitStructure.GPIO_Pin   = \
        MINISTM32_LCD_BL_PIN | MINISTM32_LCD_CS_PIN | MINISTM32_LCD_RS_PIN | \
        MINISTM32_LCD_WR_PIN | MINISTM32_LCD_RD_PIN;
    GPIO_Init(MINISTM32_LCD_CTRL_PORT, &GPIO_InitStructure);
    GPIO_SetBits(MINISTM32_LCD_CTRL_PORT, GPIO_InitStructure.GPIO_Pin);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* Read ID */
    lcd_readReg(0x0000, &DeviceCode);
    if (DeviceCode == 0x8989)
    {
        lcd_debug("LCD: found SSD1289 (ID %x)\n", DeviceCode);
    }
    else
    {
        lcd_debug("LCD: unknown device (ID %x)\n", DeviceCode);
        return -RT_ERROR;
    }

    /* Display ON Sequence */
    // Power Supply Setting:
    // Set R07h at 0021h (GON=1,DTE=0,D[1:0]=01)
    lcd_writeReg(0x0007, 0x0021);
    // Set R00h at 0001h (OSCEN=1)
    lcd_writeReg(0x0000, 0x0001);
    // Set R07h at 0023h (GON=1,DTE=0,D[1:0]=11)
    lcd_writeReg(0x0007, 0x0023);
    // Set R10h at 0000h (Exit sleep mode)
    lcd_writeReg(0x0010,0x0000);
    // Wait 30ms
    lcd_delayUs(30000);
    // Set R07h at 0033h (GON=1,DTE=1,D[1:0]=11)
    lcd_writeReg(0x0007, 0x0033);
    // Entry mode setting (Portrait:AM=0;Landscape:AM=1)
#if defined(MINISTM32_LCD_LANDSCAPE)
    lcd_writeReg(0x0011, 0x6038);
#else
    lcd_writeReg(0x0011, 0x6030);
#endif
    // LCD driver AC setting
    lcd_writeReg(0x0002, 0x0600);

    /* The other settings */
    // Power control
    lcd_writeReg(0x0003, 0xEEEE); //0xA8A4); // TODO: 0x0804 ?
    lcd_writeReg(0x000C, 0x0004); //0x0000);
    lcd_writeReg(0x000D, 0x000F); //0x080C); // TODO: 0x0808 ?
    lcd_writeReg(0x000E, 0x3000); //0x2B00); // TODO: 0x2900 ?
    lcd_writeReg(0x001E, 0x00AF); //0x00B0); // TODO: 0x00B8 ?
    // Driver output
#if defined(MINISTM32_LCD_BGR)
    lcd_writeReg(0x0001, 0x233F);   // BGR=0
#else
    lcd_writeReg(0x0001, 0x2B3F);   // BGR=1
#endif
    // Compare register
    lcd_writeReg(0x0005, 0x0000);
    lcd_writeReg(0x0006, 0x0000);
    // Horizontal porch
    lcd_writeReg(0x0016, 0xEF1C);
    // Vertical porch
    lcd_writeReg(0x0017, 0x0003);
    // Display
//    lcd_writeReg(0x0007, 0x0233);
    // Frame cycle
    lcd_writeReg(0x000B, 0x0020); // TODO: 0x0000 |(3 << 6) ?
    // Gate scan position
    lcd_writeReg(0x000F, 0x0000);
    // Vertical scroll
    lcd_writeReg(0x0041, 0x0000);
    lcd_writeReg(0x0042, 0x0000);
    // 1st screen driving position
    lcd_writeReg(0x0048, 0x0000);
    lcd_writeReg(0x0049, 0x013F);
    // 2nd screen driving position
    lcd_writeReg(0x004A, 0x0000);
    lcd_writeReg(0x004B, 0x0000);
    // Vertical RAM address position
    lcd_writeReg(0x0044, 0xEF00);
    lcd_writeReg(0x0045, 0x0000);
    lcd_writeReg(0x0046, 0x013F);
    // Gamma
    lcd_writeReg(0x0030, 0x0707);
    lcd_writeReg(0x0031, 0x0204);
    lcd_writeReg(0x0032, 0x0204);
    lcd_writeReg(0x0033, 0x0502);
    lcd_writeReg(0x0034, 0x0507);
    lcd_writeReg(0x0035, 0x0204);
    lcd_writeReg(0x0036, 0x0204);
    lcd_writeReg(0x0037, 0x0502);
    lcd_writeReg(0x003A, 0x0302);
    lcd_writeReg(0x003B, 0x0302);
    // RAM write data mask
    lcd_writeReg(0x0023, 0x0000);
    lcd_writeReg(0x0024, 0x0000);
    // Frame frequency: 50 Hz
    lcd_writeReg(0x0025, 0x0000);
    // Window x0 = 0
    lcd_writeReg(0x004E, 0x0000);
    // Window y0 = 0
    lcd_writeReg(0x004F, 0x0000);

    return RT_EOK;
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
static void lcd_setPixel(rtgui_color_t *c, int x, int y)
{
    rt_err_t ret;

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    lcd_setCursor((rt_uint16_t)x, (rt_uint16_t)y);
	lcd_writeReg(0x0022, *(rt_uint16_t *)c);

    rt_sem_release(&lcd_lock);
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
static void lcd_getPixel(rtgui_color_t *c, int x, int y)
{
    rt_err_t ret;

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    lcd_setCursor((rt_uint16_t)x, (rt_uint16_t)y);
    lcd_readData((rt_uint16_t *)c, 1);

    rt_sem_release(&lcd_lock);
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
static void lcd_drawRawHLine(rt_uint8_t *pixels, int x1, int x2, int y)
{
    rt_err_t ret;
    rt_uint16_t *data = (rt_uint16_t *)pixels;

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    lcd_setCursor((rt_uint16_t)x1, (rt_uint16_t)y);
    lcd_writePixels(data, x2 - x1 + 1);
//    lcd_debug("* %d-%d %d\n", x1, x2, y);

    rt_sem_release(&lcd_lock);
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
static void lcd_drawHLine(rtgui_color_t *c, int x1, int x2, int y)
{
    rt_err_t ret;

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

    lcd_setCursor((rt_uint16_t)x1, (rt_uint16_t)y);
    lcd_writeData(*(rt_uint16_t *)c, x2 - x1 + 1);
//    lcd_debug("LCD: H LINE (%d-%d, %d)\n", x1, x2, y);

    rt_sem_release(&lcd_lock);
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
static void lcd_drawVLine(rtgui_color_t *c, int x , int y1, int y2)
{
    rt_err_t ret;

    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&lcd_lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return;
    }

#if defined(MINISTM32_LCD_LANDSCAPE)
    lcd_writeReg(0x0011, 0x6030);           // AM=0
#else
    lcd_writeReg(0x0011, 0x6038);           // AM=1
#endif
    lcd_setCursor((rt_uint16_t)x, (rt_uint16_t)y1);
    lcd_writeData(*(rt_uint16_t *)c, y2 - y1 + 1);
#if defined(MINISTM32_LCD_LANDSCAPE)
    lcd_writeReg(0x0011, 0x6038);           // AM=0
#else
    lcd_writeReg(0x0011, 0x6030);           // AM=1
#endif
//     lcd_debug("LCD: V LINE (%d, %d-%d)\n", x, y1, y2);

    rt_sem_release(&lcd_lock);
}

/***************************************************************************//**
 * @brief
 *   LCD calibration
 *
 * @details
 *
 * @note
 *
 * @param[in] step
 *   Calibration step
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t lcd_calibration(rt_uint16_t *step)
{
    rt_uint16_t color = MINISTM32_LCD_COLOR_RED;
    rt_uint16_t offset = 30;    /* The offset between calibration ponit and lcd edge */
    rt_uint16_t x1, x2, y1, y2;
    rt_uint16_t half_len = 30;

    switch(*step)
    {
    case 0:
        lcd_clear(MINISTM32_LCD_COLOR_GREENE);
        break;
    case 1:
        if (offset > half_len)
        {
            x1 = offset - half_len;
            y1 = offset - half_len;
        }
        else
        {
            x1 = 0;
            y1 = 0;
        }
        x2 = offset + half_len;
        y2 = offset + half_len;

        lcd_drawHLine((rtgui_color_t *)&color,
            (int)x1, (int)x2,
            (int)offset);
        lcd_drawVLine((rtgui_color_t *)&color,
            (int)offset,
            (int)y1, (int)y2);
        break;
    case 2:
        if (offset > half_len)
        {
            x2 = MINISTM32_LCD_WIDTH - offset + half_len;
            y1 = offset - half_len;
        }
        else
        {
            x2 = MINISTM32_LCD_WIDTH;
            y1 = 0;
        }
        x1 = MINISTM32_LCD_WIDTH - offset - half_len;
        y2 = offset + half_len;

        lcd_drawHLine((rtgui_color_t *)&color,
            (int)x1, (int)x2,
            (int)offset);
        lcd_drawVLine((rtgui_color_t *)&color,
            (int)MINISTM32_LCD_WIDTH - offset,
            (int)y1, (int)y2);
        break;
    case 3:
        if (offset > half_len)
        {
            x1 = offset - half_len;
            y2 = MINISTM32_LCD_HEIGHT - offset + half_len;
        }
        else
        {
            x1 = 0;
            y2 = MINISTM32_LCD_HEIGHT;
        }
        x2 = offset + half_len;
        y1 = MINISTM32_LCD_HEIGHT - offset - half_len;

        lcd_drawHLine((rtgui_color_t *)&color,
            (int)x1, (int)x2,
            (int)MINISTM32_LCD_HEIGHT - offset);
        lcd_drawVLine((rtgui_color_t *)&color,
            (int)offset,
            (int)y1, (int)y2);
        break;
    case 4:
        if (offset > half_len)
        {
            x2 = MINISTM32_LCD_WIDTH - offset + half_len;
            y2 = MINISTM32_LCD_HEIGHT - offset + half_len;
        }
        else
        {
            x2 = MINISTM32_LCD_WIDTH;
            y2 = MINISTM32_LCD_HEIGHT;
        }
        x1 = MINISTM32_LCD_WIDTH - offset - half_len;
        y1 = MINISTM32_LCD_HEIGHT - offset - half_len;

        lcd_drawHLine((rtgui_color_t *)&color,
            (int)x1, (int)x2,
            (int)MINISTM32_LCD_HEIGHT - offset);
        lcd_drawVLine((rtgui_color_t *)&color,
            (int)MINISTM32_LCD_WIDTH - offset,
            (int)y1, (int)y2);
        break;
    default:
        return -RT_ERROR;
        break;
    }

    *step = offset;
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
static rt_err_t miniStm32_lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
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
static rt_err_t miniStm32_lcd_close(rt_device_t dev)
{
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
static rt_err_t miniStm32_lcd_control(rt_device_t dev, rt_uint8_t cmd, void *args)
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
		rt_memcpy(args, &lcd_info, sizeof(lcd_info));
		break;
	case RTGRAPHIC_CTRL_SET_MODE:
		break;
    case RTGRAPHIC_CTRL_LCD_CALIBRATION:
        return lcd_calibration((rt_uint16_t *)args);
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
static rt_err_t miniStm32_lcd_register(
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
	device->open		= miniStm32_lcd_open;
	device->close		= miniStm32_lcd_close;
	device->read 		= RT_NULL;
	device->write 		= RT_NULL;
	device->control 	= miniStm32_lcd_control;
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
void miniStm32_hw_lcd_init(void)
{
    rt_uint32_t flag;

    do
    {
        /* Init LCD info */
        flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE; //RT_DEVICE_FLAG_DMA_TX
#if defined(MINISTM32_LCD_BGR)
        lcd_info.pixel_format       = RTGRAPHIC_PIXEL_FORMAT_RGB565;
#else
        lcd_info.pixel_format       = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
#endif
        lcd_info.bits_per_pixel     = 16;
        lcd_info.width              = MINISTM32_LCD_WIDTH;
        lcd_info.height             = MINISTM32_LCD_HEIGHT;
        lcd_info.framebuffer        = RT_NULL;
        if (miniStm32_lcd_register(&lcd_device, LCD_DEVICE_NAME, flag, (void *)&lcd_ops) != RT_EOK)
        {
            break;
        }

        /* Init LCD lock */
        if (rt_sem_init(&lcd_lock, LCD_DEVICE_NAME, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }

        /* Init ssd1289 */
        if (ssd1289_init() != RT_EOK)
        {
            break;
        }
        lcd_clear(MINISTM32_LCD_COLOR_BLUE);

        /* Set as rtgui graphic driver */
        if (rtgui_graphic_set_device(&lcd_device) != RT_EOK)
        {
            break;
        }

        lcd_debug("LCD: H/W init OK!\n");
        return;
    } while(0);

    lcd_debug("LCD err: H/W init failed!\n");
}

#endif /* defined(MINISTM32_USING_LCD) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
