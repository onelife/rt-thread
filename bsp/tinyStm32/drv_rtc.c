/***************************************************************************//**
 * @file    drv_rtc.c
 * @brief   RTC driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author  onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-02-18   onelife     Initial creation of RTC driver for MiniSTM32
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_rtc.h"
#if defined(BSP_USING_RTC)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_RTC_DEBUG
#define rtc_debug(format,args...)           rt_kprintf(format, ##args)
#else
#define rtc_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct rt_device rtc;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *	Touch screen interrupt handler
 *
 * @details
 *
 * @note
 ******************************************************************************/
rt_inline void rtc_delayUs(rt_uint32_t us)
{
    /* This function is not that accurate */
    rt_uint32_t i = SystemCoreClock / 1000000 * us / 3;

    for(; i > 0; i--);
}

/***************************************************************************//**
 * @brief
 *  RTC counter overflow interrupt handler
 *
 * @details
 *
 * @note
 ******************************************************************************/
void rtc_overflow_isr(rt_device_t device)
{
	if (RTC_GetFlagStatus(RTC_FLAG_OW))
	{
        rt_kprintf("IR: overflow!!!\n");
	}
}

/***************************************************************************//**
 * @brief
 *   Open RTC device
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
static rt_err_t miniStm32_rtc_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Close RTC device
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
static rt_err_t miniStm32_rtc_close(rt_device_t dev)
{
    return RT_EOK;
}

/***************************************************************************//**
* @brief
*   Configure RTC device
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
static rt_err_t miniStm32_rtc_control(rt_device_t dev, rt_uint8_t cmd,
    void *args)
{
    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_time_t *)args = RTC_GetCounter();
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        /* Enable BKP clocks */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
        /* Enable write access to Backup domain */
        PWR_BackupAccessCmd(ENABLE);
        /* Change the current time */
        RTC_WaitForLastTask();
        RTC_SetCounter(*(rt_time_t *)args);
        RTC_WaitForLastTask();
        /* Mark in BKP */
        BKP_WriteBackupRegister(BKP_DR1, 0x4F4B /* "OK" */);
        /* Disable BKP clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, DISABLE);
        break;
    }

    return RT_EOK;
}


/***************************************************************************//**
* @brief
*   Initialize touch screen related hardware and register the device to kernel
*
* @details
*
* @note
*
* @return
*   Error code
******************************************************************************/
rt_err_t miniStm32_hw_rtc_init(void)
{
    miniStm32_irq_hook_init_t hook;
    NVIC_InitTypeDef nvic_init;

    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    {
        rt_kprintf("RTC: power on reset detected\n");
        RCC_ClearFlag();
    }
    if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
    {
        rt_kprintf("RTC: button reset detected\n");
    }
    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET)
    {
        rt_kprintf("RTC: S/W reset detected\n");
    }

    /* Enable PWR and BKP clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    /* Read BKP */
    if (BKP_ReadBackupRegister(BKP_DR1) != 0x4F4B /* "OK" */)
    {
        rt_uint16_t retry = 1000;

        rt_kprintf("RTC: not configured!\n");
        rt_kprintf("RTC: please configure using set_date() and set_time()\n");

        /* Enable write access to Backup domain */
        PWR_BackupAccessCmd(ENABLE);
        /* Reset Backup Domain */
        BKP_DeInit();
        /* Enable LSE */
        RCC_LSEConfig(RCC_LSE_ON);
        /* Wait till LSE is ready */
        while ((RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) && retry)
        {
            rtc_delayUs(1000);
            retry--;
        }
        if (!retry)
        {
            rt_kprintf("RTC err: LSE is not ready!\n");
            return -RT_ERROR;
        }
        /* Select LSE as RTC Clock Source */
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        /* Enable RTC Clock */
        RCC_RTCCLKCmd(ENABLE);
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        /* Set RTC prescaler: set RTC period to 1sec */
        RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
    }
    else
    {
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
    }
    /* Disable BKP clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, DISABLE);

    /* Config hook */
    hook.type       = miniStm32_irq_type_dma;
    hook.unit       = 0;
    hook.cbFunc     = rtc_overflow_isr;
    hook.userPtr    = RT_NULL;
    miniStm32_irq_hook_register(&hook);

    /* Enable interrupt and NVIC */
    RTC_ClearFlag(RTC_FLAG_OW);
    RTC_WaitForLastTask();
    RTC_ITConfig(RTC_IT_OW, ENABLE);
    RTC_WaitForLastTask();
    NVIC_ClearPendingIRQ(RTC_IRQn);
    nvic_init.NVIC_IRQChannel = RTC_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DEFAULT;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    /* Config and register device */
    rtc.type	        = RT_Device_Class_RTC;
    rtc.init 	        = RT_NULL;
    rtc.open 	        = miniStm32_rtc_open;
    rtc.close	        = miniStm32_rtc_close;
    rtc.read 	        = RT_NULL;
    rtc.write	        = RT_NULL;
    rtc.control         = miniStm32_rtc_control;
    rtc.user_data       = RT_NULL;

    return rt_device_register(&rtc, RTC_NAME, RT_DEVICE_FLAG_RDWR);
}

/*******************************************************************************
 *  Support function
 ******************************************************************************/
#include <time.h>

#if defined (__IAR_SYSTEMS_ICC__) &&  (__VER__) >= 6020000   /* for IAR 6.2 later Compiler */
#pragma module_name = "?time"
time_t (__time32)(time_t *t)                                 /* Only supports 32-bit timestamp */
#else
time_t time(time_t* t)
#endif
{
    rt_device_t rtc;
    time_t time = 0;

    rtc = rt_device_find(RTC_NAME);
    if (rtc != RT_NULL)
    {
        rtc->control(rtc, RT_DEVICE_CTRL_RTC_GET_TIME, &time);
        if (t != RT_NULL) *t = time;
    }

    return time;
}

/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
#ifdef RT_USING_FINSH
#include <finsh.h>

void set_date(rt_uint32_t year, rt_uint32_t month, rt_uint32_t day)
{
    time_t now;
    struct tm* ti;
    rt_device_t rtc;

    ti = RT_NULL;
    time(&now);

    ti = localtime(&now);
    if (ti != RT_NULL)
    {
        ti->tm_year = year - 1900;
        ti->tm_mon 	= month - 1; /* ti->tm_mon 	= month; 0~11 */
        ti->tm_mday = day;
    }
    now = mktime(ti);

    rtc = rt_device_find(RTC_NAME);
    if (rtc != RT_NULL)
    {
        rtc->control(rtc, RT_DEVICE_CTRL_RTC_SET_TIME, &now);
    }
}
FINSH_FUNCTION_EXPORT(set_date, set date. e.g: set_date(2010,2,28))

void set_time(rt_uint32_t hour, rt_uint32_t minute, rt_uint32_t second)
{
    time_t now;
    struct tm* ti;
    rt_device_t rtc;

    ti = RT_NULL;
    time(&now);

    ti = localtime(&now);
    if (ti != RT_NULL)
    {
        ti->tm_hour = hour;
        ti->tm_min 	= minute;
        ti->tm_sec 	= second;
    }
    now = mktime(ti);

    rtc = rt_device_find(RTC_NAME);
    if (rtc != RT_NULL)
    {
        rtc->control(rtc, RT_DEVICE_CTRL_RTC_SET_TIME, &now);
    }
}
FINSH_FUNCTION_EXPORT(set_time, set time. e.g: set_time(23,59,59))

void list_date(void)
{
    time_t now;

    time(&now);
    rt_kprintf("%s\n", ctime(&now));
}
FINSH_FUNCTION_EXPORT(list_date, show date and time.)
#endif

#endif /* defined(BSP_USING_RTC) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
