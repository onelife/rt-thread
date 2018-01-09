/***************************************************************************//**
 * @file    application.c
 * @brief   Demo application
 *  COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author  Bernard, onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 * LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-01-24	onelife		Initial creation for STM32 (Modified from EFN32 branch)
 * 2012-05-17   onelife     Modify photo frame DEMO according to new version of
 *  RTGUI
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <board.h>

#if defined(RT_USING_DFS)
/* dfs init */
#include <dfs_init.h>
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#if defined(BOARD_USING_SPISD)
#include "drv_sdcard.h"
#endif
#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#endif

#if 0 // defined(BOARD_USING_OLED)
//#include <rtgui/rtgui_server.h>
//#include <rtgui/rtgui_system.h>
#include <rtgui/rtgui_app.h>
#include <rtgui/widgets/widget.h>
#include <rtgui/widgets/label.h>
#include <rtgui/widgets/window.h>
#include <rtgui/widgets/box.h>
#include <rtgui/image.h>
#include <rtgui/video_mjpeg.h> 	// TESTING

 #if defined(RTGUI_USING_DFS_FILERW)
 #include <dfs_posix.h>
 #define PATH_SEPARATOR     '/'
 #endif
#endif

/* Private defines -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#if 0 // defined(BOARD_USING_OLED)
static rt_bool_t pic_view_event_handler(rtgui_object_t *object, rtgui_event_t *event)
{
	rt_bool_t result;
    rt_bool_t load = RT_FALSE;

	result = rtgui_label_event_handler(object, event);

    switch(event->type)
    {
    case RTGUI_EVENT_PAINT:
        load = RT_TRUE;
        break;
    }

    if (load)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;
        rtgui_image_t* image;

//        image = rtgui_image_create_from_file("jpg", "/test9.jpg", RT_FALSE);
        image = rtgui_image_create_from_file("bmp", "/test_2.bmp", RT_FALSE);

		dc = rtgui_dc_begin_drawing(RTGUI_WIDGET(object));
		if (dc == RT_NULL)
        {
            return result;
        }

        rtgui_widget_get_rect(RTGUI_WIDGET(object), &rect);
 //       rtgui_widget_rect_to_device(widget, &rect);
        rect.x1 +=10;
        rect.y1 +=10;

		if (image != RT_NULL)
        {
			rtgui_image_blit(image, dc, &rect);
            rtgui_image_destroy(image);
        }
        else
        {
            rt_kprintf("APP err: no image found!\n");
        }

		rtgui_dc_end_drawing(dc);
	}

	return result;
}

static void app_oled(void *parameter)
{
    /* find lcd device */
    rt_device_t oled = rt_device_find(OLED_DEVICE_NAME);
    if (oled == RT_NULL)
    {
        rt_kprintf("Can't find OLED\n");
        return;
    }

    /* read OLED info */
    struct rt_device_graphic_info oled_info;
    oled->control(oled, RTGRAPHIC_CTRL_GET_INFO, (void *)&oled_info);
    rt_kprintf("OLED size: %dX%d\n", oled_info.width, oled_info.height);

	/* create app */
	struct rtgui_app *app;
	app = rtgui_app_create(rt_thread_self(), "oled_app");
	if (app == RT_NULL)
    {
        rt_kprintf("Create app \"oled_app\" failed!\n");
        return;
    }

	struct rtgui_rect rect;
    struct rtgui_win *win;
    struct rtgui_label* label;

	rtgui_graphic_driver_get_rect(rtgui_graphic_driver_get_default(), &rect);

    /* create window */
	win = rtgui_win_create(RT_NULL, "main",
                    &rect,
                    RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
	if (win == RT_NULL)
	{
        rt_kprintf("Create window \"main\" failed!\n");
		rtgui_app_destroy(app);
        return;
	}

    /* create lable in window */
	label = rtgui_label_create("¹þÂÞ,íïÅÖ!");
    if (label == RT_NULL)
    {
        rt_kprintf("Create lable failed!\n");
        return;
    }

	RTGUI_WIDGET_TEXTALIGN(RTGUI_WIDGET(label)) = RTGUI_ALIGN_LEFT;
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(label)) = black;
    RTGUI_WIDGET_FOREGROUND(RTGUI_WIDGET(label)) = white;

	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect);
    rtgui_container_add_child(RTGUI_CONTAINER(win), RTGUI_WIDGET(label));
    rtgui_object_set_event_handler(RTGUI_OBJECT(label), pic_view_event_handler);

    rtgui_win_show(win, RT_FALSE);

    rtgui_app_run(app);
    rtgui_app_destroy(app);
}
#elif defined(BOARD_USING_OLED)
static void app_oled(void *parameter)
{
    /* find lcd device */
    rt_device_t oled = rt_device_find(OLED_DEVICE_NAME);
    if (oled == RT_NULL)
    {
        rt_kprintf("Can't find OLED\n");
        return;
    }

    oled->init(oled);

    /* read OLED info */
    struct rt_device_graphic_info oled_info;
    oled->control(oled, RTGRAPHIC_CTRL_GET_INFO, (void *)&oled_info);
    rt_kprintf("OLED size: %dX%d\n", oled_info.width, oled_info.height);
}
#endif

void rt_demo_thread_entry(void* parameter)
{
#if defined(RT_USING_DFS)
    do
    {
        rt_kprintf("APP: File system DEMO start...\n");
        /* Filesystem Initialization */
        dfs_init();

 #if defined(RT_USING_DFS_ELMFAT)
        /* init the elm chan FatFs filesystam*/
        elm_init();

  #if defined(BOARD_USING_SPISD)
        /* mount sd card fat partition 1 as root directory */
        if (dfs_mount(SPISD_DEVICE_NAME, "/", "elm", 0, 0) == 0)
        {
            rt_kprintf("APP: FatFs init OK\n");
        }
        else
        {
            rt_kprintf("APP err: FatFs init failed!\n");
        }
  #endif
 #endif
        rt_kprintf("APP: File system DEMO end.\n");
    } while (0);
#endif

#if defined(BOARD_USING_OLED)
{
    rt_kprintf("OLED DEMO start...\n");

    /* Create RTGUI app thread */
    rt_thread_t gui_oled;
    gui_oled = rt_thread_create(
        "oled",
        app_oled,
        RT_NULL,
        2048,
        25,
        10);
    if (gui_oled != RT_NULL)
    {
        rt_thread_startup(gui_oled);
    }
    else
    {
        rt_kprintf("Create gui_oled thread failed!\n");
    }

    rt_kprintf("OLED DEMO end.\n");
}
#endif

    rt_kprintf("All Demo end.\n");

    while(1)
    {
        rt_thread_sleep(10);
    }
}

int rt_application_init()
{
    rt_thread_t demo_thread;

#if defined(BOARD_USING_SPISD)
    if (miniStm32_hw_spiSd_init() != RT_EOK)
    {
        rt_kprintf("INIT: Init SD card driver failed!");
        while(1); //Or do something?
    }
#endif

    demo_thread = rt_thread_create(
        "demo",
        rt_demo_thread_entry,
        RT_NULL,
        1024,
        TASK_PRIORITY_APPLICATION,
        20);

    if(demo_thread != RT_NULL)
    {
        rt_kprintf("demo sp:%x\n", demo_thread->sp);
        rt_thread_startup(demo_thread);
    }

    return 0;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
