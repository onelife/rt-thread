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

//#include "dev_led.h"
#if defined(EFM32_USING_SPISD)
#include "drv_sdcard.h"
#endif
#ifdef RT_USING_LWIP
#include <lwip/sys.h>
#include <lwip/api.h>
#include <netif/ethernetif.h>
#endif

#if (defined(MINISTM32_USING_LCD) || defined(MINISTM32_USING_OLED) || defined(MINISTM32_USING_DOU))
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
#define APP_CMD_PHOTO_FRAME 0x00000001
//#define APP_PHOTO_FRAME

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
#if defined(APP_PHOTO_FRAME)
struct photo_event
{
    struct rtgui_event_win win;
    rt_uint32_t cmd;
	rt_uint8_t* path;
	rt_uint8_t* format;
};
#endif

/* Private variables ---------------------------------------------------------*/
static rt_uint8_t led_stack[512];
static struct rt_thread led_thread;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void led_thread_entry(void* parameter)
{
    unsigned int count=0;

    rt_hw_led_init();

    while (1)
    {
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n",count);
#endif
        count++;
        rt_hw_led_on(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 );
    }
}

#if (defined(MINISTM32_USING_LCD) || defined(MINISTM32_USING_DOU))
#if defined(MINISTM32_USING_SPISD)
static rt_bool_t pic_view_event_handler(rtgui_object_t *object, rtgui_event_t *event)
{
	rt_bool_t result;
    rt_bool_t load = RT_FALSE;

	result = rtgui_container_event_handler(object, event);

    switch(event->type)
    {
    case RTGUI_EVENT_PAINT:
        load = RT_TRUE;
        break;

    case RTGUI_EVENT_MOUSE_BUTTON:
        {
			struct rtgui_event_mouse *mouse = (struct rtgui_event_mouse *)event;

			if (mouse->button == RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_UP)
			{
                rt_kprintf("APP: left click (%x)\n", mouse->button);
			}
        }
        break;
    }

    if (load)
	{
		struct rtgui_dc* dc;
		rtgui_rect_t rect;
        rtgui_image_t* image;

//        image = rtgui_image_create_from_file("jpg", "/test9.jpg", RT_FALSE);
        image = rtgui_image_create_from_file("bmp", "/test_565.bmp", RT_FALSE);

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
#endif

static void app_lcd(void *parameter)
{
    /* find lcd device */
    rt_device_t lcd = rt_device_find(MINISTM32_DISPLAY_NAME);
    if (lcd == RT_NULL)
    {
        rt_kprintf("Can't find display\n");
        return;
    }
    lcd->open(lcd, RT_DEVICE_FLAG_RDWR);

    /* read LCD info */
    struct rt_device_graphic_info lcd_info;
    lcd->control(lcd, RTGRAPHIC_CTRL_GET_INFO, (void *)&lcd_info);
    rt_kprintf("LCD size: %dX%d\n", lcd_info.width, lcd_info.height);

	/* create app */
	struct rtgui_app *app;
	app = rtgui_app_create(rt_thread_self(), "lcd_app");
	if (app == RT_NULL)
    {
        rt_kprintf("Create app \"lcd_app\" failed!\n");
        return;
    }

	struct rtgui_rect rect1, rect2, rect3;
    struct rtgui_win *win_info, *win_main, *win_hello;
    struct rtgui_label* label;

	rtgui_graphic_driver_get_rect(rtgui_graphic_driver_get_default(), &rect1);
    rect2.x1 = rect1.x1;
    rect2.y1 = 25;
    rect2.x2 = rect1.x2;
    rect2.y2 = rect1.y2;
    rect1.y2 = 25;

    /* create info window */
	win_info = rtgui_win_create(RT_NULL, "info",
                    &rect1,
                    RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
	if (win_info == RT_NULL)
	{
        rt_kprintf("Create window \"info\" failed!\n");
		rtgui_app_destroy(app);
        return;
	}

    /* create lable in info window */
	label = rtgui_label_create("RT-Thread & RTGUI");
    if (label == RT_NULL)
    {
        rt_kprintf("Create lable failed!\n");
        return;
    }

	RTGUI_WIDGET_TEXTALIGN(RTGUI_WIDGET(label)) = RTGUI_ALIGN_LEFT;
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(label)) = red;
    RTGUI_WIDGET_FOREGROUND(RTGUI_WIDGET(label)) = white;

	rect3.x1 = rect1.x1 + 5;
	rect3.y1 = rect1.y1 + 5;
	rect3.x2 = rect1.x2 - 5;
	rect3.y2 = rect1.y2 - 5;
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect3);
    rtgui_container_add_child(RTGUI_CONTAINER(win_info), RTGUI_WIDGET(label));


    /* create main window */
	win_main = rtgui_win_create(RT_NULL, "main",
                    &rect2,
                    RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
	if (win_main == RT_NULL)
	{
        rt_kprintf("Create window \"main\" failed!\n");
		rtgui_app_destroy(app);
        return;
	}

#if defined(MINISTM32_USING_SPISD)
    rtgui_object_set_event_handler(RTGUI_OBJECT(win_main), pic_view_event_handler);
#endif

    /* create lable in main window */
	label = rtgui_label_create("EFM32GG_DK3750 Kit");
    if (label == RT_NULL)
    {
        rt_kprintf("Create lable failed!\n");
        return;
    }
	RTGUI_WIDGET_TEXTALIGN(RTGUI_WIDGET(label)) = RTGUI_ALIGN_LEFT;
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(label)) = white;
    RTGUI_WIDGET_FOREGROUND(RTGUI_WIDGET(label)) = blue;

	rect3.x1 = rect2.x1 + 5;
	rect3.y1 = rect2.y1 + 5;
	rect3.x2 = rect2.x2 - 5;
	rect3.y2 = rect2.y1 + 20;
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect3);
//TESTING    rtgui_container_add_child(RTGUI_CONTAINER(win_main), RTGUI_WIDGET(label));

    //TESTING
    struct rtgui_container *container = rtgui_container_create();
    rtgui_widget_set_rect(RTGUI_WIDGET(container), &rect2);
    rtgui_container_add_child(RTGUI_CONTAINER(win_main), RTGUI_WIDGET(container));
    struct rtgui_dc *dc = rtgui_dc_begin_drawing(RTGUI_WIDGET(container));

    /* create hello window */
	rect3.x1 = 30;
	rect3.y1 = 50;
	rect3.x2 = 210;
	rect3.y2 = 150;
	win_hello = rtgui_win_create(RT_NULL, "hello",
                    &rect3,
                    RTGUI_WIN_STYLE_DEFAULT);
	if (win_hello == RT_NULL)
	{
        rt_kprintf("Create window \"hello\" failed!\n");
		rtgui_app_destroy(app);
        return;
	}

    /* create a box */
    rtgui_box_t *box = rtgui_box_create(RTGUI_VERTICAL, 0);
	if(box == RT_NULL)
    {
        rt_kprintf("Create box failed!\n");
        return;
    }
    rtgui_container_set_box(RTGUI_CONTAINER(win_hello), box);

    label = rtgui_label_create("¹þÂÞ,íïÅÖ!");
	if(label == RT_NULL)
    {
        rt_kprintf("Create lable failed!\n");
        return;
    }
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(label)) = white;
    RTGUI_WIDGET_FOREGROUND(RTGUI_WIDGET(label)) = black;
    RTGUI_WIDGET(label)->align = RTGUI_ALIGN_CENTER_HORIZONTAL | RTGUI_ALIGN_CENTER_VERTICAL;
    rtgui_widget_set_miniwidth(RTGUI_WIDGET(label),120);
    rtgui_container_add_child(RTGUI_CONTAINER(win_hello), RTGUI_WIDGET(label));
    /* Auto layout */
    rtgui_container_layout(RTGUI_CONTAINER(win_hello));


    rtgui_win_show(win_info, RT_FALSE);
    rtgui_win_show(win_main, RT_FALSE);
//TESTING    rtgui_win_show(win_hello, RT_FALSE);
    //TESTING
    rtgui_video_mjpg_test("/test.avi", dc);

    rtgui_app_run(app);
    rtgui_app_destroy(app);
}
#endif

#if defined(MINISTM32_USING_OLED)
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
#endif

#if defined(APP_PHOTO_FRAME)
static rt_bool_t photo_view_event_handler(rtgui_object_t *object, rtgui_event_t *event)
{
	rt_bool_t result = RT_FALSE;
    struct photo_event *photo_event = (struct photo_event *)event;

	result = rtgui_container_event_handler(object, event);
    rt_kprintf("container event %x\n", event->type);

	struct rtgui_event_win* wevent = (struct rtgui_event_win*)event;
    rt_kprintf("wevent->wid %x\n", wevent->wid);

    if ((event->type == RTGUI_EVENT_COMMAND) && \
        (photo_event->cmd == APP_CMD_PHOTO_FRAME))
	{
        rtgui_rect_t rect;
        rtgui_image_t* image;
        struct rtgui_dc* dc;

        rtgui_widget_get_rect(RTGUI_WIDGET(object), &rect);
        rt_kprintf(" (%d, %d) (%d, %d)\n", rect.x1, rect.y1, rect.x2, rect.y2);
//        rtgui_widget_rect_to_device(RTGUI_WIDGET(object), &rect);
        rect.y1 +=15;

        dc = rtgui_dc_begin_drawing(RTGUI_WIDGET(object));
        if (dc == RT_NULL)
        {
            return result;
        }

        image = rtgui_image_create_from_file(photo_event->format,
            photo_event->path, RT_TRUE);
        if (image != RT_NULL)
        {
            rtgui_image_blit(image, dc, &rect);
            rtgui_image_destroy(image);
            return result;
        }

        return RT_TRUE;
    }

	return result;
}

static rt_bool_t photo_lable_event_handler(rtgui_object_t *object, rtgui_event_t *event)
{
	rt_bool_t result = RT_FALSE;

	result = rtgui_label_event_handler(object, event);
    rt_kprintf("lable event %x\n", event->type);

    if (event->type == RTGUI_EVENT_COMMAND)
	{
        struct photo_event *photo = (struct photo_event *)event;

        rtgui_label_set_text((rtgui_label_t *)object, photo->path);
        rt_kprintf("path %s\n", photo->path);
    }

	return result;
}

static void app_photo(void *parameter)
{
    struct photo_event *event = (struct photo_event *)parameter;

    /* find lcd device */
    rt_device_t lcd = rt_device_find(MINISTM32_DISPLAY_NAME);
    if (lcd == RT_NULL)
    {
        rt_kprintf("Can't find display\n");
        return;
    }
    lcd->open(lcd, RT_DEVICE_FLAG_RDWR);

    /* read LCD info */
    struct rt_device_graphic_info lcd_info;
    lcd->control(lcd, RTGRAPHIC_CTRL_GET_INFO, (void *)&lcd_info);
    rt_kprintf("LCD size: %dX%d\n", lcd_info.width, lcd_info.height);

	/* create app */
	struct rtgui_app *app;
	app = rtgui_app_create(rt_thread_self(), "pho_app");
	if (app == RT_NULL)
    {
        rt_kprintf("Create app \"pho_app\" failed!\n");
        return;
    }

	struct rtgui_rect rect1, rect2;
    struct rtgui_win *window;
    struct rtgui_label* label;

	rtgui_graphic_driver_get_rect(rtgui_graphic_driver_get_default(), &rect1);

    /* create window */
	window = rtgui_win_create(RT_NULL, "photo",
                    &rect1,
                    RTGUI_WIN_STYLE_NO_BORDER | RTGUI_WIN_STYLE_NO_TITLE);
	if (window == RT_NULL)
	{
        rt_kprintf("Create window \"photo\" failed!\n");
		rtgui_app_destroy(app);
        return;
	}
    event->win.wid = window;
    rtgui_object_set_event_handler(RTGUI_OBJECT(window), photo_view_event_handler);

    /* create lable in window */
	label = rtgui_label_create("Photo Frame Demo");
    if (label == RT_NULL)
    {
        rt_kprintf("Create lable failed!\n");
        return;
    }

	RTGUI_WIDGET_TEXTALIGN(RTGUI_WIDGET(label)) = RTGUI_ALIGN_LEFT;
	RTGUI_WIDGET_BACKGROUND(RTGUI_WIDGET(label)) = white;
    RTGUI_WIDGET_FOREGROUND(RTGUI_WIDGET(label)) = blue;

    rect2.x1 = rect1.x1;
    rect2.y1 = rect1.y1;
    rect2.x2 = rect1.x2;
    rect2.y2 = 15;
	rtgui_widget_set_rect(RTGUI_WIDGET(label), &rect2);
    rtgui_object_set_event_handler(RTGUI_OBJECT(label), photo_lable_event_handler);
	rtgui_container_add_child(RTGUI_CONTAINER(window), RTGUI_WIDGET(label));

    rtgui_win_show(window, RT_FALSE);

    rtgui_app_run(app);
    rtgui_app_destroy(app);
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

  #if defined(MINISTM32_USING_SPISD)
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

#if defined(MINISTM32_USING_TOUCH)
    do
    {
        rt_kprintf("APP: Touch screen DEMO start...\n");

        rt_device_t touch = rt_device_find("touch");
        if (touch == RT_NULL)
        {
            rt_kprintf("APP err: Can't find touch screen\n");
            break;
        }
        touch->open(touch, RT_DEVICE_OFLAG_RDWR);
//        touch->control(touch, RTGRAPHIC_CTRL_TOUCH_CALIBRATION, RT_NULL);

        rt_kprintf("APP: Touch screen DEMO end.\n");
    } while (0);
#endif

#if ((defined(MINISTM32_USING_LCD) || defined(MINISTM32_USING_DOU)) && !defined(APP_PHOTO_FRAME))
{
    rt_kprintf("LCD DEMO start...\n");

    /* Create RTGUI app thread */
    rt_thread_t gui_lcd;
    gui_lcd = rt_thread_create(
        "lcd",
        app_lcd,
        RT_NULL,
        2048,
        25,
        10);
    if (gui_lcd != RT_NULL)
    {
        rt_thread_startup(gui_lcd);
    }
    else
    {
        rt_kprintf("Create gui_lcd thread failed!\n");
    }

    rt_kprintf("LCD DEMO end.\n");
}
#endif

#if defined(MINISTM32_USING_OLED)
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


#if defined(APP_PHOTO_FRAME)
{
    rt_kprintf("Photo frame DEMO start...\n");

    DIR* dir = opendir("/photo");
    struct photo_event event;
    struct dirent* dirent;
    rt_uint8_t path[100];
    const rt_uint8_t bmp[] = "bmp";
    const rt_uint8_t jpeg[] = "jpeg";

    RTGUI_EVENT_COMMAND_INIT(&event.win);
    event.cmd = APP_CMD_PHOTO_FRAME;
    event.path = path;

    /* Create photo frame thread */
    rt_thread_t photo_app;
    photo_app = rt_thread_create(
        "pho_thd",
        app_photo,
        (void *)&event,
        2048,
        25,
        10);
    if (photo_app != RT_NULL)
    {
        rt_thread_startup(photo_app);
    }
    else
    {
        rt_kprintf("Create photo frame thread failed!\n");
    }

    /* start display photos */
    rt_thread_sleep(100);
    do
    {
        /* get a photo */
        dirent = readdir(dir);
        if (dirent == RT_NULL)
        {
            break;
        }
        if ((strcmp(dirent->d_name, ".") == 0) || \
            (strcmp(dirent->d_name, "..") == 0))
        {
            continue;
        }
        rt_sprintf(path, "%s%c%s", "/photo", PATH_SEPARATOR, dirent->d_name);

        /* display it */
        if ((rt_strstr(path, ".bmp") != RT_NULL) || \
            (rt_strstr(path, ".BMP") != RT_NULL))
        {
            event.format = &bmp[0];
            rt_kprintf("bmp: %s\n", path);
        }
        else if ((rt_strstr(path, ".jpg") != RT_NULL) || \
            (rt_strstr(path, ".JPG") != RT_NULL))
        {
            event.format = &jpeg[0];
            rt_kprintf("jpeg: %s\n", path);
        }
        else
        {
            rt_kprintf("skip: %s\n", path);
            continue;
        }

        rtgui_send(photo_app, &event.win.parent, sizeof(event));
        rt_thread_sleep(2000);
    } while (dirent != RT_NULL);
    closedir(dir);

    rt_kprintf("Photo frame end.\n");
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

#if defined(MINISTM32_USING_SPISD)
    if (miniStm32_hw_spiSd_init() != RT_EOK)
    {
        rt_kprintf("INIT: Init SD card driver failed!");
        while(1); //Or do something?
    }
#endif

    /* init led thread */
/*	if (rt_thread_init(&led_thread,
		"led",
		led_thread_entry, RT_NULL,
		(rt_uint8_t*)&led_stack[0], sizeof(led_stack),
		20, 5) == RT_EOK)
	{
        rt_thread_startup(&led_thread);
	}
    else
    {
        rt_kprintf("INIT: LED thread init failed.\n");
    }*/

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
