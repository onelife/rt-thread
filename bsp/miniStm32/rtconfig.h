/***************************************************************************//**
 * @file    rtconfig.h
 * @brief   RT-Thread config file
 * 	COPYRIGHT (C) 2009, RT-Thread Development Team
 * @author
 * @version 0.4 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 ******************************************************************************/
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define RT_USING_BSP_CMSIS

/* RT_NAME_MAX*/
#define RT_NAME_MAX					(8)

/* RT_ALIGN_SIZE*/
#define RT_ALIGN_SIZE				(4)

/* PRIORITY_MAX */
#define RT_THREAD_PRIORITY_MAX		(32)

/* Tick per Second */
#define RT_TICK_PER_SECOND			(100)

/* SECTION: RT_DEBUG */
#define RT_DEBUG
//#define RT_DEBUG_MEM 				(1)
//#define RT_DEBUG_SCHEDULER 			(1)
//#define RT_DEBUG_IPC 				(1)
#define THREAD_DEBUG
//#define IRQ_DEBUG
#define RT_USING_OVERFLOW_CHECK
//#define DFS_DEBUG
//#define RT_LWIP_DEBUG

//#define RT_IRQHDL_DEBUG
//#define RT_IIC_DEBUG
//#define RT_MISC_DEBUG
//#define RT_ADC_DEBUG
//#define RT_ACMP_DEBUG
//#define RT_TIMER_DEBUG

//#define EFM32_DEBUG
//#define MINISTM32_USART_DEBUG
//#define MINISTM32_SPI_DEBUG
//#define MINISTM32_GPIO_SPI_DEBUG
//#define MINISTM32_GPIO_SCCB_DEBUG
#define MINISTM32_RTC_DEBUG
//#define MINISTM32_SDCARD_DEBUG
#define MINISTM32_LCD_DEBUG
#define MINISTM32_OLED_DEBUG
#define MINISTM32_TOUCH_DEBUG
#define MINISTM32_IR_DEBUG
//#define EFM32_ETHERNET_DEBUG
//#define EFM32_KEYS_DEBUG
#define MINISTM32_CAMERA_DEBUG
#define MINISTM32_USB_DEBUG
#define MINISTM32_DOU_DEBUG

/* Using Hook */
//#define RT_USING_HOOK

/* Using Software Timer */
/* #define RT_USING_TIMER_SOFT */
#define RT_TIMER_THREAD_PRIO		    (4)
#define RT_TIMER_THREAD_STACK_SIZE	    (512)
#define RT_TIMER_TICK_PER_SECOND	    (10)

/* SECTION: IPC */
/* Using Semaphore*/
#define RT_USING_SEMAPHORE              /* Required by DFS and lwIP */
/* Using Mutex */
#define RT_USING_MUTEX                  /* Required by DFS */
/* Using Event */
#define RT_USING_EVENT
/* Using MailBox */
#define RT_USING_MAILBOX                /* Required by lwIP */
/* Using Message Queue */
#define RT_USING_MESSAGEQUEUE           /* Required by RTGUI */
/* SECTION: Memory Management */
/* Using Memory Pool Management*/
#define RT_USING_MEMPOOL
/* Using Dynamic Heap Management */
#define RT_USING_HEAP
/* Using Small MM */
#define RT_USING_SMALL_MEM


/* SECTION: Device Firmware Upgrade */
#define MINISTM32_USING_DFU


/* SECTION: Device System */
/* Using Device System */
#define RT_USING_DEVICE


/* SECTION: MiniSTM32 Board */
/* Note: Select one from touch screen and IR */
#define MINISTM32_USING_RTC
#define MINISTM32_USING_USART1
#define MINISTM32_USING_SPISD           /* MicroSD card */
#define MINISTM32_USING_LCD             /* TFT LCD (SSD1289) */
//#define MINISTM32_USING_OLED            /* OLED (SSD1306) */
//#define MINISTM32_USING_TOUCH           /* Touch screen */
//#define MINISTM32_USING_CAMERA          /* Camera */
//#define MINISTM32_USING_IR              /* IR remote control */
//#define EFM32_USING_KEYS                    /* Keys and joystick */
//#define MINISTM32_USING_USB
//#define MINISTM32_USING_USB_VIRTUAL_COM
//#define MINISTM32_USING_USB_HID_MOUSE
//#define MINISTM32_USING_DOU             /* Virtual display with mouse input */

#define RTGUI_VIDEO_MJPEG

#if defined(MINISTM32_USING_SPISD)
#define MINISTM32_USING_SPI1
#endif
#if defined(MINISTM32_USING_TOUCH)
#define MINISTM32_USING_GPIO_SPI1
#endif
#if defined(MINISTM32_USING_CAMERA)
#define MINISTM32_USING_GPIO_SCCB
#endif
#if defined(MINISTM32_USING_DOU)
#define MINISTM32_USING_USART3
#endif


/* USART options */
#define MINISTM32_USART_DIRECT_EXE      (1 << 0)    /* Directly execute */
#define MINISTM32_USART_9BIT            (1 << 1)    /* Word length */
#define MINISTM32_USART_REMAP(num)      (num << 2)  /* Location number */
#define MINISTM32_USART_DMA_TX          (1 << 4)    /* DMA TX */
#define MINISTM32_USART_INT_RX          (1 << 5)    /* INT RX */
#define MINISTM32_USART_CONSOLE         (1 << 6)    /* Console device */


/* SECTION: USART */
#if defined(MINISTM32_USING_USART1)
#define USART1_NAME                     "usart1"
#define USART1_USART_MODE               (MINISTM32_USART_DIRECT_EXE | \
                                        MINISTM32_USART_REMAP(0) | \
                                        MINISTM32_USART_INT_RX | \
                                        MINISTM32_USART_CONSOLE)
#define USART1_FREQUENCY                (115200)
#endif

#if defined(MINISTM32_USING_USART3)
#define USART3_NAME                     "usart3"
#define USART3_USART_MODE               (MINISTM32_USART_REMAP(0) | \
                                        MINISTM32_USART_INT_RX)
#define USART3_FREQUENCY                (57600)
#endif


/* SPI options */
#define MINISTM32_SPI_DIRECT_EXE        (1 << 0)    /* Directly execute */
#define MINISTM32_SPI_MASTER            (1 << 1)    /* Master mode */
#define MINISTM32_SPI_AUTOCS            (1 << 2)    /* Auto chip select */
/*
    0: Clock idle low, sample on rising edge
    1: Clock idle low, sample on falling edge
    2: Clock idle high, sample on falling edge
    3: Clock idle high, sample on rising edge.
*/
#define MINISTM32_SPI_CLK_MODE(mode)    (mode << 3) /* Clock mode */
#define MINISTM32_SPI_REMAP(num)        (num << 5)  /* Pin remap */
#define MINISTM32_SPI_DMA_TX            (1 << 7)    /* DMA TX */
#define MINISTM32_SPI_DMA_RX            (1 << 8)    /* DMA RX */

/* SECTION: SPI */
#if defined(MINISTM32_USING_SPI1)
#define SPI1_NAME                       "spi1"
#define SPI1_SPI_MODE                   (MINISTM32_SPI_DIRECT_EXE | \
                                        MINISTM32_SPI_MASTER | \
                                        MINISTM32_SPI_DMA_TX | \
                                        MINISTM32_SPI_REMAP(0) | \
                                        MINISTM32_SPI_CLK_MODE(3))
#endif

/* SECTION: GPIO SPI */
#if defined(MINISTM32_USING_GPIO_SPI1)
#define GPIO_SPI1_NAME                  "io_spi1"
#define GPIO_SPI1_SPI_MODE              (MINISTM32_SPI_MASTER | MINISTM32_SPI_CLK_MODE(0))
#define GPIO_SPI1_FREQUENCY             (1000000)   /* Max 1MHz */
#define GPIO_SPI1_SCK_CLOCK             (RCC_APB2Periph_GPIOC)
#define GPIO_SPI1_SCK_PORT              (GPIOC)
#define GPIO_SPI1_SCK_PIN               (GPIO_Pin_0)
#define GPIO_SPI1_MOSI_CLOCK            (RCC_APB2Periph_GPIOC)
#define GPIO_SPI1_MOSI_PORT             (GPIOC)
#define GPIO_SPI1_MOSI_PIN              (GPIO_Pin_3)
#define GPIO_SPI1_MISO_CLOCK            (RCC_APB2Periph_GPIOC)
#define GPIO_SPI1_MISO_PORT             (GPIOC)
#define GPIO_SPI1_MISO_PIN              (GPIO_Pin_2)
#define GPIO_SPI1_CS_CLOCK              (RCC_APB2Periph_GPIOC)
#define GPIO_SPI1_CS_PORT               (GPIOC)
#define GPIO_SPI1_CS_PIN                (GPIO_Pin_13)
#endif


/* SCCB options */
#define MINISTM32_SCCB_MASTER           (1 << 0)    /* Master mode */

/* SECTION: GPIO SCCB */
#if defined(MINISTM32_USING_GPIO_SCCB)
#define GPIO_SCCB_NAME                  "io_sccb"
#define GPIO_SCCB_SCCB_MODE             (MINISTM32_SCCB_MASTER)
#define GPIO_SCCB_FREQUENCY             (1000000)    /* Max 1MHz */
#define GPIO_SCCB_SCL_CLOCK             (RCC_APB2Periph_GPIOC)
#define GPIO_SCCB_SCL_PORT              (GPIOC)
#define GPIO_SCCB_SCL_PIN               (GPIO_Pin_11)
#define GPIO_SCCB_SDA_CLOCK             (RCC_APB2Periph_GPIOC)
#define GPIO_SCCB_SDA_PORT              (GPIOC)
#define GPIO_SCCB_SDA_PIN               (GPIO_Pin_12)
#endif


/* SECTION: USB */
#if defined(MINISTM32_USING_USB)
#define USB_NAME                        "usb"
#endif


/* SECTION: RTC */
#if defined(MINISTM32_USING_RTC)
#define RTC_NAME 				        "clock"
#endif


/* SECTION: Runtime library */
// #define RT_USING_NOLIBC
// #define RT_USING_NEWLIB


/* SECTION: Console options */
#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE			    (128)

/* SECTION: finsh, a C-Express shell */
#define RT_USING_FINSH
/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION


/* SECTION: RTGUI support */
#if defined(MINISTM32_USING_LCD)
#define LCD_DEVICE_NAME                 "lcd"
#define MINISTM32_DISPLAY_NAME          LCD_DEVICE_NAME
#endif

#if defined(MINISTM32_USING_OLED)
#define OLED_DEVICE_NAME                "oled"
#define MINISTM32_DISPLAY_NAME          OLED_DEVICE_NAME
#endif

#if (defined(MINISTM32_USING_LCD) || defined(MINISTM32_USING_OLED) || defined(MINISTM32_USING_DOU))
/* using RTGUI support */
#define RT_USING_RTGUI

/* name length of RTGUI object */
#define RTGUI_NAME_MAX                  (16)
/* support 16 weight font */
#define RTGUI_USING_FONT16
/* support 12 weight font */
#define RTGUI_USING_FONT12
/* support Chinese font */
#define RTGUI_USING_FONTHZ
/* use DFS as file interface */
#define RTGUI_USING_DFS_FILERW
/* use font file as Chinese font */
/* #define RTGUI_USING_HZ_FILE */
/* use Chinese bitmap font */
#define RTGUI_USING_HZ_BMP
/* use small size in RTGUI */
/* #define RTGUI_USING_SMALL_SIZE */
/* use mouse cursor */
/* #define RTGUI_USING_MOUSE_CURSOR */
/* default font size in RTGUI */
//#define RTGUI_DEFAULT_FONT_SIZE		(16)
/* RTGUI image options */
//#define RTGUI_IMAGE_XPM
//#define RTGUI_IMAGE_JPEG
#define RTGUI_IMAGE_TJPGD
//#define RTGUI_IMAGE_PNG
#define RTGUI_IMAGE_BMP
#endif /* defined(MINISTM32_USING_LCD) || defined(MINISTM32_USING_OLED) || defined(MINISTM32_USING_DOU) */

#if defined(MINISTM32_USING_TOUCH)
#define TOUCH_USING_DEVICE_NAME         GPIO_SPI1_NAME
#define TOUCH_DEVICE_NAME               "touch"
#endif /* defined(MINISTM32_USING_TOUCH) */

#if defined(MINISTM32_USING_DOU)
#define DOU_USING_DEVICE_NAME           USART3_NAME
#define DOU_DEVICE_NAME                 "dou"
#define MINISTM32_DISPLAY_NAME          DOU_DEVICE_NAME
#endif /* defined(MINISTM32_USING_DOU) */

#if defined(MINISTM32_USING_CAMERA)
#define CAMERA_USING_DEVICE_NAME        GPIO_SCCB_NAME
#define CAMERA_DEVICE_NAME              "camera"
#define CAMERA_USING_DISPLAY_NAME       MINISTM32_DISPLAY_NAME
#endif /* defined(MINISTM32_USING_CAMERA) */


/* SECTION: device filesystem */
#if (defined(RT_USING_NEWLIB) || defined(MINISTM32_USING_SPISD) || defined(RT_USING_RTGUI))
#define RT_USING_DFS
/* the max number of mounted filesystem */
#define DFS_FILESYSTEMS_MAX             (2)
/* the max number of opened files 		*/
#define DFS_FD_MAX                      (4)
/* the max number of cached sector 		*/
#define DFS_CACHE_MAX_NUM               (4)
#endif /* defined(RT_USING_NEWLIB) || defined(MINISTM32_USING_SPISD) */
#if defined(MINISTM32_USING_SPISD)
#define SPISD_USING_DEVICE_NAME         SPI1_NAME
#define SPISD_DEVICE_NAME               "spiSd"

#define RT_USING_DFS_ELMFAT
#define DFS_ELMFAT_INTERFACE_EFM
#endif /* defined(MINISTM32_USING_SPISD) */
#if defined(RT_USING_NEWLIB)
//#define RT_USING_DFS_DEVFS
#endif /* defined(RT_USING_NEWLIB) */

//#define RT_DFS_ELM_WORD_ACCESS
/* Reentrancy (thread safe) of the FatFs module.  */
//#define RT_DFS_ELM_REENTRANT
/* Number of volumes (logical drives) to be used. */
//#define RT_DFS_ELM_DRIVES			(2)
/* #define RT_DFS_ELM_USE_LFN			(1) */
//#define RT_DFS_ELM_MAX_LFN			(255)
/* Maximum sector size to be handled. */
//#define RT_DFS_ELM_MAX_SECTOR_SIZE  (512)


/* SECTION: LWIP */
#if defined(EFM32_USING_ETHERNET)
//#define RT_USING_LWIP
//#define RT_USING_NETUTILS
//#define RT_LWIP_DHCP
/* LwIP uses RT-Thread Memory Management */
#define RT_LWIP_USING_RT_MEM
/* Enable ICMP protocol*/
#define RT_LWIP_ICMP
/* Enable ICMP protocol*/
//#define RT_LWIP_IGMP
/* Enable UDP protocol*/
#define RT_LWIP_UDP
/* Enable TCP protocol*/
#define RT_LWIP_TCP
/* Enable DHCP */
//#define RT_LWIP_DHCP
/* Enable DNS */
//#define RT_LWIP_DNS

/* the number of simulatenously active TCP connections*/
#define RT_LWIP_TCP_PCB_NUM			    (5)

/* Using DHCP */
/* #define RT_LWIP_DHCP */

/* ip address of target*/
#define RT_LWIP_IPADDR0				    (192)
#define RT_LWIP_IPADDR1				    (168)
#define RT_LWIP_IPADDR2				    (1)
#define RT_LWIP_IPADDR3				    (118)
/* gateway address of target*/
#define RT_LWIP_GWADDR0				    (192)
#define RT_LWIP_GWADDR1				    (168)
#define RT_LWIP_GWADDR2				    (1)
#define RT_LWIP_GWADDR3				    (1)
/* mask address of target*/
#define RT_LWIP_MSKADDR0 			    (255)
#define RT_LWIP_MSKADDR1 			    (255)
#define RT_LWIP_MSKADDR2 			    (255)
#define RT_LWIP_MSKADDR3 			    (0)

/* tcp thread options */
#define RT_LWIP_TCPTHREAD_PRIORITY	    (12)
#define RT_LWIP_TCPTHREAD_MBOX_SIZE	    (10)
#define RT_LWIP_TCPTHREAD_STACKSIZE	    (1024)
/* ethernet if thread options */
#define RT_LWIP_ETHTHREAD_PRIORITY 	    (15)
#define RT_LWIP_ETHTHREAD_MBOX_SIZE	    (10)
#define RT_LWIP_ETHTHREAD_STACKSIZE	    (512)
#endif /* defined(EFM32_USING_ETHERNET) */


/* Exported functions ------------------------------------------------------- */

#endif /* __RTTHREAD_CFG_H__ */
