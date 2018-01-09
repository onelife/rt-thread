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
//#define BOARD_LED_DEBUG
//#define MINISTM32_USART_DEBUG
//#define MINISTM32_SPI_DEBUG
#define BOARD_SDIO_DEBUG
#define MINISTM32_RTC_DEBUG
//#define MINISTM32_SDCARD_DEBUG
#define BOARD_SDIOSD_DEBUG
//#define MINISTM32_OLED_DEBUG
//#define EFM32_ETHERNET_DEBUG
//#define EFM32_KEYS_DEBUG
#define BOARD_USB_DEBUG

/* Using Hook */
//#define RT_USING_HOOK


/* SECTION: System Timer */
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


/* SECTION: Device System */
/* Using Device System */
#define RT_USING_DEVICE


/* SECTION: blackCoreStm32 Board */
/* Note: Select one from touch screen and IR */
// #define BOARD_USING_LED1
// #define BOARD_USING_LED2
#define MINISTM32_USING_RTC
#define BOARD_USING_USART1
// #define BOARD_USING_SPISD               /* MicroSD card */
// #define BOARD_USING_SDIOSD          	/* MicroSD card */
// #define BOARD_USING_OLED                /* OLED (SSD1306) */
// #define BOARD_USING_USB
// #define BOARD_USING_USB_VIRTUAL_COM


#if (defined(BOARD_USING_OLED) || defined(BOARD_USING_SPISD))
#define MINISTM32_USING_SPI2
#endif

#if defined(BOARD_USING_SPISD)
#define SD_INTERFACE_SPI
#elif defined(BOARD_USING_SDIOSD)
#define BOARD_USING_SDIO
#define SD_INTERFACE_SDIO
#endif


/* USART options */
#define MINISTM32_USART_DIRECT_EXE      (1 << 0)    /* Directly execute */
#define MINISTM32_USART_9BIT            (1 << 1)    /* Word length */
#define MINISTM32_USART_REMAP(num)      (num << 2)  /* Location number */
#define MINISTM32_USART_DMA_TX          (1 << 4)    /* DMA TX */
#define MINISTM32_USART_INT_RX          (1 << 5)    /* INT RX */
#define BOARD_USART_CONSOLE         	(1 << 6)    /* Console device */


/* SECTION: USART */
#if defined(BOARD_USING_USART1)
#define USART1_NAME                     "usart1"
#define USART1_USART_MODE               (MINISTM32_USART_DIRECT_EXE | \
                                        MINISTM32_USART_REMAP(0) | \
                                        MINISTM32_USART_INT_RX | \
                                        BOARD_USART_CONSOLE)
#define USART1_FREQUENCY                (115200)
#endif

#if defined(BOARD_USING_USART3)
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
#if defined(MINISTM32_USING_SPI2)
#define SPI2_NAME                       "spi2"
#define SPI2_SPI_MODE                   (MINISTM32_SPI_DIRECT_EXE | \
                                        MINISTM32_SPI_MASTER | \
                                        MINISTM32_SPI_DMA_TX | \
                                        MINISTM32_SPI_REMAP(0) | \
                                        MINISTM32_SPI_CLK_MODE(0))
#endif

/* SECTION: SDIO */
#if defined(BOARD_USING_SDIO)
#define SDIO_NAME                       "sdio"
#endif

/* SECTION: LED */
#if defined(BOARD_USING_LED1)
#define LED1_NAME                  		"led1"
#endif
#if defined(BOARD_USING_LED2)
#define LED2_NAME                  		"led2"
#endif


/* SECTION: RTC */
#if defined(MINISTM32_USING_RTC)
#define RTC_NAME 				        "clock"
#endif


/* SECTION: USB */
#if defined(BOARD_USING_USB)
#define USB_NAME                        "usb"
#endif


/* SECTION: LUA */
#define RT_USING_LUA
#if defined(RT_USING_LUA)
#define FINSH_THREAD_STACK_SIZE         (2048)
//#define LUA_PACK_VALUE
//#define ELUA_ENDIAN_LITTLE
#define RT_LUA_OPTRAM
#endif

/* SECTION: Runtime library */
// #define RT_USING_NOLIBC
#if defined(RT_USING_LUA)
#define RT_USING_LIBC
#endif


/* SECTION: Console options */
#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE			    (128)

/* SECTION: finsh, a C-Express shell */
#define RT_USING_FINSH
/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION


/* OLED options */
//#define INTERFACE_IIC                   (1 << 0)    /* IIC */
//#define INTERFACE_3WIRE_SPI             (1 << 1)    /* 3-wire SPI */
#define INTERFACE_4WIRE_SPI             (1 << 3)    /* 4-wire SPI */
//#define INTERFACE_8BIT_68XX             (1 << 4)    /* 8-bit 68xx parallel */
#define INTERFACE_8BIT_80XX             (1 << 5)    /* 8-bit 80xx parallel */

/* SECTION: OLED */
#if defined(BOARD_USING_OLED)
#define OLED_USING_DEVICE_NAME          SPI2_NAME
#define OLED_DEVICE_NAME                "oled"
#define OLED_DEVICE_INTERFACE           INTERFACE_4WIRE_SPI
#define BOARD_DISPLAY_NAME              OLED_DEVICE_NAME
#endif

/* SECTION: RTGUI support */
#if defined(BOARD_USING_OLED)
/* using RTGUI support */
#define RT_USING_RTGUI

/* name length of RTGUI object */
#define RTGUI_NAME_MAX                  (16)
/* support 16 weight font */
//#define RTGUI_USING_FONT16
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
//#define RTGUI_IMAGE_TJPGD
//#define RTGUI_IMAGE_PNG
//#define RTGUI_IMAGE_BMP
#endif /* defined(BOARD_USING_OLED) */


/* SECTION: device filesystem */
#if (defined(BOARD_USING_SPISD) || defined(RT_USING_RTGUI))
#define RT_USING_DFS
/* the max number of mounted filesystem */
#define DFS_FILESYSTEMS_MAX             (2)
/* the max number of opened files 		*/
#define DFS_FD_MAX                      (4)
/* the max number of cached sector 		*/
#define DFS_CACHE_MAX_NUM               (4)
#endif /* defined(BOARD_USING_SPISD) || defined(RT_USING_RTGUI) */

#if defined(BOARD_USING_SPISD) || defined(BOARD_USING_SDIOSD)
#if defined(BOARD_USING_SPISD)
#define SPISD_USING_DEVICE_NAME         SPI2_NAME
#define SPISD_DEVICE_NAME               "spiSd"
#endif

#if defined(BOARD_USING_SDIOSD)
#define SDIOSD_USING_DEVICE_NAME        SDIO_NAME
#define SDIOSD_DEVICE_NAME              "sdioSd"
#endif

#define RT_USING_DFS_ELMFAT
#define DFS_ELMFAT_INTERFACE_EFM
#endif /* defined(BOARD_USING_SPISD) || defined(BOARD_USING_SDIOSD) */

#if defined(RT_USING_NEWLIB)
//#define RT_USING_DFS_DEVFS
#endif /* defined(RT_USING_NEWLIB) */

#if defined(RT_USING_DFS)
//#define RT_DFS_ELM_WORD_ACCESS
/* Reentrancy (thread safe) of the FatFs module.  */
//#define RT_DFS_ELM_REENTRANT
/* Number of volumes (logical drives) to be used. */
//#define RT_DFS_ELM_DRIVES			(2)
/* #define RT_DFS_ELM_USE_LFN			(1) */
//#define RT_DFS_ELM_MAX_LFN			(255)
/* Maximum sector size to be handled. */
#define RT_DFS_ELM_MAX_SECTOR_SIZE      (2048)  // 512: 0~2GB
#endif /* defined(RT_USING_DFS) */


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
