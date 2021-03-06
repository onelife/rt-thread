#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 1024
#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_SMALL_MEM
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "vcom"
#define RT_VER_NUM 0x40002
#define ARCH_ARM
#define RT_USING_CPU_FFS
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M4

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_USING_MSH_ONLY
#define FINSH_ARG_MAX 10

/* Device virtual file system */

#define RT_USING_DFS
#define DFS_USING_WORKDIR
#define DFS_FILESYSTEMS_MAX 2
#define DFS_FILESYSTEM_TYPES_MAX 2
#define DFS_FD_MAX 16
#define RT_USING_DFS_ELMFAT
#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 4096
#define RT_DFS_ELM_REENTRANT

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 64
#define RT_USING_SPI
#define RT_USING_SFUD
#define RT_SFUD_USING_SFDP
// #define RT_SFUD_USING_FLASH_INFO_TABLE
#define RT_USING_SDIO
#define RT_USING_PIN
#define RT_USING_RTC

/* Using Hardware Crypto drivers */


/* Using WiFi */


/* Using USB */

#define RT_USING_USB_DEVICE
#define RT_USBD_THREAD_STACK_SZ 4096
// #define USB_VENDOR_ID 0x0FFE
// #define USB_PRODUCT_ID 0x0001
#define RT_USB_DEVICE_CDC
#define RT_VCOM_TX_USE_DMA

/* POSIX layer and C standard library */


/* Network */

/* Socket abstraction layer */


/* Network interface device */

#define RT_USING_NETDEV
#define NETDEV_USING_IFCONFIG
#define NETDEV_USING_PING
#define NETDEV_USING_NETSTAT
#define NETDEV_USING_AUTO_DEFAULT
#define NETDEV_IPV4 1

/* light weight TCP/IP stack */

#define RT_USING_LWIP
#define RT_USING_LWIP210
#define RT_LWIP_UDP
#define RT_LWIP_TCP
#define RT_LWIP_RAW
#define RT_LWIP_ICMP
#define RT_LWIP_DNS
#define RT_LWIP_DHCP
#define RT_LWIP_IPERF
#define IP_SOF_BROADCAST 1
#define IP_SOF_BROADCAST_RECV 1
#define RT_LWIP_MEM_USE_POOLS
#define RT_MEMP_NUM_NETCONN 8
#define RT_LWIP_PBUF_NUM 64
#define RT_LWIP_RAW_PCB_NUM 4
#define RT_LWIP_UDP_PCB_NUM 4
#define RT_LWIP_TCP_PCB_NUM 4
// #define RT_LWIP_TCP_SEG_NUM xxx
#define RT_LWIP_TCP_WND (1460 * 16)
#define RT_LWIP_TCP_SND_BUF (1460 * 16)
#define RT_LWIP_TCPTHREAD_PRIORITY 10
#define RT_LWIP_TCPTHREAD_MBOX_SIZE 8
#define RT_LWIP_TCPTHREAD_STACKSIZE 2048
// #define RT_LWIP_ETHTHREAD_PRIORITY 12
// #define RT_LWIP_ETHTHREAD_STACKSIZE 2048
// #define RT_LWIP_ETHTHREAD_MBOX_SIZE 8
#define RT_LWIP_REASSEMBLY_FRAG
#define RT_LWIP_USING_HW_CHECKSUM
#define RT_LWIP_USING_PING
#define LWIP_NETIF_STATUS_CALLBACK 1
#define LWIP_NETIF_LINK_CALLBACK 1
#define LWIP_WND_SCALE 1
#define TCP_RCV_SCALE 1
#define TCP_OVERSIZE 1
#define LWIP_NETIF_TX_SINGLE_PBUF 1
#define LWIP_SUPPORT_CUSTOM_PBUF 1
#define MEMP_NUM_NETBUF 32  // for socket
#define MEMP_NUM_TCPIP_MSG_INPKT 32
#define MEMP_NUM_TCPIP_MSG_API 32  // for socket
#define MEMP_NUM_SYS_TIMEOUT 32    // for socket (16+)
#define SO_REUSE 1
#define LWIP_SO_RCVTIMEO 1
#define LWIP_SO_SNDTIMEO 1
#define LWIP_SO_RCVBUF 1
#define LWIP_NO_RX_THREAD
#define LWIP_NO_TX_THREAD
// #define RT_LWIP_DEBUG
// #define RT_LWIP_TCP_DEBUG
// #define RT_LWIP_TCP_INPUT_DEBUG

/* Modbus master and slave stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */

#define RT_USING_ULOG
#define ULOG_OUTPUT_LVL LOG_LVL_INFO /*LOG_LVL_DBG*/
#define ULOG_ASSERT_ENABLE
#define ULOG_USING_COLOR
#define ULOG_OUTPUT_TIME
#define ULOG_OUTPUT_LEVEL
#define ULOG_OUTPUT_TAG
#define ULOG_OUTPUT_THREAD_NAME
#define ULOG_BACKEND_USING_CONSOLE

/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */


/* multimedia packages */


/* tools packages */


/* system packages */


/* peripheral libraries and drivers */


/* miscellaneous packages */


/* samples: kernel and components samples */

#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32F4

/* Hardware Drivers Config */

#define SOC_STM32F407ZG

/* Onboard Peripheral Drivers */


/* On-chip Peripheral Drivers */

#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART3
#define BSP_USING_USBD
// #define BSP_USING_SRAM
#define BSP_USING_EXT_FMC_IO
#define BSP_USING_FMC
// #define BSP_USING_SPI_FLASH
// #define BSP_USING_SPI
#define BSP_USING_SPI2
#define BSP_SPI2_TX_USING_DMA
#define BSP_SPI2_RX_USING_DMA
// #define BSP_USING_SDCARD
#define BSP_USING_SDIO
#define SDIO_MAX_FREQ 12000000
#define BSP_USING_ETH
#define BSP_USING_ETH_0COPY
#define PHY_USING_LAN8720A
#define RT_PHY_ADDR 0
#define BSP_USING_ONCHIP_RTC

/* Board extended module Drivers */


#endif
