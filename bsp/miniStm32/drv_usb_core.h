/***************************************************************************//**
 * @file    drv_usb_core.h
 * @brief   USB UART driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2011, RT-Thread Development Team
 * @author  onelife
 * @version 0.4 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-02-24   onelife     Initial creation of USB UART module driver for
 *  MiniSTM32
s ******************************************************************************/
#ifndef __DRV_USB_CORE_H__
#define __DRV_USB_CORE_H__

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_conf.h"
#include "drv_usb_desc.h"

/* Exported defines ----------------------------------------------------------*/
#define bool    rt_bool_t
#define TRUE    RT_TRUE
#define FALSE   RT_FALSE
#define NULL    RT_NULL

#define USB_ENDPOINT_BUFFER_SIZE            (512)


#define USB_TASK_STACK_SIZE                 (512)
#define USB_RX_MESSAGE_SIZE                 (2)
#define USB_RX_MESSAGE_QUEUE_SIZE           (10)
#define USB_DMA_QUEUE_SIZE                  (5)

#define USB_WAIT_TIME_TX                    (RT_TICK_PER_SECOND / 100 * 3)
#define USB_RETRY_TIMES_RX                  (10)

/* Status options */
#define USB_STATUS_START                    (1 << 0)
#define USB_STATUS_READ_ONLY                (1 << 1)
#define USB_STATUS_WRITE_ONLY               (1 << 2)


#define USB_DEVICE_DESCRIPTOR_TYPE              (0x01)
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       (0x02)
#define USB_STRING_DESCRIPTOR_TYPE              (0x03)
#define USB_INTERFACE_DESCRIPTOR_TYPE           (0x04)
#define USB_ENDPOINT_DESCRIPTOR_TYPE            (0x05)

/* Exported types ------------------------------------------------------------*/
typedef enum _DEVICE_STATE
{
    UNCONNECTED,
    ATTACHED,
    POWERED,
    SUSPENDED,
    ADDRESSED,
    CONFIGURED
} DEVICE_STATE;

typedef enum _RESUME_STATE
{
    RESUME_EXTERNAL,
    RESUME_INTERNAL,
    RESUME_LATER,
    RESUME_WAIT,
    RESUME_START,
    RESUME_ON,
    RESUME_OFF,
    RESUME_ESOF
} RESUME_STATE;

typedef struct usb_device_property
{
    rt_bool_t (*Process_Status_IN)(void);
    rt_bool_t (*Process_Status_OUT)(void);
    RESULT (*Class_Data_Setup)(uint8_t RequestNo);
    RESULT (*Class_NoData_Setup)(uint8_t RequestNo);
} usb_device_property_t;

typedef struct usb_property
{
    rt_uint8_t              table;
    const usb_device_property_t *preperty[DEVICE_NUMBER];
} usb_property_t;

typedef struct usb_endpoint_register
{
    rt_uint8_t              num;
    rt_uint16_t             type;
    rt_uint16_t             inBufSize;
    rt_uint16_t             outBufSize;
    void (*inFunc)(void);
    void (*outFunc)(void);
} usb_endpoint_register_t;

struct miniStm32_usb_buffer
{
    rt_uint8_t              size;
    rt_uint16_t             addr;
};

struct miniStm32_usb_device
{
    volatile rt_uint8_t     bIntPackSOF;        /* SOFs received between 2 consecutive packets */
    volatile rt_uint16_t    wIstr;              /* ISTR register last read value */
    volatile rt_uint32_t    bDeviceState;       /* USB device status */
    volatile rt_bool_t      fSuspendEnabled;    /* true when suspend is possible */

    rt_uint16_t             epType[ENDPOINT_NUMBER - 1];
    struct miniStm32_usb_buffer epInBuf[ENDPOINT_NUMBER - 1];
    struct miniStm32_usb_buffer epOutBuf[ENDPOINT_NUMBER - 1];
    usb_property_t          property;
    volatile rt_uint16_t    status;
    struct rt_semaphore     lock;
};

struct miniStm32_usb_cmd_message
{
    rt_uint32_t             cmd;
    rt_uint32_t             other;
    rt_size_t               size;
    rt_uint8_t              *ptr;
};

struct miniStm32_usb_ret_message
{
    rt_uint32_t             cmd;
    rt_uint32_t             other;
    rt_size_t               size;
    rt_err_t                ret;
};

union miniStm32_usb_exec_message
{
    struct miniStm32_usb_cmd_message cmd;
    struct miniStm32_usb_ret_message ret;
};

struct miniStm32_usb_task
{
    struct rt_thread        thread;
    struct rt_messagequeue  rx_msgs;
    struct rt_event         tx_evts;
    rt_uint8_t              stack[USB_TASK_STACK_SIZE];
    rt_uint8_t              rx_msg_pool[USB_RX_MESSAGE_QUEUE_SIZE * \
                                (USB_RX_MESSAGE_SIZE + 4)];
};

struct miniStm32_usb_unit
{
    struct miniStm32_usb_task task;
    struct rt_device        device;
    struct miniStm32_usb_device usb;
};

struct miniStm32_usb_unit_init
{
    rt_uint8_t              number;
    rt_uint32_t             config;
    const rt_uint8_t        *name;
    struct miniStm32_usb_unit *unit;
};

/* Exported constants --------------------------------------------------------*/
extern const device_descriptor_t _DeviceDescriptor;
extern const configuration_descriptor_set_t _ConfigDescriptor;

/* Exported variables --------------------------------------------------------*/
extern struct miniStm32_usb_unit usb;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef CTR_CALLBACK
void CTR_Callback(void);
#endif

#ifdef DOVR_CALLBACK
void DOVR_Callback(void);
#endif

#ifdef ERR_CALLBACK
void ERR_Callback(void);
#endif

#ifdef WKUP_CALLBACK
void WKUP_Callback(void);
#endif

#ifdef SUSP_CALLBACK
void SUSP_Callback(void);
#endif

#ifdef RESET_CALLBACK
void RESET_Callback(void);
#endif

#ifdef SOF_CALLBACK
void SOF_Callback(void);
#endif

#ifdef ESOF_CALLBACK
void ESOF_Callback(void);
#endif

void Suspend(struct miniStm32_usb_unit *cfg);
void Resume_Init(struct miniStm32_usb_unit *cfg);
void Resume(struct miniStm32_usb_unit *cfg, RESUME_STATE eResumeSetVal);
RESULT PowerOn(void);
RESULT PowerOff(void);

rt_err_t miniStm32_hw_usb_core_init(void);
void usb_endpoint_register(struct miniStm32_usb_unit *cfg, usb_endpoint_register_t *ep);
void usb_device_register(struct miniStm32_usb_unit *cfg, const usb_device_property_t *prep);


#endif /* __DRV_USB_CORE_H__ */
