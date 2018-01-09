/***************************************************************************//**
 * @file    drv_usb_core.c
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
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_usb_core.h"

#if defined (BSP_USING_USB_VIRTUAL_COM)
#include "drv_usb_com.h"
#endif
#if defined (BSP_USING_USB_HID_MOUSE)
#include "drv_usb_mouse.h"
#endif

#if defined(BSP_USING_USB)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef BSP_USB_DEBUG
#define usb_debug(format,args...)           rt_kprintf(format, ##args)
#else
#define usb_debug(format,args...)
#endif

/* Private function prototypes -----------------------------------------------*/
void usb_core_reset(struct bsp_usb_unit *cfg);
void Enter_LowPowerMode(struct bsp_usb_unit *cfg);
void Leave_LowPowerMode(struct bsp_usb_unit *cfg);

static void usb_nop_process(void);

static void usb_device_init(void);
static void usb_device_reset(void);
static void usb_device_statusIn(void);
static void usb_device_statusOut(void);
static RESULT usb_device_dataSetup(uint8_t RequestNo);
static RESULT usb_device_noDataSetup(uint8_t RequestNo);
static void usb_device_setConfiguration(void);
static void usb_device_setDeviceAddress (void);
static RESULT usb_device_getInterfaceSetting(uint8_t Interface, uint8_t AlternateSetting);
static uint8_t *usb_device_getDeviceDescriptor(uint16_t Length);
static uint8_t *usb_device_getConfigDescriptor(uint16_t Length);
static uint8_t *usb_device_getStringDescriptor(uint16_t Length);

static void usb_user_setConfiguration(void);
static void usb_user_setDeviceAddress (void);

/* Public variables ----------------------------------------------------------*/
struct bsp_usb_unit usb = {0};

/* ISTR register last read value */
__IO uint16_t wIstr;

/* USB device table */
DEVICE Device_Table =
{
    ENDPOINT_NUMBER,
    1
};
/* Device property */
DEVICE_PROP Device_Property =
{
    usb_device_init,
    usb_device_reset,
    usb_device_statusIn,
    usb_device_statusOut,
    usb_device_dataSetup,
    usb_device_noDataSetup,
    usb_device_getInterfaceSetting,
    usb_device_getDeviceDescriptor,
    usb_device_getConfigDescriptor,
    usb_device_getStringDescriptor,
    0,
    0x40      /*MAX PACKET SIZE*/
};
/* User standard requests */
USER_STANDARD_REQUESTS User_Standard_Requests =
{
    usb_nop_process, //usb_user_getConfiguration,
    usb_user_setConfiguration,
    usb_nop_process, //usb_user_getInterface,
    usb_nop_process, //usb_user_setInterface,
    usb_nop_process, //usb_user_getStatus,
    usb_nop_process, //usb_user_clearFeature,
    usb_nop_process, //usb_user_setEndPointFeature,
    usb_nop_process, //usb_user_setDeviceFeature,
    usb_user_setDeviceAddress
};

/* CTR service routines */
void (*pEpInt_IN[7])(void) =
{
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process
};
void (*pEpInt_OUT[7])(void) =
{
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process,
    usb_nop_process
};

/* Private variables ---------------------------------------------------------*/
struct
{
    __IO RESUME_STATE eState;
    __IO uint8_t bESOFcnt;
}ResumeS;

/*** Device descriptor ***/
const device_descriptor_t _DeviceDescriptor =
{
    sizeof(device_descriptor_t),            // bLength
    DSC_TYPE_DEVICE,                        // bDescriptorType
    LE(VER_USB),                            // bcdUSB
    0xEF,                                   // bDeviceClass  (Misc)
    0x02,                                   // bDeviceSubClass  (common)
    0x01,                                   // bDeviceProtocol  (IAD)
    USB_ENDP0_RX_BUF_SIZE,                  // bMaxPacketSize0
    LE(VID),                                // idVendor
    LE(PID),                                // idProduct
    LE(DEV_REV),                            // bcdDevice
    0x01,                                   // iManufacturer
    0x02,                                   // iProduct
    0x03,                                   // iSerialNumber
    0x01                                    // bNumConfigurations
};  //end of DeviceDesc

/*** Configuration descriptor set ***/
const configuration_descriptor_set_t _ConfigDescriptor =
{
    /*** Configuration descriptor ***/
    {
        sizeof(configuration_descriptor_t), // bLength
        DSC_TYPE_CONFIG,                    // bDescriptorType
        LE(sizeof(configuration_descriptor_set_t)), // bTotalLength
        DSC_NUM_INTERFACE,				    // bNumInterfaces
        0x01,							    // bConfigurationValue
        0x00,							    // iConfiguration
        DSC_CNFG_ATR_BASE | \
        DSC_CNFG_ATR_SELF_POWERED,		    // bmAttributes
        DSC_CNFG_MAXPOWER(100),			    // bMaxPower (mA)
        								    // <= 100mA: Low power
        							        // <= 500mA: High power
    },

#if defined(BSP_USING_USB_VIRTUAL_COM)
    USB_VIRTUAL_COM_ConfigDescriptor
#endif

#if defined(BSP_USING_USB_HID_MOUSE)
    USB_HID_MOUSE_ConfigDescriptor
#endif
};  //end of Configuration*/

//
// String descriptors
//
#define DSC_STRING0_LEN (4)
const rt_uint8_t _StringLangID[DSC_STRING0_LEN] =
{
    DSC_STRING0_LEN,
    DSC_TYPE_STRING,
    0x09,
    0x04 /* LangID = 0x0409: U.S. English */
};

#define DSC_STRING1_LEN (sizeof("piao.sg/onelife") * 2)
const rt_uint8_t _StringVendor[DSC_STRING1_LEN] =
{
    DSC_STRING1_LEN,                        /* Size of Vendor string */
    DSC_TYPE_STRING,                        /* bDescriptorType*/
    /* Manufacturer: "piao.sg/onelife" */
    'p', 0, 'i', 0, 'a', 0, 'o', 0, '.', 0, 's', 0, 'g', 0, '/', 0,
    'o', 0, 'n', 0, 'e', 0, 'l', 0, 'i', 0, 'f', 0, 'e', 0
};

#define DSC_STRING2_LEN (sizeof("Modified MiniSTM32") * 2)
const rt_uint8_t _StringProduct[DSC_STRING2_LEN] =
{
    DSC_STRING2_LEN,                        /* bLength */
    DSC_TYPE_STRING,                        /* bDescriptorType */
    /* Product name: "Modified MiniSTM32" */
    'M', 0, 'o', 0, 'd', 0, 'i', 0, 'f', 0, 'i', 0, 'e', 0, 'd', 0,
    ' ', 0, 'M', 0, 'i', 0, 'n', 0, 'i', 0, 'S', 0, 'T', 0, 'M', 0,
    '3', 0, '2', 0
};

#define DSC_STRING3_LEN (sizeof("123456789012") * 2)
rt_uint8_t _StringSerial[DSC_STRING3_LEN] =
{
    DSC_STRING3_LEN,                        /* bLength */
    DSC_TYPE_STRING,                        /* bDescriptorType */
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0,
    '9', 0, '0', 0, '1', 0, '2', 0
};

ONE_DESCRIPTOR Device_Descriptor =
{
    (uint8_t *)&_DeviceDescriptor,
    sizeof(device_descriptor_t)
};

ONE_DESCRIPTOR Config_Descriptor =
{
    (uint8_t *)&_ConfigDescriptor,
    sizeof(configuration_descriptor_set_t)
};

ONE_DESCRIPTOR String_Descriptor[4] =
{
    {(uint8_t *)_StringLangID, DSC_STRING0_LEN},
    {(uint8_t *)_StringVendor, DSC_STRING1_LEN},
    {(uint8_t *)_StringProduct, DSC_STRING2_LEN},
    {(uint8_t *)_StringSerial, DSC_STRING3_LEN}
};

extern rt_uint8_t USB_UART_StringSerial[];
extern uint8_t USB_Tx_State, USB_Tx_State2;
extern uint8_t *USB_Tx_Buffer, *USB_Tx_Buffer2;

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_getSerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode(Device_Serial0, &_StringSerial[2] , 8);
    IntToUnicode(Device_Serial1, &_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : PowerOn
* Description    :
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
RESULT PowerOn(void)
{
    uint16_t wRegVal;

    /*** CNTR_PWDN = 0 ***/
    wRegVal = CNTR_FRES;
    _SetCNTR(wRegVal);

    /*** CNTR_FRES = 0 ***/
    wInterrupt_Mask = 0;
    _SetCNTR(wInterrupt_Mask);
    /*** Clear pending interrupts ***/
    _SetISTR(0);
    /*** Set interrupt mask ***/
    wInterrupt_Mask = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;
    _SetCNTR(wInterrupt_Mask);

    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : PowerOff
* Description    : handles switch-off conditions
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
RESULT PowerOff()
{
    /* disable all interrupts and force USB reset */
    _SetCNTR(CNTR_FRES);
    /* clear interrupt status register */
    _SetISTR(0);
    /* switch-off device */
    _SetCNTR(CNTR_FRES | CNTR_PDWN);

    /* sw variables reset */
    /* ... */

    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Suspend
* Description    : sets suspend mode operating conditions
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Suspend(struct bsp_usb_unit *cfg)
{
    uint16_t wCNTR;
    /* suspend preparation */
    /* ... */

    /* macrocell enters suspend mode */
    wCNTR = _GetCNTR();
    wCNTR |= CNTR_FSUSP;
    _SetCNTR(wCNTR);

    /* ------------------ ONLY WITH BUS-POWERED DEVICES ---------------------- */
    /* power reduction */
    /* ... on connected devices */

    /* force low-power mode in the macrocell */
    wCNTR = _GetCNTR();
    wCNTR |= CNTR_LPMODE;
    _SetCNTR(wCNTR);

    /* switch-off the clocks */
    /* ... */
    Enter_LowPowerMode(cfg);
}

/*******************************************************************************
* Function Name  : Resume_Init
* Description    : Handles wake-up restoring normal operations
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Resume_Init(struct bsp_usb_unit *cfg)
{
    uint16_t wCNTR;

    /* ------------------ ONLY WITH BUS-POWERED DEVICES ---------------------- */
    /* restart the clocks */
    /* ...  */

    /* CNTR_LPMODE = 0 */
    wCNTR = _GetCNTR();
    wCNTR &= (~CNTR_LPMODE);
    _SetCNTR(wCNTR);

    /* restore full power */
    /* ... on connected devices */
    Leave_LowPowerMode(cfg);

    /* reset FSUSP bit */
    _SetCNTR(IMR_MSK);

    /* reverse suspend preparation */
    /* ... */
}

/*******************************************************************************
* Function Name  : Resume
* Description    : This is the state machine handling resume operations and
*                 timing sequence. The control is based on the Resume structure
*                 variables and on the ESOF interrupt calling this subroutine
*                 without changing machine state.
* Input          : a state machine value (RESUME_STATE)
*                  RESUME_ESOF doesn't change ResumeS.eState allowing
*                  decrementing of the ESOF counter in different states.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Resume(struct bsp_usb_unit *cfg, RESUME_STATE eResumeSetVal)
{
    uint16_t wCNTR;

    if (eResumeSetVal != RESUME_ESOF)
        ResumeS.eState = eResumeSetVal;

    switch (ResumeS.eState)
    {
    case RESUME_EXTERNAL:
        Resume_Init(cfg);
        ResumeS.eState = RESUME_OFF;
    break;
    case RESUME_INTERNAL:
        Resume_Init(cfg);
        ResumeS.eState = RESUME_START;
    break;
    case RESUME_LATER:
        ResumeS.bESOFcnt = 2;
        ResumeS.eState = RESUME_WAIT;
    break;
    case RESUME_WAIT:
        ResumeS.bESOFcnt--;
        if (ResumeS.bESOFcnt == 0)
            ResumeS.eState = RESUME_START;
        break;
    case RESUME_START:
        wCNTR = _GetCNTR();
        wCNTR |= CNTR_RESUME;
        _SetCNTR(wCNTR);
        ResumeS.eState = RESUME_ON;
        ResumeS.bESOFcnt = 10;
        break;
    case RESUME_ON:
        ResumeS.bESOFcnt--;
        if (ResumeS.bESOFcnt == 0)
        {
            wCNTR = _GetCNTR();
            wCNTR &= (~CNTR_RESUME);
            _SetCNTR(wCNTR);
            ResumeS.eState = RESUME_OFF;
        }
        break;
    case RESUME_OFF:
    case RESUME_ESOF:
    default:
        ResumeS.eState = RESUME_OFF;
        break;
    }
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(struct bsp_usb_unit *cfg)
{
    /* Set the device state to suspend */
    cfg->usb.bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(struct bsp_usb_unit *cfg)
{
    /* Set the device state to the correct state */
    if (Device_Info.Current_Configuration != 0)
    {
        /* Device configured */
        cfg->usb.bDeviceState = CONFIGURED;
    }
    else
    {
        cfg->usb.bDeviceState = ATTACHED;
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_init.
* Description    : Virtual COM Port Mouse init routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_device_init(void)
{
    usb_debug("USB: usb_device_init\n");
    /* Update the serial number string descriptor with the data from the unique
    ID*/
    usb_getSerialNum();
    Device_Info.Current_Configuration = 0;
    /* Connect the device */
    PowerOn();
    /* Perform basic device initialization operations */
    USB_SIL_Init();

    usb.usb.bDeviceState = UNCONNECTED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_device_reset(void)
{
    /* Set Virtual_Com_Port DEVICE as not configured */
    Device_Info.Current_Configuration = 0;

    /* Current Feature initialization */
    Device_Info.Current_Feature = Config_Descriptor.Descriptor[7];

    /* Set Virtual_Com_Port DEVICE with the default Interface*/
    Device_Info.Current_Interface = 0;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_device_statusIn(void)
{
    rt_uint8_t i;

    for (i = 0; i < DEVICE_NUMBER; i++)
    {
        if ((usb.usb.property.table & (1 << i)) && \
            (usb.usb.property.preperty[i]->Process_Status_IN != RT_NULL))
        {
            if (usb.usb.property.preperty[i]->Process_Status_IN() == RT_TRUE)
            {
                return;
            }
        }
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_Out
* Description    : Virtual COM Port Status OUT Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_device_statusOut(void)
{
    rt_uint8_t i;

    for (i = 0; i < DEVICE_NUMBER; i++)
    {
        if ((usb.usb.property.table & (1 << i)) && \
            (usb.usb.property.preperty[i]->Process_Status_OUT != RT_NULL))
        {
            if (usb.usb.property.preperty[i]->Process_Status_OUT() == RT_TRUE)
            {
                return;
            }
        }
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
static RESULT usb_device_dataSetup(uint8_t RequestNo)
{
    rt_uint8_t i;
    RESULT ret = USB_UNSUPPORT;

    for (i = 0; i < DEVICE_NUMBER; i++)
    {
        if ((usb.usb.property.table & (1 << i)) && \
            (usb.usb.property.preperty[i]->Class_Data_Setup != RT_NULL))
        {
            if ((ret = usb.usb.property.preperty[i]->Class_Data_Setup(RequestNo)) \
                == USB_SUCCESS)
            {
                return ret;
            }
        }
    }

    return ret;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_NoData_Setup.
* Description    : handle the no data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
static RESULT usb_device_noDataSetup(uint8_t RequestNo)
{
    rt_uint8_t i;
    RESULT ret = USB_UNSUPPORT;

    for (i = 0; i < DEVICE_NUMBER; i++)
    {
        if ((usb.usb.property.table & (1 << i)) && \
            (usb.usb.property.preperty[i]->Class_NoData_Setup != RT_NULL))
        {
            if ((ret = usb.usb.property.preperty[i]->Class_NoData_Setup(RequestNo)) \
                == USB_SUCCESS)
            {
                return ret;
            }
        }
    }

    return ret;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to configured.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_device_setConfiguration(void)
{
    if (Device_Info.Current_Configuration != 0)
    {
        /* Device configured */
        usb.usb.bDeviceState = CONFIGURED;
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_device_setDeviceAddress (void)
{
    usb.usb.bDeviceState = ADDRESSED;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Get_Interface_Setting.
* Description    : test the interface and the alternate setting according to the
*                  supported one.
* Input1         : uint8_t: Interface : interface number.
* Input2         : uint8_t: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
static RESULT usb_device_getInterfaceSetting(uint8_t Interface, uint8_t AlternateSetting)
{
    if (AlternateSetting > 0)
    {
        return USB_UNSUPPORT;
    }
    else if (Interface > Config_Descriptor.Descriptor[4])
    {
        return USB_UNSUPPORT;
    }
    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
* Description    : Gets the device descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
static uint8_t *usb_device_getDeviceDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetConfigDescriptor.
* Description    : get the configuration descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
static uint8_t *usb_device_getConfigDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetStringDescriptor
* Description    : Gets the string descriptors according to the needed index
* Input          : Length.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
static uint8_t *usb_device_getStringDescriptor(uint16_t Length)
{
    uint8_t wValue0 = Device_Info.USBwValue0;
    if (wValue0 > 4)
    {
        return NULL;
    }
    else
    {
        return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to configured.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_user_setConfiguration(void)
{
    if (Device_Info.Current_Configuration != 0)
    {
        /* Device configured */
        usb.usb.bDeviceState = CONFIGURED;
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_user_setDeviceAddress (void)
{
    usb.usb.bDeviceState = ADDRESSED;
}

/***************************************************************************//**
 * @brief
 *  USART RX data valid interrupt handler
 *
 * @details
 *
 * @note
 *  9-bit USART mode has not implemented yet and USART slave mode is untested
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void bsp_usb_wakeup_isr(rt_device_t dev)
{
    struct bsp_usb_unit *cfg;

    cfg = (struct bsp_usb_unit *)dev->user_data;
    if (cfg->usb.status & USB_STATUS_START)
    {
        //usb_debug("USB: wakeup_isr\n");
    }
}

/***************************************************************************//**
 * @brief
 *  USART RX data valid interrupt handler
 *
 * @details
 *
 * @note
 *  9-bit USART mode has not implemented yet and USART slave mode is untested
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void bsp_usb_lowPriority_isr(rt_device_t dev)
{
    struct bsp_usb_unit *cfg;

    cfg = (struct bsp_usb_unit *)dev->user_data;
    cfg->usb.wIstr = _GetISTR();
    if ((cfg->usb.status & USB_STATUS_START) && !(cfg->usb.wIstr & ISTR_SOF))
    {
        if (rt_mq_send(
            &cfg->task.rx_msgs,
            (void *)&cfg->usb.wIstr,
            USB_RX_MESSAGE_SIZE) != RT_EOK)
        {
            //usb_debug("USB err: get mq failed!\n");
        }
    }

#if (IMR_MSK & ISTR_CTR)
    if (cfg->usb.wIstr & ISTR_CTR & wInterrupt_Mask)
    {
        /* servicing of the endpoint correct transfer interrupt */
        /* clear of the CTR flag into the sub */
        CTR_LP();
#ifdef CTR_CALLBACK
        CTR_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_RESET)
    if (cfg->usb.wIstr & ISTR_RESET & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_RESET);
        usb_core_reset(cfg);
        Device_Property.Reset();
#ifdef RESET_CALLBACK
        RESET_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_DOVR)
    if (cfg->usb.wIstr & ISTR_DOVR & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_DOVR);
#ifdef DOVR_CALLBACK
        DOVR_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_ERR)
    if (cfg->usb.wIstr & ISTR_ERR & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_ERR);
#ifdef ERR_CALLBACK
        ERR_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_WKUP)
    if (cfg->usb.wIstr & ISTR_WKUP & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_WKUP);
        Resume(cfg, RESUME_EXTERNAL);
#ifdef WKUP_CALLBACK
        WKUP_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_SUSP)
    if (cfg->usb.wIstr & ISTR_SUSP & wInterrupt_Mask)
    {
        /* check if SUSPEND is possible */
        if (cfg->usb.fSuspendEnabled)
        {
            Suspend(cfg);
        }
        else
        {
        /* if not possible then resume after xx ms */
            Resume(cfg, RESUME_LATER);
        }
        /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
        _SetISTR((uint16_t)CLR_SUSP);
#ifdef SUSP_CALLBACK
        SUSP_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

#if (IMR_MSK & ISTR_SOF)
    if (cfg->usb.wIstr & ISTR_SOF & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_SOF);
        cfg->usb.bIntPackSOF++;

#ifdef SOF_CALLBACK
        SOF_Callback();
#endif
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

#if (IMR_MSK & ISTR_ESOF)
    if (cfg->usb.wIstr & ISTR_ESOF & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_ESOF);
        /* resume handling timing is made with ESOFs */
        Resume(cfg, RESUME_ESOF); /* request without change of the machine state */

#ifdef ESOF_CALLBACK
        ESOF_Callback();
#endif
    }
#endif
}

/*******************************************************************************
* Function Name  : NOP_Process
* Description    : No operation function.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_nop_process(void)
{
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Reset
* Description    : Virtual_Com_Port Mouse reset routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void usb_core_reset(struct bsp_usb_unit *cfg)
{
    rt_uint8_t i;
    rt_uint16_t addr;

    SetBTABLE(USB_BUF_TABLE_ADDR);

    /* Initialize Endpoint 0 */
    SetEPType(ENDP0, EP_CONTROL);
    SetEPTxAddr(ENDP0, USB_ENDP0_TX_BUF_ADDR);
    SetEPTxStatus(ENDP0, EP_TX_STALL);
    SetEPRxAddr(ENDP0, USB_ENDP0_RX_BUF_ADDR);
    SetEPRxCount(ENDP0, USB_ENDP0_RX_BUF_SIZE);
    SetEPRxValid(ENDP0);
    Clear_Status_Out(ENDP0);

    addr = USB_BUF_TABLE_ADDR + USB_BUF_TABLE_SIZE + \
        USB_ENDP0_TX_BUF_SIZE + USB_ENDP0_RX_BUF_SIZE;

    /* Initialize other Endpoints */
    for (i = 0; i < ENDPOINT_NUMBER - 1; i++)
    {
        if (!cfg->usb.epInBuf[i].size && !cfg->usb.epOutBuf[i].size)
        {
            continue;
        }
        SetEPType(i + 1, cfg->usb.epType[i]);
        if (cfg->usb.epInBuf[i].size)
        {
            SetEPTxAddr(i + 1, addr);
            SetEPTxStatus(i + 1, EP_TX_NAK);
            cfg->usb.epInBuf[i].addr = addr;
            addr += cfg->usb.epInBuf[i].size;
        }
        else
        {
            SetEPTxStatus(i + 1, EP_TX_DIS);
        }
        if (cfg->usb.epOutBuf[i].size)
        {
            SetEPRxAddr(i + 1, addr);
            SetEPRxCount(i + 1, cfg->usb.epOutBuf[i].size);
            SetEPRxStatus(i + 1, EP_RX_VALID);
            cfg->usb.epOutBuf[i].addr = addr;
            addr += cfg->usb.epOutBuf[i].size;
        }
        else
        {
            SetEPRxStatus(i + 1, EP_RX_DIS);
        }
    }

    if (addr > USB_ENDPOINT_BUFFER_SIZE)
    {
        rt_kprintf("USB err: no enough buffer for endpoint\n");
    }
    RT_ASSERT(addr < USB_ENDPOINT_BUFFER_SIZE);

    /* Set this device to response on default address (0) */
    SetDeviceAddress(0);

    cfg->usb.bDeviceState = ATTACHED;
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer (uint16_t len)
{

    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if(USB_Tx_State != 1)
    {
        USB_Tx_State = 1;

        USB_SIL_Write(EP1_IN, USB_Tx_Buffer, len);
        SetEPTxValid(ENDP1);
    }
}

void Handle_USBAsynchXfer2 (uint16_t len)
{

    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if(USB_Tx_State2 != 1)
    {
        USB_Tx_State2 = 1;

        USB_SIL_Write(EP3_IN, USB_Tx_Buffer2, len);
        SetEPTxValid(ENDP3);
    }
}

/***************************************************************************//**
* @brief
*   Register SPI device
*
* @details
*
* @note
*
* @param[in] device
*   Pointer to device descriptor
*
* @param[in] name
*   Device name
*
* @param[in] flag
*   Configuration flags
*
* @param[in] spi
*   Pointer to SPI device descriptor
*
* @return
*   Error code
******************************************************************************/
void usb_core_task_main_loop(void *parameter)
{
    struct bsp_usb_unit *cfg;
    rt_uint16_t wIstr;
    union bsp_usb_exec_message *exec_msg;
    rt_thread_t self;

    cfg = (struct bsp_usb_unit *)parameter;
    self = rt_thread_self();

	if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        USB_RX_MESSAGE_SIZE,
		sizeof(cfg->task.rx_msg_pool),
		RT_IPC_FLAG_FIFO) != RT_EOK)
	{
        rt_kprintf("USB: init mq failed!\n");
        return;
	}
    usb_debug("USB: mq init OK\n");

	if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("USB: init event failed!\n");
            return;
        }
    usb_debug("USB: event init OK\n");

    /* Init USB */
    USB_Init();
    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;
    cfg->usb.status |= USB_STATUS_START;
    usb_debug("USB: enter main loop\n");

USB_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
        usb_debug("USB: waiting\n");
		if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&wIstr,
            USB_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
		{
            continue;
		}

        if (cfg->usb.wIstr & ISTR_RESET & wInterrupt_Mask)
        {
            rt_uint8_t i;

            for (i = 0; i < ENDPOINT_NUMBER - 1; i++)
            {
                usb_debug("USB: EP%d, type %x, in %d, out %d\n",
                    i + 1, cfg->usb.epType[i],
                    cfg->usb.epInBuf[i].addr,
                    cfg->usb.epOutBuf[i].addr);
            }
        }
        usb_debug("USB: wIstr %x\n", wIstr);
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize SPI device
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
static rt_err_t usb_core_init(rt_device_t dev)
{
    return rt_thread_startup(&usb.task.thread);
}

/***************************************************************************//**
* @brief
*   Initialize all USART module related hardware and register USART device to
*   kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t bsp_hw_usb_core_init(void)
{
    EXTI_InitTypeDef    exti_init;
    NVIC_InitTypeDef    nvic_init;
    bsp_irq_hook_init_t hook;

    usb.usb.bDeviceState = UNCONNECTED;
    usb.usb.fSuspendEnabled = RT_TRUE;

    do
    {
        /* Config and enable clock */
        RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

        /* Config weakup interrupt */
        hook.type       = bsp_irq_type_exti;
        hook.unit       = 18;
        hook.cbFunc     = bsp_usb_wakeup_isr;
        hook.userPtr    = (void *)&usb.device;
        bsp_irq_hook_register(&hook);

        EXTI_ClearFlag(EXTI_Line18);
        exti_init.EXTI_Line     = EXTI_Line18;
        exti_init.EXTI_Mode     = EXTI_Mode_Interrupt;
        exti_init.EXTI_Trigger  = EXTI_Trigger_Rising;
        exti_init.EXTI_LineCmd  = ENABLE;
        EXTI_Init(&exti_init);

        NVIC_ClearPendingIRQ(USBWakeUp_IRQn);
        nvic_init.NVIC_IRQChannel = USBWakeUp_IRQn;
        nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
        nvic_init.NVIC_IRQChannelSubPriority = BSP_IRQ_PRI_COM;
        nvic_init.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic_init);

        /* Config USB low priority interrupt */
        hook.type       = bsp_irq_type_usb;
        hook.unit       = 0;
        hook.cbFunc     = bsp_usb_lowPriority_isr;
        hook.userPtr    = (void *)&usb.device;
        bsp_irq_hook_register(&hook);

        NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
        nvic_init.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
        nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
        nvic_init.NVIC_IRQChannelSubPriority = BSP_IRQ_PRI_COM;
        nvic_init.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic_init);

        if (rt_thread_init(
            &usb.task.thread,
            USB_NAME,
            usb_core_task_main_loop,
            (void *)&usb,
            (void *)&usb.task.stack,
            sizeof(usb.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }

        /* Config and register device */
        usb.device.type         = RT_Device_Class_USBDevice;
        usb.device.rx_indicate  = RT_NULL;
        usb.device.tx_complete   = RT_NULL;
        usb.device.init          = usb_core_init;
        usb.device.open          = RT_NULL;
        usb.device.close         = RT_NULL;
        usb.device.read          = RT_NULL;
        usb.device.write         = RT_NULL;
        usb.device.control       = RT_NULL;
        usb.device.user_data     = (void *)&usb;
        if (rt_device_register(&usb.device, USB_NAME, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            break;
        }

        usb_debug("USB: H/W init OK\n");
        return RT_EOK;
    } while (0);

    usb_debug("USB: H/W init failed!\n");
    return -RT_ERROR;
}

/***************************************************************************//**
 * @brief
 *   Initialize SPI device
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
void usb_endpoint_register(struct bsp_usb_unit *cfg,
    usb_endpoint_register_t *ep)
{
    if (ep->num > 0)
    {   /* Start from EP1 */
        cfg->usb.epType[ep->num - 1] = ep->type;
        cfg->usb.epInBuf[ep->num - 1].size = ep->inBufSize;
        cfg->usb.epOutBuf[ep->num - 1].size = ep->outBufSize;

        if (ep->inFunc)
        {
            pEpInt_IN[ep->num - 1] = ep->inFunc;
       }
        if (ep->outFunc)
        {
            pEpInt_OUT[ep->num - 1] = ep->outFunc;
       }

        usb_debug("USB: EP%d, type %x, in %d, out %d, inFunc %x, outFunc %x\n",
            ep->num, cfg->usb.epType[ep->num - 1],
            cfg->usb.epInBuf[ep->num - 1].size,
            cfg->usb.epOutBuf[ep->num - 1].size,
            pEpInt_IN[ep->num - 1],
            pEpInt_OUT[ep->num - 1]);
    }
}

rt_uint8_t usb_find_entry(rt_uint8_t table)
{
    rt_uint8_t i;

    for (i = 0; i < DEVICE_NUMBER; i++)
    {
        if (((table >> i) & 0x01) == 0x00)
        {
            return i;
        }
    }
}

void usb_device_register(struct bsp_usb_unit *cfg,
    const usb_device_property_t *prep)
{
    if (prep)
    {
        rt_uint8_t i = usb_find_entry(cfg->usb.property.table);

        cfg->usb.property.preperty[i] = prep;
        cfg->usb.property.table |= 1 << i;

        usb_debug("USB: preperty entry %d, table %x (%x %x %x %x)\n",
            i, cfg->usb.property.table,
            cfg->usb.property.preperty[i]->Process_Status_IN,
            cfg->usb.property.preperty[i]->Process_Status_OUT,
            cfg->usb.property.preperty[i]->Class_Data_Setup,
            cfg->usb.property.preperty[i]->Class_NoData_Setup);
    }
}

/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
/*#ifdef RT_USING_FINSH
#include <finsh.h>

void debug(void)
{
    rt_kprintf(" ERROR %d\n", error);
}
FINSH_FUNCTION_EXPORT(debug, debug.)
#endif
*/
#endif /* defined(BSP_USING_USB) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
