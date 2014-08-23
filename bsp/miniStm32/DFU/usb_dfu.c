/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_prop.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : All processings related to DFU demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/***************************************************************************//**
 * @addtogroup STM32_DFU
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "dfu_mal.h"
#include "usb_desc.h"
#include "usb_dfu.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void usb_nop_process(void);
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

static void DFU_init(void);
static void DFU_Reset(void);
static void DFU_SetConfiguration(void);
static void DFU_SetDeviceAddress (void);
static void DFU_Status_In (void);
static void DFU_Status_Out (void);
static RESULT DFU_Data_Setup(uint8_t);
static RESULT DFU_NoData_Setup(uint8_t);
static RESULT DFU_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
static uint8_t *DFU_GetDeviceDescriptor(uint16_t );
static uint8_t *DFU_GetConfigDescriptor(uint16_t);
static uint8_t *DFU_GetStringDescriptor(uint16_t);
static void DFU_write_crc (void);

/* Private variables ---------------------------------------------------------*/
stm32_usb_device_t usb = {0};
stm32_dfu_unit_t dfu ={0};

__IO uint16_t wIstr;  /* ISTR register last read value */

struct
{
  __IO RESUME_STATE eState;
  __IO uint8_t bESOFcnt;
}ResumeS;

DEVICE Device_Table =
{
    ENDPOINT_NUMBER,
    1
};

/* Device property */
DEVICE_PROP Device_Property =
{
    DFU_init,
    DFU_Reset,
    DFU_Status_In,
    DFU_Status_Out,
    DFU_Data_Setup,
    DFU_NoData_Setup,
    DFU_Get_Interface_Setting,
    DFU_GetDeviceDescriptor,
    DFU_GetConfigDescriptor,
    DFU_GetStringDescriptor,
    0,
    bMaxPacketSize0     /*MAX PACKET SIZE*/
};
/* User standard requests */
USER_STANDARD_REQUESTS User_Standard_Requests =
{
    DFU_GetConfiguration,
    DFU_SetConfiguration,
    DFU_GetInterface,
    DFU_SetInterface,
    DFU_GetStatus,
    DFU_ClearFeature,
    DFU_SetEndPointFeature,
    DFU_SetDeviceFeature,
    DFU_SetDeviceAddress
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

uint8_t DFU_DeviceDescriptor[DFU_SIZ_DEVICE_DESC] =
  {
    0x12,   /* bLength */
    0x01,   /* bDescriptorType */
    0x00,   /* bcdUSB, version 1.00 */
    0x01,
    0x00,   /* bDeviceClass : See interface */
    0x00,   /* bDeviceSubClass : See interface*/
    0x00,   /* bDeviceProtocol : See interface */
    bMaxPacketSize0, /* bMaxPacketSize0 0x40 = 64 */
    0x83,   /* idVendor     (0483) */
    0x04,
    0x11,   /* idProduct (0xDF11) DFU PiD*/
    0xDF,
    0x00,   /* bcdDevice*/
    0x02,

    0x01,   /* iManufacturer : index of string Manufacturer  */
    0x02,   /* iProduct      : index of string descriptor of product*/
    0x03,   /* iSerialNumber : index of string serial number*/

    0x01    /*bNumConfigurations */
  };

uint8_t DFU_ConfigDescriptor[DFU_SIZ_CONFIG_DESC] =
  {
    0x09,   /* bLength: Configuration Descriptor size */
    0x02,   /* bDescriptorType: Configuration */
    DFU_SIZ_CONFIG_DESC, /* wTotalLength: Bytes returned */
    0x00,
    0x01,   /* bNumInterfaces: 1 interface */
    0x01,   /* bConfigurationValue: */
    /*      Configuration value */
    0x00,   /* iConfiguration: */
    /*      Index of string descriptor */
    /*      describing the configuration */
    0xC0,   /* bmAttributes: */
    /*      bus powered */
    0x32,   /* MaxPower 100 mA */
    /* 09 */

    /************ Descriptor of DFU interface 0 Alternate setting 0 *********/
    0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: */
    /*      Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x00,   /* bNumEndpoints*/
    0xFE,   /* bInterfaceClass: Application Specific Class Code */
    0x01,   /* bInterfaceSubClass : Device Firmware Upgrade Code */
    0x02,   /* nInterfaceProtocol: DFU mode protocol */
    0x04,   /* iInterface: */
    /* Index of string descriptor */
    /* 18 */

    /************ Descriptor of DFU interface 0 Alternate setting 1  **********/

    0x09,   /* bLength: Interface Descriptor size */
    0x04,   /* bDescriptorType: */
    /*      Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x01,   /* bAlternateSetting: Alternate setting */
    0x00,   /* bNumEndpoints*/
    0xFE,   /* bInterfaceClass: Application Specific Class Code */
    0x01,   /* bInterfaceSubClass : Device Firmware Upgrade Code */
    0x02,   /* nInterfaceProtocol: DFU mode protocol */
    0x05,   /* iInterface: */
    /* Index of string descriptor */
    /* 27 */

    /******************** DFU Functional Descriptor********************/
    0x09,   /*blength = 9 Bytes*/
    0x21,   /* DFU Functional Descriptor*/
    0x0B,   /*bmAttribute

                                             bitCanDnload             = 1      (bit 0)
                                             bitCanUpload             = 1      (bit 1)
                                             bitManifestationTolerant = 0      (bit 2)
                                             bitWillDetach            = 1      (bit 3)
                                             Reserved                          (bit4-6)
                                             bitAcceleratedST         = 0      (bit 7)*/
    0xFF,   /*DetachTimeOut= 255 ms*/
    0x00,
    wTransferSizeB0,
    wTransferSizeB1,          /* TransferSize = 1024 Byte*/
    0x1A,                     /* bcdDFUVersion*/
    0x01
    /***********************************************************/
    /*36*/

  };

uint8_t DFU_StringLangId[DFU_SIZ_STRING_LANGID] =
  {
    DFU_SIZ_STRING_LANGID,
    0x03,
    0x09,
    0x04    /* LangID = 0x0409: U.S. English */
  };


uint8_t DFU_StringVendor[DFU_SIZ_STRING_VENDOR] =
  {
    DFU_SIZ_STRING_VENDOR,
    0x03,
    /* Manufacturer: "STMicroelectronics" */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
  };

uint8_t DFU_StringProduct[DFU_SIZ_STRING_PRODUCT] =
  {
    DFU_SIZ_STRING_PRODUCT,
    0x03,
    /* Product name: "STM32 DFU" */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'D', 0, 'F', 0, 'U', 0
  };

uint8_t DFU_StringSerial[DFU_SIZ_STRING_SERIAL] =
  {
    DFU_SIZ_STRING_SERIAL,
    0x03,
    /* Serial number */
    'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
  };

uint8_t DFU_StringInterface0[DFU_SIZ_STRING_INTERFACE0] =
{
    DFU_SIZ_STRING_INTERFACE0,
    0x03,
    // Interface 0: "@Internal Flash   /0x08000000/06*002Ka,506*002Kg"
    '@', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'n', 0, 'a', 0,
    'l', 0, ' ', 0, 'F', 0, 'l', 0, 'a', 0, 's', 0, 'h', 0, ' ', 0,
    ' ', 0,

    '/', 0, '0', 0, 'x', 0, '0', 0, '8', 0, '0', 0, '0', 0, '0', 0,
    '0', 0, '0', 0, '0', 0,

    '/', 0, '0', 0, '6', 0, '*', 0, '0', 0, '0', 0, '2', 0, 'K', 0,
    'a', 0, ',', 0, '5', 0, '0', 0, '6', 0, '*', 0, '0', 0, '0', 0,
    '2', 0, 'K', 0, 'g', 0
};

#if 1
uint8_t DFU_StringInterface1[DFU_SIZ_STRING_INTERFACE1] =
{
    DFU_SIZ_STRING_INTERFACE1,
    0x03,
    // Interface 1: "@ SPI Flash: M25P64  /0x00000000/128*064Kg"
    '@', 0, 'S', 0, 'P', 0, 'I', 0, ' ', 0, 'F', 0, 'l', 0, 'a', 0,
    's', 0, 'h', 0, ' ', 0, ':', 0, ' ', 0, 'M', 0, '2', 0, '5', 0,
    'P', 0, '6', 0, '4', 0,

    '/', 0, '0', 0, 'x', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0,
    '0', 0, '0', 0, '0', 0, '/', 0, '1', 0, '2', 0, '8', 0, '*', 0,
    '6', 0, '4', 0, 'K', 0, 'g', 0
};
#endif /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */


ONE_DESCRIPTOR Device_Descriptor =
  {
    (uint8_t*)DFU_DeviceDescriptor,
    DFU_SIZ_DEVICE_DESC
  };

ONE_DESCRIPTOR Config_Descriptor =
  {
    (uint8_t*)DFU_ConfigDescriptor,
    DFU_SIZ_CONFIG_DESC
  };

 ONE_DESCRIPTOR DFU_String_Descriptor[6] =
  {
    {       (uint8_t*)DFU_StringLangId,          DFU_SIZ_STRING_LANGID       },
    {       (uint8_t*)DFU_StringVendor,          DFU_SIZ_STRING_VENDOR       },
    {       (uint8_t*)DFU_StringProduct,         DFU_SIZ_STRING_PRODUCT      },
    {       (uint8_t*)DFU_StringSerial,          DFU_SIZ_STRING_SERIAL       },
    {       (uint8_t*)DFU_StringInterface0,      DFU_SIZ_STRING_INTERFACE0   },
    {       (uint8_t*)DFU_StringInterface1,      DFU_SIZ_STRING_INTERFACE1   }
  };

/* Extern variables ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : NOP_Process
* Description    : No operation function.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void usb_nop_process(void)
{
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
  _SetCNTR(CNTR_FRES + CNTR_PDWN);

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
void Suspend(void)
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
  Enter_LowPowerMode();

}

/*******************************************************************************
* Function Name  : Resume_Init
* Description    : Handles wake-up restoring normal operations
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Resume_Init(void)
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
  Leave_LowPowerMode();

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
void Resume(RESUME_STATE eResumeSetVal)
{
  uint16_t wCNTR;

  if (eResumeSetVal != RESUME_ESOF)
    ResumeS.eState = eResumeSetVal;

  switch (ResumeS.eState)
  {
    case RESUME_EXTERNAL:
      Resume_Init();
      ResumeS.eState = RESUME_OFF;
      break;
    case RESUME_INTERNAL:
      Resume_Init();
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
     #ifdef STM32F10X_CL
      OTGD_FS_SetRemoteWakeup();
     #else
      wCNTR = _GetCNTR();
      wCNTR |= CNTR_RESUME;
      _SetCNTR(wCNTR);
     #endif /* STM32F10X_CL */
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
* Function Name  : DNLOAD
* Description    : Download routine.
* Input          : Length.
* Output         : None.
* Return         : Pointer to data.
*******************************************************************************/
static uint8_t *DNLOAD (uint16_t Length)
{
    DEVICE_INFO *pInfo = &Device_Info;
    uint16_t offset;

    /* wBlockNum value updated */
    dfu.wBlockNum = ((uint32_t)pInfo->USBwValues.bw.bb1 << 8) + \
        pInfo->USBwValues.bw.bb0;

    /* wlength value updated*/
    dfu.wLength = ((uint32_t)pInfo->USBwLengths.bw.bb1 << 8) + \
        pInfo->USBwLengths.bw.bb0;

    offset = pInfo->Ctrl_Info.Usb_wOffset;

    dfu.bState = STATE_dfuDNLOAD_SYNC;
    dfu.bwPollTimeout = 0;

    if (Length == 0)
    {
        pInformation->Ctrl_Info.Usb_wLength = dfu.wLength - offset;
        return NULL;
    }
    return(MAL_Buffer + offset);
}

/*******************************************************************************
* Function Name  : UPLOAD
* Description    : Upload routine.
* Input          : Length.
* Output         : None.
* Return         : Pointer to data.
*******************************************************************************/
static uint8_t *UPLOAD(uint16_t Length)
{
    DEVICE_INFO *pInfo = &Device_Info;
    uint16_t offset;
    uint8_t *Phy_Addr = NULL;
    uint32_t Addr = 0;

    /* wBlockNum value updated */
    dfu.wBlockNum = ((uint32_t)pInfo->USBwValues.bw.bb1 << 8) + \
        pInfo->USBwValues.bw.bb0;

    /* wlength value updated*/
    dfu.wLength = ((uint32_t)pInfo->USBwLengths.bw.bb1 << 8) + \
        pInfo->USBwLengths.bw.bb0;

    offset = pInformation->Ctrl_Info.Usb_wOffset;

    if (dfu.wBlockNum == 0)  /* Get Command */
    {
        if (dfu.wLength > 3)
        {
            dfu.bState = STATE_dfuIDLE;
            dfu.bwPollTimeout = 0;
        }
        else
        {
            dfu.bState = STATE_dfuUPLOAD_IDLE;
            dfu.bwPollTimeout = 0;
        }

        MAL_Buffer[0] = CMD_GETCOMMANDS;
        MAL_Buffer[1] = CMD_SETADDRESSPOINTER;
        MAL_Buffer[2] = CMD_ERASE;

        if (Length == 0)
        {
          pInformation->Ctrl_Info.Usb_wLength = 3 ;
          return NULL;
        }
        return(&MAL_Buffer[0]);
    }
    else if (dfu.wBlockNum > 1)     // TODO: =1?
    {
        dfu.bState = STATE_dfuUPLOAD_IDLE ;
        dfu.bwPollTimeout = 0;

        Addr = ((dfu.wBlockNum - 2) * wTransferSize) + dfu.wPointer;    /* Change is Accelerated*/
        Phy_Addr = MAL_Read(Addr, dfu.wLength);

        if (Length == 0)
        {
            pInformation->Ctrl_Info.Usb_wLength = dfu.wLength - offset ;
            return NULL;
        }
        return(Phy_Addr + offset);
    }
    else  /* unsupported wBlockNum */
    {
        dfu.bState = STATUS_ERRSTALLEDPKT;
        dfu.bwPollTimeout = 0;
        return NULL;
    }
}

/*******************************************************************************
* Function Name  : GETSTATE.
* Description    : Get State request routine.
* Input          : Length.
* Output         : None.
* Return         : Pointer to data.
*******************************************************************************/
static uint8_t *GETSTATE(uint16_t Length)
{
    if (Length == 0)
    {
        pInformation->Ctrl_Info.Usb_wLength = 1 ;
        return NULL;
    }
    else
    {
        return(&dfu.bState);
    }
}

/*******************************************************************************
* Function Name  : GETSTATUS.
* Description    : Get Status request routine.
* Input          : Length.
* Output         : None.
* Return         : Pointer to data.
*******************************************************************************/
static uint8_t *GETSTATUS(uint16_t Length)
{
    switch (dfu.bState)
    {
    case STATE_dfuDNLOAD_SYNC:
        if (dfu.wLength != 0)
        {
            dfu.bState = STATE_dfuDNBUSY;
            dfu.bwPollTimeout = 0;
            if ((dfu.wBlockNum == 0) && (MAL_Buffer[0] == CMD_ERASE))
            {
                MAL_GetStatus(AppAddress, 0, dfu.bufStatus);
            }
            else
            {
                MAL_GetStatus(AppAddress, 1, dfu.bufStatus);
            }
        }
        else  /* (wlength==0)*/
        {
            dfu.bState = STATE_dfuDNLOAD_IDLE;
            dfu.bwPollTimeout = 0;
        }
        break;

    case STATE_dfuMANIFEST_SYNC :
        if (dfu.bManifestState == Manifest_In_Progress)
        {
            dfu.bState = STATE_dfuMANIFEST;
            dfu.bwPollTimeout = 0;          /* 1ms */
        }
        else if ((dfu.bManifestState == Manifest_complete) && \
            (Config_Descriptor.Descriptor[29] & 0x04))
        {
            dfu.bState = STATE_dfuIDLE;
            dfu.bwPollTimeout = 0;
        }
        break;
    default :
        break;
    }

    if (Length == 0)
    {
        pInformation->Ctrl_Info.Usb_wLength = 6 ;
        return NULL;
    }
    else
    {
        dfu.bufStatus[1] = (uint8_t)(dfu.bwPollTimeout & 0x000000FF);
        dfu.bufStatus[2] = (uint8_t)((dfu.bwPollTimeout >> 8) & 0x000000FF);
        dfu.bufStatus[3] = (uint8_t)((dfu.bwPollTimeout >> 16) & 0x000000FF);
        return(dfu.bufStatus);
    }
}


/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
    Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
    Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

    Device_Serial0 += Device_Serial2;

    if (Device_Serial0 != 0)
    {
        IntToUnicode (Device_Serial0, &DFU_StringSerial[2] , 8);
        IntToUnicode (Device_Serial1, &DFU_StringSerial[18], 4);
    }
}



/*******************************************************************************
* Function Name  : DFU_init.
* Description    : DFU init routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_init(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Update the serial number string descriptor with the data from the unique ID*/
  Get_SerialNum();

  pInfo->Current_Configuration = 0;

  /* Connect the device */
  PowerOn();

  /* Perform basic device initialization operations */
  USB_SIL_Init();

  /* Enable USB interrupts */
  USB_Interrupts_Config();

  usb.bDeviceState = UNCONNECTED;
}

/*******************************************************************************
* Function Name  : DFU_Reset.
* Description    : DFU reset routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_Reset(void)
{
  /* Set DFU_DEVICE as not configured */
  Device_Info.Current_Configuration = 0;

  /* Current Feature initialization */
  pInformation->Current_Feature = DFU_ConfigDescriptor[7];

#ifdef STM32F10X_CL
  /* EP0 is already configured in DFU_Init by OTG_DEV_Init() function
      No Other endpoints needed for this firmware */
#else
  _SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
  _SetEPType(ENDP0, EP_CONTROL);
  _SetEPTxStatus(ENDP0, EP_TX_NAK);
  _SetEPRxAddr(ENDP0, ENDP0_RXADDR);
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
  _SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  SetEPTxCount(ENDP0, Device_Property.MaxPacketSize);
  Clear_Status_Out(ENDP0);
  SetEPRxValid(ENDP0);

  /* Set this device to response on default address */
  SetDeviceAddress(0);
#endif /* STM32F10X_CL */

  /* Set the new control state of the device to Attached */
  usb.bDeviceState = ATTACHED;
}
/*******************************************************************************
* Function Name  : DFU_SetConfiguration.
* Description    : Update the device state to configured.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_SetConfiguration(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    usb.bDeviceState = CONFIGURED;
  }
}
/*******************************************************************************
* Function Name  : DFU_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_SetDeviceAddress (void)
{
  usb.bDeviceState = ADDRESSED;
}
/*******************************************************************************
* Function Name  : DFU_Status_In.
* Description    : DFU status IN routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_Status_In(void)
{
}

/*******************************************************************************
* Function Name  : DFU_Status_Out.
* Description    : DFU status OUT routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_Status_Out (void)
{
    DEVICE_INFO *pInfo = &Device_Info;
    uint32_t Addr;

    if (pInfo->USBbRequest == DFU_GETSTATUS)
    {
        if (dfu.bState == STATE_dfuDNBUSY)
        {
            if (dfu.wBlockNum == 0)     /* Decode the Special Command*/
            {
                if ((MAL_Buffer[0] == CMD_GETCOMMANDS) && (dfu.wLength == 1))
                {
                }
                else if ((MAL_Buffer[0] == CMD_SETADDRESSPOINTER) && (dfu.wLength == 5))
                {
                    dfu.wPointer = MAL_Buffer[1];
                    dfu.wPointer += MAL_Buffer[2] << 8;
                    dfu.wPointer += MAL_Buffer[3] << 16;
                    dfu.wPointer += MAL_Buffer[4] << 24;
                }
                else if ((MAL_Buffer[0] ==  CMD_ERASE) && (dfu.wLength == 5))
                {
                    dfu.wPointer = MAL_Buffer[1];
                    dfu.wPointer += MAL_Buffer[2] << 8;
                    dfu.wPointer += MAL_Buffer[3] << 16;
                    dfu.wPointer += MAL_Buffer[4] << 24;
                    MAL_Erase(dfu.wPointer);
                }
            }
            else if(dfu.wBlockNum > 1)  /* Download Command */ // TODO: =1?
            {
                Addr = ((dfu.wBlockNum - 2) * wTransferSize) + dfu.wPointer;
                MAL_Write(Addr, dfu.wLength);
            }
            dfu.wLength = 0;
            dfu.wBlockNum = 0;

            dfu.bState =  STATE_dfuDNLOAD_SYNC;
            dfu.bwPollTimeout = 0;
            return;
        }
        else if(dfu.bState == STATE_dfuMANIFEST) /* Manifestation in progress*/
        {
            DFU_write_crc();
            return;
        }
    }
    return;
}

/*******************************************************************************
* Function Name  : DFU_Data_Setup.
* Description    : Handle the data class specific requests.
* Input          : RequestNb.
* Output         : None.
* Return         : USB_SUCCESS or USB_UNSUPPORT.
*******************************************************************************/
static RESULT DFU_Data_Setup(uint8_t RequestNo)
{
    uint8_t *(*CopyRoutine)(uint16_t);
    CopyRoutine = NULL;

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
        if ((RequestNo == DFU_UPLOAD) && ((dfu.bState == STATE_dfuIDLE) || \
            (dfu.bState == STATE_dfuUPLOAD_IDLE)))
        {
            CopyRoutine = UPLOAD;
        }
        else if ((RequestNo == DFU_DNLOAD) && ((dfu.bState == STATE_dfuIDLE) || \
            (dfu.bState == STATE_dfuDNLOAD_IDLE)))
        {
            dfu.bState = STATE_dfuDNLOAD_SYNC;
            CopyRoutine = DNLOAD;
        }
        else if (RequestNo == DFU_GETSTATE)
        {
            CopyRoutine = GETSTATE;
        }
        else if (RequestNo == DFU_GETSTATUS)
        {
            CopyRoutine = GETSTATUS;
        }
    }

    if (CopyRoutine == NULL)
    {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);

    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : DFU_NoData_Setup.
* Description    : Handle the No data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_SUCCESS or USB_UNSUPPORT.
*******************************************************************************/
static RESULT DFU_NoData_Setup(uint8_t RequestNo)
{
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
        /*DFU_NDLOAD*/
        if (RequestNo == DFU_DNLOAD)
        {
            /* End of DNLOAD operation*/
            if ((dfu.bState == STATE_dfuDNLOAD_IDLE) || \
                (dfu.bState == STATE_dfuIDLE))
            {
                dfu.bManifestState = Manifest_In_Progress;
                dfu.bState = STATE_dfuMANIFEST_SYNC;
                dfu.bwPollTimeout = 0;
                return USB_SUCCESS;
            }
        }
        /*DFU_UPLOAD*/
        else if (RequestNo == DFU_UPLOAD)
        {
            dfu.bState = STATE_dfuIDLE;
            dfu.bwPollTimeout = 0;
            return USB_SUCCESS;
        }
        /*DFU_CLRSTATUS*/
        else if (RequestNo == DFU_CLRSTATUS)
        {

            if (dfu.bState == STATE_dfuERROR)
            {
                dfu.bState = STATE_dfuIDLE;
                dfu.bStatus = STATUS_OK;
                dfu.bwPollTimeout = 0;
                dfu.iString = 0;
            }
            else
            {   /*State Error*/
                dfu.bState = STATE_dfuERROR;
                dfu.bStatus = STATUS_ERRUNKNOWN;/*bStatus*/
                dfu.bwPollTimeout = 0;
                dfu.iString = 0;
            }
            return USB_SUCCESS;
        }
        /*DFU_ABORT*/
        else if (RequestNo == DFU_ABORT)
        {
            if ((dfu.bState == STATE_dfuIDLE) || \
                (dfu.bState == STATE_dfuDNLOAD_SYNC) || \
                (dfu.bState == STATE_dfuDNLOAD_IDLE) || \
                (dfu.bState == STATE_dfuMANIFEST_SYNC) || \
                (dfu.bState == STATE_dfuUPLOAD_IDLE))
            {
                dfu.bState = STATE_dfuIDLE;
                dfu.bStatus = STATUS_OK;
                dfu.bwPollTimeout = 0;
                dfu.iString = 0;

                dfu.wBlockNum = 0;
                dfu.wLength = 0;
            }
            return USB_SUCCESS;
        }
    }

    return USB_UNSUPPORT;
} /* End of DFU_NoData_Setup */

/*******************************************************************************
* Function Name  : DFU_GetDeviceDescriptor.
* Description    : Gets the device descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the device descriptor.
*******************************************************************************/
static uint8_t *DFU_GetDeviceDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/*******************************************************************************
* Function Name  : DFU_GetConfigDescriptor.
* Description    : Gets the configuration descriptor.
* Input          : Length.
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
static uint8_t *DFU_GetConfigDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData (Length, &Config_Descriptor);
}

/*******************************************************************************
* Function Name  : DFU_GetStringDescriptor.
* Description    : Gets the string descriptors according to the needed index.
* Input          : Length.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
static uint8_t *DFU_GetStringDescriptor(uint16_t Length)
{
  uint8_t wValue0 = pInformation->USBwValue0;

  if (wValue0 > 8)
  {
    return NULL;
  }
  else
  {
    return Standard_GetDescriptorData(Length, &DFU_String_Descriptor[wValue0]);
  }
}

/*******************************************************************************
* Function Name  : DFU_Get_Interface_Setting.
* Description    : tests the interface and the alternate setting according to the
*                  supported one.
* Input          : - Interface : interface number.
*                  - AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : USB_SUCCESS or USB_UNSUPPORT.
*******************************************************************************/
static RESULT DFU_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
  if (AlternateSetting > 3)
  {
    return USB_UNSUPPORT;   /* In this application we don't have more than 3 AlternateSettings */
  }
  else if (Interface > 2)
  {
    return USB_UNSUPPORT; /* In this application we have only 1 interfaces */
  }

  return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : DFU_write_crc.
* Description    : DFU Write CRC routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void DFU_write_crc(void)
{
    dfu.bManifestState = Manifest_complete;

    if (Config_Descriptor.Descriptor[20] & 0x04)
    {
        dfu.bState = STATE_dfuMANIFEST_SYNC;
        dfu.bwPollTimeout = 0;
        return;
    }
    else
    {
        dfu.bState = STATE_dfuMANIFEST_WAIT_RESET;
        dfu.bwPollTimeout = 0;
        Reset_Device();
        return;
    }
}

/*******************************************************************************
* Function Name  : USB_Istr
* Description    : STR events interrupt service routine
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void USB_Istr(void)
{

    wIstr = _GetISTR();

#if (IMR_MSK & ISTR_CTR)
    if (wIstr & ISTR_CTR & wInterrupt_Mask)
    {
        /* servicing of the endpoint correct transfer interrupt */
        /* clear of the CTR flag into the sub */
        CTR_LP();
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_RESET)
    if (wIstr & ISTR_RESET & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_RESET);
        Device_Property.Reset();
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_DOVR)
    if (wIstr & ISTR_DOVR & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_DOVR);
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_ERR)
    if (wIstr & ISTR_ERR & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_ERR);
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_WKUP)
    if (wIstr & ISTR_WKUP & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_WKUP);
        Resume(RESUME_EXTERNAL);
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_SUSP)
    if (wIstr & ISTR_SUSP & wInterrupt_Mask)
    {
        /* check if SUSPEND is possible */
        if (usb.fSuspendEnabled)
        {
            Suspend();
        }
        else
        {
            /* if not possible then resume after xx ms */
            Resume(RESUME_LATER);
        }
        /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
        _SetISTR((uint16_t)CLR_SUSP);
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_SOF)
    if (wIstr & ISTR_SOF & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_SOF);
    }
#endif
    /*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/
#if (IMR_MSK & ISTR_ESOF)
    if (wIstr & ISTR_ESOF & wInterrupt_Mask)
    {
        _SetISTR((uint16_t)CLR_ESOF);
        /* resume handling timing is made with ESOFs */
        Resume(RESUME_ESOF); /* request without change of the machine state */
    }
#endif
} /* USB_Istr */

/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
    /* Set the device state to suspend */
    usb.bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
    DEVICE_INFO *pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if (pInfo->Current_Configuration != 0)
    {
        /* Device configured */
        usb.bDeviceState = CONFIGURED;
    }
    else
    {
        usb.bDeviceState = ATTACHED;
    }
}

/*******************************************************************************
* Function Name  : DFU_Button_Config.
* Description    : Configures the DFU selector Button to enter DFU Mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void DFU_Button_Config(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(KEY_CLOCK, ENABLE);

    gpio_init.GPIO_Pin      = KEY_PIN;
    gpio_init.GPIO_Mode     = GPIO_Mode_IPD;
    GPIO_Init(KEY_PORT, &gpio_init);
}

/*******************************************************************************
* Function Name  : DFU_Button_Read.
* Description    : Reads the DFU selector Button to enter DFU Mode.
* Input          : None.
* Output         : None.
* Return         : Status
*******************************************************************************/
uint8_t DFU_Button_Read (void)
{
    return GPIO_ReadInputDataBit(KEY_PORT, KEY_PIN);
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : Reset_Device.
* Description    : Reset the device.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Reset_Device(void)
{
  NVIC_SystemReset();
}

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

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
