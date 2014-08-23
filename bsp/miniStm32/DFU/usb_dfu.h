/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DFU_H
#define __USB_DFU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "usb_lib.h"

#include "usb_conf.h"


#define KEY_CLOCK                   (RCC_APB2Periph_GPIOA)
#define KEY_PORT                    (GPIOA)
#define KEY_PIN                     (GPIO_Pin_0)

/* Flash memory address from where user application will be loaded */
#define AppAddress                      (0x08003000)

#define DFU_GetConfiguration          NOP_Process
//#define DFU_SetConfiguration          NOP_Process
#define DFU_GetInterface              NOP_Process
#define DFU_SetInterface              NOP_Process
#define DFU_GetStatus                 NOP_Process
#define DFU_ClearFeature              NOP_Process
#define DFU_SetEndPointFeature        NOP_Process
#define DFU_SetDeviceFeature          NOP_Process
//#define DFU_SetDeviceAddress          NOP_Process

/* Exported types ------------------------------------------------------------*/
typedef struct usb_device_property
{
    bool (*Process_Status_IN)(void);
    bool (*Process_Status_OUT)(void);
    RESULT (*Class_Data_Setup)(uint8_t RequestNo);
    RESULT (*Class_NoData_Setup)(uint8_t RequestNo);
} usb_device_property_t;

typedef struct usb_property
{
    uint8_t                 table;
    const usb_device_property_t *preperty;
} usb_property_t;

typedef struct stm32_usb_device
{
    volatile uint8_t        bIntPackSOF;        /* SOFs received between 2 consecutive packets */
    volatile uint16_t       wIstr;              /* ISTR register last read value */
    volatile uint32_t       bDeviceState;       /* USB device status */
    volatile bool           fSuspendEnabled;    /* true when suspend is possible */

    uint16_t                epType[ENDPOINT_NUMBER - 1];
    usb_property_t          property;
} stm32_usb_device_t;

#define bStatus             bufStatus[0]
#define bState              bufStatus[4]
#define iString             bufStatus[5]

typedef struct stm32_dfu_unit
{
    uint32_t                bwPollTimeout;
    uint8_t                 bError;
    uint32_t                wPointer;
    uint32_t                wBlockNum;
    uint32_t                wLength;
    uint8_t                 bManifestState;
    uint8_t                 bufStatus[6];
} stm32_dfu_unit_t;

/*---------------------------------------------------------------------*/
/*  DFU definitions                                                    */
/*---------------------------------------------------------------------*/
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

typedef enum _DEVICE_STATE
{
  UNCONNECTED,
  ATTACHED,
  POWERED,
  SUSPENDED,
  ADDRESSED,
  CONFIGURED
} DEVICE_STATE;

/**************************************************/
/* DFU Requests                                   */
/**************************************************/

typedef enum _DFU_REQUESTS {
  DFU_DNLOAD = 1,
  DFU_UPLOAD,
  DFU_GETSTATUS,
  DFU_CLRSTATUS,
  DFU_GETSTATE,
  DFU_ABORT
} DFU_REQUESTS;

/**************************************************/
/* DFU Requests  DFU states                       */
/**************************************************/
typedef enum _DFU_STATES {
    STATE_appIDLE = 0,
    STATE_appDETACH,
    STATE_dfuIDLE,
    STATE_dfuDNLOAD_SYNC,
    STATE_dfuDNBUSY,
    STATE_dfuDNLOAD_IDLE,
    STATE_dfuMANIFEST_SYNC,
    STATE_dfuMANIFEST,
    STATE_dfuMANIFEST_WAIT_RESET,
    STATE_dfuUPLOAD_IDLE,
    STATE_dfuERROR
} DFU_STATES;

/**************************************************/
/* DFU Requests  DFU status                       */
/**************************************************/
typedef enum _DFU_ERRORS {
    STATUS_OK= 0,
    STATUS_ERRTARGET,
    STATUS_ERRFILE,
    STATUS_ERRWRITE,
    STATUS_ERRERASE,
    STATUS_ERRCHECK_ERASED,
    STATUS_ERRPROG,
    STATUS_ERRVERIFY,
    STATUS_ERRADDRESS,
    STATUS_ERRNOTDONE,
    STATUS_ERRFIRMWARE,
    STATUS_ERRVENDOR,
    STATUS_ERRUSBR,
    STATUS_ERRPOR,
    STATUS_ERRUNKNOWN,
    STATUS_ERRSTALLEDPKT
} DFU_ERRORS;

/**************************************************/
/* DFU Requests  DFU states Manifestation State   */
/**************************************************/

#define Manifest_complete           0x00
#define Manifest_In_Progress        0x01


/**************************************************/
/* Special Commands  with Download Request        */
/**************************************************/

#define CMD_GETCOMMANDS              0x00
#define CMD_SETADDRESSPOINTER        0x21
#define CMD_ERASE                    0x41

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void DFU_Button_Config(void);
uint8_t  DFU_Button_Read(void);

void USB_Istr(void);


/* External variables --------------------------------------------------------*/
extern stm32_usb_device_t usb;
extern stm32_dfu_unit_t dfu;

#endif  /*__USB_DFU_H*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
