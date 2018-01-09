/***************************************************************************//**
 * @file    drv_sdio.c
 * @brief   SDIO driver of RT-Thread RTOS
 *  COPYRIGHT (C) 2011, RT-Thread Development Team
 * @author  onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2016-03-10   onelife     Initial creation of SDIO module driver
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup OneBoard
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_sdio.h"
#if defined(BSP_USING_SDIO)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SPI1_DR_Base                        (SPI1_BASE + 0x0C)
#define SPI2_DR_Base                        (SPI2_BASE + 0x0C)
#define SPI3_DR_Base                        (SPI3_BASE + 0x0C)

#define DMA2_Channel_SDIO                   (DMA2_Channel4)
#define DMA2_Channel_SDIO_Num               (10)
#define DMA2_Channel_SDIO_IRQn              (DMA2_Channel4_IRQn)

/* Private macro -------------------------------------------------------------*/
#ifdef BSP_SDIO_DEBUG
#define sdio_debug(format,args...)        	rt_kprintf(format, ##args)
#else
#define sdio_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct board_sdio_task_struct *sdio_task;
static struct board_sdio_unit_struct sdio1;
#if (SDIO_MODE & SDIO_CONFIG_DMA_MODE)
static struct board_sdio_dma_mode sdio_dma_tx;
#else
static struct board_sdio_int_mode sdio_int_tx;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Copy From STM32 Example Code ----------------------------------------------*/
/**
  * @brief  Checks for error conditions for CMD0.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

  while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))
  {
    timeout--;
  }

  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}


/**
  * @brief  Checks for error conditions for R7 response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp7Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t timeout = SDIO_CMD0TIMEOUT;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
  {
    timeout--;
    status = SDIO->STA;
  }

  if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
  {
    /*!< Card is not V2.0 complient or card does not support the set voltage range */
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }

  if (status & SDIO_FLAG_CMDREND)
  {
    /*!< Card is SD V2.0 compliant */
    errorstatus = SD_OK;
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    return(errorstatus);
  }
  return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R1 response.
  * @param  cmd: The sent command index.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp1Error(uint8_t cmd)
{
  while (!(SDIO->STA & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
  }

  SDIO->ICR = SDIO_STATIC_FLAGS;

  return (SD_Error)(SDIO->RESP1 &  SD_OCR_ERRORBITS);
}

/**
  * @brief  Checks for error conditions for R3 (OCR) response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp3Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R2 (CID or CSD) response.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp2Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/**
  * @brief  Checks for error conditions for R6 (RCA) response.
  * @param  cmd: The sent command index.
  * @param  prca: pointer to the variable that will contain the SD card relative 
  *         address RCA. 
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Check response received is of desired command */
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*!< We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {
    *prca = (uint16_t) (response_r1 >> 16);
    return(errorstatus);
  }

  if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_R6_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_R6_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  return(errorstatus);
}

/**
  * @brief  Enables or disables the SDIO wide bus mode.
  * @param  NewState: new state of the SDIO wide bus mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /*!< Get SCR Register */
  errorstatus = FindSCR(RCA, scr);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /*!< If wide bus operation to be enabled */
  if (NewState == ENABLE)
  {
    /*!< If requested card supports wide bus operation */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
      SDIO_CmdInitStructure.SDIO_Argument = 0x2;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }   /*!< If wide bus operation to be disabled */
  else
  {
    /*!< If requested card supports 1 bit mode operation */
    if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}

/***************************************************************************//**
 * @brief
 *  DMA for SDIO interrupt handler
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void board_sdio_dma_isr(rt_device_t dev)
{
    struct board_sdio_device *sdio;
    struct board_sdio_dma_mode *dma_tx;
    struct board_sdio_dma_node *dma_node;
    rt_uint32_t level;

    sdio = (struct board_sdio_device *)dev->user_data;
    dma_tx = (struct board_sdio_dma_mode *)sdio->tx_mode;
    dma_node = dma_tx->list_head;

    RT_ASSERT((dev->flag & RT_DEVICE_FLAG_DMA_TX) && (dma_node != RT_NULL));

    /* Invoke call to notify tx complete */
    if (dev->tx_complete != RT_NULL)
    {
        dev->tx_complete(dev, dma_node->data_ptr);
    }

    /* Remove the node */
    level = rt_hw_interrupt_disable();
    dma_tx->list_head = dma_node->next;
    if (dma_tx->list_head == RT_NULL)  /* data link empty */
    {
        dma_tx->list_tail = RT_NULL;
    }
    rt_hw_interrupt_enable(level);

    /* Release the node */
    rt_mp_free(dma_node);

    SPI_I2S_DMACmd(spi->spi_device, SPI_I2S_DMAReq_Tx, DISABLE);
    DMA_Cmd(dma_tx->dma_chn, DISABLE);

    if (dma_tx->list_head != RT_NULL)
    {
        /* Enable DMA TX */
        dma_tx->dma_chn->CMAR = (rt_uint32_t)dma_tx->list_head->data_ptr;
        dma_tx->dma_chn->CNDTR = (rt_uint32_t)dma_tx->list_head->data_size;
        DMA_Cmd(dma_tx->dma_chn, ENABLE);
        SPI_I2S_DMACmd(spi->spi_device, SPI_I2S_DMAReq_Tx, ENABLE);
    }
    else
    {
        /* Reset status */
        spi->status &= ~(rt_uint16_t)SPI_STATUS_TX_BUSY;
    }
}

static rt_err_t board_sdio_dma(
    rt_uint32_t dir,
    struct board_sdio_device *sdio,
    rt_uint32_t *buffer,
    rt_uint16_t size)
{
    struct board_sdio_dma_mode *dma_trx;
    struct board_sdio_dma_node *dma_node;
    rt_uint32_t level;

    dma_trx = (struct board_sdio_dma_mode *)\
        ((dir == DMA_DIR_PeripheralDST)? sdio->tx_mode: sdio->rx_mode);
    
    /* Allocate a data node */
    dma_node = (struct board_sdio_dma_node *)rt_mp_alloc(
            &(dma_trx->dma_mp), RT_WAITING_FOREVER);
    if (dma_node == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    dma_node->data_ptr  = buffer;
    dma_node->data_size = size;
    dma_node->next      = RT_NULL;

    /* Insert the node */
    level = rt_hw_interrupt_disable();

    dma_node->prev = dma_trx->list_tail;
    if (dma_trx->list_tail != RT_NULL)
    {
        dma_trx->list_tail->next = dma_node;
    }
    dma_trx->list_tail = dma_node;

    if (dma_trx->list_head == RT_NULL)
    {
        dma_trx->list_head = dma_node;

        /* Enable DMA TX */
        sdio->status |= (rt_uint16_t)\
            ((dir == DMA_DIR_PeripheralDST)? SDIO_STATUS_TX_BUSY: SDIO_STATUS_RX_BUSY);
        dma_trx->dma_chn->CCR = dma_trx->dma_chn->CCR & ~(0x00000010) | dir;
        dma_trx->dma_chn->CMAR = (rt_uint32_t)buffer;
        dma_trx->dma_chn->CNDTR = (rt_uint32_t)size / 4;
        DMA_Cmd(dma_trx->dma_chn, ENABLE);

        rt_hw_interrupt_enable(level);

        SDIO_DMACmd(ENABLE);
    }
    else
    {
        rt_hw_interrupt_enable(level);
    }

    return RT_EOK;
}

#define board_sdio_dma_tx(sdio, buffer, size) \
    board_sdio_dma(DMA_DIR_PeripheralDST, sdio, buffer, size)
#define board_sdio_dma_rx(sdio, buffer, size) \
        board_sdio_dma(DMA_DIR_PeripheralSRC, sdio, buffer, size)

/***************************************************************************//**
 * @brief
 *  SPI RX data valid interrupt handler
 *
 * @details
 *
 * @note
 *  9-bit SPI mode has not implemented yet and SPI slave mode is untested
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void board_sdio_rx_isr(rt_device_t dev)
{
    struct board_sdio_device     *spi;
    struct board_sdio_int_mode   *int_rx;

    /* interrupt mode receive */
    RT_ASSERT(dev->flag & RT_DEVICE_FLAG_INT_RX);
    spi = (struct board_sdio_device *)(dev->user_data);
    int_rx = (struct board_sdio_int_mode *)(spi->rx_mode);

    /* Set status */
    spi->status |= SPI_STATUS_RX_BUSY;

    /* save into rx buffer */
    while (spi->spi_device->SR & SPI_I2S_FLAG_RXNE)
    {
        rt_base_t level;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        /* save character */
        int_rx->data_ptr[int_rx->save_index] = \
            (rt_uint8_t)(spi->spi_device->DR & 0x00ff);
        int_rx->save_index ++;
        if (int_rx->save_index >= SPI_RX_BUFFER_SIZE)
            int_rx->save_index = 0;

        /* if the next position is read index, discard this 'read char' */
        if (int_rx->save_index == int_rx->read_index)
        {
            if (++int_rx->read_index >= SPI_RX_BUFFER_SIZE)
            {
                int_rx->read_index = 0;
            }
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }

    /* Reset status */
    spi->status &= ~(rt_uint16_t)SPI_STATUS_RX_BUSY;
    /* invoke callback */
    if (dev->rx_indicate != RT_NULL)
    {
        rt_size_t rx_len;

        /* get rx length */
        rx_len = int_rx->read_index > int_rx->save_index ?
            SPI_RX_BUFFER_SIZE - int_rx->read_index + int_rx->save_index : \
            int_rx->save_index - int_rx->read_index;

        dev->rx_indicate(dev, rx_len);
    }
}

/***************************************************************************//**
 * @brief
 *   Open SPI device
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
static rt_err_t sdio_task_exec(union board_sdio_exec_message *exec_msg, rt_bool_t nonblock)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &sdio_tasks->rx_msgs,
            (void *)&exec_msg,
            SDIO_RX_MESSAGE_SIZE);
        if ((ret == -RT_EFULL) && !rt_hw_interrupt_check())
        {
            rt_thread_sleep(SDIO_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        sdio_debug("SDIO err: send cmd failed! (%x)\n", ret);
        return ret;
    }

    if (!nonblock)
    {
        if (rt_hw_interrupt_check())
        {
            ret = rt_event_recv(
                &sdio_tasks->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_NO,
                &exec_msg->ret.cmd);
        }
        else
        {
            ret = rt_event_recv(
                &sdio_tasks->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &exec_msg->ret.cmd);
        }
        if (ret != RT_EOK)
        {
            sdio_debug("SDIO err: receive event failed! (%x)\n", ret);
        }
    }

    return ret;
}

static void sdio_task_open(struct board_sdio_unit_struct *cfg,
   union board_sdio_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_uint16_t oflag;

    oflag = (rt_uint16_t)exec_msg->cmd.other;
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_RDONLY)
    {
        cfg->sdio.status |= SPI_STATUS_READ_ONLY;
    }
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_WRONLY)
    {
        cfg->sdio.status |= SPI_STATUS_WRITE_ONLY;
    }
    if (oflag & (rt_uint16_t)RT_DEVICE_OFLAG_NONBLOCKING)
    {
        cfg->sdio.status |= SPI_STATUS_NONBLOCKING;
    }

    cfg->sdio.counter++;

    sdio_debug("SDIO: Open with flag %x\n", oflag);
    exec_msg->ret.ret = RT_EOK;
}

static void sdio_task_close(struct board_sdio_unit_struct *cfg,
    union board_sdio_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    cfg->sdio.counter--;

    exec_msg->ret.ret = RT_EOK;
}

static void sdio_task_read(struct board_sdio_unit_struct *cfg,
    union board_sdio_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->sdio.status & SDIO_STATUS_WRITE_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }

    pos = (rt_off_t)exec_msg->cmd.other;

    rt_uint8_t inst_len = *(exec_msg->cmd.ptr);
    rt_uint8_t *inst_ptr = exec_msg->cmd.ptr + 1;
    rt_uint8_t *rx_buf = *((rt_uint8_t **)(exec_msg->cmd.ptr + inst_len + 1));
    rt_off_t i;
    rt_uint8_t retry;

    ptr = inst_ptr;
    len = inst_len;
    sdio_debug("SDIO: RX, TX INS (%d)\n", len);
    /* Write instructions */
    while (len)
    {
        
        len--;
    }

    /* Flushing RX */
    *(rt_uint16_t *)0x00 = cfg->spi.spi_device->DR;

    ptr = rx_buf;
    len = exec_msg->cmd.size;
    sdio_debug("SPI%d: RX data (%d)\n", cfg->spi.number, len);
    /* Read data */
    while (len)
    {
        /* dummy write */
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
        cfg->spi.spi_device->DR = 0x00ff;
        /* read a byte of data */
        retry = 0;
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_RXNE))
        {
            if (++retry > SPI_RETRY_TIMES_RX)
            {
                exec_msg->ret.size = exec_msg->cmd.size - len;
                exec_msg->ret.ret = -RT_ERROR;
                return;
            }
        }
        *(ptr++) = (rt_uint8_t)(cfg->spi.spi_device->DR & 0x00ff);
        len--;
    }

    exec_msg->ret.size = exec_msg->cmd.size - len;
    exec_msg->ret.ret = RT_EOK;
}

static void spi_task_write(struct board_sdio_unit_struct *cfg,
    union board_sdio_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->sdio.status & SDIO_STATUS_READ_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }

    pos = (rt_off_t)exec_msg->cmd.other;

    rt_uint8_t inst_len = *(exec_msg->cmd.ptr);
    rt_uint8_t *inst_ptr = exec_msg->cmd.ptr + 1;
    rt_uint8_t *tx_buf = *((rt_uint8_t **)(exec_msg->cmd.ptr + inst_len + 1));

    ptr = inst_ptr;
    len = inst_len;
    sdio_debug("SPI%d: TX INS (%d)\n", cfg->spi.number, len);
    /* Write instructions */
    while (len)
    {
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
        cfg->spi.spi_device->DR = (rt_uint16_t)*(ptr++);
        len--;
    }

    ptr = tx_buf;
    len = exec_msg->cmd.size;
    sdio_debug("SPI%d: TX DATA (%d)\n", cfg->spi.number, len);
    /* Write data */
    if (cfg->device.flag & RT_DEVICE_FLAG_STREAM)
    {
        if (*(ptr + len - 1) == '\n')
        {
            *(ptr + len - 1) = '\r';
            *(ptr + len++) = '\n';
            *(ptr + len) = 0;
        }
    }
    if ((cfg->spi.status & SPI_STATUS_START) && \
        (cfg->device.flag & RT_DEVICE_FLAG_DMA_TX) && (len > 8))
    {
        /* DMA mode */
        sdio_debug("SPI%d: DMA TX DATA\n", cfg->spi.number);
        exec_msg->ret.ret = board_sdio_dma_tx(&cfg->spi,
            (rt_uint32_t *)ptr,
            (rt_uint16_t)len);
        if (exec_msg->ret.ret != RT_EOK)
        {
            exec_msg->ret.size = 0;
            return;
        }
        exec_msg->ret.size = exec_msg->cmd.size;
    }
    else
    {
        /* polling mode */
        sdio_debug("SPI%d: Polling TX DATA\n", cfg->spi.number);
        while (len)
        {
            while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
            cfg->spi.spi_device->DR = (rt_uint16_t)*(ptr++);
            len--;
        }

        exec_msg->ret.size = exec_msg->cmd.size - len;
        exec_msg->ret.ret = RT_EOK;
    }
}

static void sdio_task_control(struct board_sdio_unit_struct *cfg,
    union board_sdio_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    switch (exec_msg->cmd.other)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* Suspend device */
        SDIO_ClockCmd(DISABLE);
        cfg->device.flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* Resume device */
        SDIO_ClockCmd(ENABLE);
        cfg->device.flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_MODE_BLOCKING:
        /* Blocking mode operation */
        cfg->sdio.status &= ~(rt_uint16_t)SDIO_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_MODE_NONBLOCKING:
        /* Non-blocking mode operation */
        cfg->sdio.status |= (rt_uint16_t)SDIO_STATUS_NONBLOCKING;
        break;
    }

    exec_msg->ret.ret = RT_EOK;
}

/***************************************************************************//**
* @brief
*   SDIO device main loop
*
* @details
*
* @note
*
* @param[in] parameter
*   Pointer to board_sdio_unit_struct
*
* @return
*   None
******************************************************************************/
void sdio_task_main_loop(void *parameter)
{
    struct board_sdio_unit_struct *cfg;
    union board_sdio_exec_message *p_exec_msg;
    rt_thread_t self;
    rt_bool_t chk_block;

    cfg = (struct board_sdio_unit_struct *)parameter;
    self = rt_thread_self();

    if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        SPI_RX_MESSAGE_SIZE,
        sizeof(cfg->task.rx_msg_pool),
        RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("SDIO: init mq failed!\n");
        return;
    }

    if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("SDIO: init event failed!\n");
            return;
        }

    /* Enable SDIO */
    SDIO_SetPowerState(SDIO_PowerState_ON);
    SDIO_ClockCmd(ENABLE);
    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;
    cfg->sdio.status |= SDIO_STATUS_START;
    sdio_debug("SDIO: enter main loop\n");

SPI_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
        if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&p_exec_msg,
            SDIO_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        chk_block = RT_FALSE;
        switch (p_exec_msg->cmd.cmd)
        {
        case SDIO_COMMAND_STATUS:
            p_exec_msg->ret.other = (rt_uint32_t)cfg->sdio.status;
            break;

        case SDIO_COMMAND_OPEN:
            sdio_task_open(cfg, p_exec_msg);
            break;

        case SDIO_COMMAND_CLOSE:
            sdio_task_close(cfg, p_exec_msg);
            break;

        case SDIO_COMMAND_READ:
            sdio_task_read(cfg, p_exec_msg);
            chk_block = RT_TRUE;
            break;

        case SDIO_COMMAND_WRITE:
            sdio_task_write(cfg, p_exec_msg);
            chk_block = RT_TRUE;
            break;

        case SDIO_COMMAND_CONTROL:
            sdio_task_control(cfg, p_exec_msg);
            break;

        default:
            break;
        }

        if (chk_block && (cfg->sdio.status & SDIO_STATUS_NONBLOCKING))
        {
            rt_free(p_exec_msg);
            continue;
        }

        rt_event_send(&cfg->task.tx_evts, p_exec_msg->ret.cmd);
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize SDIO device
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
static rt_err_t board_sdio_init (rt_device_t dev)
{
    return rt_thread_startup(&sdio_task->thread);
}

/***************************************************************************//**
 * @brief
 *   Open SDIO device
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
static rt_err_t board_sdio_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct board_sdio_device *sdio;
    union board_sdio_exec_message exec_msg;

    sdio = (struct board_sdio_device *)(dev->user_data);

    exec_msg.cmd.cmd = SDIO_COMMAND_OPEN;
    exec_msg.cmd.other = oflag;

    if (!(sdio->status & SDIO_STATUS_START) && \
    (sdio->status & SDIO_STATUS_DIRECT_EXE))
    {
        sdio_task_open(&sdio1, &exec_msg);

        return RT_EOK;
    }
    
    return sdio_task_exec(&exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *   Close SDIO device
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
static rt_err_t board_sdio_close(rt_device_t dev)
{
    struct board_sdio_device *sdio;
    union board_sdio_exec_message exec_msg;

    sdio = (struct board_sdio_device *)(dev->user_data);

    if (!(sdio->status & SPI_STATUS_START) && \
    (sdio->status & SPI_STATUS_DIRECT_EXE))
    {
        sdio_task_close(&sdio1, &exec_msg);

        return RT_EOK;
    }

    exec_msg.cmd.cmd = SPI_COMMAND_CLOSE;
    return sdio_task_exec(&exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *  Read from SDIO device
 *
 * @details
 *
 * @note
 *  Slave mode has not tested yet
 *
 * @param[in] dev
 *  Pointer to device descriptor
 *
 * @param[in] pos
 *  Offset
 *
 * @param[in] buffer
 *  Poniter to the buffer
 *
 * @param[in] size
 *  Buffer size in byte
 *
 * @return
 *  Number of read bytes
 ******************************************************************************/
static rt_size_t board_sdio_read (
    rt_device_t     dev,
    rt_off_t        pos,
    void            *buffer,
    rt_size_t       size)

{
    rt_err_t    ret;
    struct board_sdio_device *sdio;
    union board_sdio_exec_message exec_msg;
    rt_bool_t nonblock;

    sdio = (struct board_sdio_device *)(dev->user_data);

    if (!(sdio->status & SDIO_STATUS_START) && \
        (sdio->status & SDIO_STATUS_DIRECT_EXE))
    {
        while(sdio1.sdio.status & SDIO_STATUS_RX_BUSY);

        exec_msg.cmd.cmd = SDIO_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        spi_task_read(&sdio1, &exec_msg);

        return RT_EOK;
    }

    do
    {
        union board_sdio_exec_message *p_exec_msg;
        
        exec_msg.cmd.cmd = SDIO_COMMAND_STATUS;
        do
        {
            ret = sdio_task_exec(&exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & SDIO_STATUS_RX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(SDIO_COMMAND_WAIT_TIME);
                }
            }
            else
            {
                break;
            }
        } while (1);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & SDIO_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
            p_exec_msg = rt_malloc(sizeof(union board_sdio_exec_message));
            if (p_exec_msg == RT_NULL)
            {
                ret = RT_ENOMEM;
                break;
            }
        }
        else
        {
            nonblock = RT_FALSE;
            p_exec_msg = &exec_msg;
        }

        /* Lock device */
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&sdio->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&sdio->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        p_exec_msg->cmd.cmd = SDIO_COMMAND_READ;
        p_exec_msg->cmd.size = size;
        p_exec_msg->cmd.ptr = (rt_uint8_t *)buffer;
        p_exec_msg->cmd.other = (rt_uint32_t)pos;
        ret = sdio_task_exec(p_exec_msg, nonblock);
        {
            rt_sem_release(&sdio->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&sdio->lock);
    } while (0);

    if (ret != RT_EOK)
    {
        rt_set_errno(ret);
        return 0;
    }
    if (exec_msg.ret.ret != RT_EOK)
    {
        rt_set_errno(exec_msg.ret.ret);
    }
    return exec_msg.ret.size;
}

/***************************************************************************//**
 * @brief
 *   Write to SDIO device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] pos
 *   Offset
 *
 * @param[in] buffer
 *   Poniter to the buffer
 *
 * @param[in] size
 *   Buffer size in byte
 *
 * @return
 *   Number of written bytes
 ******************************************************************************/
static rt_size_t board_sdio_write (
    rt_device_t     dev,
    rt_off_t        pos,
    const void*     buffer,
    rt_size_t       size)

{
    rt_err_t    ret;
    struct board_sdio_device *sdio;
    union board_sdio_exec_message exec_msg;
    rt_bool_t nonblock;

    sdio = (struct board_sdio_device *)(dev->user_data);

    if (!(sdio->status & SDIO_STATUS_START) && \
        (sdio->status & SDIO_STATUS_DIRECT_EXE))
    {
        while(sdio1.sdio.status & SDIO_STATUS_RX_BUSY);

        exec_msg.cmd.cmd = SDIO_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        spi_task_write(cfg, &exec_msg);

        return RT_EOK;
    }

    do
    {
        union board_sdio_exec_message *p_exec_msg;
    
        exec_msg.cmd.cmd = SDIO_COMMAND_STATUS;
        do
        {
            ret = sdio_task_exec(&exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & SDIO_STATUS_TX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(SDIO_COMMAND_WAIT_TIME);
                }
            }
            else
            {
                break;
            }
        } while (1);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & SDIO_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
            p_exec_msg = rt_malloc(sizeof(union board_sdio_exec_message));
            if (p_exec_msg == RT_NULL)
            {
                ret = RT_ENOMEM;
                break;
            }
        }
        else
        {
            nonblock = RT_FALSE;
            p_exec_msg = &exec_msg;
        }

        /* Lock device */
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&sdio->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&sdio->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        p_exec_msg->cmd.cmd = SDIO_COMMAND_WRITE;
        p_exec_msg->cmd.size = size;
        p_exec_msg->cmd.ptr = (rt_uint8_t *)buffer;
        p_exec_msg->cmd.other = (rt_uint32_t)pos;
        ret = spi_task_exec(p_exec_msg, nonblock);
        {
            rt_sem_release(&sdio->lock);
            break;
        }

        /* Unlock device */
        rt_sem_release(&sdio->lock);
    } while (0);

    if (ret != RT_EOK)
    {
        rt_set_errno(ret);
        return 0;
    }
    if (exec_msg.ret.ret != RT_EOK)
    {
        rt_set_errno(exec_msg.ret.ret);
    }
    return exec_msg.ret.size;
}

/***************************************************************************//**
* @brief
*   Configure SDIO device
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
static rt_err_t board_sdio_control (
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    rt_err_t ret;
    struct board_sdio_device *sdio;
    union board_sdio_exec_message exec_msg;

    sdio = (struct board_sdio_device *)(dev->user_data);

    if (!(sdio->status & SDIO_STATUS_START) && \
    (sdio->status & SDIO_STATUS_DIRECT_EXE))
    {
        sdio_task_control(&sdio1, &exec_msg);

        return RT_EOK;
    }

    /* Lock device */
    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&sdio->lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&sdio->lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return 0;
    }

    exec_msg.cmd.cmd = SDIO_COMMAND_CONTROL;
    exec_msg.cmd.ptr = (rt_uint8_t *)args;
    exec_msg.cmd.other = (rt_uint32_t)cmd;
    ret = spi_task_exec(&exec_msg, RT_FALSE);

    /* Unlock device */
    rt_sem_release(&sdio->lock);

    return ret;
}

/***************************************************************************//**
* @brief
*   Initialize the specified SDIO unit
*
* @details
*
* @note
*
* @param[in] init
*   Initialization structure
*
* @return
*   Error code
******************************************************************************/
static rt_err_t board_sdio_unit_init(struct board_sdio_unit_init *init)
{
    /*          STM32 SDIO Pin assignment
     *          =========================    
     *          +-----------------------------------------------------------+
     *          |                     Pin assignment                        |
     *          +-----------------------------+---------------+-------------+
     *          |  STM32 SDIO Pins            |     SD        |    Pin      |
     *          +-----------------------------+---------------+-------------+
     *          |      SDIO D2                |   D2          |    1        |
     *          |      SDIO D3                |   D3          |    2        |
     *          |      SDIO CMD               |   CMD         |    3        |
     *          |                             |   VCC         |    4 (3.3 V)|
     *          |      SDIO CLK               |   CLK         |    5        |
     *          |                             |   GND         |    6 (0 V)  |
     *          |      SDIO D0                |   D0          |    7        |
     *          |      SDIO D1                |   D1          |    8        |  
     *          +-----------------------------+---------------+-------------+  
     */
    SD_Error            ret = SD_OK;
    rt_device_t         device;
    struct board_sdio_device *sdio;
    struct board_sdio_dma_mode *dma_tx;
    struct board_sdio_int_mode *int_tx;
    rt_uint32_t         flag;
    GPIO_InitTypeDef    gpio_init;
    SDIO_InitTypeDef    sdio_init;
    SDIO_CmdInitTypeDef cmd_init;
    rt_uint32_t         sd_type = SD_STD_CAPACITY;
    rt_uint32_t         response = 0;
    rt_uint32_t         count = 0;
    rt_uint32_t         valid_voltage = 0;
    // SD_CardInfo         sd_info;
    DMA_InitTypeDef     dma_init;
    NVIC_InitTypeDef    nvic_init;
    rt_uint32_t         tx_irq;
    rt_uint8_t          tx_chn;
    bsp_irq_hook_init_t hook;

    device = &(init->unit)->device;
    sdio = &(init->unit)->sdio;
    sdio->card_type = SDIO_STD_CAPACITY_SD_CARD_V1_1;
    dma_tx = (struct board_sdio_dma_mode *)sdio->tx_mode;
    int_tx = (struct board_sdio_int_mode *)sdio->tx_mode;
    flag = RT_DEVICE_FLAG_RDWR;

    do
    {
        sdio->status        = init->config & SDIO_STATUS_MASK;
        sdio->sdio_device   = SDIO;
        
        /* Init lock */
        if (rt_sem_init(&sdio->lock, init->name, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }
        
        /* Enable clock */
        RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_AHBPeriph_SDIO), 
            ENABLE);

        if (init->config & SDIO_CONFIG_DMA_MODE)
        {
            /* Pre-config DMA */
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
            dma_tx->dma_chn = DMA2_Channel_SDIO;
            tx_irq          = DMA2_Channel_SDIO_IRQn;
            tx_chn          = DMA2_Channel_SDIO_Num;
        }

        /* Config GPIO */
        gpio_init.GPIO_Speed    = GPIO_Speed_50MHz;
        /* D0, D1, D2, D3, CLK */
        gpio_init.GPIO_Pin      = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
        gpio_init.GPIO_Mode     = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &gpio_init);
        /* CMD */
        gpio_init.GPIO_Pin      = GPIO_Pin_2;
        GPIO_Init(GPIOD, &gpio_init);

        /* Init SDIO unit */
        SDIO_ClockCmd(DISABLE);
        SDIO_SetPowerState(SDIO_PowerState_OFF);
        SDIO_DeInit();
        sdio_init.SDIO_ClockDiv             = SDIO_TRANSFER_CLK_DIV; 
        sdio_init.SDIO_ClockEdge            = SDIO_ClockEdge_Rising;
        sdio_init.SDIO_ClockBypass          = SDIO_ClockBypass_Disable;
        sdio_init.SDIO_ClockPowerSave       = SDIO_ClockPowerSave_Disable;
        sdio_init.SDIO_BusWide              = SDIO_BusWide_1b;
        sdio_init.SDIO_HardwareFlowControl  = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&sdio_init);
        SDIO_SetPowerState(SDIO_PowerState_ON);
        SDIO_ClockCmd(ENABLE);
        
        /* CMD0: GO_IDLE_STATE */
        /* No CMD response required */
        cmd_init.SDIO_Argument = 0x0;
        cmd_init.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE;
        cmd_init.SDIO_Response = SDIO_Response_No;
        cmd_init.SDIO_Wait = SDIO_Wait_No;
        cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand(&cmd_init);
        ret = CmdError();
        if (ret != SD_OK)
        {
            break;
        }
        
        /* CMD8: SEND_IF_COND */
        /* Send CMD8 to verify SD card interface operating condition */
        /* Argument: - [31:12]: Reserved (shall be set to '0')
                     - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
                     - [7:0]: Check Pattern (recommended 0xAA) */
        /* CMD Response: R7 */
        cmd_init.SDIO_Argument = SD_CHECK_PATTERN;
        cmd_init.SDIO_CmdIndex = SDIO_SEND_IF_COND;
        cmd_init.SDIO_Response = SDIO_Response_Short;
        cmd_init.SDIO_Wait = SDIO_Wait_No;
        cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand(&cmd_init);
        ret = CmdResp7Error();
        if (ret == SD_OK)
        {
            sdio->card_type = SDIO_STD_CAPACITY_SD_CARD_V2_0; /* SD Card 2.0 */
            sd_type = SD_HIGH_CAPACITY;
        }
        else
        {
            /* CMD55 */
            cmd_init.SDIO_Argument = 0x00;
            cmd_init.SDIO_CmdIndex = SD_CMD_APP_CMD;
            cmd_init.SDIO_Response = SDIO_Response_Short;
            cmd_init.SDIO_Wait = SDIO_Wait_No;
            cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand(&cmd_init);
            ret = CmdResp1Error(SD_CMD_APP_CMD);
        }
        /* CMD55 */
        cmd_init.SDIO_Argument = 0x00;
        cmd_init.SDIO_CmdIndex = SD_CMD_APP_CMD;
        cmd_init.SDIO_Response = SDIO_Response_Short;
        cmd_init.SDIO_Wait = SDIO_Wait_No;
        cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
        SDIO_SendCommand(&cmd_init);
        ret = CmdResp1Error(SD_CMD_APP_CMD);

        /* If ret is Command TimeOut, it is a MMC card */
        /* If ret is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch) or SD card 1.x */
        if (ret == SD_OK)
        {
            /* SD CARD */
            /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
            while ((!valid_voltage) && (count < SD_MAX_VOLT_TRIAL))
            {
                /* SEND CMD55 APP_CMD with RCA as 0 */
                cmd_init.SDIO_Argument = 0x00;
                cmd_init.SDIO_CmdIndex = SD_CMD_APP_CMD;
                cmd_init.SDIO_Response = SDIO_Response_Short;
                cmd_init.SDIO_Wait = SDIO_Wait_No;
                cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
                SDIO_SendCommand(&cmd_init);
                ret = CmdResp1Error(SD_CMD_APP_CMD);
                if (ret != SD_OK)
                {
                    break;
                }
                
                cmd_init.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | sd_type;
                cmd_init.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
                cmd_init.SDIO_Response = SDIO_Response_Short;
                cmd_init.SDIO_Wait = SDIO_Wait_No;
                cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
                SDIO_SendCommand(&cmd_init);
                ret = CmdResp3Error();
                if (ret != SD_OK)
                {
                    break;
                }

                response = SDIO_GetResponse(SDIO_RESP1);
                valid_voltage = (((response >> 31) == 1) ? 1 : 0);
                count++;
            }
            if (ret != SD_OK)
            {
                break;
            }
            if (count >= SD_MAX_VOLT_TRIAL)
            {
              ret = SD_INVALID_VOLTRANGE;
              break;
            }

            if (response &= SD_HIGH_CAPACITY)
            {
              sdio->card_type = SDIO_HIGH_CAPACITY_SD_CARD;
            }
        }/* else MMC Card */
        

        /* Config DMA transfer */
        if (init->config & SDIO_CONFIG_DMA_MODE)
        {
            /* Init DMA TX queue */
            dma_tx->list_head = dma_tx->list_tail = RT_NULL;

            /* Init memory pool */
            if (rt_mp_init(&dma_tx->dma_mp,
                init->name,
                dma_tx->mem_pool,
                sizeof(dma_tx->mem_pool),
                sizeof(struct board_sdio_dma_node)) != RT_EOK)
            {
                break;
            }

            /* Config DMA */
            DMA_DeInit(dma_tx->dma_chn);
            dma_init.DMA_PeripheralBaseAddr = sdio->sdio_device->FIFO;
            dma_init.DMA_MemoryBaseAddr     = (uint32_t)0x00;
            dma_init.DMA_DIR                = DMA_DIR_PeripheralDST;    // RX: DMA_DIR_PeripheralSRC
            dma_init.DMA_BufferSize         = 1;
            dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
            dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable;
            dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
            dma_init.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
            dma_init.DMA_Mode               = DMA_Mode_Normal;
            dma_init.DMA_Priority           = DMA_Priority_High;
            dma_init.DMA_M2M                = DMA_M2M_Disable;
            DMA_Init(dma_tx->dma_chn, &dma_init);

            /* Config hook */
            hook.type       = bsp_irq_type_dma;
            hook.unit       = tx_chn;
            hook.cbFunc     = board_sdio_dma_isr;
            hook.userPtr    = device;
            bsp_irq_hook_register(&hook);

            /* Enable interrupt and NVIC */
            SDIO_ClearFlag(0xFFFFFFFF);
            DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);
            DMA_ITConfig(dma_tx->dma_chn, DMA_IT_TC | DMA_IT_TE, ENABLE);
            SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
            // SDIO_DMACmd(ENABLE);
            NVIC_ClearPendingIRQ(tx_irq);
            nvic_init.NVIC_IRQChannel = tx_irq;
            nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
            nvic_init.NVIC_IRQChannelSubPriority = BSP_IRQ_PRI_DMA;
            nvic_init.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&nvic_init);
        }
        else /* Config INT transfer */
        {
            /* Init INT RX structure */
            int_tx->read_index = 0;
            int_tx->save_index = 0;

            /* Config hook */
            hook.type       = bsp_irq_type_sdio;
            hook.unit       = 0;
            hook.cbFunc     = board_sdio_int_tx_isr;
            hook.userPtr    = device;
            bsp_irq_hook_register(&hook);

            /* Enable interrupt and NVIC */
            SDIO_ClearFlag(0xFFFFFFFF);
            SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
            NVIC_ClearPendingIRQ(SDIO_IRQn);
            nvic_init.NVIC_IRQChannel = SDIO_IRQn;
            nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
            nvic_init.NVIC_IRQChannelSubPriority = BSP_IRQ_PRI_COM;
            nvic_init.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&nvic_init);
        }

        /* Config and register device */
        device->type        = RT_Device_Class_SDIO;
        device->rx_indicate = RT_NULL;
        device->tx_complete = RT_NULL;
        device->init        = board_sdio_init;
        device->open        = board_sdio_open;
        device->close       = board_sdio_close;
        device->read        = board_sdio_read;
        device->write       = board_sdio_write;
        device->control     = board_sdio_control;
        device->user_data   = (void *)sdio;

        if (init->config & SDIO_CONFIG_DMA_MODE)
        {
            flag |= RT_DEVICE_FLAG_DMA_TX;
        }
        return rt_device_register(device, init->name, flag);
    } while(0);

    rt_kprintf("SDIO err: Unit init failed (%d)!\n", ret);
    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize all SDIO module related hardware and register device to kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t board_hw_sdio_init(void)
{
    struct board_sdio_unit_init init;

    do
    {
        sdio_task           = &sdio1.task;
#if (SDIO_MODE & SDIO_CONFIG_DMA_MODE)
        sdio1.sdio.tx_mode  = (void *)&sdio_dma_tx;
#else
        sdio1.sdio.tx_mode  = (void *)&sdio_int_rx;
#endif
        init.config         = SDIO_MODE;
        init.name           = SDIO_NAME;
        init.unit           = &sdio1;
        if (board_sdio_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &sdio1.task.thread,
            init.name,
            sdio_task_main_loop,
            (void *)&sdio1,
            (void *)&sdio1.task.stack,
            sizeof(sdio1.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }

        sdio_debug("SDIO: H/W init OK!\n");
        return RT_EOK;
    } while (0);

    rt_kprintf("SDIO err: H/W init failed!\n");
    return -RT_ERROR;
}

#endif /* defined(BSP_USING_SDIO) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
