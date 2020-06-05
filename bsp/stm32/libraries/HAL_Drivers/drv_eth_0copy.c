/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-06-01     onelife      add zero-copy support
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "drv_config.h"
#include "drv_eth.h"
#define LOG_TAG "drv.eth"
#include "drv_log.h"

#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "netif/ethernetif.h"

#ifndef BSP_USING_ETH_0COPY
#error BSP_USING_ETH_0COPY is not enabled
#endif
#ifndef LWIP_NO_RX_THREAD
#error LWIP_NO_RX_THREAD should enable for zero-copy
#endif
#ifndef LWIP_NO_TX_THREAD
#error LWIP_NO_TX_THREAD should enable for zero-copy
#endif

/* Private define ------------------------------------------------------------*/
#define ETH_RXBUFQSZ (ETH_RXBUFNB + ETH_RXBUFNB / 2)

/* Private typedef -----------------------------------------------------------*/
typedef struct {
  rt_uint32_t front, rear, size;
  rt_uint32_t capacity;
  void **array;
} queue_t;

typedef struct lwip_custom_pbuf {
    struct pbuf_custom custom_p;
    uint8_t *buf;
} lwip_custom_pbuf_t;

struct rt_stm32_eth {
    /* inherit from ethernet device */
    struct eth_device parent;
#ifndef PHY_USING_INTERRUPT_MODE
    struct rt_timer poll_timer;
#endif
    rt_uint8_t mac[NETIF_MAX_HWADDR_LEN];
    rt_bool_t linkup;
    rt_bool_t duplex;
    rt_uint8_t speed;
};

/* Private macro -------------------------------------------------------------*/
#define queue_init(q, b)                     \
  {                                          \
    q.front = 0;                             \
    q.rear = sizeof(b) / sizeof(void *) - 1; \
    q.size = 0;                              \
    q.capacity = q.rear + 1;                 \
    q.array = b;                             \
  }

/* Private variables ---------------------------------------------------------*/
static ETH_DMADescTypeDef txDescTab[ETH_TXBUFNB]
    __attribute__((aligned(4), section(".fast")));
static ETH_DMADescTypeDef rxDescTab[ETH_RXBUFNB]
    __attribute__((aligned(4), section(".fast")));
static rt_uint8_t rxBuf[ETH_RXBUFQSZ * ETH_RX_BUF_SIZE]
    __attribute__((aligned(4), section(".mac")));
LWIP_MEMPOOL_DECLARE(PBUF_RX, ETH_RXBUFQSZ, sizeof(lwip_custom_pbuf_t),
    "RX pbuf pool");
static void *_rx_buf_queue[ETH_RXBUFQSZ];
static void *_tx_free_queue[ETH_TXBUFNB];
static queue_t rx_buf_queue;
static queue_t tx_free_queue;

static  ETH_HandleTypeDef ethHdl;
static struct rt_stm32_eth ethDev;

/* External function prototypes ----------------------------------------------*/
extern void phy_reset(void);

/* Private function prototypes -----------------------------------------------*/
static rt_bool_t is_full(queue_t *queue);
static rt_bool_t is_empty(queue_t *queue);
static rt_bool_t queue_put(queue_t *queue, void *item);
static void *queue_get(queue_t *queue);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief Custom pbuf free function
 * @retval None
 */
static void lwip_custom_pbuf_free(struct pbuf *p) {
  lwip_custom_pbuf_t *cp = (lwip_custom_pbuf_t *)p;
  // rt_base_t level;

  // level = rt_hw_interrupt_disable();
  (void)queue_put(&rx_buf_queue, (void *)cp->buf);
  // rt_hw_interrupt_enable(level);
  LWIP_MEMPOOL_FREE(PBUF_RX, cp);
}

static void link_monitor(void *param) {
  uint32_t bsr;
  rt_bool_t linkup;
  (void)param;

  HAL_ETH_ReadPHYRegister(&ethHdl, PHY_BSR, &bsr);
  linkup =
      (bsr & (PHY_AUTONEGO_COMPLETE | PHY_LINKED_STATUS)) ? RT_TRUE : RT_FALSE;

  if (linkup) {
    uint32_t sr;

    linkup = RT_TRUE;
    HAL_ETH_ReadPHYRegister(&ethHdl, PHY_SR, &sr);
    ethDev.speed = PHY_Status_SPEED_100M(sr) ? 100 : 10;
    ethDev.duplex = PHY_Status_FULL_DUPLEX(sr) ? RT_TRUE : RT_FALSE;
  }

  if (ethDev.linkup != linkup) {
    ethDev.linkup = linkup;
    if (linkup)
      netif_set_link_up(ethDev.parent.netif);
    else
      netif_set_link_down(ethDev.parent.netif);
  }
}

static rt_err_t rt_stm32_eth_init(rt_device_t dev) {
  do {
    /* enable interrupt */
    HAL_NVIC_SetPriority(ETH_IRQn, 0x07, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
    /* start eth */
    if (HAL_OK != HAL_ETH_Start(&ethHdl)) break;
    /* start link monitor */
    if (RT_EOK != rt_timer_start(&ethDev.poll_timer)) break;

    LOG_D("eth init ok");
    return RT_EOK;
  } while (0);

  LOG_E("eth init err");
  return -RT_ERROR;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag) {
  LOG_D("eth open");
  return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev) {
  LOG_D("eth close");
  return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void *buffer,
                                   rt_size_t size) {
  LOG_D("eth read");
  rt_set_errno(-RT_ENOSYS);
  return 0;
}

static rt_size_t rt_stm32_eth_write(rt_device_t dev, rt_off_t pos,
                                    const void *buffer, rt_size_t size) {
  LOG_D("eth write");
  rt_set_errno(-RT_ENOSYS);
  return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, int cmd, void *args) {
  switch (cmd) {
    case NIOCTL_GADDR:
      /* get mac address */
      if (args)
        rt_memcpy(args, ethDev.mac, NETIF_MAX_HWADDR_LEN);
      else
        return -RT_ERROR;
      break;

    default:
      break;
  }

  return RT_EOK;
}

/**
 * @brief Ethernet TX API function
 * @retval Error code
 */
static rt_err_t rt_stm32_eth_tx(rt_device_t dev, struct pbuf *p) {
  /*! \note
   * The assumption is ETH_TX_BUF_SIZE bigger than the largest frame length.
   */
  rt_err_t ret;

  /* free tx_free_queue */
  if (((ethHdl.Instance->DMASR & ETH_DMASR_TPS) == ETH_DMASR_TPS_Stopped) ||
      ((ethHdl.Instance->DMASR & ETH_DMASR_TPS) == ETH_DMASR_TPS_Suspended)) {
    while (!is_empty(&tx_free_queue)) {
      struct pbuf *done_p = (struct pbuf *)queue_get(&tx_free_queue);
      if (done_p && done_p->ref) pbuf_free(done_p);
    }
  }

  if (__HAL_ETH_DMA_GET_FLAG(&ethHdl, ETH_DMASR_TUS))
    __HAL_ETH_DMA_CLEAR_FLAG(&ethHdl, ETH_DMASR_TUS);

  ret = RT_ERROR;
  do {
    if (p->len != p->tot_len) {
      LOG_D("tx clone %d %d", p->len, p->tot_len);
      /* merge to single pbuf */
      if (RT_NULL == (p = pbuf_clone(PBUF_RAW, PBUF_RAM, p))) {
        LOG_E("tx pbuf_clone err");
        break;
      }
    } else {
      pbuf_ref(p);
    }

#if ETH_PAD_SIZE
    if (pbuf_header(p, ETH_PAD_SIZE)) break;
#endif

    /* put pbuf into tx_free_queue */
    if (!queue_put(&tx_free_queue, (void *)p)) {
      pbuf_free(p);
      LOG_E("tx_free_queue full");
      break;
    }

    /* wait for lock */
    while (ethHdl.Lock == HAL_LOCKED) {
      LOG_E("tx waiting");
      rt_thread_mdelay(10);
    }

    /* do tx */
    ethHdl.TxDesc->Buffer1Addr = (rt_uint32_t)p->payload;
    if (HAL_OK != HAL_ETH_TransmitFrame(&ethHdl, p->tot_len)) {
      LOG_E("tx err");
      break;
    }

    ret = RT_EOK;
  } while (0);

  return ret;
}

#if 0
static struct pbuf *rt_stm32_eth_rx(rt_device_t dev) {
  /*! \note
   * The assumption is ETH_RX_BUF_SIZE bigger than the largest frame length.
   */
    struct pbuf *p = RT_NULL;

    LOG_W("eth_rx");
    do {
        uint8_t i;
        uint8_t *newBuf;
        lwip_custom_pbuf_t *cp;

        /* get data */
        if (HAL_OK != HAL_ETH_GetReceivedFrame_IT(&ethHdl))
            break;
        if (ethHdl.RxFrameInfos.SegCount < 1)
            break;
        if (ethHdl.RxFrameInfos.SegCount > 1) {
            LOG_W("SegCount %d", ethHdl.RxFrameInfos.SegCount);
            /* too large, then drop */
            ETH_DMADescTypeDef *rxDesc = ethHdl.RxFrameInfos.FSRxDesc;
            for (i = 0; i < ethHdl.RxFrameInfos.SegCount; i++) {
                rxDesc->Status |= ETH_DMARXDESC_OWN;
                rxDesc = (ETH_DMADescTypeDef *)(rxDesc->Buffer2NextDescAddr);
            }
            break;
        }

        /* prepare new rx buf */
        newBuf = queue_get(&rx_buf_queue);
        if (RT_NULL == newBuf) {
            LOG_W("rx_buf_queue empty");
            break;
        }

        /* get custom pbuf */
        cp = (lwip_custom_pbuf_t *)LWIP_MEMPOOL_ALLOC(PBUF_RX);
        if (RT_NULL == cp) {
            LOG_W("PBUF_RX empty");
            if (RT_NULL != newBuf)
                queue_put(&rx_buf_queue, newBuf);
            break;
        }
        cp->custom_p.custom_free_function = lwip_custom_pbuf_free;
        cp->buf = (uint8_t *)ethHdl.RxFrameInfos.buffer;

        /* get pbuf */
        p = pbuf_alloced_custom(PBUF_RAW, ethHdl.RxFrameInfos.length,
                                         PBUF_REF,
                                         &cp->custom_p,
                                         (void *)ethHdl.RxFrameInfos.buffer,
                                         ETH_RX_BUF_SIZE);
        if (RT_NULL == p) {
            LOG_W("custom pbuf empty");
            if (RT_NULL != newBuf)
                queue_put(&rx_buf_queue, newBuf);
            if (RT_NULL != cp)
                LWIP_MEMPOOL_FREE(PBUF_RX, cp);
            break;
        }

        ethHdl.RxFrameInfos.FSRxDesc->Buffer1Addr = (rt_uint32_t)newBuf;
        ethHdl.RxFrameInfos.FSRxDesc->Status |= ETH_DMARXDESC_OWN;
        ethHdl.RxFrameInfos.SegCount = 0;
    } while (0);

    if (__HAL_ETH_DMA_GET_FLAG(&ethHdl, ETH_DMASR_RBUS))
      __HAL_ETH_DMA_CLEAR_FLAG(&ethHdl, ETH_DMASR_RBUS);

    return p;
}
#endif

/** @defgroup QueueByArray Simple queue implemented by array
  * @brief  https://www.geeksforgeeks.org/queue-set-1introduction-and-array-implementation/ 
  * @{
  */

/**
  * @brief  Check if queue full (when size becomes equal to the capacity)
  * @param  queue pointer to "queue_t" structure
  * @retval status:   RT_TRUE   Queue is full
  *                   RT_FALSE  Queue is not full
  */
static rt_bool_t is_full(queue_t *queue) { return (queue->size == queue->capacity); }

/**
  * @brief  Check if queue empty (when size is 0)
  * @param  queue pointer to "queue_t" structure
  * @retval status:   RT_TRUE   Queue is empty
  *                   RT_FALSE  Queue is not empty
  */
static rt_bool_t is_empty(queue_t *queue) { return (queue->size == 0); }

/**
  * @brief  Add an item to queue. It changes rear and size.
  * @param  queue pointer to "queue_t" structure
  * @param  item  pointer to item
  * @retval status:   RT_TRUE   Operation succeeded
  *                   RT_FALSE  Operation failed
  */
static rt_bool_t queue_put(queue_t *queue, void *item) {
  if (queue->size == queue->capacity) return RT_FALSE;
  queue->rear = (queue->rear + 1) % queue->capacity;
  queue->array[queue->rear] = item;
  queue->size = queue->size + 1;
  return RT_TRUE;
}

/**
  * @brief  Remove an item from queue. It changes front and size.
  * @param  queue pointer to "queue_t" structure
  * @retval item pointer to item or RT_NULL when queue is empty
  */
static void *queue_get(queue_t *queue) {
  void *item;
  if (queue->size == 0) return RT_NULL;
  item = queue->array[queue->front];
  queue->front = (queue->front + 1) % queue->capacity;
  queue->size = queue->size - 1;
  return item;
}
/**
  * @}
  */

/* Public functions ----------------------------------------------------------*/
/**
 * @brief Ethernet H/W init
 * @retval Error code
 */
int rt_hw_stm32_eth_init(void) {
  queue_init(rx_buf_queue, _rx_buf_queue);
  queue_init(tx_free_queue, _tx_free_queue);
  LWIP_MEMPOOL_INIT(PBUF_RX);

  phy_reset();

  do {
    uint8_t i;
    rt_uint32_t temp;
    uint8_t addr = 0xFF;

    /* handle init */
    ethHdl.Instance = ETH;
    ethHdl.Init.MACAddr = (rt_uint8_t *)&ethDev.mac[0];
    // ethHdl.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
    ethHdl.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
    ethHdl.Init.Speed = ETH_SPEED_10M;
    ethHdl.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
    ethHdl.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
    ethHdl.Init.RxMode = ETH_RXINTERRUPT_MODE;
#ifdef RT_LWIP_USING_HW_CHECKSUM
    ethHdl.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
#else
    ethHdl.Init.ChecksumMode = ETH_CHECKSUM_BY_SOFTWARE;
#endif
#ifdef RT_PHY_ADDR
    ethHdl.Init.PhyAddress = RT_PHY_ADDR;
#else
    /* search PHY chip */
    for (i = 0; i <= 0x1F; i++) {
      ethHdl.Init.PhyAddress = i;
      HAL_ETH_ReadPHYRegister(&ethHdl, PHY_ID1_REG, (rt_uint32_t *)&temp);
      if ((0xFFFF != temp) && (0x0000 != temp)) {
        addr = i;
        break;
      }
    }
    if (0xFF == addr) {
      LOG_E("No PHY detected");
      break;
    }
#endif

    ethDev.linkup = RT_FALSE;
    ethDev.duplex = RT_FALSE;
    ethDev.speed = 0;
    /* STMICROELECTRONICS OUI: 00-80-E1 */
    ethDev.mac[0] = 0x00;
    ethDev.mac[1] = 0x80;
    ethDev.mac[2] = 0xE1;
    /* generate MAC addr from 96bit unique ID (only for test). */
    ethDev.mac[3] = *(rt_uint8_t *)(UID_BASE + 4);
    ethDev.mac[4] = *(rt_uint8_t *)(UID_BASE + 2);
    ethDev.mac[5] = *(rt_uint8_t *)(UID_BASE + 0);
    ethDev.parent.parent.init = rt_stm32_eth_init;
    ethDev.parent.parent.open = rt_stm32_eth_open;
    ethDev.parent.parent.close = rt_stm32_eth_close;
    ethDev.parent.parent.read = rt_stm32_eth_read;
    ethDev.parent.parent.write = rt_stm32_eth_write;
    ethDev.parent.parent.control = rt_stm32_eth_control;
    ethDev.parent.parent.user_data = RT_NULL;
    ethDev.parent.eth_rx = RT_NULL;
    ethDev.parent.eth_tx = rt_stm32_eth_tx;
    rt_timer_init(&ethDev.poll_timer, "lnkMon", link_monitor, RT_NULL,
                  RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);

    __HAL_RCC_ETH_CLK_ENABLE();

    /* eth init */
    (void)HAL_ETH_DeInit(&ethHdl);
    if (HAL_OK != HAL_ETH_Init(&ethHdl)) break;
    /* rxBuf is dummy param */
    (void)HAL_ETH_DMATxDescListInit(&ethHdl, txDescTab, rxBuf, ETH_TXBUFNB);
    (void)HAL_ETH_DMARxDescListInit(&ethHdl, rxDescTab, rxBuf, ETH_RXBUFNB);
    for (i = 0; i < ETH_RXBUFQSZ; i++) {
      if (i < ETH_RXBUFNB) {
        rxDescTab[i].Buffer1Addr = (rt_uint32_t)(&rxBuf[i * ETH_RX_BUF_SIZE]);
      } else {
        queue_put(&rx_buf_queue, (void *)(&rxBuf[i * ETH_RX_BUF_SIZE]));
      }
    }

    /* register eth device */
    if (RT_EOK != eth_device_init(&(ethDev.parent), "e0")) break;

    LOG_D("eth h/w init ok");
    return RT_EOK;
  } while (0);

  LOG_E("eth h/w init err");
  return -RT_ERROR;
}
INIT_DEVICE_EXPORT(rt_hw_stm32_eth_init);

/**
 * @brief Ethernet ISR
 * @retval None
 */
void ETH_IRQHandler(void) {
  rt_interrupt_enter();
  HAL_ETH_IRQHandler(&ethHdl);
  rt_interrupt_leave();
}

/**
 * @brief Ethernet RX callback in ISR
 * @retval None
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth) {
  /*! \note
   * The assumption is ETH_RX_BUF_SIZE bigger than the largest frame length.
   */
  do {
    ETH_DMADescTypeDef *rxDesc;
    uint8_t i;
    uint8_t *newBuf;
    lwip_custom_pbuf_t *cp;
    struct pbuf *p;

    /* get data */
    if (HAL_OK != HAL_ETH_GetReceivedFrame_IT(heth)) break;
    if (heth->RxFrameInfos.SegCount < 1) break;
    if (heth->RxFrameInfos.SegCount > 1) {
        rt_kprintf("SegCount %d\n", heth->RxFrameInfos.SegCount);
      /* too large, then drop */
      rxDesc = heth->RxFrameInfos.FSRxDesc;
      for (i = 0; i < heth->RxFrameInfos.SegCount; i++) {
        rxDesc->Status |= ETH_DMARXDESC_OWN;
        rxDesc = (ETH_DMADescTypeDef *)(rxDesc->Buffer2NextDescAddr);
      }
      heth->RxFrameInfos.SegCount = 0;
      LINK_STATS_INC(link.drop);
      continue;
    }

    /* prepare new rx buf */
    newBuf = queue_get(&rx_buf_queue);
    if (RT_NULL == newBuf) break;

    /* get custom pbuf */
    cp = (lwip_custom_pbuf_t *)LWIP_MEMPOOL_ALLOC(PBUF_RX);
    if (RT_NULL == cp) {
      if (RT_NULL != newBuf) queue_put(&rx_buf_queue, newBuf);
      break;
    }
    cp->custom_p.custom_free_function = lwip_custom_pbuf_free;
    cp->buf = (uint8_t *)(heth->RxFrameInfos.buffer);

    /* get pbuf */
    p = pbuf_alloced_custom(PBUF_RAW, heth->RxFrameInfos.length, PBUF_REF,
                            &cp->custom_p, (void *)(heth->RxFrameInfos.buffer),
                            ETH_RX_BUF_SIZE);
    if (RT_NULL == p) {
      if (RT_NULL != newBuf) queue_put(&rx_buf_queue, newBuf);
      if (RT_NULL != cp) LWIP_MEMPOOL_FREE(PBUF_RX, cp);
      break;
    }

    heth->RxFrameInfos.FSRxDesc->Buffer1Addr = (rt_uint32_t)newBuf;
    heth->RxFrameInfos.FSRxDesc->Status |= ETH_DMARXDESC_OWN;
    heth->RxFrameInfos.SegCount = 0;

    if (ERR_OK != ethDev.parent.netif->input(p, ethDev.parent.netif)) {
      LWIP_DEBUGF(NETIF_DEBUG, ("netif.input error\n"));
      pbuf_free(p);
      LINK_STATS_INC(link.drop);
      break;
    }
    LINK_STATS_INC(link.recv);
  } while (1);

  if (__HAL_ETH_DMA_GET_FLAG(heth, ETH_DMASR_RBUS))
    __HAL_ETH_DMA_CLEAR_FLAG(heth, ETH_DMASR_RBUS);
}

/**
 * @brief Ethernet error callback in ISR
 * @retval None
 */
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth) {
  rt_kprintf("*** eth err ***");
}
