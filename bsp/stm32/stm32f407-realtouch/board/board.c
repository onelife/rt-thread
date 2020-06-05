/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-05-22     onelife      
 */

#include "drv_spi.h"
#include "spi_flash.h"
#include "spi_flash_sfud.h"
#include "dfs_fs.h"

#include "board.h"

/**
 * @brief Workaround of "vcom" as console issue
 * @retval Error code
 */
static int bsp_fix_vcom_as_console(void) {
  rt_device_t vcom;

  if (rt_strcmp(RT_CONSOLE_DEVICE_NAME, "vcom")) return 0;

  vcom = rt_device_find("vcom");
  if (!vcom) return -1;

  (void)rt_console_set_device("vcom");
  (void)rt_device_close(vcom);
  if (!rt_device_open(vcom, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |
                                #ifdef RT_VCOM_TX_USE_DMA
                                RT_DEVICE_FLAG_DMA_TX |
                                #endif
                                RT_DEVICE_FLAG_STREAM)) {
    return 0;
  }
  return -1;
}
INIT_COMPONENT_EXPORT(bsp_fix_vcom_as_console);

/**
 * @brief Init SPI Flash
 * @retval Error code
 */
static int bsp_spi_flash_init(void) {
    rt_hw_spi_device_attach("spi2", "spi20", GPIOG, GPIO_PIN_10);

    if (!rt_sfud_flash_probe("W25Q64", "spi20"))
        return -1;
    return 0;
}
INIT_COMPONENT_EXPORT(bsp_spi_flash_init);

/**
 * @brief Mount SPI Flash and SD card
 * @retval Error code
 */
static int bsp_fs_mount(void) {
  if (dfs_mount("W25Q64", "/", "elm", 0, 0)) return -1;
  if (dfs_mount("sd0", "/sd", "elm", 0, 0)) return -1;
  return 0;
}
INIT_ENV_EXPORT(bsp_fs_mount);

/**
 * @brief Hard reset PHY chip
 * @retval None
 */
void phy_reset(void) {
  rt_tick_t tick_stop;

  HAL_GPIO_WritePin(STM32_PHY_NRST_PORT, STM32_PHY_NRST_PIN, GPIO_PIN_RESET);
  tick_stop = rt_tick_get() + rt_tick_from_millisecond(50);
  while (rt_tick_get() < tick_stop)  __NOP();
  HAL_GPIO_WritePin(STM32_PHY_NRST_PORT, STM32_PHY_NRST_PIN, GPIO_PIN_SET);
}

/**
 * @brief Release PHY chip reset pin
 * @retval Error code
 */
int bsp_phy_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Pin = STM32_PHY_NRST_PIN,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(STM32_PHY_NRST_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(STM32_PHY_NRST_PORT, STM32_PHY_NRST_PIN, GPIO_PIN_SET);
  return 0;
}
INIT_BOARD_EXPORT(bsp_phy_init);

#ifdef FINSH_USING_MSH
static int sram_test(void) {
  int i = 0;
  uint32_t start_time = 0, time_cast = 0;
  char data_width = 2;
  uint16_t data = 0;

  /* write data */
  rt_kprintf("Writing %ld bytes data...\n", STM32_EXT_SRAM_SIZE);
  start_time = rt_tick_get();
  for (i = 0; i < STM32_EXT_SRAM_SIZE / data_width; i++) {
    *(__IO uint16_t *)(STM32_EXT_SRAM_START_ADRESS + i * data_width) =
        (uint16_t)0x5555;
  }
  time_cast = rt_tick_get() - start_time;
  rt_kprintf(
      "Writing data done. Total time: %d.%03dS\n",
      time_cast / RT_TICK_PER_SECOND,
      time_cast % RT_TICK_PER_SECOND / ((RT_TICK_PER_SECOND * 1 + 999) / 1000));

  /* read data */
  rt_kprintf("Reading data...\n");
  for (i = 0; i < STM32_EXT_SRAM_SIZE / data_width; i++) {
    if (0x5555 !=
        *(__IO uint16_t *)(STM32_EXT_SRAM_START_ADRESS + i * data_width)) {
      rt_kprintf("SRAM test failed!\n");
      break;
    }
  }

  if (i >= STM32_EXT_SRAM_SIZE / data_width) rt_kprintf("SRAM test success!\n");

  return RT_EOK;
}
MSH_CMD_EXPORT(sram_test, sram test);

#if defined(RT_USING_LWIP) && defined(RT_LWIP_IPERF)
#include "lwip/apps/lwiperf.h"

static rt_bool_t test_done = RT_FALSE;

static void iperf_report(void *arg, enum lwiperf_report_type report_type,
                         const ip_addr_t *local_addr, u16_t local_port,
                         const ip_addr_t *remote_addr, u16_t remote_port,
                         u32_t bytes_transferred, u32_t ms_duration,
                         u32_t bandwidth_kbitpsec) {
  (void)arg;
  (void)report_type;
  (void)remote_port;
  (void)report_type;
  rt_kprintf("TX %d bytes in %d ms \t=> %d kbps\n", bytes_transferred,
             ms_duration, bandwidth_kbitpsec);
  test_done = RT_TRUE;
}

static void iperf_client(uint8_t argc, char **argv) {
  const char help[] = "iperf_client server_ip\n";
  rt_uint32_t ip;
  void *handle;

  if (argc < 2) {
    rt_kprintf(help);
    return;
  }

  ip = ipaddr_addr(argv[1]);
  if (IPADDR_NONE == ip) {
    rt_kprintf("Invalid server address.\n");
    return;
  }

  rt_kprintf("Start iPerf client with %s:5001\n", (char *)argv[1]);
  {
    const ip_addr_t remote_addr = IPADDR4_INIT(ip);
    handle =
        lwiperf_start_tcp_client_default(&remote_addr, iperf_report, RT_NULL);
        // lwiperf_start_tcp_server_default(NULL,NULL);
  }

  if (RT_NULL == handle) {
    rt_kprintf("Cannot connect to server\n");
    return;
  }

  while (!test_done)
    rt_thread_delay(RT_TICK_PER_SECOND * 2);
  lwiperf_abort(handle);
  rt_kprintf("iPerf test done\n");
}
MSH_CMD_EXPORT(iperf_client, iPerf test client);

#endif /* defined(RT_USING_LWIP) && defined(RT_LWIP_IPERF) */
#endif /* FINSH_USING_MSH */
