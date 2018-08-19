/**
 *  @filename   :   epdif.cpp
 *  @brief      :   Implements EPD interface functions
 *                  Users have to implement all the functions in epdif.cpp
 *  @author     :   Yehui from Waveshare
 *
 *  Copyright (C) Waveshare     August 10 2017
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "epdif.h"

#include <string.h>
#include "driver/gpio.h"
#include "freertos/task.h"

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19

// To speed up transfers, every SPI transfer sends a bunch of lines. This define
// specifies how many. More means more memory use, but less overhead for setting
// up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

namespace {

// This function is called (in irq context!) just before a transmission
// starts. It will set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  int dc = (int)t->user;
  gpio_set_level(static_cast<gpio_num_t>(DC_PIN), dc);
}

}  // namespace

EpdIf::EpdIf() {};

EpdIf::~EpdIf() {};

void EpdIf::DigitalWrite(int pin, int value) {
  gpio_set_level(static_cast<gpio_num_t>(pin), value);
}

int EpdIf::DigitalRead(int pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin));
}

void EpdIf::DelayMs(unsigned int delaytime) {
  vTaskDelay(delaytime / portTICK_RATE_MS);
}

void EpdIf::SpiTransfer(uint8_t data, Type type) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  // Payload is 8 bits
  t.length = 8;
  t.tx_buffer = &data;
  switch (type) {
    case Type::kCommand:
      t.user = (void*)0; // D/C needs to be set to 0 when command
      break;
    case Type::kData:
      t.user = (void*)1; // D/C needs to be set to 1 when data
      break;
  }
  ret = spi_device_transmit(spi_handle_, &t);
  assert(ret == ESP_OK);            //Should have had no issues.
}

int EpdIf::IfInit(void) {
  spi_bus_config_t buscfg = {};
  buscfg.miso_io_num = PIN_NUM_MISO;
  buscfg.mosi_io_num = PIN_NUM_MOSI;
  buscfg.sclk_io_num = PIN_NUM_CLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = PARALLEL_LINES*320*2+8;

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 2*1000*1000; // 2 MHz
  devcfg.mode = 0;
  devcfg.spics_io_num = CS_PIN;
  // We want to be able to queue 7 transactions at a time
  devcfg.queue_size = 7;
  // Specify pre-transfer callback to handle D/C line
  devcfg.pre_cb = lcd_spi_pre_transfer_callback;

  esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  if (ret == ESP_OK) {
    // Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle_);
  } else {
    printf("%s: spi_bus_initialize failed (%s)\n",
           __func__, esp_err_to_name(ret));
  }

  if (ret != ESP_OK) {
    printf("%s: spi_bus_add_device failed (%s)\n",
           __func__, esp_err_to_name(ret));
  }

  gpio_set_direction(static_cast<gpio_num_t>(DC_PIN), GPIO_MODE_OUTPUT);
  gpio_set_direction(static_cast<gpio_num_t>(RST_PIN), GPIO_MODE_OUTPUT);
  gpio_set_direction(static_cast<gpio_num_t>(BUSY_PIN), GPIO_MODE_INPUT);
  return 0;
}
