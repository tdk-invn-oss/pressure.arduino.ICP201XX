/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#ifndef ICP201xx_H
#define ICP201xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include <stdint.h>

extern "C" {
#include "ICP201xx/Icp201xx.h"
#include "ICP201xx/Icp201xxDriver.h"
}
// This defines the handler called when receiving an irq
typedef void (*ICP201xx_irq_handler)(void);

class ICP201xx {
  public:
    ICP201xx(TwoWire &i2c,bool address_lsb, uint32_t freq);
    ICP201xx(TwoWire &i2c,bool address_lsb);
    ICP201xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    ICP201xx(SPIClass &spi,uint8_t chip_select_id);
    int begin();
    int start(void);
    int getData(float& pressure, float& temperature);
    int singleMeasure(float& pressure_kp, float& temperature_C);
    int startFifoInterrupt(uint8_t intpin, ICP201xx_irq_handler handler, uint8_t fifo_watermark);
    int startPressureInterrupt(uint8_t intpin, ICP201xx_irq_handler handler, float pressure);
    int startPressureChangeInterrupt(uint8_t intpin, ICP201xx_irq_handler handler, float pressure_delta);
    uint8_t clear_interrupt_status(void);
    inv_icp201xx_t icp_device;
    uint8_t i2c_address;
    TwoWire *i2c;
    uint8_t spi_cs;
    SPIClass *spi;
    ICP201xx_irq_handler irq_callback;
    bool spi_in_transaction;
    uint32_t clk_freq;

  protected:
    bool use_spi;
    bool irq_enabled;
    int enableInterrupt(uint8_t intpin, ICP201xx_irq_handler handler);
    void inv_icp201xx_app_warmup(icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode);
};

#endif // ICP201xx_H
