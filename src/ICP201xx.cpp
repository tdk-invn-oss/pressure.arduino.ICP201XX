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
 
#include "Arduino.h"
#include "ICP201xx.h"
#include <math.h>


static int i2c_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int i2c_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int spi_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int spi_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static void irq_handler(void);

// i2c and SPI interfaces are used from C driver callbacks, without any knowledge of the object
// As they are declared as static, they will be overriden each time a new ICP201xx object is created
// i2c
#define I2C_DEFAULT_CLOCK 400000
#define I2C_MAX_CLOCK 1000000
#define ICP201xx_I2C_ADDRESS 0x63
// spi
#define SPI_READ 0x80
#define SPI_DEFAULT_CLOCK 6000000
#define SPI_MAX_CLOCK 12000000

static  ICP201xx_irq_handler irq_callback = NULL;
static  ICP201xx* icp_ptr = NULL;

// ICP201xx constructor for I2c interface
ICP201xx::ICP201xx(TwoWire &i2c_ref,bool lsb, uint32_t freq) {
  i2c = &i2c_ref; 
  i2c_address = ICP201xx_I2C_ADDRESS + (lsb ? 0x1 : 0);
  use_spi = false;
  spi = NULL;
  spi_cs = 0;
  spi_in_transaction = false;
  irq_enabled = false;
  irq_callback = NULL;
  if ((freq <= I2C_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = I2C_DEFAULT_CLOCK;
  }
}

// ICP201xx constructor for I2c interface, default freq
ICP201xx::ICP201xx(TwoWire &i2c_ref,bool lsb) {
  i2c = &i2c_ref; 
  i2c_address = ICP201xx_I2C_ADDRESS + (lsb ? 0x1 : 0);
  use_spi = false;
  spi = NULL;
  spi_cs = 0;
  spi_in_transaction = false;
  irq_enabled = false;
  irq_callback = NULL;
  clk_freq = I2C_DEFAULT_CLOCK;
}

// ICP201xx constructor for spi interface
ICP201xx::ICP201xx(SPIClass &spi_ref,uint8_t cs_id, uint32_t freq) {
  spi = &spi_ref;
  spi_cs = cs_id; 
  use_spi = true;
  spi_in_transaction = false;
  i2c = NULL;
  i2c_address = 0;
  irq_enabled = false;
  irq_callback = NULL;
  if ((freq <= SPI_MAX_CLOCK) && (freq >= 100000))
  {
    clk_freq = freq;
  } else {
    clk_freq = SPI_DEFAULT_CLOCK;
  }
}

// ICP201xx constructor for spi interface, default freq
ICP201xx::ICP201xx(SPIClass &spi_ref,uint8_t cs_id) {
  spi = &spi_ref;
  spi_cs = cs_id; 
  use_spi = true;
  spi_in_transaction = false;
  i2c = NULL;
  i2c_address = 0;
  irq_enabled = false;
  irq_callback = NULL;
  clk_freq = SPI_DEFAULT_CLOCK;
}

/* starts communication with the ICP201xx */
int ICP201xx::begin() {
  inv_icp201xx_serif_t icp_serif;
  int rc = 0;
  uint8_t who_am_i;
  uint8_t icp_version;

  if (!use_spi) {
    i2c->begin();
    i2c->setClock(clk_freq);
    icp_serif.if_mode = ICP201XX_IF_I2C;
    icp_serif.read_reg  = i2c_read;
    icp_serif.write_reg = i2c_write;
  } else {
    spi->begin();
    pinMode(spi_cs,OUTPUT);
    digitalWrite(spi_cs,HIGH);
    icp_serif.if_mode = ICP201XX_IF_4_WIRE_SPI;
    icp_serif.read_reg  = spi_read;
    icp_serif.write_reg = spi_write;
  }
  /* Initialize serial interface between MCU and Icp201xx */
  icp_serif.context   = (void*)this;        /* no need */
  icp_serif.max_read  = 2048; /* maximum number of bytes allowed per serial read */
  icp_serif.max_write = 2048; /* maximum number of bytes allowed per serial write */
  icp_ptr = this;
  rc = inv_icp201xx_init(&icp_device, &icp_serif);
  if (rc != INV_ERROR_SUCCESS) {
    return rc;
  }
  rc = inv_icp201xx_soft_reset(&icp_device);
  if (rc != INV_ERROR_SUCCESS) {
    return rc;
  }
  
  /* Check WHOAMI */
  rc =  inv_icp201xx_get_devid_version(&icp_device,&who_am_i,&icp_version);
  if(rc != 0) {
    return -2;
  }
  if (who_am_i != EXPECTED_DEVICE_ID) {
    return -3;
  }
  
  /* Boot up OTP config */
  rc = inv_icp201xx_OTP_bootup_cfg(&icp_device);
  if(rc != 0) {
    return rc;
  }
  
  // successful init, return 0
  return 0;
}

int ICP201xx::start(void) {
  int rc = 0;
  rc = inv_icp201xx_soft_reset(&icp_device);
  rc = inv_icp201xx_config(&icp_device,ICP201XX_OP_MODE0,ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
  inv_icp201xx_app_warmup(ICP201XX_OP_MODE0,ICP201XX_MEAS_MODE_CONTINUOUS);

  return rc;
}

int ICP201xx::singleMeasure(float& pressure_kp, float& temperature_C)
{
	inv_icp201xx_trigger_meas(&icp_device);
	delay(20);
	return getData(pressure_kp,temperature_C);
}

int ICP201xx::getData(float& pressure, float& temperature) {
  uint8_t fifo_packets;
  uint8_t fifo_data[6];
  int32_t data_press,data_temp;
  /** Read measurements count in FIFO **/
  if ( inv_icp201xx_get_fifo_count(&icp_device,&fifo_packets) )
    return -2;

  if (fifo_packets)
  {
    inv_icp201xx_get_fifo_data(&icp_device,1,fifo_data);
    if (i2c != NULL) {
      /* Perform dummy read to 0x00 register as last transaction after FIFO read for I2C interface */
      do{
        uint8_t dummy_reg = 0;
        i2c_read(&icp_device, 0, &dummy_reg, 1);
      }while(0);
    }
    inv_icp201xx_process_raw_data(&icp_device,1,fifo_data,&data_press,&data_temp);
    
    /** P = (POUT/2^17)*40kPa + 70kPa **/
    if (data_press & 0x080000 )
      data_press |= 0xFFF00000;
    pressure = ((float)(data_press) *40 /131072) +70;
    pressure = round(pressure * 1000.0)/1000.0;

    /* T = (TOUT/2^18)*65C + 25C */
    if (data_temp & 0x080000 )
      data_temp |= 0xFFF00000;
    temperature = ((float)( data_temp )*65 /262144 ) + 25;
    temperature = round(temperature*10.0)/10.0;

    return 0;
  }
  return -1;
}


int ICP201xx::enableInterrupt(uint8_t intpin, ICP201xx_irq_handler handler)
{
  if(handler == NULL) {
    return -1;
  }
  if(irq_enabled)
  {
    // IRQ already enabled
    return -2;
  }
  irq_callback = handler;
  pinMode(intpin,INPUT);
  attachInterrupt(intpin,&irq_handler,LOW);
  irq_enabled = true;
  return 0;
}

int ICP201xx::startFifoInterrupt(uint8_t intpin, ICP201xx_irq_handler handler, uint8_t fifo_watermark) {
  int rc = 0;
  if(enableInterrupt(intpin, handler))
  {
    return -1;
  }
  rc = inv_icp201xx_soft_reset(&icp_device);
  inv_icp201xx_flush_fifo(&icp_device);
  rc |= inv_icp201xx_config(&icp_device,ICP201XX_OP_MODE0,ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
  rc |= inv_icp201xx_set_press_notification_config(&icp_device, 0, 0, 0);
  rc |= inv_icp201xx_set_fifo_notification_config(&icp_device, ICP201XX_INT_MASK_FIFO_WMK_HIGH,fifo_watermark,0);

  inv_icp201xx_app_warmup(ICP201XX_OP_MODE0,ICP201XX_MEAS_MODE_CONTINUOUS);
  return rc;
}

int ICP201xx::startPressureInterrupt(uint8_t intpin, ICP201xx_irq_handler handler, float pressure) {
  int16_t pressure_value;
  int rc = 0;
  if(enableInterrupt(intpin, handler))
  {
    return -1;
  }
  rc = inv_icp201xx_soft_reset(&icp_device);
  rc |= inv_icp201xx_config(&icp_device,ICP201XX_OP_MODE0,ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
  rc |= inv_icp201xx_set_fifo_notification_config(&icp_device, 0,0,0);
  pressure_value = round((8192.0)*(pressure-70.0)/40);
  rc |= inv_icp201xx_set_press_notification_config(&icp_device, ICP201XX_INT_MASK_PRESS_ABS, pressure_value, 0);
  return rc;
}

int ICP201xx::startPressureChangeInterrupt(uint8_t intpin, ICP201xx_irq_handler handler, float pressure_delta) {
  int16_t pressure_delta_value;
  int rc = 0;
  if(enableInterrupt(intpin, handler))
  {
    return -1;
  }
  rc = inv_icp201xx_soft_reset(&icp_device);
  rc |= inv_icp201xx_config(&icp_device,ICP201XX_OP_MODE0,ICP201XX_FIFO_READOUT_MODE_PRES_TEMP);
  rc |= inv_icp201xx_set_fifo_notification_config(&icp_device, 0,0,0);
  pressure_delta_value = round((16384.0 * pressure_delta)/80);
  rc |= inv_icp201xx_set_press_notification_config(&icp_device, ICP201XX_INT_MASK_PRESS_DELTA,0, pressure_delta_value);
  return rc;
}

static int i2c_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  ICP201xx* obj = (ICP201xx*)ctx;
  obj->i2c->beginTransmission(obj->i2c_address);
  obj->i2c->write(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    obj->i2c->write(wbuffer[i]);
  }
  obj->i2c->endTransmission();
  return 0;
}

static int i2c_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  ICP201xx* obj = (ICP201xx*)ctx;
  uint16_t rx_bytes = 0;

  obj->i2c->beginTransmission(obj->i2c_address);
  obj->i2c->write(reg);
  obj->i2c->endTransmission(false);
  rx_bytes = obj->i2c->requestFrom(obj->i2c_address, rlen);
  if (rlen == rx_bytes) {
    for(uint8_t i = 0; i < rx_bytes; i++) {
      rbuffer[i] = obj->i2c->read();
    }
    return 0;
  } else {
    return -1;
  }
}

/*
 * spi_write might be called for writing read or write accesses.
 * First byte differenciate between those 0x33 => write, 0x3C => read
 */
static int spi_write(void * ctx, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
  (void)reg;
  ICP201xx* obj = (ICP201xx*)ctx;
  
  if(!obj->spi_in_transaction)
  {
    /* This is regsiter address stage */
    digitalWrite(obj->spi_cs,LOW);
    obj->spi->beginTransaction(SPISettings(obj->clk_freq, MSBFIRST, SPI_MODE3));
    for(uint8_t i = 0; i < wlen; i++) {
      obj->spi->transfer(wbuffer[i]);
    }
    obj->spi_in_transaction = true;
  } else {
    // this is a data write => end the transaction
    for(uint8_t i = 0; i < wlen; i++) {
      obj->spi->transfer(wbuffer[i]);
    }
    obj->spi->endTransaction();
    digitalWrite(obj->spi_cs,HIGH);
    obj->spi_in_transaction = false;
  }
  return 0;
}

static int spi_read(void * ctx, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
  (void)reg;
  ICP201xx* obj = (ICP201xx*)ctx;
  // Read is always started by a write
  obj->spi->transfer(rbuffer,rlen);
  obj->spi->endTransaction();
  digitalWrite(obj->spi_cs,HIGH);
  obj->spi_in_transaction = false;
  return 0;
}

/* ICP201xx warm up. 
 * If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values. 
 * Therefore the first 14 pressure output values are discarded.
 **/
void ICP201xx::inv_icp201xx_app_warmup(icp201xx_op_mode_t op_mode, icp201xx_meas_mode_t meas_mode)
{
  volatile uint8_t fifo_packets = 0;
  uint8_t fifo_packets_to_skip = 14;

  do{
    fifo_packets = 0;
    if ( !inv_icp201xx_get_fifo_count(&icp_device,(uint8_t*)&fifo_packets) && ( fifo_packets >= fifo_packets_to_skip ) )
    {
      uint8_t i_status = 0;
      inv_icp201xx_flush_fifo(&icp_device);

      inv_icp201xx_get_int_status(&icp_device,&i_status);
      if ( i_status )
        inv_icp201xx_clear_int_status(&icp_device,i_status);
      break;
    }
    delayMicroseconds(2);
  } while (1);

}

static void irq_handler(void)
{
  if(icp_ptr != NULL)
  {
    uint8_t i_status;
    if(icp_ptr->irq_callback != NULL)
      icp_ptr->irq_callback();
    inv_icp201xx_get_int_status(&icp_ptr->icp_device,&i_status);
    if ( i_status )
      inv_icp201xx_clear_int_status(&icp_ptr->icp_device,i_status);
  }
}