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
 
#include "ICP201xx.h"
#include "conversion_helper.h"

// Instantiate an ICP201xx with LSB address set to 0
ICP201xx ICP(Wire,0);

volatile uint8_t irq_received = 0;

void irq_handler(void) {
  irq_received = 1;
}

void setup() {
  int ret;
  Serial.begin(115200);
  while(!Serial) {}

  // Initializing the ICP201xx
  ret = ICP.begin();
  if (ret != 0) {
    Serial.print("ICP201xx initialization failed: ");
    Serial.println(ret);
    while(1);
  }
  // Enable interrupt on pin 2, Fifo watermark=10
  ICP.startFifoInterrupt(2,irq_handler,10);
}

void loop() {
  // Wait for interrupt to read data from fifo (ODR = 25Hz, 10 samples in FIFO)
  if(irq_received) {
    float pressure_kP = 0;
    float temperature_C = 0;
    irq_received = 0;
    ICP.clear_interrupt_status();
    while(ICP.getData(pressure_kP,temperature_C) == 0)
    {
      // Format data for Serial Plotter
      Serial.print("Pressure(kP):");
      Serial.println(pressure_kP);
      Serial.print("Temp(C):");
      Serial.println(temperature_C);
      Serial.print("Altitude(m):");
      Serial.println(convertToHeight(pressure_kP,temperature_C));
    }
  }
  delay(1);
}
