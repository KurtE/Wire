/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

#include "Wire.h"

// Only T3.6
#if defined(WIRE_DEFINE_WIRE1) && defined(__MK66FX1M0__)

#include "kinetis.h"
#include <string.h> // for memcpy
#include "core_pins.h"
//#include "HardwareSerial.h"
#include "Wire.h"

TwoWire3::TwoWire3() : TwoWire()
{
	kinetisk_pi2c = &KINETIS_I2C3;
	sda_pin_num = 56;
	scl_pin_num = 57;
}


void TwoWire3::begin(void)
{
	//serial_begin(BAUD2DIV(115200));
	//serial_print("\nWire Begin\n");

	slave_mode = 0;
	SIM_SCGC1 |= SIM_SCGC1_I2C3; // TODO: use bitband
	kinetisk_pi2c->C1 = 0;

	// Currently only pin each
	// SCL
	CORE_PIN56_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE; 
	// SDA
	CORE_PIN57_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
	setClock(100000);
	kinetisk_pi2c->C2 = I2C_C2_HDRS;
	kinetisk_pi2c->C1 = I2C_C1_IICEN;
}

void TwoWire3::begin(uint8_t address)
{
	begin();
	kinetisk_pi2c->A1 = address << 1;
	slave_mode = 1;
	kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE;
	NVIC_ENABLE_IRQ(IRQ_I2C3);
}


void TwoWire3::setSDA(uint8_t pin)
{
}

void TwoWire3::setSCL(uint8_t pin)
{
}

uint8_t TwoWire3::checkSIM_SCG()
{
	return ( SIM_SCGC1 & SIM_SCGC1_I2C3)? 1 : 0;  //Test to see if we have done a begin...

}


void TwoWire3::end()
{
	if (!(SIM_SCGC1 & SIM_SCGC1_I2C3)) return;
	NVIC_DISABLE_IRQ(IRQ_I2C3);
	kinetisk_pi2c->C1 = 0;

	CORE_PIN56_CONFIG = 0;
	CORE_PIN57_CONFIG = 0;

	SIM_SCGC1 &= ~SIM_SCGC1_I2C3; // TODO: use bitband
}

void i2c3_isr(void)
{
	if (Wire3.isr() ) {
		// Hack that mainline function says to set
		attachInterrupt(Wire3.sda_pin_num, TwoWire3::sda_rising_isr, RISING);
	}

}

// Detects the stop condition that terminates a slave receive transfer.
// Sadly, the I2C in Kinetis K series lacks the stop detect interrupt
// This pin change interrupt hack is needed to detect the stop condition
void TwoWire3::sda_rising_isr(void)
{
	//digitalWrite(3, HIGH);
	if (!(I2C3_S & I2C_S_BUSY)) {
		detachInterrupt(Wire3.sda_pin_num);
		if (Wire3.user_onReceive != NULL) {
			Wire3.rxBufferIndex = 0;
			Wire3.user_onReceive(Wire3.rxBufferLength);
		}
		//delayMicroseconds(100);
	} else {
		if (++Wire3.irqcount >= 2 || !Wire3.slave_mode) {
			detachInterrupt(Wire3.sda_pin_num);
		}
	}
	//digitalWrite(3, LOW);
}


// Chapter 44: Inter-Integrated Circuit (I2C) - Page 1012
//  kinetisk_pi2c->A1      // I2C Address Register 1
//  kinetisk_pi2c->F       // I2C Frequency Divider register
//  kinetisk_pi2c->C1      // I2C Control Register 1
//  kinetisk_pi2c->S       // I2C Status register
//  kinetisk_pi2c->D       // I2C Data I/O register
//  kinetisk_pi2c->C2      // I2C Control Register 2
//  kinetisk_pi2c->FLT     // I2C Programmable Input Glitch Filter register

//TwoWire Wire = TwoWire();
TwoWire3 Wire3;


#endif //  __MK66FX1M0__ 

