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
#if defined(__MK66FX1M0__)

#include "kinetis.h"
#include <string.h> // for memcpy
#include "core_pins.h"
//#include "HardwareSerial.h"
#include "Wire.h"

uint8_t TwoWire3::rxBuffer[BUFFER_LENGTH];
uint8_t TwoWire3::rxBufferIndex = 0;
uint8_t TwoWire3::rxBufferLength = 0;
uint8_t TwoWire3::txBuffer[BUFFER_LENGTH+1];
uint8_t TwoWire3::txBufferIndex = 0;
uint8_t TwoWire3::txBufferLength = 0;
uint8_t TwoWire3::transmitting = 0;
uint8_t TwoWire3::sda_pin_num = 56;
uint8_t TwoWire3::scl_pin_num = 57;
void (*TwoWire3::user_onRequest)(void) = NULL;
void (*TwoWire3::user_onReceive)(int) = NULL;


TwoWire3::TwoWire3()
{
}

static uint8_t slave_mode = 0;
static uint8_t irqcount=0;


void TwoWire3::begin(void)
{
	//serial_begin(BAUD2DIV(115200));
	//serial_print("\nWire Begin\n");

	slave_mode = 0;
	SIM_SCGC1 |= SIM_SCGC1_I2C3; // TODO: use bitband
	I2C3_C1 = 0;

	// Currently only pin each
	// SCL
	CORE_PIN56_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE; 
	// SDA
	CORE_PIN57_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
	setClock(100000);
	I2C3_C2 = I2C_C2_HDRS;
	I2C3_C1 = I2C_C1_IICEN;
	//pinMode(3, OUTPUT);
	//pinMode(4, OUTPUT);
}

void TwoWire3::setClock(uint32_t frequency)
{
	if (!(SIM_SCGC1 & SIM_SCGC1_I2C3)) return;

#if F_BUS == 120000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV1152; // 104 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV288; // 416 kHz
	} else {
		I2C3_F = I2C_F_DIV128; // 0.94 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 108000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV1024; // 105 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV256; //  422 kHz
	} else {
		I2C3_F = I2C_F_DIV112; // 0.96 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 96000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV960; // 100 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV240; // 400 kHz
	} else {
		I2C3_F = I2C_F_DIV96; // 1.0 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 90000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV896; // 100 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV224; // 402 kHz
	} else {
		I2C3_F = I2C_F_DIV88; // 1.02 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 80000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV768; // 104 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV192; // 416 kHz
	} else {
		I2C3_F = I2C_F_DIV80; // 1.0 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 72000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV640; // 112 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV192; // 375 kHz
	} else {
		I2C3_F = I2C_F_DIV72; // 1.0 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 64000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV640; // 100 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV160; // 400 kHz
	} else {
		I2C3_F = I2C_F_DIV64; // 1.0 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 60000000
	if (frequency < 400000) {
		I2C3_F = 0x2C;	// 104 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x1C; // 416 kHz
	} else {
		I2C3_F = 0x12; // 938 kHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 56000000
	if (frequency < 400000) {
		I2C3_F = 0x2B;	// 109 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x1C; // 389 kHz
	} else {
		I2C3_F = 0x0E; // 1 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 54000000
	if (frequency < 400000) {
		I2C3_F = I2C_F_DIV512;	// 105 kHz
	} else if (frequency < 1000000) {
		I2C3_F = I2C_F_DIV128; // 422 kHz
	} else {
		I2C3_F = I2C_F_DIV56; // 0.96 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 48000000
	if (frequency < 400000) {
		I2C3_F = 0x27;	// 100 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x1A; // 400 kHz
	} else {
		I2C3_F = 0x0D; // 1 MHz
	}
	I2C3_FLT = 4;
#elif F_BUS == 40000000
	if (frequency < 400000) {
		I2C3_F = 0x29;	// 104 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x19; // 416 kHz
	} else {
		I2C3_F = 0x0B; // 1 MHz
	}
	I2C3_FLT = 3;
#elif F_BUS == 36000000
	if (frequency < 400000) {
		I2C3_F = 0x28;	// 113 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x19; // 375 kHz
	} else {
		I2C3_F = 0x0A; // 1 MHz
	}
	I2C3_FLT = 3;
#elif F_BUS == 24000000
	if (frequency < 400000) {
		I2C3_F = 0x1F; // 100 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x12; // 375 kHz
	} else {
		I2C3_F = 0x02; // 1 MHz
	}
	I2C3_FLT = 2;
#elif F_BUS == 16000000
	if (frequency < 400000) {
		I2C3_F = 0x20; // 100 kHz
	} else if (frequency < 1000000) {
		I2C3_F = 0x07; // 400 kHz
	} else {
		I2C3_F = 0x00; // 800 MHz
	}
	I2C3_FLT = 1;
#elif F_BUS == 8000000
	if (frequency < 400000) {
		I2C3_F = 0x14; // 100 kHz
	} else {
		I2C3_F = 0x00; // 400 kHz
	}
	I2C3_FLT = 1;
#elif F_BUS == 4000000
	if (frequency < 400000) {
		I2C3_F = 0x07; // 100 kHz
	} else {
		I2C3_F = 0x00; // 200 kHz
	}
	I2C3_FLT = 1;
#elif F_BUS == 2000000
	I2C3_F = 0x00; // 100 kHz
	I2C3_FLT = 1;
#else
#error "F_BUS must be 120, 108, 96, 9, 80, 72, 64, 60, 56, 54, 48, 40, 36, 24, 16, 8, 4 or 2 MHz"
#endif
}

void TwoWire3::setSDA(uint8_t pin)
{
}

void TwoWire3::setSCL(uint8_t pin)
{
}

void TwoWire3::begin(uint8_t address)
{
	begin();
	I2C3_A1 = address << 1;
	slave_mode = 1;
	I2C3_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
	NVIC_ENABLE_IRQ(IRQ_I2C3);
}

void TwoWire3::end()
{
	if (!(SIM_SCGC1 & SIM_SCGC1_I2C3)) return;
	NVIC_DISABLE_IRQ(IRQ_I2C3);
	I2C3_C1 = 0;

	CORE_PIN56_CONFIG = 0;
	CORE_PIN57_CONFIG = 0;

	SIM_SCGC1 &= ~SIM_SCGC1_I2C3; // TODO: use bitband
}

void i2c3_isr(void)
{
	uint8_t status, c1, data;
	static uint8_t receiving=0;

	status = I2C3_S;
	//serial_print(".");
	if (status & I2C_S_ARBL) {
		// Arbitration Lost
		I2C3_S = I2C_S_ARBL;
		//serial_print("a");
		if (receiving && TwoWire3::rxBufferLength > 0) {
			// TODO: does this detect the STOP condition in slave receive mode?

		}
		if (!(status & I2C_S_IAAS)) return;
	}
	if (status & I2C_S_IAAS) {
		//serial_print("\n");
		// Addressed As A Slave
		if (status & I2C_S_SRW) {
			//serial_print("T");
			// Begin Slave Transmit
			receiving = 0;
			TwoWire3::txBufferLength = 0;
			if (TwoWire3::user_onRequest != NULL) {
				TwoWire3::user_onRequest();
			}
			if (TwoWire3::txBufferLength == 0) {
				// is this correct, transmitting a single zero
				// when we should send nothing?  Arduino's AVR
				// implementation does this, but is it ok?
				TwoWire3::txBufferLength = 1;
				TwoWire3::txBuffer[0] = 0;
			}
			I2C3_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
			I2C3_D = TwoWire3::txBuffer[0];
			TwoWire3::txBufferIndex = 1;
		} else {
			// Begin Slave Receive
			//serial_print("R");
			receiving = 1;
			TwoWire3::rxBufferLength = 0;
			I2C3_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
			data = I2C3_D;
		}
		I2C3_S = I2C_S_IICIF;
		return;
	}
	c1 = I2C3_C1;
	if (c1 & I2C_C1_TX) {
		// Continue Slave Transmit
		//serial_print("t");
		if ((status & I2C_S_RXAK) == 0) {
			//serial_print(".");
			// Master ACK'd previous byte
			if (TwoWire3::txBufferIndex < TwoWire3::txBufferLength) {
				I2C3_D = TwoWire3::txBuffer[TwoWire3::txBufferIndex++];
			} else {
				I2C3_D = 0;
			}
			I2C3_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
		} else {
			//serial_print("*");
			// Master did not ACK previous byte
			I2C3_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
			data = I2C3_D;
		}
	} else {
		// Continue Slave Receive
		irqcount = 0;
		attachInterrupt(TwoWire3::sda_pin_num, TwoWire3::sda_rising_isr, RISING);
		//digitalWriteFast(4, HIGH);
		data = I2C3_D;
		//serial_phex(data);
		if (TwoWire3::rxBufferLength < BUFFER_LENGTH && receiving) {
			TwoWire3::rxBuffer[TwoWire3::rxBufferLength++] = data;
		}
		//digitalWriteFast(4, LOW);
	}
	I2C3_S = I2C_S_IICIF;
}

// Detects the stop condition that terminates a slave receive transfer.
// Sadly, the I2C in Kinetis K series lacks the stop detect interrupt
// This pin change interrupt hack is needed to detect the stop condition
void TwoWire3::sda_rising_isr(void)
{
	//digitalWrite(3, HIGH);
	if (!(I2C3_S & I2C_S_BUSY)) {
		detachInterrupt(sda_pin_num);
		if (user_onReceive != NULL) {
			rxBufferIndex = 0;
			user_onReceive(rxBufferLength);
		}
		//delayMicroseconds(100);
	} else {
		if (++irqcount >= 2 || !slave_mode) {
			detachInterrupt(sda_pin_num);
		}
	}
	//digitalWrite(3, LOW);
}


// Chapter 44: Inter-Integrated Circuit (I2C) - Page 1012
//  I2C3_A1      // I2C Address Register 1
//  I2C3_F       // I2C Frequency Divider register
//  I2C3_C1      // I2C Control Register 1
//  I2C3_S       // I2C Status register
//  I2C3_D       // I2C Data I/O register
//  I2C3_C2      // I2C Control Register 2
//  I2C3_FLT     // I2C Programmable Input Glitch Filter register

static uint8_t i2c_status(void)
{
	static uint32_t p=0xFFFF;
	uint32_t s = I2C3_S;
	if (s != p) {
		//Serial.printf("(%02X)", s);
		p = s;
	}
	return s;
}

static void i2c_wait(void)
{
#if 0
	while (!(I2C3_S & I2C_S_IICIF)) ; // wait
	I2C3_S = I2C_S_IICIF;
#endif
	//Serial.write('^');
	while (1) {
		if ((i2c_status() & I2C_S_IICIF)) break;
	}
	I2C3_S = I2C_S_IICIF;
}

void TwoWire3::beginTransmission(uint8_t address)
{
	txBuffer[0] = (address << 1);
	transmitting = 1;
	txBufferLength = 1;
}

size_t TwoWire3::write(uint8_t data)
{
	if (transmitting || slave_mode) {
		if (txBufferLength >= BUFFER_LENGTH+1) {
			setWriteError();
			return 0;
		}
		txBuffer[txBufferLength++] = data;
		return 1;
	}
	return 0;
}

size_t TwoWire3::write(const uint8_t *data, size_t quantity)
{
	if (transmitting || slave_mode) {
		size_t avail = BUFFER_LENGTH+1 - txBufferLength;
		if (quantity > avail) {
			quantity = avail;
			setWriteError();
		}
		memcpy(txBuffer + txBufferLength, data, quantity);
		txBufferLength += quantity;
		return quantity;
	}
	return 0;
}

void TwoWire3::flush(void)
{
}


uint8_t TwoWire3::endTransmission(uint8_t sendStop)
{
	uint8_t i, status, ret=0;

	// clear the status flags
	I2C3_S = I2C_S_IICIF | I2C_S_ARBL;
	// now take control of the bus...
	if (I2C3_C1 & I2C_C1_MST) {
		// we are already the bus master, so send a repeated start
		//Serial.print("rstart:");
		I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
	} else {
		// we are not currently the bus master, so wait for bus ready
		//Serial.print("busy:");
		uint32_t wait_begin = millis();
		while (i2c_status() & I2C_S_BUSY) {
			//Serial.write('.') ;
			if (millis() - wait_begin > 15) {
				// bus stuck busy too long
				I2C3_C1 = 0;
				I2C3_C1 = I2C_C1_IICEN;
				//Serial.println("abort");
				return 4;
			}
		}
		// become the bus master in transmit mode (send start)
		slave_mode = 0;
		I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
	}
	// wait until start condition establishes control of the bus
	while (1) {
		status = i2c_status();
		if ((status & I2C_S_BUSY)) break;
	}
	// transmit the address and data
	for (i=0; i < txBufferLength; i++) {
		I2C3_D = txBuffer[i];
		//Serial.write('^');
		while (1) {
			status = i2c_status();
			if ((status & I2C_S_IICIF)) break;
			if (!(status & I2C_S_BUSY)) break;
		}
		I2C3_S = I2C_S_IICIF;
		//Serial.write('$');
		status = i2c_status();
		if ((status & I2C_S_ARBL)) {
			// we lost bus arbitration to another master
			// TODO: what is the proper thing to do here??
			//Serial.printf(" c1=%02X ", I2C3_C1);
			I2C3_C1 = I2C_C1_IICEN;
			ret = 4; // 4:other error
			break;
		}
		if (!(status & I2C_S_BUSY)) {
			// suddenly lost control of the bus!
			I2C3_C1 = I2C_C1_IICEN;
			ret = 4; // 4:other error
			break;
		}
		if (status & I2C_S_RXAK) {
			// the slave device did not acknowledge
			if (i == 0) {
				ret = 2; // 2:received NACK on transmit of address
			} else {
				ret = 3; // 3:received NACK on transmit of data 
			}
			sendStop = 1;
			break;
		}
	}
	if (sendStop) {
		// send the stop condition
		I2C3_C1 = I2C_C1_IICEN;
		// TODO: do we wait for this somehow?
	}
	transmitting = 0;
	//Serial.print(" ret=");
	//Serial.println(ret);
	return ret;
}


uint8_t TwoWire3::requestFrom(uint8_t address, uint8_t length, uint8_t sendStop)
{
	uint8_t tmp __attribute__((unused));
	uint8_t status, count=0;

	rxBufferIndex = 0;
	rxBufferLength = 0;
	//serial_print("requestFrom\n");
	// clear the status flags
	I2C3_S = I2C_S_IICIF | I2C_S_ARBL;
	// now take control of the bus...
	if (I2C3_C1 & I2C_C1_MST) {
		// we are already the bus master, so send a repeated start
		I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
	} else {
		// we are not currently the bus master, so wait for bus ready
		while (i2c_status() & I2C_S_BUSY) ;
		// become the bus master in transmit mode (send start)
		slave_mode = 0;
		I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
	}
	// send the address
	I2C3_D = (address << 1) | 1;
	i2c_wait();
	status = i2c_status();
	if ((status & I2C_S_RXAK) || (status & I2C_S_ARBL)) {
		// the slave device did not acknowledge
		// or we lost bus arbitration to another master
		I2C3_C1 = I2C_C1_IICEN;
		return 0;
	}
	if (length == 0) {
		// TODO: does anybody really do zero length reads?
		// if so, does this code really work?
		I2C3_C1 = I2C_C1_IICEN | (sendStop ? 0 : I2C_C1_MST);
		return 0;
	} else if (length == 1) {
		I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
	} else {
		I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST;
	}
	tmp = I2C3_D; // initiate the first receive
	while (length > 1) {
		i2c_wait();
		length--;
		if (length == 1) I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
		if (count < BUFFER_LENGTH) {
			rxBuffer[count++] = I2C3_D;
		} else {
			tmp = I2C3_D;
		}
	}
	i2c_wait();
	I2C3_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
	if (count < BUFFER_LENGTH) {
		rxBuffer[count++] = I2C3_D;
	} else {
		tmp = I2C3_D;
	}
	if (sendStop) I2C3_C1 = I2C_C1_IICEN;
	rxBufferLength = count;
	return count;
}

int TwoWire3::available(void)
{
	return rxBufferLength - rxBufferIndex;
}

int TwoWire3::read(void)
{
	if (rxBufferIndex >= rxBufferLength) return -1;
	return rxBuffer[rxBufferIndex++];
}

int TwoWire3::peek(void)
{
	if (rxBufferIndex >= rxBufferLength) return -1;
	return rxBuffer[rxBufferIndex];
}





// alternate function prototypes

uint8_t TwoWire3::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire3::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire3::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire3::beginTransmission(int address)
{
	beginTransmission((uint8_t)address);
}

uint8_t TwoWire3::endTransmission(void)
{
	return endTransmission(true);
}

void TwoWire3::begin(int address)
{
	begin((uint8_t)address);
}

void TwoWire3::onReceive( void (*function)(int) )
{
	user_onReceive = function;
}

void TwoWire3::onRequest( void (*function)(void) )
{
	user_onRequest = function;
}

//TwoWire Wire = TwoWire();
TwoWire3 Wire3;


#endif //  __MK66FX1M0__ 

