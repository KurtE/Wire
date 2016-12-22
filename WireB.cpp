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

#if defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)
#if defined (WIRE_DEFINE_WIRE1) || defined(WIRE_DEFINE_WIRE2) || defined(WIRE_DEFINE_WIRE3)
#include "kinetis.h"
#include <string.h> // for memcpy
#include "core_pins.h"
//#include "HardwareSerial.h"
#include "Wire.h"


TwoWireB::TwoWireB()
{
    receiving = 0;      // Our we receiving...
    irqcount = 0;
    transmitting = 0;
    user_onRequest = 0;
    user_onReceive = 0;
    rxBufferIndex = 0;
	rxBufferLength = 0;
	txBufferIndex = 0;
	txBufferLength = 0;

}



void TwoWireB::setClock(uint32_t frequency)
{
	if (!checkSIM_SCG()) {
		Serial.println("Set Clock failed SIM_SCG");
		return;
	}
	Serial.printf("setClock frequency: %u Bus: %u\n", frequency, F_BUS);

#if F_BUS == 120000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV1152; // 104 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV288; // 416 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV128; // 0.94 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 108000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV1024; // 105 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV256; //  422 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV112; // 0.96 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 96000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV960; // 100 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV240; // 400 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV96; // 1.0 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 90000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV896; // 100 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV224; // 402 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV88; // 1.02 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 80000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV768; // 104 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV192; // 416 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV80; // 1.0 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 72000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV640; // 112 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV192; // 375 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV72; // 1.0 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 64000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV640; // 100 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV160; // 400 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV64; // 1.0 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 60000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x2C;	// 104 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x1C; // 416 kHz
	} else {
		kinetisk_pi2c->F = 0x12; // 938 kHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 56000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x2B;	// 109 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x1C; // 389 kHz
	} else {
		kinetisk_pi2c->F = 0x0E; // 1 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 54000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = I2C_F_DIV512;	// 105 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = I2C_F_DIV128; // 422 kHz
	} else {
		kinetisk_pi2c->F = I2C_F_DIV56; // 0.96 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 48000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x27;	// 100 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x1A; // 400 kHz
	} else {
		kinetisk_pi2c->F = 0x0D; // 1 MHz
	}
	kinetisk_pi2c->FLT = 4;
#elif F_BUS == 40000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x29;	// 104 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x19; // 416 kHz
	} else {
		kinetisk_pi2c->F = 0x0B; // 1 MHz
	}
	kinetisk_pi2c->FLT = 3;
#elif F_BUS == 36000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x28;	// 113 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x19; // 375 kHz
	} else {
		kinetisk_pi2c->F = 0x0A; // 1 MHz
	}
	kinetisk_pi2c->FLT = 3;
#elif F_BUS == 24000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x1F; // 100 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x12; // 375 kHz
	} else {
		kinetisk_pi2c->F = 0x02; // 1 MHz
	}
	kinetisk_pi2c->FLT = 2;
#elif F_BUS == 16000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x20; // 100 kHz
	} else if (frequency < 1000000) {
		kinetisk_pi2c->F = 0x07; // 400 kHz
	} else {
		kinetisk_pi2c->F = 0x00; // 800 MHz
	}
	kinetisk_pi2c->FLT = 1;
#elif F_BUS == 8000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x14; // 100 kHz
	} else {
		kinetisk_pi2c->F = 0x00; // 400 kHz
	}
	kinetisk_pi2c->FLT = 1;
#elif F_BUS == 4000000
	if (frequency < 400000) {
		kinetisk_pi2c->F = 0x07; // 100 kHz
	} else {
		kinetisk_pi2c->F = 0x00; // 200 kHz
	}
	kinetisk_pi2c->FLT = 1;
#elif F_BUS == 2000000
	kinetisk_pi2c->F = 0x00; // 100 kHz
	kinetisk_pi2c->FLT = 1;
#else
#error "F_BUS must be 120, 108, 96, 9, 80, 72, 64, 60, 56, 54, 48, 40, 36, 24, 16, 8, 4 or 2 MHz"
#endif
}



uint8_t TwoWireB::isr(void)
{
	uint8_t retval = 0;
	uint8_t status, c1, data;

	status = kinetisk_pi2c->S;
	//serial_print(".");
	if (status & I2C_S_ARBL) {
		// Arbitration Lost
		kinetisk_pi2c->S = I2C_S_ARBL;
		//serial_print("a");
		if (receiving && rxBufferLength > 0) {
			// TODO: does this detect the STOP condition in slave receive mode?


		}
		if (!(status & I2C_S_IAAS)) return 0;
	}
	if (status & I2C_S_IAAS) {
		//serial_print("\n");
		// Addressed As A Slave
		if (status & I2C_S_SRW) {
			//serial_print("T");
			// Begin Slave Transmit
			receiving = 0;
			txBufferLength = 0;
			if (user_onRequest != NULL) {
				user_onRequest();
			}
			if (txBufferLength == 0) {
				// is this correct, transmitting a single zero
				// when we should send nothing?  Arduino's AVR
				// implementation does this, but is it ok?
				txBufferLength = 1;
				txBuffer[0] = 0;
			}
			kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
			kinetisk_pi2c->D = txBuffer[0];
			txBufferIndex = 1;
		} else {
			// Begin Slave Receive
			//serial_print("R");
			receiving = 1;
			rxBufferLength = 0;
			kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE;
			data = kinetisk_pi2c->D;
		}
		kinetisk_pi2c->S = I2C_S_IICIF;
		return 0;
	}
	#if defined(KINETISL)
	c1 = kinetisk_pi2c->FLT;
	if ((c1 & I2C_FLT_STOPF) && (c1 & I2C_FLT_STOPIE)) {
		kinetisk_pi2c->FLT = c1 & ~I2C_FLT_STOPIE;
		if (user_onReceive != NULL) {
			rxBufferIndex = 0;
			user_onReceive(rxBufferLength);
		}
	}
	#endif
	c1 = kinetisk_pi2c->C1;
	if (c1 & I2C_C1_TX) {
		// Continue Slave Transmit
		//serial_print("t");
		if ((status & I2C_S_RXAK) == 0) {
			//serial_print(".");
			// Master ACK'd previous byte
			if (txBufferIndex < txBufferLength) {
				kinetisk_pi2c->D = txBuffer[txBufferIndex++];
			} else {
				kinetisk_pi2c->D = 0;
			}
			kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
		} else {
			//serial_print("*");
			// Master did not ACK previous byte
			kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_IICIE;
			data = kinetisk_pi2c->D;
		}
	} else {
		// Continue Slave Receive
		irqcount = 0;
		#if defined(KINETISK)
		retval = 1;
		#elif defined(KINETISL)
		kinetisk_pi2c->FLT |= I2C_FLT_STOPIE;
		#endif
		//digitalWriteFast(4, HIGH);
		data = kinetisk_pi2c->D;
		//serial_phex(data);
		if (rxBufferLength < BUFFER_LENGTH && receiving) {
			rxBuffer[rxBufferLength++] = data;
		}
		//digitalWriteFast(4, LOW);
	}
	kinetisk_pi2c->S = I2C_S_IICIF;
	return retval;
}


// Chapter 44: Inter-Integrated Circuit (I2C) - Page 1012
//  kinetisk_pi2c->A1      // I2C Address Register 1
//  I2C1_F       // I2C Frequency Divider register
//  I2C1_C1      // I2C Control Register 1
//  I2C1_S       // I2C Status register
//  I2C1_D       // I2C Data I/O register
//  I2C1_C2      // I2C Control Register 2
//  I2C1_FLT     // I2C Programmable Input Glitch Filter register

uint8_t TwoWireB::i2c_status(void)
{
#if 0	
	static uint32_t p=0xFFFF;
	uint32_t s = kinetisk_pi2c->S;
	if (s != p) {
		//Serial.printf("(%02X)", s);
		p = s;
	}
	return s;
#else
	return kinetisk_pi2c->S;
#endif	
}

void TwoWireB::i2c_wait(void)
{
#if 0
	while (!(kinetisk_pi2c->S & I2C_S_IICIF)) ; // wait
	kinetisk_pi2c->S = I2C_S_IICIF;
#endif
	//Serial.write('^');
	while (1) {
		if ((i2c_status() & I2C_S_IICIF)) break;
	}
	kinetisk_pi2c->S = I2C_S_IICIF;
}

void TwoWireB::beginTransmission(uint8_t address)
{
	txBuffer[0] = (address << 1);
	transmitting = 1;
	txBufferLength = 1;
}

size_t TwoWireB::write(uint8_t data)
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

size_t TwoWireB::write(const uint8_t *data, size_t quantity)
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

void TwoWireB::flush(void)
{
}


uint8_t TwoWireB::endTransmission(uint8_t sendStop)
{
	uint8_t i, status, ret=0;

	// clear the status flags
	kinetisk_pi2c->S = I2C_S_IICIF | I2C_S_ARBL;
	// now take control of the bus...
	if (kinetisk_pi2c->C1 & I2C_C1_MST) {
		// we are already the bus master, so send a repeated start
		//Serial.print("rstart:");
		kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
	} else {
		// we are not currently the bus master, so wait for bus ready
		//Serial.print("busy:");
		uint32_t wait_begin = millis();
		while (i2c_status() & I2C_S_BUSY) {
			//Serial.write('.') ;
			if (millis() - wait_begin > 15) {
				// bus stuck busy too long
				kinetisk_pi2c->C1 = 0;
				kinetisk_pi2c->C1 = I2C_C1_IICEN;
				//Serial.println("abort");
				return 4;
			}
		}
		// become the bus master in transmit mode (send start)
		slave_mode = 0;
		kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
	}
	// wait until start condition establishes control of the bus
	while (1) {
		status = i2c_status();
		if ((status & I2C_S_BUSY)) break;
	}
	// transmit the address and data
	for (i=0; i < txBufferLength; i++) {
		kinetisk_pi2c->D = txBuffer[i];
		//Serial.write('^');
		while (1) {
			status = i2c_status();
			if ((status & I2C_S_IICIF)) break;
			if (!(status & I2C_S_BUSY)) break;
		}
		kinetisk_pi2c->S = I2C_S_IICIF;
		//Serial.write('$');
		status = i2c_status();
		if ((status & I2C_S_ARBL)) {
			// we lost bus arbitration to another master
			// TODO: what is the proper thing to do here??
			//Serial.printf(" c1=%02X ", kinetisk_pi2c->C1);
			kinetisk_pi2c->C1 = I2C_C1_IICEN;
			ret = 4; // 4:other error
			break;
		}
		if (!(status & I2C_S_BUSY)) {
			// suddenly lost control of the bus!
			kinetisk_pi2c->C1 = I2C_C1_IICEN;
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
		kinetisk_pi2c->C1 = I2C_C1_IICEN;
		// TODO: do we wait for this somehow?
	}
	transmitting = 0;
	//Serial.print(" ret=");
	//Serial.println(ret);
	return ret;
}


uint8_t TwoWireB::requestFrom(uint8_t address, uint8_t length, uint8_t sendStop)
{
	uint8_t tmp __attribute__((unused));
	uint8_t status, count=0;

	rxBufferIndex = 0;
	rxBufferLength = 0;
	//serial_print("requestFrom\n");
	// clear the status flags
	kinetisk_pi2c->S = I2C_S_IICIF | I2C_S_ARBL;
	// now take control of the bus...
	if (kinetisk_pi2c->C1 & I2C_C1_MST) {
		// we are already the bus master, so send a repeated start
		kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
	} else {
		// we are not currently the bus master, so wait for bus ready
		while (i2c_status() & I2C_S_BUSY) ;
		// become the bus master in transmit mode (send start)
		slave_mode = 0;
		kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
	}
	// send the address
	kinetisk_pi2c->D = (address << 1) | 1;
	i2c_wait();
	status = i2c_status();
	if ((status & I2C_S_RXAK) || (status & I2C_S_ARBL)) {
		// the slave device did not acknowledge
		// or we lost bus arbitration to another master
		kinetisk_pi2c->C1 = I2C_C1_IICEN;
		return 0;
	}
	if (length == 0) {
		// TODO: does anybody really do zero length reads?
		// if so, does this code really work?
		kinetisk_pi2c->C1 = I2C_C1_IICEN | (sendStop ? 0 : I2C_C1_MST);
		return 0;
	} else if (length == 1) {
		kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
	} else {
		kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST;
	}
	tmp = kinetisk_pi2c->D; // initiate the first receive
	while (length > 1) {
		i2c_wait();
		length--;
		if (length == 1) kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
		if (count < BUFFER_LENGTH) {
			rxBuffer[count++] = kinetisk_pi2c->D;
		} else {
			tmp = kinetisk_pi2c->D;
		}
	}
	i2c_wait();
	kinetisk_pi2c->C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
	if (count < BUFFER_LENGTH) {
		rxBuffer[count++] = kinetisk_pi2c->D;
	} else {
		tmp = kinetisk_pi2c->D;
	}
	if (sendStop) kinetisk_pi2c->C1 = I2C_C1_IICEN;
	rxBufferLength = count;
	return count;
}

int TwoWireB::available(void)
{
	return rxBufferLength - rxBufferIndex;
}

int TwoWireB::read(void)
{
	if (rxBufferIndex >= rxBufferLength) return -1;
	return rxBuffer[rxBufferIndex++];
}

int TwoWireB::peek(void)
{
	if (rxBufferIndex >= rxBufferLength) return -1;
	return rxBuffer[rxBufferIndex];
}





// alternate function prototypes

uint8_t TwoWireB::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWireB::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWireB::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWireB::beginTransmission(int address)
{
	beginTransmission((uint8_t)address);
}

uint8_t TwoWireB::endTransmission(void)
{
	return endTransmission(true);
}

void TwoWireB::begin(int address)
{
	begin((uint8_t)address);
}

void TwoWireB::onReceive( void (*function)(int) )
{
	user_onReceive = function;
}

void TwoWireB::onRequest( void (*function)(void) )
{
	user_onRequest = function;
}

#endif // __MK20DX256__ || __MK64FX512__ || __MK66FX1M0__ || __MKL26Z64__
#endif
