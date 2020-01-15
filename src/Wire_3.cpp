
#include "Wire.h"

#if defined(__MK66FX1M0__)
constexpr TwoWire::I2C_Hardware_t TwoWire::i2c3_hardware = {
	SIM_SCGC1, SIM_SCGC1_I2C3,
	56, 255, 255, 255, 255,
	2, 0, 0, 0, 0,
	57, 255, 255, 255, 255,
	2, 0, 0, 0, 0,
	IRQ_I2C3
};

#define MAKE_CONST(x) (__builtin_constant_p(x) ? (x) : (x))

#ifdef WIRE_IMPLEMENT_WIRE3
constexpr uintptr_t i2c3_addr = uintptr_t(MAKE_CONST(&KINETIS_I2C3));
TwoWire Wire3(i2c3_addr, TwoWire::i2c3_hardware);
void i2c3_isr(void) { Wire3.isr(); }
#endif

#endif