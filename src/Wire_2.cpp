
#include "Wire.h"


#if defined(__IMXRT1062__)
//#include "debug/printf.h"

PROGMEM
constexpr TwoWire::I2C_Hardware_t TwoWire::i2c4_hardware = {
	CCM_CCGR6, CCM_CCGR6_LPI2C4_SERIAL(CCM_CCGR_ON),
		{{25, 0 | 0x10, &IOMUXC_LPI2C4_SDA_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
		{{24, 0 | 0x10, &IOMUXC_LPI2C4_SCL_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	IRQ_LPI2C4
};
TwoWire Wire2(&IMXRT_LPI2C4, TwoWire::i2c4_hardware);

#elif defined(__arm__) && defined(TEENSYDUINO) && ( defined(__MK64FX512__) || defined(__MK66FX1M0__))

constexpr TwoWire::I2C_Hardware_t TwoWire::i2c2_hardware = {
	SIM_SCGC1, SIM_SCGC1_I2C2,
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	4, 255, 255, 255, 255,
	5, 0, 0, 0, 0,
	3, 26, 255, 255, 255,
	5, 5, 0, 0, 0,
#endif
	IRQ_I2C2
};

#define MAKE_CONST(x) (__builtin_constant_p(x) ? (x) : (x))
#ifdef WIRE_IMPLEMENT_WIRE2
constexpr uintptr_t i2c2_addr = uintptr_t(MAKE_CONST(&KINETIS_I2C2));
TwoWire Wire2(i2c2_addr, TwoWire::i2c2_hardware);
void i2c2_isr(void) { Wire2.isr(); }
#endif

#endif