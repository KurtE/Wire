
#include "Wire.h"


#if defined(__IMXRT1062__)
//#include "debug/printf.h"

PROGMEM
constexpr TwoWire::I2C_Hardware_t TwoWire::i2c1_hardware = {
	CCM_CCGR2, CCM_CCGR2_LPI2C1(CCM_CCGR_ON),
		{{18, 3 | 0x10, &IOMUXC_LPI2C1_SDA_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
		{{19, 3 | 0x10, &IOMUXC_LPI2C1_SCL_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	IRQ_LPI2C1
};
TwoWire Wire(&IMXRT_LPI2C1, TwoWire::i2c1_hardware);

#elif defined(__arm__) && defined(TEENSYDUINO) && (defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))

#if !defined(WIRE_HAS_STOP_INTERRUPT)
void sda_rising_isr0(void)
{
	Wire.sda_rising_isr();
}
#endif

constexpr TwoWire::I2C_Hardware_t TwoWire::i2c0_hardware = {
	SIM_SCGC4, SIM_SCGC4_I2C0,
#if defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__)
	18, 17, 255, 255, 255,
	2, 2, 0, 0, 0,
	19, 16, 255, 255, 255,
	2, 2, 0, 0, 0,
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	18, 17, 34, 8, 48,
	2, 2, 5, 7, 2,
	19, 16, 33, 7, 47,
	2, 2, 5, 7, 2,
#endif
	IRQ_I2C0
	#if !defined(WIRE_HAS_STOP_INTERRUPT)
	,&sda_rising_isr0
	#endif
};


#define MAKE_CONST(x) (__builtin_constant_p(x) ? (x) : (x))

#ifdef WIRE_IMPLEMENT_WIRE
constexpr uintptr_t i2c0_addr = uintptr_t(MAKE_CONST(&KINETIS_I2C0));
TwoWire Wire(i2c0_addr, TwoWire::i2c0_hardware);
void i2c0_isr(void) { Wire.isr(); }
#endif

#endif