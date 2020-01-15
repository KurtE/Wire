
#include "Wire.h"


#if defined(__IMXRT1062__)
//#include "debug/printf.h"
PROGMEM
constexpr TwoWire::I2C_Hardware_t TwoWire::i2c3_hardware = {
	CCM_CCGR2, CCM_CCGR2_LPI2C3(CCM_CCGR_ON),
		{{17, 1 | 0x10, &IOMUXC_LPI2C3_SDA_SELECT_INPUT, 2}, {36, 2 | 0x10, &IOMUXC_LPI2C3_SDA_SELECT_INPUT, 1}},
		{{16, 1 | 0x10, &IOMUXC_LPI2C3_SCL_SELECT_INPUT, 2}, {37, 2 | 0x10, &IOMUXC_LPI2C3_SCL_SELECT_INPUT, 1}},
	IRQ_LPI2C3
};
TwoWire Wire1(&IMXRT_LPI2C3, TwoWire::i2c3_hardware);

#elif defined(__arm__) && defined(TEENSYDUINO) && (defined(__MKL26Z64__) || defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))

#if !defined(WIRE_HAS_STOP_INTERRUPT)
void sda_rising_isr1(void)
{
	Wire1.sda_rising_isr();
}
#endif

#if defined(__MKL26Z64__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
constexpr TwoWire::I2C_Hardware_t TwoWire::i2c1_hardware = {
	SIM_SCGC4, SIM_SCGC4_I2C1,
#if defined(__MKL26Z64__)
	23, 255, 255, 255, 255,
	2, 0, 0, 0, 0,
	22, 255, 255, 255, 255,
	2, 0, 0, 0, 0,
#elif defined(__MK20DX256__)
	30, 255, 255, 255, 255,
	2, 0, 0, 0, 0,
	29, 255, 255, 255, 255,
	2, 0, 0, 0, 0,
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	38, 58, 255, 255, 255,
	2, 6, 0, 0, 0,
	37, 59, 255, 255, 255,
	2, 6, 0, 0, 0,
#endif
	IRQ_I2C1
	#if !defined(WIRE_HAS_STOP_INTERRUPT)
	,&sda_rising_isr1
	#endif
};
#endif

#define MAKE_CONST(x) (__builtin_constant_p(x) ? (x) : (x))
#ifdef WIRE_IMPLEMENT_WIRE1
constexpr uintptr_t i2c1_addr = uintptr_t(MAKE_CONST(&KINETIS_I2C1));
TwoWire Wire1(i2c1_addr, TwoWire::i2c1_hardware);
void i2c1_isr(void) { Wire1.isr(); }
#endif

#endif