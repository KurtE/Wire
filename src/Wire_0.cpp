
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


#endif

