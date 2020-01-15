
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

#endif

