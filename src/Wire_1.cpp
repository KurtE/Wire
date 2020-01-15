
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

#endif

