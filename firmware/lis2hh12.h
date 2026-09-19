/* Driver for the ST LIS2HH12 3-axis accelerometer over I2C.
 * Register map / bit layout verified against the official datasheet
 * (DocID025344 Rev 5) and https://github.com/STMicroelectronics/lis2hh12-pid
 */
#ifndef _LIS2HH12_H
#define _LIS2HH12_H

#include "lib_i2c.h"

// I2C address depends on the SA0 pin: GND -> 0x19, VDD -> 0x1D
#define LIS2HH12_ADDR_SA0_LOW  0x19
#define LIS2HH12_ADDR_SA0_HIGH 0x1D

#define LIS2HH12_WHO_AM_I     0x0F
#define LIS2HH12_WHO_AM_I_VAL 0x41

#define LIS2HH12_CTRL1        0x20
#define LIS2HH12_CTRL2        0x21
#define LIS2HH12_CTRL3        0x22
#define LIS2HH12_CTRL4        0x23
#define LIS2HH12_CTRL5        0x24
#define LIS2HH12_STATUS       0x27
#define LIS2HH12_OUT_X_L      0x28
#define LIS2HH12_FIFO_CTRL    0x2E
#define LIS2HH12_FIFO_SRC     0x2F
#define LIS2HH12_IG_CFG1      0x30
#define LIS2HH12_IG_SRC1      0x31
#define LIS2HH12_IG_THS_X1    0x32
#define LIS2HH12_IG_THS_Y1    0x33
#define LIS2HH12_IG_THS_Z1    0x34
#define LIS2HH12_IG_DUR1      0x35

// CTRL1: HR=1, ODR=100Hz(011), BDU=1, Zen=Yen=Xen=1
#define LIS2HH12_CTRL1_VAL    0xBF
// CTRL4: FS=+-2g, IF_ADD_INC=1 (required for burst reads)
#define LIS2HH12_CTRL4_VAL    (LIS2HH12_FS_2G | 0x04)

// CTRL3 bits
#define LIS2HH12_CTRL3_FIFO_EN   0x80
#define LIS2HH12_CTRL3_STOP_FTH  0x40
#define LIS2HH12_CTRL3_INT1_IG1  0x08
#define LIS2HH12_CTRL3_INT1_DRDY 0x01

// CTRL5 self-test bits (ST2,ST1 at bits 3,2)
#define LIS2HH12_ST_NORMAL    0x00
#define LIS2HH12_ST_POSITIVE  0x04
#define LIS2HH12_ST_NEGATIVE  0x08

// FIFO_CTRL FMODE[2:0] at bits 7:5
#define LIS2HH12_FIFO_BYPASS           (0x0 << 5)
#define LIS2HH12_FIFO_FIFO             (0x1 << 5)
#define LIS2HH12_FIFO_STREAM           (0x2 << 5)
#define LIS2HH12_FIFO_STREAM_TO_FIFO   (0x3 << 5)
#define LIS2HH12_FIFO_BYPASS_TO_STREAM (0x4 << 5)
#define LIS2HH12_FIFO_BYPASS_TO_FIFO   (0x7 << 5)

// FIFO_SRC bits
#define LIS2HH12_FIFO_SRC_FTH   0x80
#define LIS2HH12_FIFO_SRC_OVR   0x40
#define LIS2HH12_FIFO_SRC_EMPTY 0x20
#define LIS2HH12_FIFO_SRC_FSS   0x1F

// IG_CFG1 bits
#define LIS2HH12_IG_XLIE 0x01
#define LIS2HH12_IG_XHIE 0x02
#define LIS2HH12_IG_YLIE 0x04
#define LIS2HH12_IG_YHIE 0x08
#define LIS2HH12_IG_ZLIE 0x10
#define LIS2HH12_IG_ZHIE 0x20
#define LIS2HH12_IG_6D   0x40
#define LIS2HH12_IG_AOI  0x80

// IG_SRC1 bits
#define LIS2HH12_IG_SRC_XL 0x01
#define LIS2HH12_IG_SRC_XH 0x02
#define LIS2HH12_IG_SRC_YL 0x04
#define LIS2HH12_IG_SRC_YH 0x08
#define LIS2HH12_IG_SRC_ZL 0x10
#define LIS2HH12_IG_SRC_ZH 0x20
#define LIS2HH12_IG_SRC_IA 0x40

// mg per LSB, high-resolution (16-bit) mode, per full-scale range
// (datasheet Table 3: 0.061 / 0.122 / 0.244 mg/digit @ +-2g / +-4g / +-8g)
#define LIS2HH12_MG_PER_LSB_2G   61
#define LIS2HH12_MG_PER_LSB_4G  122
#define LIS2HH12_MG_PER_LSB_8G  244

// CTRL4 FS[1:0] is at bits 5:4. 01 (0x1) is not a valid setting.
#define LIS2HH12_FS_2G (0x0 << 4)
#define LIS2HH12_FS_4G (0x2 << 4)
#define LIS2HH12_FS_8G (0x3 << 4)

// IG_THS1 1 LSB ~= FS/128 (established ST convention for this generator)
#define LIS2HH12_IG_THS_MG_PER_LSB 16

static inline int lis2hh12_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	return i2c_sendBytes(I2C1, addr, buf, 2);
}

static inline int lis2hh12_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
	return i2c_readReg_buffer(I2C1, addr, reg, val, 1);
}

static inline int lis2hh12_find(uint8_t *out_addr)
{
	uint8_t who;
	uint8_t candidates[2] = {LIS2HH12_ADDR_SA0_LOW, LIS2HH12_ADDR_SA0_HIGH};

	for (int i = 0; i < 2; i++)
	{
		if (!lis2hh12_read_reg(candidates[i], LIS2HH12_WHO_AM_I, &who)
			&& who == LIS2HH12_WHO_AM_I_VAL)
		{
			*out_addr = candidates[i];
			return 1;
		}
	}
	return 0;
}

static inline void lis2hh12_init(uint8_t addr)
{
	lis2hh12_write_reg(addr, LIS2HH12_CTRL4, LIS2HH12_CTRL4_VAL);
	lis2hh12_write_reg(addr, LIS2HH12_CTRL1, LIS2HH12_CTRL1_VAL);
}

// n: 0=power-down 1=10Hz 2=50Hz 3=100Hz 4=200Hz 5=400Hz 6=800Hz
static inline void lis2hh12_set_odr(uint8_t addr, uint8_t n)
{
	// keep HR=1, BDU=1, Xen=Yen=Zen=1; just change the ODR field
	lis2hh12_write_reg(addr, LIS2HH12_CTRL1, 0x8F | ((n & 0x7) << 4));
}

static inline int lis2hh12_read_xyz(uint8_t addr, int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t raw[6];

	if (i2c_readReg_buffer(I2C1, addr, LIS2HH12_OUT_X_L, raw, 6))
		return 0;

	*x = (int16_t)((raw[1] << 8) | raw[0]);
	*y = (int16_t)((raw[3] << 8) | raw[2]);
	*z = (int16_t)((raw[5] << 8) | raw[4]);
	return 1;
}

static inline int32_t lis2hh12_to_mg(int16_t raw, int32_t mg_per_lsb)
{
	return ((int32_t)raw * mg_per_lsb) / 1000;
}

// g: 2, 4 or 8. Returns the new mg_per_lsb scale factor, or 0 if invalid.
static inline int32_t lis2hh12_set_scale(uint8_t addr, int g)
{
	uint8_t fs;
	int32_t mg_per_lsb;

	if (g == 2) { fs = LIS2HH12_FS_2G; mg_per_lsb = LIS2HH12_MG_PER_LSB_2G; }
	else if (g == 4) { fs = LIS2HH12_FS_4G; mg_per_lsb = LIS2HH12_MG_PER_LSB_4G; }
	else if (g == 8) { fs = LIS2HH12_FS_8G; mg_per_lsb = LIS2HH12_MG_PER_LSB_8G; }
	else return 0;

	lis2hh12_write_reg(addr, LIS2HH12_CTRL4, fs | 0x04); // keep IF_ADD_INC
	return mg_per_lsb;
}

#endif
