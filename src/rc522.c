#include "rc522.h"

#include <stdlib.h>
#include <string.h>

#define MAKE_READ_ADDR(reg) (0x80 | (reg << 1))
#define MAKE_WRITE_ADDR(reg) (reg << 1)

#define READ_BIT(byte, mask) (byte & mask)
#define SET_BIT(mask) (1 << mask)
#define RET_ERROR (-1)
#define RET_OK (0)

#define RC522_VERSION_V1_0 (0x91U)
#define RC522_VERSION_V2_0 (0x92U)

#define RC522_DIGITAL_SELF_TEST_VALUES_V1_0 (unsigned char[]){       \
		0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,     \
		0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E, \
		0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41, \
		0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79  \
	}

#define RC522_DIGITAL_SELF_TEST_VALUES_V2_0   (unsigned char[]){    \
		0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,     \
		0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49, \
		0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9, \
		0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F  \
	}

struct rc_io {
	itf_write_ptr write;
	itf_read_ptr read;
	itf_pwr set_pwr;
};

struct rc522_handle {
	struct rc_io io;
	bool powered_on;
	unsigned char version;
};

/** @brief Reads only one register
 * 	@param[in] handle Holds the function ptrs to IO.
 * 	@param[in] reg RC522_REG...
 *  @retval -1 Reading failed
 *  @retval value of the register
 */
static unsigned char priv_read_reg(rc522_handle *handle, unsigned char reg) {
	unsigned char data_in_out[2] = { MAKE_READ_ADDR(reg), 0x00, };
	int status = 0;
	status = handle->io.read(2, &data_in_out[0]);
	return status == RET_ERROR ? RET_ERROR : data_in_out[0];
}

/**
 * @brief Reads bunch of data from bunch of registers.
 * @param[in] handle Holds the function ptrs to IO.
 * @param[in] reg RC522_REG...
 * @param[in] n Number of data to read.
 * @param[inout] data Holds the addresses for registers to read from and also is the array for data.
 * @retval RET_ERROR
 * @retval RC_OK
 */
static int priv_read_data(rc522_handle *handle, unsigned char reg, size_t n,
		unsigned char data[static n]) {
	unsigned char data_in_out[n];
	for (size_t i = 0; i < n; i++) {
		data_in_out[i] = MAKE_READ_ADDR(reg);
	}

	int status = 0;
	status = handle->io.read(n, &data_in_out[0]);
	for (size_t i = 0; i < n; i++) {
		data[i] = data_in_out[i];
	}
	return status;;
}

/**
 * @brief Write single value to single register.
 * @param[in] handle Holds function ptr to IOs
 * @param[in] reg RC522_REG...
 * @param[in] data value to write register with.
 * @retval RET_ERROR
 * @retval RC_OK
 */
static int priv_write_reg(rc522_handle *handle, unsigned char reg,
		unsigned char data) {
	unsigned char data_to_send[2] = { [0] = MAKE_WRITE_ADDR(reg), [1] = data };

	return handle->io.write(2, &data_to_send[0]);
}

/**
 * @brief Writes multiple values to single register. Mainly used for FIFO manipulations.
 * @param[in] handle Holds function ptr to IOs
 * @param[in] reg RC522_REG...
 * @param[in] n length of data array
 * @param[in] data values to write the register with.
 * @retval RET_ERROR
 * @retval RC_OK
 */
static int priv_write_data(rc522_handle *handle, unsigned char reg, size_t n,
		unsigned char data[static n]) {
	unsigned char data_to_send[n + 1];
	data_to_send[0] = MAKE_WRITE_ADDR(reg);

	for (size_t i = 0; i < n; i++) {
		data_to_send[i + 1] = data[i];
	}

	return handle->io.write(n + 1, &data_to_send[0]);
}

/**
 * @brief Arguments are functions that depends on the platform.
 * @param fptr_write Must return -1 when error occurs and 0 when ok.
 * @param fptr_read Must return -1 when error occurs and 0 when ok.
 * @param fptr_set_pwr Powers on the device
 * @retval NULL
 * @retval handle
 */
rc522_handle* rc522_init(itf_write_ptr fptr_write, itf_read_ptr fptr_read,
		itf_pwr fptr_set_pwr) {
	if (fptr_write == NULL || fptr_read == NULL || fptr_set_pwr == NULL) {
		return NULL;
	}

	rc522_handle *handle = calloc(sizeof(rc522_handle), 1);
	if (handle == NULL) {
		return NULL;
	}
	handle->io.write = fptr_write;
	handle->io.read = fptr_read;
	handle->io.set_pwr = fptr_set_pwr;

	rc522_hw_pwr(handle, true);
	priv_write_reg(handle, RC522_REG_TX_MODE, 0x00);
	priv_write_reg(handle, RC522_REG_RX_MODE, 0x00);

	priv_write_reg(handle, RC522_REG_MOD_WIDTH, 0x26);
	priv_write_reg(handle, RC522_REG_T_MODE, 0x80);
	priv_write_reg(handle, RC522_REG_T_PRESCALER, 0xA9);
	priv_write_reg(handle, RC522_REG_T_RELOAD_H, 0x03);
	priv_write_reg(handle, RC522_REG_T_RELOAD_L, 0xE8);
	priv_write_reg(handle, RC522_REG_TX_ASK, 0x40);
	priv_write_reg(handle, RC522_REG_MODE, 0x3D);

	unsigned char data = priv_read_reg(handle, RC522_REG_TX_CTRL);
	if( (data & REG_TX_CTRL_TX1_RF_EN_MASK) && (data & REG_TX_CTRL_TX2_RF_EN_MASK))
	{
		priv_write_reg(handle, RC522_REG_TX_CTRL, REG_TX_CTRL_TX1_RF_EN_MASK | REG_TX_CTRL_TX2_RF_EN_MASK);
	}
	return handle;
}

/**
 * @brief Hardware power on via pin.
 * @param[in] handle pointer with IO function pointers.
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_hw_pwr(rc522_handle *handle, bool on_off) {
	if (handle == NULL) {
		return RET_ERROR;
	}

	handle->io.set_pwr(on_off);
	unsigned char reg = priv_read_reg(handle, RC522_REG_COMMAND);
	handle->powered_on = READ_BIT(reg, REG_CMD_PWR_DOWN_MASK);
	if (handle->powered_on == on_off) {
		return RET_OK;
	}
	return RET_ERROR;
}

/**
 * @brief Hardware power on via pin.
 * @param[in] handle pointer with IO function pointers.
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_sw_pwr(rc522_handle *handle, bool on_off) {
	if (handle == NULL) {
		return RET_ERROR;
	}

	unsigned char data = priv_read_reg(handle, RC522_REG_COMMAND);
	if (on_off == true) {
		data |= REG_CMD_PWR_DOWN_MASK;
	} else {
		data &= ~REG_CMD_PWR_DOWN_MASK;
	}

	if (priv_write_reg(handle, RC522_REG_COMMAND, data) == RET_ERROR) {
		return RET_ERROR;
	}
	handle->powered_on = false;
	return RET_OK;
}

/**
 * @param[in] handle pointer with IO function pointers.
 * @retval RET_ERROR
 * @retval software version (0x91 for v1.0, 0x92 for v2.0)
 */
int rc522_read_software_version(rc522_handle *handle) {
	if (handle == NULL) {
		return RET_ERROR;
	}
	int version = priv_read_reg(handle, RC522_REG_VERSION);
	handle->version = version;

	return version == RET_ERROR ? RET_ERROR : version;
}

/**
 * @brief Send command to chip
 * @param[in] handle pointer with IO function pointers.
 * @param[in] cmd RC522_CMD_...
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_send_cmd(rc522_handle *handle, rc522_cmd cmd) {
	if (handle == NULL) {
		return RET_ERROR;
	}
	unsigned char data = (cmd << REG_CMD_ARBITRARY_POS);
	return priv_write_reg(handle, RC522_REG_COMMAND, data);
}

/**
 * @brief Read number of bytes in FIFO buffer
 * @param[in] handle pointer with IO function pointers.
 * @param[in] cmd RC522_CMD_...
 * @retval RET_ERROR
 * @retval FIFO count
 */
size_t rc522_get_fifo_count(rc522_handle *handle) {
	if (handle == NULL) {
		return RET_ERROR;
	}
	unsigned char data = priv_read_reg(handle, RC522_REG_FIFO_LEVEL);
	size_t len = data & REG_FIFO_LEVEL_VALUE_MASK;
	if (len > 64) {
		return RET_ERROR;
	}
	return len;
}

/**
 * @brief Resets FIFO counter.
 * @param[in] handle pointer with IO function pointers.
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_flush_fifo_count(rc522_handle *handle) {
	if (handle == NULL) {
		return RET_ERROR;
	}

	unsigned char data = 0x80;
	return priv_write_reg(handle, RC522_REG_FIFO_LEVEL, data);
}

/**
 * @brief Read FIFO buffer. First make sure you read the number of bytes that are present in FIFO. Use RC522_get_fifo_count for that.
 * @warning FIFO can have max 64 bytes, so if n is above, this function returns error.
 * @param[in] handle pointer with IO function pointers.
 * @param[in] n number of bytes to read from FIFO
 * @param[in] data array for FIFO bytes.
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_read_fifo(rc522_handle *handle, size_t n,
		unsigned char data[static n]) {
	if (handle == NULL || n > RC522_FIFO_CNT_MAX) {
		return RET_ERROR;
	}

	return priv_read_data(handle, RC522_REG_FIFO_DATA, n, data);
}

/**
 * @brief Writes bytes to FIFO buffer.
 * @warning FIFO can have max 64 bytes, so if n is above, this function returns error. This function does not check whether there is space in buffer. If it is important, the caller must take care of that.
 * @param[in] handle pointer with IO function pointers.
 * @param[in] n number of bytes to read from FIFO
 * @param[in] data array for FIFO bytes.
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_write_fifo(rc522_handle *handle, size_t n,
		unsigned char data[static n]) {
	if (handle == NULL) {
		return RET_ERROR;
	}

	int ret = priv_write_data(handle, RC522_REG_FIFO_DATA, n, data);
	return ret;
}

rc522_cmd rc522_get_active_cmd(rc522_handle *handle)
{
	unsigned char data = priv_read_reg(handle, RC522_REG_COMMAND) & REG_CMD_ARBITRARY_MASK;
	return data;
}

/**
 * @brief Performs digital self test.
 * @warning Before performing self test, the caller must read software version or function will fail! The bytes in FIFO differs from version to version.
 * @param[in] handle pointer with IO function pointers.
 * @retval RET_ERROR
 * @retval RC_OK
 */
int rc522_perform_digital_self_test(rc522_handle *handle) {
	if (handle == NULL) {
		return RET_ERROR;
	}

	unsigned char compare_vals[RC522_FIFO_CNT_MAX] = { 0 };
	if (handle->version == RC522_VERSION_V1_0) {
		for (size_t i = 0; i < RC522_FIFO_CNT_MAX; i++) {
			compare_vals[i] = RC522_DIGITAL_SELF_TEST_VALUES_V1_0 [i];
		}
	} else if (handle->version == RC522_VERSION_V2_0) {
		for (size_t i = 0; i < RC522_FIFO_CNT_MAX; i++) {
			compare_vals[i] = RC522_DIGITAL_SELF_TEST_VALUES_V2_0 [i];
		}
	} else {
		return RET_ERROR;
	}
	unsigned char data[RC522_FIFO_CNT_MAX] = { 0 };

	rc522_send_cmd(handle, RC522_CMD_SOFT_RST);

	/* Wait until chip powers up again. */
	while(rc522_get_active_cmd(handle) != RC522_CMD_IDLE);

	/* Clear internal buffer by writing 25 bytes of 0x00 first to buffer and then isuing the MEM cmd. */
	rc522_write_fifo(handle, 25, &data[0]);
	rc522_send_cmd(handle, RC522_CMD_MEM);

	/* Wait until MEM cmd finishes. */
	while(rc522_get_active_cmd(handle) != RC522_CMD_IDLE);

	const unsigned char enable_self_test = (0x09 << REG_AUTO_TEST_SELF_TEST_POS);
	priv_write_reg(handle, RC522_REG_AUTO_TEST, enable_self_test);

	rc522_flush_fifo_count(handle);
	rc522_write_fifo(handle, 1, &data[0]);
	rc522_send_cmd(handle, RC522_CMD_CALC_CRC);

	//while crc is active
	size_t fifo_count = rc522_get_fifo_count(handle);
	while(rc522_get_active_cmd(handle) != RC522_CMD_IDLE);
	rc522_read_fifo(handle, fifo_count, data);
	for (size_t i = 0; i < RC522_FIFO_CNT_MAX; i++) {
		if (data[i] != compare_vals[i]) {
			return RET_ERROR;
		}
	}
	return RET_OK;
}
