#include "rc522.h"
#include <stdlib.h>
#include <string.h>

#define MFRC_MAKE_READ_ADDR(reg) (0x80 | (reg << 1))
#define MFRC_MAKE_WRITE_ADDR(reg) (reg << 1)

#define READ_BIT(byte, mask) (byte & mask)
#define SET_BIT(mask) (1 << mask)
#define MFRC_ERROR (-1)
#define MFRC_OK (0)

#define MFRC_VERSION_V1_0 (0x91U)
#define MFRC_VERSION_V2_0 (0x92U)

#define MFRC_DIGITAL_SELF_TEST_VALUES_V1_0 (unsigned char[]){       \
		0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C, 0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,     \
		0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A, 0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E, \
		0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC, 0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41, \
		0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02, 0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79  \
	}

#define MFRC_DIGITAL_SELF_TEST_VALUES_V2_0   (unsigned char[]){    \
		0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95, 0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,     \
		0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82, 0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49, \
		0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81, 0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9, \
		0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D, 0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F  \
	}

struct mfrc_io {
	itf_write_ptr write;
	itf_read_ptr read;
	itf_pwr set_pwr;
};

struct mfrc522_handle {
	struct mfrc_io io;
	bool powered_on;
	unsigned char version;
};

/** @brief Reads only one register
 * 	@param[in] handle Holds the function ptrs to IO.
 * 	@param[in] reg MFRC522_REG...
 *  @retval -1 Reading failed
 *  @retval value of the register
 */
static unsigned char priv_read_reg(mfrc522_handle *handle, unsigned char reg) {
	unsigned char data_in_out[2] = { MFRC_MAKE_READ_ADDR(reg), 0x00, };
	int status = 0;
	status = handle->io.read(2, &data_in_out[0]);
	return status == MFRC_ERROR ? MFRC_ERROR : data_in_out[0];
}

/**
 * @brief Reads bunch of data from bunch of registers.
 * @param[in] handle Holds the function ptrs to IO.
 * @param[in] reg MFRC522_REG...
 * @param[in] n Number of data to read.
 * @param[inout] data Holds the addresses for registers to read from and also is the array for data.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
static int priv_read_data(mfrc522_handle *handle, unsigned char reg, size_t n,
		unsigned char data[static n]) {
	unsigned char data_in_out[n];
	for (size_t i = 0; i < n; i++) {
		data_in_out[i] = MFRC_MAKE_READ_ADDR(reg);
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
 * @param[in] reg MFRC522_REG...
 * @param[in] data value to write register with.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
static int priv_write_reg(mfrc522_handle *handle, unsigned char reg,
		unsigned char data) {
	unsigned char data_to_send[2] = { [0] = MFRC_MAKE_WRITE_ADDR(reg), [1] = data };

	return handle->io.write(2, &data_to_send[0]);
}

/**
 * @brief Writes multiple values to single register. Mainly used for FIFO manipulations.
 * @param[in] handle Holds function ptr to IOs
 * @param[in] reg MFRC522_REG...
 * @param[in] n length of data array
 * @param[in] data values to write the register with.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
static int priv_write_data(mfrc522_handle *handle, unsigned char reg, size_t n,
		unsigned char data[static n]) {
	unsigned char data_to_send[n + 1];
	data_to_send[0] = MFRC_MAKE_WRITE_ADDR(reg);

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
mfrc522_handle* mfrc522_init(itf_write_ptr fptr_write, itf_read_ptr fptr_read,
		itf_pwr fptr_set_pwr) {
	if (fptr_write == NULL || fptr_read == NULL || fptr_set_pwr == NULL) {
		return NULL;
	}

	mfrc522_handle *handle = calloc(sizeof(mfrc522_handle), 1);
	if (handle == NULL) {
		return NULL;
	}
	handle->io.write = fptr_write;
	handle->io.read = fptr_read;
	handle->io.set_pwr = fptr_set_pwr;
	return handle;
}

/**
 * @brief Hardware power on via pin.
 * @param[in] handle pointer with IO function pointers.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_hw_pwr(mfrc522_handle *handle, bool on_off) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}

	handle->io.set_pwr(on_off);
	unsigned char reg = priv_read_reg(handle, MFRC522_REG_COMMAND);
	handle->powered_on = READ_BIT(reg, REG_CMD_PWR_DOWN_MASK);
	if (handle->powered_on == on_off) {
		return MFRC_OK;
	}
	return MFRC_ERROR;
}

/**
 * @brief Hardware power on via pin.
 * @param[in] handle pointer with IO function pointers.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_sw_pwr(mfrc522_handle *handle, bool on_off) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}

	unsigned char data = priv_read_reg(handle, MFRC522_REG_COMMAND);
	if (on_off == true) {
		data |= REG_CMD_PWR_DOWN_MASK;
	} else {
		data &= ~REG_CMD_PWR_DOWN_MASK;
	}

	if (priv_write_reg(handle, MFRC522_REG_COMMAND, data) == MFRC_ERROR) {
		return MFRC_ERROR;
	}
	handle->powered_on = false;
	return MFRC_OK;
}

/**
 * @param[in] handle pointer with IO function pointers.
 * @retval MFRC_ERROR
 * @retval software version (0x91 for v1.0, 0x92 for v2.0)
 */
int mfrc522_read_software_version(mfrc522_handle *handle) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}
	int version = priv_read_reg(handle, MFRC522_REG_VERSION);
	handle->version = version;

	return version == MFRC_ERROR ? MFRC_ERROR : version;
}

/**
 * @brief Send command to chip
 * @param[in] handle pointer with IO function pointers.
 * @param[in] cmd MFRC_CMD_...
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_send_cmd(mfrc522_handle *handle, mfrc522_cmd cmd) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}
	unsigned char data = (cmd << REG_CMD_ARBITRARY_POS);
	return priv_write_reg(handle, MFRC522_REG_COMMAND, data);
}

/**
 * @brief Read number of bytes in FIFO buffer
 * @param[in] handle pointer with IO function pointers.
 * @param[in] cmd MFRC_CMD_...
 * @retval MFRC_ERROR
 * @retval FIFO count
 */
size_t mfrc522_get_fifo_count(mfrc522_handle *handle) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}
	unsigned char data = priv_read_reg(handle, MFRC522_REG_FIFO_LEVEL);
	size_t len = data & REG_FIFO_LEVEL_VALUE_MASK;
	if (len > 64) {
		return MFRC_ERROR;
	}
	return len;
}

/**
 * @brief Resets FIFO counter.
 * @param[in] handle pointer with IO function pointers.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_flush_fifo_count(mfrc522_handle *handle) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}

	unsigned char data = 0x80;
	return priv_write_reg(handle, MFRC522_REG_FIFO_LEVEL, data);
}

/**
 * @brief Read FIFO buffer. First make sure you read the number of bytes that are present in FIFO. Use mfrc522_get_fifo_count for that.
 * @warning FIFO can have max 64 bytes, so if n is above, this function returns error.
 * @param[in] handle pointer with IO function pointers.
 * @param[in] n number of bytes to read from FIFO
 * @param[in] data array for FIFO bytes.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_read_fifo(mfrc522_handle *handle, size_t n,
		unsigned char data[static n]) {
	if (handle == NULL || n > MFRC522_FIFO_CNT_MAX) {
		return MFRC_ERROR;
	}

	return priv_read_data(handle, MFRC522_REG_FIFO_DATA, n, data);
}

/**
 * @brief Writes bytes to FIFO buffer.
 * @warning FIFO can have max 64 bytes, so if n is above, this function returns error. This function does not check whether there is space in buffer. If it is important, the caller must take care of that.
 * @param[in] handle pointer with IO function pointers.
 * @param[in] n number of bytes to read from FIFO
 * @param[in] data array for FIFO bytes.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_write_fifo(mfrc522_handle *handle, size_t n,
		unsigned char data[static n]) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}

	return priv_write_data(handle, MFRC522_REG_FIFO_DATA, n, data);
}

mfrc522_cmd mfrc522_get_active_cmd(mfrc522_handle *handle)
{
	unsigned char data = priv_read_reg(handle, MFRC522_REG_COMMAND) & REG_CMD_ARBITRARY_MASK;
	return data;
}

/**
 * @brief Performs digital self test.
 * @warning Before performing self test, the caller must read software version or function will fail! The bytes in FIFO differs from version to version.
 * @param[in] handle pointer with IO function pointers.
 * @retval MFRC_ERROR
 * @retval MFRC_OK
 */
int mfrc522_perform_digital_self_test(mfrc522_handle *handle) {
	if (handle == NULL) {
		return MFRC_ERROR;
	}

	unsigned char compare_vals[MFRC522_FIFO_CNT_MAX] = { 0 };
	if (handle->version == MFRC_VERSION_V1_0) {
		for (size_t i = 0; i < MFRC522_FIFO_CNT_MAX; i++) {
			compare_vals[i] = MFRC_DIGITAL_SELF_TEST_VALUES_V1_0 [i];
		}
	} else if (handle->version == MFRC_VERSION_V2_0) {
		for (size_t i = 0; i < MFRC522_FIFO_CNT_MAX; i++) {
			compare_vals[i] = MFRC_DIGITAL_SELF_TEST_VALUES_V2_0 [i];
		}
	} else {
		return MFRC_ERROR;
	}
	unsigned char data[MFRC522_FIFO_CNT_MAX] = { 0 };

	mfrc522_send_cmd(handle, MFRC_CMD_SOFT_RST);

	/* Wait until chip powers up again. */
	while(mfrc522_get_active_cmd(handle) != MFRC_CMD_IDLE);

	/* Clear internal buffer by writing 25 bytes of 0x00 first to buffer and then isuing the MEM cmd. */
	mfrc522_write_fifo(handle, 25, &data[0]);
	mfrc522_send_cmd(handle, MFRC_CMD_MEM);

	/* Wait until MEM cmd finishes. */
	while(mfrc522_get_active_cmd(handle) != MFRC_CMD_IDLE);

	const unsigned char enable_self_test = (0x09 << REG_AUTO_TEST_SELF_TEST_POS);
	priv_write_reg(handle, MFRC522_REG_AUTO_TEST, enable_self_test);

	mfrc522_flush_fifo_count(handle);
	mfrc522_write_fifo(handle, 1, &data[0]);
	mfrc522_send_cmd(handle, MFRC_CMD_CALC_CRC);

	//while crc is active
	size_t fifo_count = mfrc522_get_fifo_count(handle);
	while(mfrc522_get_active_cmd(handle) != MFRC_CMD_IDLE);
	mfrc522_read_fifo(handle, fifo_count, data);
	for (size_t i = 0; i < MFRC522_FIFO_CNT_MAX; i++) {
		if (data[i] != compare_vals[i]) {
			return MFRC_ERROR;
		}
	}
	return MFRC_OK;
}
