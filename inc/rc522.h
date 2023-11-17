#ifndef __RC522_H__
#define __RC522_H__

#include "rc522_registers.h"
#include <stdbool.h>
#include <stdlib.h>

#define RC522_FIFO_CNT_MAX (64)

typedef int (*itf_write_ptr)(size_t, unsigned char[]);
typedef int (*itf_read_ptr)(size_t, unsigned char[]);
typedef void (*itf_pwr)(bool);

typedef enum rc522_commands {
  RC522_CMD_IDLE = 0x00,
  RC522_CMD_MEM = 0x01,
  RC522_CMD_GEN_RAND_ID = 0x02,
  RC522_CMD_CALC_CRC = 0x03,
  RC522_CMD_TX = 0x04,
  RC522_CMD_NO_CMD_CHANGE = 0x07,
  RC522_CMD_RX = 0x08,
  RC522_CMD_TX_RX = 0x0C,
  RC522_CMD_MF_AUTH = 0x0E,
  RC522_CMD_SOFT_RST = 0x0F,
} rc522_cmd;

typedef enum crc_presets {
	RC522_CRC_PRESET_0000 = 0x00,
	RC522_CRC_PRESET_6363 = 0x01,
	RC522_CRC_PRESET_A671 = 0x02,
	RC522_CRC_PRESET_FFFF = 0x03,
} rc522_crc_preset;

typedef enum tx_speeds {
	RC522_TX_SPEED_106kBd = 0x00,
	RC522_TX_SPEED_212kBd = 0x01,
	RC522_TX_SPEED_424kBd = 0x02,
	RC522_TX_SPEED_848kBd = 0x03,
} rc522_tx_speed;

typedef struct rc522_handle rc522_handle;

rc522_handle *rc522_init(itf_write_ptr fptr_write, itf_read_ptr fptr_read,
                             itf_pwr fptr_set_pwr);

int rc522_read_software_version(rc522_handle *handle);
int rc522_hw_pwr(rc522_handle *handle, bool on_off);
int rc522_sw_pwr(rc522_handle *handle, bool on_off);
int rc522_send_cmd(rc522_handle *handle, rc522_cmd cmd);
size_t rc522_get_fifo_count(rc522_handle *handle);
int rc522_flush_fifo_count(rc522_handle *handle);
int rc522_read_fifo(rc522_handle *handle, size_t n,
                      unsigned char data[static n]);
int rc522_write_fifo(rc522_handle *handle, size_t n,
                       unsigned char data[static n]);
rc522_cmd rc522_get_active_cmd(rc522_handle *handle);
int rc522_enable_digital_self_test(rc522_handle *handle);
#endif
