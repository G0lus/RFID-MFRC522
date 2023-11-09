#ifndef __RC522_H__
#define __RC522_H__

#include "rc522_registers.h"
#include <stdbool.h>
#include <stdlib.h>

#define MFRC522_FIFO_CNT_MAX (64)

typedef int (*itf_write_ptr)(size_t, unsigned char[]);
typedef int (*itf_read_ptr)(size_t, unsigned char[]);
typedef void (*itf_pwr)(bool);

typedef enum mfrc522_command {
  MFRC_CMD_IDLE = 0x00,
  MFRC_CMD_MEM = 0x01,
  MFRC_CMD_GEN_RAND_ID = 0x02,
  MFRC_CMD_CALC_CRC = 0x03,
  MFRC_CMD_TX = 0x04,
  MFRC_CMD_NO_CMD_CHANGE = 0x07,
  MFRC_CMD_RX = 0x08,
  MFRC_CMD_TX_RX = 0x0C,
  MFRC_CMD_MF_AUTH = 0x0E,
  MFRC_CMD_SOFT_RST = 0x0F,
} mfrc522_cmd;

typedef struct mfrc522_handle mfrc522_handle;

mfrc522_handle *mfrc522_init(itf_write_ptr fptr_write, itf_read_ptr fptr_read,
                             itf_pwr fptr_set_pwr);

int mfrc522_read_software_version(mfrc522_handle *handle);
int mfrc522_hw_pwr(mfrc522_handle *handle, bool on_off);
int mfrc522_sw_pwr(mfrc522_handle *handle, bool on_off);
int mfrc522_send_cmd(mfrc522_handle *handle, mfrc522_cmd cmd);
size_t mfrc522_get_fifo_count(mfrc522_handle *handle);
int mfrc522_flush_fifo_count(mfrc522_handle *handle);
int mfrc522_read_fifo(mfrc522_handle *handle, size_t n,
                      unsigned char data[static n]);
int mfrc522_write_fifo(mfrc522_handle *handle, size_t n,
                       unsigned char data[static n]);
mfrc522_cmd mfrc522_get_active_cmd(mfrc522_handle *handle);
int mfrc522_enable_digital_self_test(mfrc522_handle *handle);
#endif
