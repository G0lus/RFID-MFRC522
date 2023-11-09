#ifndef RC522_REGISTERS_H
#define RC522_REGISTERS_H

/* MFRC522 Commands */
/* NO action; Cancel the current command */
#define PCD_IDLE 0x00
/* Authentication Key */
#define PCD_AUTHENT 0x0E
/* Receive Data */
#define PCD_RECEIVE 0x08
/* Transmit data */
#define PCD_TRANSMIT 0x04
/* Transmit and receive data */
#define PCD_TRANSCEIVE 0x0C
/* Reset */
#define PCD_RESETPHASE 0x0F
/* CRC Calculate */
#define PCD_CALCCRC 0x03

/* Mifare_One card command word */
/* Find the antenna area does not enter hibernation */
#define PICC_REQIDL 0x26
/* Find all the cards antenna area */
#define PICC_REQALL 0x52
/* Anti-collision */
#define PICC_ANTICOLL 0x93
/* Election card */
#define PICC_SElECTTAG 0x93
/* Authentication key A */
#define PICC_AUTHENT1A 0x60
/* Authentication key B */
#define PICC_AUTHENT1B 0x61
/* Read Block */
#define PICC_READ 0x30
/* Write block */
#define PICC_WRITE 0xA0
/* Debit */
#define PICC_DECREMENT 0xC0
/* Recharge */
#define PICC_INCREMENT 0xC1
/* Transfer block data to the buffer */
#define PICC_RESTORE 0xC2
/* Save the data in the buffer */
#define PICC_TRANSFER 0xB0
/* Sleep */
#define PICC_HALT 0x50

/* MFRC522 Registers and bitmasks */
/* Page 0: Command and Status */

/* Starts and stops command execution */
#define MFRC522_REG_COMMAND 0x01

/* Analog part if receiver is switched off */
#define REG_CMD_RCV_OFF_POS 5
#define REG_CMD_RCV_OFF_MASK (1 << REG_CMD_RCV_OFF_POS)

/* Soft power-down mode entered */
#define REG_CMD_PWR_DOWN_POS 4
#define REG_CMD_PWR_DOWN_MASK (1 << REG_CMD_PWR_DOWN_POS)
/* Activates command based on Command value. Section 10.3 of datasheet. */
#define REG_CMD_ARBITRARY_POS 0
#define REG_CMD_ARBITRARY_MASK (0x0F << REG_CMD_ARBITRARY_POS)

/* Register to control what irq requests gets propagated to irq pin */
#define MFRC522_REG_COM_IT_EN 0x02
/* Signal on irq pin is inverted with respect to Status1Reg register's IRQ bit
 */
#define REG_COM_IT_IRQ_INV_POS 7
#define REG_COM_IT_IRQ_INV_MASK (1 << REG_COM_IT_IRQ_INV_POS)
/* Transmitter irq request */
#define REG_COM_IT_IRQ_TX_POS 6
#define REG_COM_IT_IRQ_TX_MASK (1 << REG_COM_IT_IRQ_TX_POS)
/* Receiver irq request */
#define REG_COM_IT_IRQ_RX_POS 5
#define REG_COM_IT_IRQ_RX_MASK (1 << REG_COM_IT_IRQ_RX_POS)
/* Idle irq request */
#define REG_COM_IT_IRQ_IDLE_POS 4
#define REG_COM_IT_IRQ_IDLE_MASK (1 << REG_COM_IT_IRQ_IDLE_POS)
/* High alert irq request */
#define REG_COM_IT_IRQ_HI_ALERT_POS 3
#define REG_COM_IT_IRQ_HI_ALERT_MASK (1 << REG_COM_IT_IRQ_HI_ALERT_POS)
/* Low alert irq request */
#define REG_COM_IT_IRQ_LO_ALERT_POS 2
#define REG_COM_IT_IRQ_LO_ALERT_MASK (1 << REG_COM_IT_IRQ_LO_ALERT_POS)
/* Error irq requst */
#define REG_COM_IT_IRQ_ERR_POS 1
#define REG_COM_IT_IRQ_ERR_MASK (1 << REG_COM_IT_IRQ_ERR_POS)
/* Timer irq request */
#define REG_COM_IT_IRQ_TIM_POS 0
#define REG_COM_IT_IRQ_TIM_MASK (1 << REG_COM_IT_IRQ_TIM_POS)

/* Register to control what irq requests gets propagated to irq pin - same as
 * ComIEnReg*/
#define MFRC522_REG_DIV_I_EN 0x03
/* 1 - irq pin is standard CMOS out pin | 0 - irq pin is OpenDrain output pin */
#define REG_DIV_I_EN_IRQ_PUSH_PULL_POS 7
#define REG_DIV_I_EN_IRQ_PUSH_PULL_MASK (1 << DIV_I_EN_IRQ_PUSH_PULL_POS)
/* MFIN active irq request */
#define REG_DIV_I_EN_MFIN_ACT_IT_POS 4
#define REG_DIV_I_EN_MFIN_ACT_IT_MASK (1 << DIV_I_EN_MFIN_ACT_IT_POS)
/* CRC irq request */
#define REG_DIV_I_EN_CRC_IRQ_POS 2
#define REG_DIV_I_EN_CRC_IRQ_MASK (1 << DIV_I_EN_CRC_IRQ_POS)

/* Irq request bits. All bits are cleared by software. */
#define MFRC522_REG_COMM_IRQ 0x04
/* Indicates thet the marked bits in the ComIrqReg register are set or cleared
 */
#define REG_COMM_IRQ_SET1_POS 7
#define REG_COMM_IRQ_SET1_MASK (1 << REG_COMM_IRQ_SET1_POS)
/* Set immediately after last bit of tx data was sent out */
#define REG_COMM_IRQ_TX_IRQ_POS 6
#define REG_COMM_IRQ_TX_IRQ_MASK (1 << REG_COMM_IRQ_TX_IRQ_POS)
/* Receiver has detected the end of valid data stream. If the RxModeReg
 * register's RxNoErr bit is set to logic 1, the RxIRq bit is only set to logic
 * 1 when data bytes are available in the FIFO. */
#define REG_COMM_IRQ_RX_IRQ_POS 5
#define REG_COMM_IRQ_RX_IRQ_MASK (1 << REG_COMM_IRQ_RX_IRQ_POS)
/* Set after command terminates or an unknown command is started */
#define REG_COMM_IRQ_IDLE_IRQ_POS 4
#define REG_COMM_IRQ_IDLE_IRQ_MASK (1 << REG_COMM_IRQ_IDLE_IRQ_POS)
/* HiAlert is set. Can be cleared only by writing logic 0 in Set1's bit */
#define REG_COMM_IRQ_HI_ALERT_IRQ_POS 3
#define REG_COMM_IRQ_HI_ALERT_IRQ_MASK (1 << REG_COMM_IRQ_HI_ALERT_IRQ_POS)
/* LoAlert is set. Can be cleared only by writing logic 0 in Set1's bit */
#define REG_COMM_IRQ_LO_ALERT_IRQ_POS 2
#define REG_COMM_IRQ_LO_ALERT_IRQ_MASK (1 << REG_COMM_IRQ_LO_ALERT_IRQ_POS)
/* Any error in error register is set */
#define REG_COMM_IRQ_ERR_IRQ_POS 1
#define REG_COMM_IRQ_ERR_IRQ_MASK (1 << REG_COMM_IRQ_ERR_IRQ_POS)

/* Irq request bits. All bits are cleared by software. */
#define MFRC522_REG_DIV_IRQ 0x05
/* Indicates thet the marked bits in the DivIrqReg register are set or cleared
 */
#define REG_DIV_IRQ_SET2_POS 7
#define REG_DIV_IRQ_SET2_MASK (1 << REG_DIV_IRQ_SET2_POS)
/* MFIN is active. Activated on any edge of signal - falling and rising. */
#define REG_DIV_IRQ_MFIN_ACT_IT_POS 4
#define REG_DIV_IRQ_MFIN_ACT_IT_MASK (1 << REG_DIV_IRQ_MFIN_ACT_IT_POS)
/* CalcCRC Command is active and all data is processed*/
#define REG_DIV_IRQ_CRC_IRQ_POS 2
#define REG_DIV_IRQ_CRC_IRQ_MASK (1 << REG_DIV_IRQ_CRC_IRQ_POS)

/* Error bits showing the error status of the last command */
#define MFRC522_REG_ERROR 0x06
/* Error in writing data to FIFO Buffer. Either during MFAutenth cmd or time
 * between sending the last bit on the RF interface and receiving last bit on
 * the RF interface. */
#define REG_ERROR_WR_POS 7
#define REG_ERROR_WR_MASK (1 << REG_ERROR_WR_POS)
/* Internal temperature sensor detects overheating. Antennas are switched off */
#define REG_ERROR_TEMP_POS 6
#define REG_ERROR_TEMP_MASK (1 << REG_ERROR_TEMP_POS)
/* Writing to full FIFO Buffer triggers this error */
#define REG_ERROR_BUFF_OVRFL_POS 4
#define REG_ERROR_BUFF_OVRFL_MASK (1 << REG_ERROR_BUFF_OVRFL_POS)
/* Bit-collision detected. Cleared automatically at receiver start-up phase.
 * Only valid during bitwise anticollision at 106 kBd. Always set to logic 0
 * during comms at 212 kBd, 424 kBd, 848 kBd */
#define REG_ERROR_COLL_POS 3
#define REG_ERROR_COLL_MASK (1 << REG_ERROR_COLL_POS)
/* CRC Calculation fails. Automatically cleared on start-up phase */
#define REG_ERROR_CRC_POS 2
#define REG_ERROR_CRC_MASK (1 << REG_ERROR_CRC_POS)
/* Parity check failed. Automatically cleared on start-up phase. Only valid on
 * ISO/IEC 14443 A/MIFARE comm at 106 kBd */
#define REG_ERROR_PARITY_POS 1
#define REG_ERROR_PARITY_MASK (1 << REG_ERROR_PARITY_POS)
/* SOF is incorrect. Auto cleared at start-up phase. Only valid for 106 kBd. Set
 * if number of received bytes are incorrect during MFAuthent command. */
#define REG_ERROR_PROTOCOL_POS 0
#define REG_ERROR_PROTOCOL_MASK (1 << REG_ERROR_PROTOCOL_POS)

/* Contains status bits of the CRC, interrupt and FIFO buffer */
#define MFRC522_REG_STATUS1 0x07
/* The CRC result is zero. For data transmission and reception, the CRCOk bit is
 * undefined:, you should use ErrorReg register's CRCErr bit.
 * During calculation the value is 0, after correct calculation it changes to 1.
 */
#define REG_STAT1_CRC_OK_POS 6
#define REG_STAT1_CRC_OK_MASK (1 << REG_STAT1_CRC_OK_POS)
/* The CRC calculaton is finished.
 * Only valid for the CRC coprocessor calculation using CalcCRC command */
#define REG_STAT1_CRC_RDY_POS 5
#define REG_STAT1_CRC_RDY_MASK (1 << REG_STAT1_CRC_RDY_POS)
/* Indicates if any interrupt source requests attention with respect to the
 * settings of the irq enable bits: see the ComIEnReg and DivIEnReg registers
 */
#define REG_STAT1_IRQ_POS 4
#define REG_STAT1_IRQ_MASK (1 << REG_STAT1_IRQ_POS)
/* MFRC522’s timer unit is running, i.e. the timer will decrement the
 * TCounterValReg register with the next timer clock. */
#define REG_STAT1_TRUNNING_POS 3
#define REG_STAT1_TRUNNING_MASK (1 << REG_STAT1_TRUNNING_POS)
/* the number of bytes stored in the FIFO buffer corresponds to equation:
 * HiAlert = (64 - FIFOLength) <= WaterLevel */
#define REG_STAT1_HIALERT_POS 1
#define REG_STAT1_HIALERT_MASK (1 << REG_STAT1_HIALERT_POS)
/* the number of bytes stored in the FIFO buffer corresponds to equation:
 * LoAlert = FIFOLength <= WaterLevel */
#define REG_STAT1_LOALERT_POS 0
#define REG_STAT1_LOALERT_MASK (1 << REG_STAT1_LOALERT_POS)

/* Contains status bits of the receiver, transmitter and data mode detector. */
#define MFRC522_REG_STATUS2 0x08
/* clears the temperature error if the temperature is below the alarm limit of
 * 125 deg C */
#define REG_STAT2_TEMP_SENS_CLEAR_POS 7
#define REG_STAT2_TEMP_SENS_CLEAR_MASK (1 << REG_STAT2_TEMP_SENS_CLEAR_POS)
/* I2C Input filter settngs: 1 - I2C Hgh Speed is used. 0 - normal I2C protocol.
 */
#define REG_STAT2_I2C_FORCE_HS_POS 6
#define REG_STAT2_I2C_FORCE_HS_MASK (1 << REG_STAT2_I2C_FORCE_HS_POS)
/* Indcates that MIFARE Crypto1 unit is switched on and all comm with card is
 * encrypted. Can only be set to 1 by a successful execution of MFAuthent
 * command. */
#define REG_STAT2_MF_CRYPTO_1_ON_POS 3
#define REG_STAT2_MF_CRYPTO_1_ON_MASK (1 << REG_STAT2_MF_CRYPTO_1_ON_POS)
/* Shows the state of the transmitter and receiver state machines:
 * 000 - idle
 * 001 - wait for the BitFramingReg register’s StartSend bit
 * 010 - TxWait: wait until RF field is present if the TModeReg
 * register’s TxWaitRF bit is set to logic 1
 * the minimum time for TxWait is defined by the TxWaitReg
 * register
 * 011 - transmitting
 * 100 - RxWait: wait until RF field is present if the TModeReg
 * register’s TxWaitRF bit is set to logic 1
 * the minimum time for RxWait is defined by the
 * RxWaitReg register
 * 101 - wait for data
 * 110 - receiving */
#define REG_STAT2_MODEM_STATE_POS 0
/* defines a warning level to indicate a FIFO buffer overflow or underflow:
 * Status1Reg register’s HiAlert bit is set to logic 1 if the remaining number
 * of bytes in the FIFO buffer space is equal to, or less than the defined
 * number of WaterLevel bytes Status1Reg register’s LoAlert bit is set to logic
 * 1 if equal to, or less than the WaterLevel bytes in the FIFO buffer Remark:
 * to calculate values for HiAlert and LoAlert see Section 9.3.1.8 on page 42.
 */
#define REG_STAT2_MODEM_STATE_MASK (0x3F << REG_STAT2_MODEM_STATE_POS)

/* Input and output of 64 byte FIFO buffer */
#define MFRC522_REG_FIFO_DATA 0x09
/* Number of bytes stored in the FIFO buffer */
#define MFRC522_REG_FIFO_LEVEL 0x0A
#define REG_FIFO_LEVEL_VALUE_POS 0
#define REG_FIFO_LEVEL_VALUE_MASK (0x7F << REG_FIFO_LEVEL_VALUE_POS)
#define REG_FIFO_LEVEL_FLUSH_BUF_POS 7
#define REG_FIFO_LEVEL_FLUSH_BUF_MASK (1 << REG_FIFO_LEVEL_FLUSH_BUF_POS)

/* Level for FIFO underflow and overflow */
#define MFRC522_REG_WATER_LEVEL 0x0B
#define REG_WATER_LVL_VALUE_POS 0
#define REG_WATER_LVL_VALUE_MASK (63 << REG_WATER_LVL_VALUE_POS)

/* MISC control registers */
#define MFRC522_REG_CONTROL 0x0C
#define REG_CTRL_T_STOP_NOW_POS 7
#define REG_CTRL_T_STOP_NOW_MASK (1 << REG_CTRL_T_STOP_NOW_POS)
#define REG_CTRL_T_START_NOW_POS 6
#define REG_CTRL_T_START_NOW_MASK (1 << REG_CTRL_T_START_NOW_POS)
/* indicates the number of vald bits in the last received byte */
#define REG_CTRL_RX_LAST_BTS_POS 0
#define REG_CTRL_RX_LAST_BTS_MASK (8 << REG_CTRL_RX_LAST_BTS_POS)

/* Adjustments for bit-oriented frames */
#define MFRC522_REG_BIT_FRMNG 0x0D
/* Starts the transmission of data. Only valid in combination with the
 * Transceive command. */
#define REG_BIT_FRMNG_START_SEND_POS 7
#define REG_BIT_FRMNG_START_SEND_MASK (1 << REG_BIT_FRMNG_START_SEND_POS)
/* used for reception of bit-oriented frames: defines the bit position for the
 * first bit received to be stored in the FIFO buffer example:
 * 0 LSB of the received bit is stored at bit position 0, the second received
 * bit is stored at bit position 1 1 LSB of the received bit is stored at bit
 * position 1, the second received bit is stored at bit position 2 7 LSB of the
 * received bit is stored at bit position 7, the second received bit is stored
 * in the next byte that follows at bit position 0 These bits are only to be
 * used for bitwise anticollision at 106 kBd, for all other modes they are set
 * to 0. */
#define REG_BIT_FRMNG_RX_ALIGN_POS 4
#define REG_BIT_FRMNG_RX_ALIGN_MASK (7 << REG_BIT_FRMNG_RX_ALIGN_POS)
/* used for transmission of bit oriented frames: defines the number of bits of
 * the last byte that will be transmitted
 * 000b indicates that all bits of the last byte will be transmitted. */
#define REG_BIT_FRMNG_TX_LAST_BITS_POS 0
#define REG_BIT_FRMNG_TX_LAST_BITS_MASK (7 << REG_BIT_FRMNG_TX_LAST_BITS_POS)

/* Bit position of the first bit-collision detected on the RF interface */
#define MFRC522_REG_COLL 0x0E
/* All received bits will be cleared after collision. Only used during bitwise
 * anticollision at 106 kBd, otherwise it is set to logic 1. */
#define REG_COLL_VALS_AFTER_COLL_POS 7
#define REG_COLL_VALS_AFTER_COLL_MASK (1 << REG_COLL_VALS_AFTER_COLL_POS)
/* No collision detected or the position of the collision is out of the range of
 * CollPos[4:0]. */
#define REG_COLL_COLL_POS_NOT_VALID_POS 5
#define REG_COLL_COLL_POS_NOT_VALID_MASK (1 << REG_COLL_COLL_POS_NOT_VALID_POS)
/* Shows the bit position of the first detected collision in a received frame.
 * Only data bits are interpreted. Example:
 * 00h indicates a bit-collision in the 32nd bit
 * 01h indicates a bit-collision in the 1st bit
 * 08h indicates a bit-collision in the 8 th bit.
 * These bits will only be interpreted if the CollPosNotValid bit is set to
 * logic 0. */
#define REG_COLL_FIRST_COLL_POS_POS 0
#define REG_COLL_FIRST_COLL_POS_MASK (31 << REG_COLL_FIRST_COLL_POS_POS)

// Page 1: Communication
/* Reserved for future use */
#define MFRC522_REG_RESERVED10 0x10
/* Defines general modes for transmitting and receiving */
#define MFRC522_REG_MODE 0x11
/* Defines transmission data rate and framing */
#define MFRC522_REG_TX_MODE 0x12
/* Defines reception data rate and framing */
#define MFRC522_REG_RX_MODE 0x13
/* Controls logical behaviour of the antena driver pins TX1 and TX2 */
#define MFRC522_REG_TX_CONTROL 0x14
/* Controls the setting of the transmission modulation */
#define MFRC522_REG_TX_ASK 0x15
/* Selects the internal sources of antenna driver */
#define MFRC522_REG_TX_SEL 0x16
/* Selects internal receiver settings */
#define MFRC522_REG_RX_SEL 0x17
/* Selects thresholds for the bit decoder */
#define MFRC522_REG_RX_THRESHOLD 0x18
/* Defines demodulator settings */
#define MFRC522_REG_DEMOD 0x19
/* Reserved */
#define MFRC522_REG_RESERVED11 0x1A
/* Reserved */
#define MFRC522_REG_RESERVED12 0x1B
/* Controls some MIFARE communication transmit params */
#define MFRC522_REG_MIFARE 0x1C
/* Controls some MIFARE communication receive params */
#define MFRC522_REG_RESERVED13 0x1D
/* Reserved */
#define MFRC522_REG_RESERVED14 0x1E
/* Selects the speed of the serial UART interface */
#define MFRC522_REG_SERIALSPEED 0x1F

// Page 2: CFG
/* Reserved */
#define MFRC522_REG_RESERVED20 0x20
/* CRC Result MSB */
#define MFRC522_REG_CRC_RESULT_M 0x21
/* CRC Result LSB */
#define MFRC522_REG_CRC_RESULT_L 0x22
/* Reserved */
#define MFRC522_REG_RESERVED21 0x23
/* Controls the ModWidth setting */
#define MFRC522_REG_MOD_WIDTH 0x24
/* Reserved */
#define MFRC522_REG_RESERVED22 0x25
/* Configures the receiver gain */
#define MFRC522_REG_RF_CFG 0x26
/* Selects the conductance of the antenna driver pins TX1 and TX2 for modulation
 */
#define MFRC522_REG_GS_N 0x27
/* Defines the conductance of the p-driver output during periods of no
 * modulation */
#define MFRC522_REG_CWGS_PREG 0x28
/* Defines the conductance of the p-driver output during periods of modulation
 */
#define MFRC522_REG__MODGS_PREG 0x29
/* Defines settings for the internal timer */
#define MFRC522_REG_T_MODE 0x2A
/* Defines settings for the internal timer */
#define MFRC522_REG_T_PRESCALER 0x2B
/* Defines the 16-bit timer reload value (MSB) */
#define MFRC522_REG_T_RELOAD_H 0x2C
/* Defines the 16-bit timer reload value (MSB) */
#define MFRC522_REG_T_RELOAD_L 0x2D
/* Shows the 16-bit timer value (MSB) */
#define MFRC522_REG_T_COUNTER_VALUE_H 0x2E
/* Shows the 16-bit timer value (LSB) */
#define MFRC522_REG_T_COUNTER_VALUE_L 0x2F

// Page 3:TestRegister
/* Reserved */
#define MFRC522_REG_RESERVED30 0x30
/* General test signal configuration */
#define MFRC522_REG_TEST_SEL1 0x31
/* General test signal configuration and PRBS control */
#define MFRC522_REG_TEST_SEL2 0x32
/* Enables pin output driver on pins D1 to D7 */
#define MFRC522_REG_TEST_PIN_EN 0x33
/* Defines the values for D1 to D7 when it is used as an I/O bus */
#define MFRC522_REG_TEST_PIN_VALUE 0x34
/* Shows the status of the internal test bus */
#define MFRC522_REG_TEST_BUS 0x35

/* Controls the digital self test */
#define MFRC522_REG_AUTO_TEST 0x36
/* internal signal processing in the receiver chain is performed non-linearly which increases the operating distance in communication modes at 106 kBd */
#define REG_AUTO_TEST_AMP_RCV_POS 6
#define REG_AUTO_TEST_AMP_RCV_MASK (1 << REG_AUTO_TEST_AMP_RCV_POS)
/* Enables digital self test. Enabled by value 1001b == 0x09. */
#define REG_AUTO_TEST_SELF_TEST_POS 0
#define REG_AUTO_TEST_SELF_TEST_MASK (0x0F << REG_AUTO_TEST_SELF_TEST_POS)

/* Shows the software version */
#define MFRC522_REG_VERSION 0x37
/* Controls the pins AUX1 and AUX2 */
#define MFRC522_REG_ANALOG_TEST 0x38
/* Defines the test value for TestDAC1 */
#define MFRC522_REG_TEST_DAC1 0x39
/* Defines the test value fot TestDAC2 */
#define MFRC522_REG_TEST_DAC2 0x3A
/* Shows the value of ADC I and Q channels */
#define MFRC522_REG_TEST_ADC 0x3B
/* Reserved */
#define MFRC522_REG_RESERVED31 0x3C
/* Reserved */
#define MFRC522_REG_RESERVED32 0x3D
/* Reserved */
#define MFRC522_REG_RESERVED33 0x3E
/* Reserved */
#define MFRC522_REG_RESERVED34 0x3F

#endif
