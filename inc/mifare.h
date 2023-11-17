#ifndef MIFARE_H
#define MIFARE_H

/* Mifare_One card command word */
/* Find the antenna area does not enter hibernation */
#define MIFARE_REQIDL 0x26
/* Find all the cards antenna area */
#define MIFARE_REQALL 0x52
/* Anti-collision */
#define MIFARE_ANTICOLL 0x93
/* Selection card */
#define MIFARE_SElECTTAG 0x93
/* Authentication key A */
#define MIFARE_AUTHENT1A 0x60
/* Authentication key B */
#define MIFARE_AUTHENT1B 0x61
/* Read Block */
#define MIFARE_READ 0x30
/* Write block */
#define MIFARE_WRITE 0xA0
/* Debit */
#define MIFARE_DECREMENT 0xC0
/* Recharge */
#define MIFARE_INCREMENT 0xC1
/* Transfer block data to the buffer */
#define MIFARE_RESTORE 0xC2
/* Save the data in the buffer */
#define MIFARE_TRANSFER 0xB0
/* Sleep */
#define MIFARE_HALT 0x50


#endif
