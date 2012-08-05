#ifndef	_SERIAL_H
#define	_SERIAL_H

#include	<stdint.h>
#include	<avr/io.h>
#include	<avr/pgmspace.h>

// initialise serial subsystem
void serial_init(void);

// return number of characters in the receive buffer, and number of spaces in the send buffer
uint8_t serial_rxchars(void);
// uint8_t serial_txchars(void);

// read one character
uint8_t serial_popchar(void);
// send one character
void serial_writechar(uint8_t data);

// read/write many characters
// uint8_t serial_recvblock(uint8_t *block, int blocksize);
void serial_writeblock(void *data, int datalen);

void serial_writestr(uint8_t *data);

// write from flash
void serial_writeblock_P(PGM_P data, int datalen);
void serial_writestr_P(PGM_P data);

#endif	/* _SERIAL_H */
#ifndef	_SERMSG_H
#define	_SERMSG_H

#include	<stdint.h>

// functions for sending hexadecimal
void serwrite_hex4(uint8_t v);
void serwrite_hex8(uint8_t v);
void serwrite_hex16(uint16_t v);
void serwrite_hex32(uint32_t v);

// functions for sending decimal
#define	serwrite_uint8(v)		serwrite_uint32(v)
#define	serwrite_int8(v)		serwrite_int32(v)
#define	serwrite_uint16(v)	serwrite_uint32(v)
#define	serwrite_int16(v)		serwrite_int32(v)

void serwrite_uint32(uint32_t v);
void serwrite_int32(int32_t v);

void serwrite_uint32_vf(uint32_t v, uint8_t fp);
void serwrite_int32_vf(int32_t v, uint8_t fp);

#endif	/* _SERMSG_H */
#ifndef	_SERSENDF_H
#define	_SERSENDF_H

#include	<avr/pgmspace.h>

void sersendf(char *format, ...)		__attribute__ ((format (printf, 1, 2)));
void sersendf_P(PGM_P format, ...)	__attribute__ ((format (printf, 1, 2)));

#endif	/* _SERSENDF_H */
