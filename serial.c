#include	"serial.h"

/** \file
	\brief Serial subsystem

	Teacup's serial subsystem is a powerful, thoroughly tested and highly modular serial management system.

	It uses ringbuffers for both transmit and receive, and intelligently decides whether to wait or drop transmitted characters if the buffer is full.

	It also supports XON/XOFF flow control of the receive buffer, to help avoid overruns.
*/

#include	<avr/interrupt.h>
#include	"memory_barrier.h"

#include	"common.h"
#include	"arduino.h"

/// size of TX and RX buffers. MUST be a \f$2^n\f$ value
#define		BUFSIZE			64

/// ascii XOFF character
#define		ASCII_XOFF	19
/// ascii XON character
#define		ASCII_XON		17

/// rx buffer head pointer. Points to next available space.
volatile uint8_t rxhead = 0;
/// rx buffer tail pointer. Points to last character in buffer
volatile uint8_t rxtail = 0;
/// rx buffer
volatile uint8_t rxbuf[BUFSIZE];

/// tx buffer head pointer. Points to next available space.
volatile uint8_t txhead = 0;
/// tx buffer tail pointer. Points to last character in buffer
volatile uint8_t txtail = 0;
/// tx buffer
volatile uint8_t txbuf[BUFSIZE];

/// check if we can read from this buffer
#define	buf_canread(buffer)			((buffer ## head - buffer ## tail    ) & (BUFSIZE - 1))
/// read from buffer
#define	buf_pop(buffer, data)		do { data = buffer ## buf[buffer ## tail]; buffer ## tail = (buffer ## tail + 1) & (BUFSIZE - 1); } while (0)

/// check if we can write to this buffer
#define	buf_canwrite(buffer)		((buffer ## tail - buffer ## head - 1) & (BUFSIZE - 1))
/// write to buffer
#define	buf_push(buffer, data)	do { buffer ## buf[buffer ## head] = data; buffer ## head = (buffer ## head + 1) & (BUFSIZE - 1); } while (0)

/*
	ringbuffer logic:
	head = written data pointer
	tail = read data pointer

	when head == tail, buffer is empty
	when head + 1 == tail, buffer is full
	thus, number of available spaces in buffer is (tail - head) & bufsize

	can write:
	(tail - head - 1) & (BUFSIZE - 1)

	write to buffer:
	buf[head++] = data; head &= (BUFSIZE - 1);

	can read:
	(head - tail) & (BUFSIZE - 1)

	read from buffer:
	data = buf[tail++]; tail &= (BUFSIZE - 1);
*/

#ifdef	XONXOFF
#define		FLOWFLAG_STATE_XOFF	0
#define		FLOWFLAG_SEND_XON		1
#define		FLOWFLAG_SEND_XOFF	2
#define		FLOWFLAG_STATE_XON	4
// initially, send an XON
volatile uint8_t flowflags = FLOWFLAG_SEND_XON;
#endif

/// initialise serial subsystem
///
/// set up baud generator and interrupts, clear buffers
void serial_init()
{
#if BAUD > 38401
	UCSR0A = MASK(U2X0);
	UBRR0 = (((F_CPU / 8) / BAUD) - 0.5);
#else
	UCSR0A = 0;
	UBRR0 = (((F_CPU / 16) / BAUD) - 0.5);
#endif

	UCSR0B = MASK(RXEN0) | MASK(TXEN0);
	UCSR0C = MASK(UCSZ01) | MASK(UCSZ00);

	UCSR0B |= MASK(RXCIE0) | MASK(UDRIE0);
}

/*
	Interrupts
*/

/// receive interrupt
///
/// we have received a character, stuff it in the rx buffer if we can, or drop it if we can't
#ifdef	USART_RX_vect
ISR(USART_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	if (buf_canwrite(rx))
		buf_push(rx, UDR0);
	else {
		uint8_t trash;

		// not reading the character makes the interrupt logic to swamp us with retries, so better read it and throw it away
		trash = UDR0;
		(void)trash; // remove warning
	}

	#ifdef	XONXOFF
	if (flowflags & FLOWFLAG_STATE_XON && buf_canwrite(rx) <= 16) {
		// the buffer has only 16 free characters left, so send an XOFF
		// more characters might come in until the XOFF takes effect
		flowflags = FLOWFLAG_SEND_XOFF | FLOWFLAG_STATE_XON;
		// enable TX interrupt so we can send this character
		UCSR0B |= MASK(UDRIE0);
	}
	#endif

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

/// transmit buffer ready interrupt
///
/// provide the next character to transmit if we can, otherwise disable this interrupt
#ifdef	USART_UDRE_vect
ISR(USART_UDRE_vect)
#else
ISR(USART0_UDRE_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	#ifdef	XONXOFF
	if (flowflags & FLOWFLAG_SEND_XON) {
		UDR0 = ASCII_XON;
		flowflags = FLOWFLAG_STATE_XON;
	}
	else if (flowflags & FLOWFLAG_SEND_XOFF) {
		UDR0 = ASCII_XOFF;
		flowflags = FLOWFLAG_STATE_XOFF;
	}
	else
	#endif
	if (buf_canread(tx))
		buf_pop(tx, UDR0);
	else
		UCSR0B &= ~MASK(UDRIE0);

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

/*
	Read
*/

/// check how many characters can be read
uint8_t serial_rxchars()
{
	return buf_canread(rx);
}

/// read one character
uint8_t serial_popchar()
{
	uint8_t c = 0;

	// it's imperative that we check, because if the buffer is empty and we pop, we'll go through the whole buffer again
	if (buf_canread(rx))
		buf_pop(rx, c);

	#ifdef	XONXOFF
	if ((flowflags & FLOWFLAG_STATE_XON) == 0 && buf_canread(rx) <= 16) {
		// the buffer has (BUFSIZE - 16) free characters again, so send an XON
		flowflags = FLOWFLAG_SEND_XON;
		UCSR0B |= MASK(UDRIE0);
	}
	#endif

	return c;
}

/*
	Write
*/

/// send one character
void serial_writechar(uint8_t data)
{
	// check if interrupts are enabled
	if (SREG & MASK(SREG_I)) {
		// if they are, we should be ok to block since the tx buffer is emptied from an interrupt
		for (;buf_canwrite(tx) == 0;);
		buf_push(tx, data);
	}
	else {
		// interrupts are disabled- maybe we're in one?
		// anyway, instead of blocking, only write if we have room
		if (buf_canwrite(tx))
			buf_push(tx, data);
	}
	// enable TX interrupt so we can send this character
	UCSR0B |= MASK(UDRIE0);
}

/// send a whole block
void serial_writeblock(void *data, int datalen)
{
	int i;

	for (i = 0; i < datalen; i++)
		serial_writechar(((uint8_t *) data)[i]);
}

/// send a string- look for null byte instead of expecting a length
void serial_writestr(uint8_t *data)
{
	uint8_t i = 0, r;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = data[i++]))
		serial_writechar(r);
}

/**
	Write block from FLASH

	Extensions to output flash memory pointers. This prevents the data to
	become part of the .data segment instead of the .code segment. That means
	less memory is consumed for multi-character writes.

	For single character writes (i.e. '\n' instead of "\n"), using
	serial_writechar() directly is the better choice.
*/
void serial_writeblock_P(PGM_P data, int datalen)
{
	int i;

	for (i = 0; i < datalen; i++)
		serial_writechar(pgm_read_byte(&data[i]));
}

/// Write string from FLASH
void serial_writestr_P(PGM_P data)
{
	uint8_t r, i = 0;
	// yes, this is *supposed* to be assignment rather than comparison, so we break when r is assigned zero
	while ((r = pgm_read_byte(&data[i++])))
		serial_writechar(r);
}

/** \file sermsg.c
	\brief primitives for sending numbers over the serial link
*/

#include	"serial.h"

/** write a single hex digit
	\param v hex digit to write, higher nibble ignored
*/
void serwrite_hex4(uint8_t v) {
	v &= 0xF;
	if (v < 10)
		serial_writechar('0' + v);
	else
		serial_writechar('A' - 10 + v);
}

/** write a pair of hex digits
	\param v byte to write. One byte gives two hex digits
*/
void serwrite_hex8(uint8_t v) {
	serwrite_hex4(v >> 4);
	serwrite_hex4(v & 0x0F);
}

/** write four hex digits
	\param v word to write
*/
void serwrite_hex16(uint16_t v) {
	serwrite_hex8(v >> 8);
	serwrite_hex8(v & 0xFF);
}

/** write eight hex digits
	\param v long word to write
*/
void serwrite_hex32(uint32_t v) {
	serwrite_hex16(v >> 16);
	serwrite_hex16(v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const uint32_t powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned int
	\param v number to send
*/
void serwrite_uint32(uint32_t v) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
		serial_writechar(t + '0');
	}
	while (e--);
}

/** write decimal digits from a long signed int
	\param v number to send
*/
void serwrite_int32(int32_t v) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32(v);
}

/** write decimal digits from a long unsigned int
\param v number to send
*/
void serwrite_uint32_vf(uint32_t v, uint8_t fp) {
	uint8_t e, t;

	for (e = 9; e > 0; e--) {
		if (v >= powers[e])
			break;
	}

	if (e < fp)
		e = fp;

	do
	{
		for (t = 0; v >= powers[e]; v -= powers[e], t++);
		serial_writechar(t + '0');
		if (e == fp)
			serial_writechar('.');
	}
	while (e--);
}

/** write decimal digits from a long signed int
\param v number to send
*/
void serwrite_int32_vf(int32_t v, uint8_t fp) {
	if (v < 0) {
		serial_writechar('-');
		v = -v;
	}

	serwrite_uint32_vf(v, fp);
}

/** \file sersendf.c
	\brief Simplified printf implementation
*/

#include	<stdarg.h>
#include	<avr/pgmspace.h>

#include	"serial.h"

// void sersendf(char *format, ...) {
// 	va_list args;
// 	va_start(args, format);
//
// 	uint16_t i = 0;
// 	uint8_t c, j = 0;
// 	while ((c = format[i++])) {
// 		if (j) {
// 			switch(c) {
// 				case 'l':
// 					j = 4;
// 					break;
// 				case 'u':
// 					if (j == 4)
// 						serwrite_uint32(va_arg(args, uint32_t));
// 					else
// 						serwrite_uint16(va_arg(args, uint16_t));
// 					j = 0;
// 					break;
// 				case 'd':
// 					if (j == 4)
// 						serwrite_int32(va_arg(args, int32_t));
// 					else
// 						serwrite_int16(va_arg(args, int16_t));
// 					j = 0;
// 					break;
// 				case 'p':
// 				case 'x':
// 					serial_writestr_P(str_ox);
// 					if (j == 4)
// 						serwrite_hex32(va_arg(args, uint32_t));
// 					else
// 						serwrite_hex16(va_arg(args, uint16_t));
// 					j = 0;
// 					break;
// 				case 'c':
// 					serial_writechar(va_arg(args, uint16_t));
// 					j = 0;
// 					break;
// 				case 's':
// 					serial_writestr(va_arg(args, uint8_t *));
// 					j = 0;
// 					break;
// 				default:
// 					serial_writechar(c);
// 					j = 0;
// 					break;
// 			}
// 		}
// 		else {
// 			if (c == '%') {
// 				j = 2;
// 			}
// 			else {
// 				serial_writechar(c);
// 			}
// 		}
// 	}
// 	va_end(args);
// }

/** \brief Simplified printf
	\param format pointer to output format specifier string stored in FLASH.
	\param ... output data

	Implements only a tiny subset of printf's format specifiers :-

	%[ls][udcx%]

	l - following data is (32 bits)\n
	s - following data is short (8 bits)\n
	none - following data is 16 bits.

	u - unsigned int\n
	d - signed int\n
	q - signed int with decimal before the third digit from the right\n
	c - character\n
	x - hex\n
	% - send a literal % character

	Example:

	\code sersendf_P(PSTR("X:%ld Y:%ld temp:%u.%d flags:%sx Q%su/%su%c\n"), target.X, target.Y, current_temp >> 2, (current_temp & 3) * 25, dda.allflags, mb_head, mb_tail, (queue_full()?'F':(queue_empty()?'E':' '))) \endcode
*/
void sersendf_P(PGM_P format, ...) {
	va_list args;
	va_start(args, format);

	uint16_t i = 0;
	uint8_t c = 1, j = 0;
	while ((c = pgm_read_byte(&format[i++]))) {
		if (j) {
			switch(c) {
				case 's':
					j = 1;
					break;
				case 'l':
					j = 4;
					break;
				case 'u':
					if (j == 4)
						serwrite_uint32(va_arg(args, uint32_t));
					else
						serwrite_uint16(va_arg(args, uint16_t));
					j = 0;
					break;
				case 'd':
					if (j == 4)
						serwrite_int32(va_arg(args, int32_t));
					else
						serwrite_int16(va_arg(args, int16_t));
					j = 0;
					break;
				case 'c':
					serial_writechar(va_arg(args, uint16_t));
					j = 0;
					break;
				case 'x':
					serial_writestr_P(PSTR("0x"));
					if (j == 4)
						serwrite_hex32(va_arg(args, uint32_t));
					else if (j == 1)
						serwrite_hex8(va_arg(args, uint16_t));
					else
						serwrite_hex16(va_arg(args, uint16_t));
					j = 0;
					break;
/*				case 'p':
					serwrite_hex16(va_arg(args, uint16_t));*/
				case 'q':
					serwrite_int32_vf(va_arg(args, int32_t), 3);
					j = 0;
					break;
				default:
					serial_writechar(c);
					j = 0;
					break;
			}
		}
		else {
			if (c == '%') {
				j = 2;
			}
			else {
				serial_writechar(c);
			}
		}
	}
	va_end(args);
}
