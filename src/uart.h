// Commander X16 Emulator
// All rights reserved. License: 2-clause BSD

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	w6551   = 0,
	pc16550 = 1,
	sc16752 = 2,
} uart_type_t;

// We store all of the flags separately
typedef struct {
	uart_type_t type;
	int         fd;

	uint32_t clock_hz;          // whatever you want
	uint32_t baud_rate;         // computed rate
	uint16_t baud_rate_divisor; // plenty large enough

	uint8_t rx_sr, tx_sr; // incoming/outgoing shift registers
	bool    rx_sr_has_value, tx_sr_has_value;
	bool    overrun; // received a value when rx_sr had one already

	uint32_t ns_accumulated;
	uint8_t  total_word_size; // data bits + stop bits + parity bits
} uart_t;

#define IER_ERBFI 0x1 // RX
#define IER_ETBEI 0x2 // TX
#define IER_ELSI 0x4  // Line status
#define IER_EDSSI 0x8 // Modem status
#define IER_MASK 0xf

#define LCR_WLS 0x3    // Word Length Select
#define LCR_STB 0x4    // Stop bits
#define LCR_PEN 0x8    // Parity
#define LCR_EPS 0x10   // Even parity
#define LCR_STICK 0x20 // Stick parity
#define LCR_BREAK 0x40 // Set break
#define LCR_DLAB 0x80  // Divisor latch
#define LCR_MASK 0xff

#define MCR_DTR 0x1   // DTR
#define MCR_RTS 0x2   // RTS
#define MCR_OUT1 0x4  // Out1
#define MCR_OUT2 0x8  // Out2
#define MCR_LOOP 0x10 // Loopback
#define MCR_MASK 0x1f

#define LSR_DR 0x1      // Data ready
#define LSR_OE 0x2      // Overrun error
#define LSR_PE 0x4      // Parity error
#define LSR_FE 0x8      // Framing error
#define LSR_BI 0x10     // Break interrupt
#define LSR_THRE 0x20   // Transmitter holding
#define LSR_TEMT 0x40   // Transmitter empty
#define LSR_RXFIFO 0x80 // Error in RCVR FIFO
#define LSR_MASK 0x00

#define MSR_DCTS 0x1 // delta CTS
#define MSR_DDSR 0x2 // delta DSR
#define MSR_TERI 0x4 // trailing edge RI
#define MSR_DDCD 0x8 // delta DCD
#define MSR_CTS 0x10 // CTS
#define MSR_DSR 0x20 // DSR
#define MSR_RI 0x40  // RI
#define MSR_DCD 0x80 // DCD
#define MSR_MASK 0x00

typedef struct {
	uart_t base;

	// rx & tx interact with the buffer
	uint8_t ier; // interrupt enable
	// interrupt identity & fifo control
	// interact with fancier things
	uint8_t lcr; // line control
	uint8_t mcr; // modem control
	uint8_t lsr; // line status
	uint8_t msr; // modem status
	uint8_t scr; // scratch register
	// divisor latch is stored in baud_rate_divisor

	// buffers
        // bytes enter the buffer at [buflevel] and exit at [0]
	bool    fifo_enabled;
	uint8_t rcvr_fifo_trigger_level;
	uint8_t txbufsize;
	uint8_t rxbufsize;
        uint8_t txbuflevel;
        uint8_t rxbuflevel;
	uint8_t txbuffer[64];
	uint8_t rxbuffer[64];
} uart_16550_t;

// any defines for 6551s

#define CR_BAUD 0x0f
#define CR_RXCLK 0x10
#define CR_WL 0x60
#define CR_STOP 0x80

#define CMD_DTR 0x01
#define CMD_RXINT 0x02
#define CMD_TXCTRL 0x0c
#define CMD_ECHO 0x10
#define CMD_PARITY 0xe0

#define STATUS_PE 0x01
#define STATUS_FE 0x02
#define STATUS_OE 0x04  // Overrun
#define STATUS_RXF 0x08 // RX Full
#define STATUS_TXE 0x10 // TX Empty
#define STATUS_DCD 0x20
#define STATUS_DSR 0x40
#define STATUS_INT 0x80 // Interrupt has occurred

typedef struct {
	uart_t base;

	uint8_t control;
	uint8_t command;
	uint8_t status;
} uart_6551_t;

extern uart_t *uart;

uart_t *make_uart(int fd, uart_type_t);
uint8_t uart_read(uart_t *, uint8_t reg, bool debug);
void    uart_write(uart_t *, uint8_t reg, uint8_t value);
void    uart_step(uart_t *, uint8_t MHZ, unsigned clocks);
bool    uart_irq(uart_t *);

#endif
