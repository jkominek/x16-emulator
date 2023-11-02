// Commander X16 Emulator
// All rights reserved. License: 2-clause BSD

#include "uart.h"
#include "memory.h"
#include "serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <termios.h>

uart_t *uart = NULL;

uint8_t pc16550_read(uart_16550_t *, uint8_t, bool);
void    pc16550_write(uart_16550_t *, uint8_t, uint8_t);
uint8_t w6551_read(uart_6551_t *, uint8_t, bool);
void    w6551_write(uart_6551_t *, uint8_t, uint8_t);

// Three types of UARTs targeted
// w6551   The 6551 ACIA, represented by the W65C51S, since it
//         is still being produced, making it the easiest member
//         of the class to check behavior for, if needed.
// pc16550 An ideal 16550 with any bugs in behavior fixed
// sc16752 The NXP SC16C752B, an extension of the 16550 with a
//         larger buffer, high max speed, and other improvements.
// Why support these three? If someone wanted to make real hardware,
// the W6551 would be for the most exacting of purists, wanting the
// closest to MOS hardware they could get. The 16550 would be for
// anyone insisting on through-hole parts. And the 16752 for anyone
// who isn't worried about such things and wants the cheapest/best

void
pc16550_init(uart_16550_t *uart)
{
	uart->fifo_enabled            = 0;
	uart->rcvr_fifo_trigger_level = 1;
	// the 16550 derivatives only have a buffer if you turn it on.
	uart->txbufsize = uart->rxbufsize = 1;

	// defaults for LCR
	pc16550_write(uart, 0x7, 0x00);
}

uint8_t
pc16550_read(uart_16550_t *uart, uint8_t reg, bool debug)
{
	switch (reg) {
		case 0: {
			// receiver buffer register
			uint8_t buf;
			int     ret = read(uart->base.fd, &buf, 1);
			// we could check for errors, but, eh.
			if (ret > 0) {
				return buf;
			} else {
				return 0;
			}
		} break;
		case 1: {
			// interrupt enable register
			return uart->ier;
		} break;
		case 2: {
			// interrupt identity register
			// generate from what we know
		} break;
		case 3: {
			// line control register
			return uart->lcr;
		} break;
		case 4: {
			// modem control register
			return uart->mcr;
		} break;
		case 5: {
			// line status register
			return uart->lsr;
		} break;
		case 6: {
			// modem status register
			return uart->msr;
		} break;
		case 7: {
			// scratch register
			return uart->scr;
		} break;
	}
	return 0;
}

void
uart_update_baudrate(uart_t *uart)
{
	uart->baud_rate = uart->clock_hz / (16 * (uint32_t)(uart->baud_rate_divisor));

	// we could check for errors, but there's no way to report
	// them.
	struct termios tty;
	tcgetattr(uart->fd, &tty);

	// TODO depending on platform these won't be available and
	// we'll just have to set the nearest speed
	cfsetispeed(&tty, uart->baud_rate);
	cfsetospeed(&tty, uart->baud_rate);

	tcsetattr(uart->fd, TCSANOW, &tty);
}

void
pc16550_write(uart_16550_t *uart, uint8_t reg, uint8_t value)
{
	switch (reg) {
		case 0: {
			if (uart->lcr & LCR_DLAB) {
				// divisor latch least significant
				uart->base.baud_rate_divisor = (uart->base.baud_rate_divisor & 0xff00) | value;
				uart_update_baudrate((uart_t *)uart);
				break;
			}
			// transmitter holding register
			int ret = write(uart->base.fd, &value, 1);
			if (ret != 1) {
				// err
			}
		} break;
		case 1: {
			if (uart->lcr & LCR_DLAB) {
				// divisor latch most significant
				uart->base.baud_rate_divisor = (uart->base.baud_rate_divisor & 0x00ff) | (value << 8);
				uart_update_baudrate((uart_t *)uart);
				break;
			}
			// interrupt enable register
		} break;
		case 2: {
			// fifo control register
			bool reset_tx = 0, reset_rx = 0;
			if (value & 0x1) {
				// fifo on
				uart->fifo_enabled = 1;
				uart->txbufsize    = 64;
				uart->rxbufsize    = 64;
				reset_rx           = (value & 0x2);
				reset_tx           = (value & 0x4);
				switch ((value & 0xc0) >> 6) {
					case 0: uart->rcvr_fifo_trigger_level = 1;
					case 1: uart->rcvr_fifo_trigger_level = 4;
					case 2: uart->rcvr_fifo_trigger_level = 8;
					case 3: uart->rcvr_fifo_trigger_level = 14;
				}
			} else {
				// fifo off
				uart->fifo_enabled = 0;
				uart->txbufsize    = 1;
				uart->rxbufsize    = 1;
				// clear both buffers
				reset_rx = 1;
				reset_tx = 1;
				// force trigger level down
				uart->rcvr_fifo_trigger_level = 1;
			}
			if (reset_rx) {
			}
			if (reset_tx) {
			}
		} break;
		case 3: {
			// line control register
			uart->lcr                  = value & LCR_MASK;
			uart->base.total_word_size = 2; // one start, one stop

			struct termios tty;
			tcgetattr(uart->base.fd, &tty);
			tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);

			// even/odd is PARODD
			// stick parity is CMSPAR
			// pretty sure we can't do anything with stick parity
			if (uart->lcr & LCR_PEN) {
				tty.c_cflag |= PARENB;
				uart->base.total_word_size += 1;
			}

			if (uart->lcr & LCR_STB) {
				tty.c_cflag |= CSTOPB;
				uart->base.total_word_size += 1;
			}

			switch (uart->lcr & LCR_WLS) {
				case 0:
					tty.c_cflag |= CS5;
					uart->base.total_word_size += 5;
					break;
				case 1:
					tty.c_cflag |= CS6;
					uart->base.total_word_size += 6;
					break;
				case 2:
					tty.c_cflag |= CS7;
					uart->base.total_word_size += 7;
					break;
				default:
					tty.c_cflag |= CS8;
					uart->base.total_word_size += 8;
					break;
			}

			// TODO worry about break someday

			tcsetattr(uart->base.fd, TCSANOW, &tty);
		} break;
		case 4: {
			// modem control register
			// TODO use the values
			uart->mcr = value & MCR_MASK;
		} break;
		case 5: {
			// line status register
			// writing to this register is undefined
			uart->lsr = value & LSR_MASK;
		} break;
		case 6: {
			// modem status register
			// read-only?
		} break;
		case 7: {
			uart->scr = value;
		} break;
	}
}

bool
pc16550_irq(uart_16550_t *uart)
{
	return 0;
}

void
pc16550_io_update(uart_16550_t *uart, bool rx_io, bool tx_io)
{
	// populate tx_sr, pull from rx_sr
	// check errors, update interrupts
}

void
w6551_init(uart_6551_t *uart)
{
	w6551_write(uart, 0x2, 0);
	w6551_write(uart, 0x3, 0);
}

uint8_t
w6551_read(uart_6551_t *uart, uint8_t reg, bool debug)
{
	switch (reg) {
		case 0: {
			// read
			if (uart->base.rx_sr_has_value) {
				uart->base.rx_sr_has_value = 0;
				return uart->base.rx_sr;
			} else {
				return 0;
			}
		} break;
		case 1:
			// Status Register
			uart->status = 0;
			// Parity and framing errors will be
			// handled in w6551_io_update, if we
			// can detect them at all.
			if (uart->base.overrun) {
				uart->status |= STATUS_OE;
				uart->base.overrun = 0;
			}
			if (uart->base.rx_sr_has_value)
				uart->status |= STATUS_RXF;
			if (!uart->base.tx_sr_has_value)
				uart->status |= STATUS_TXE;
			// dcd
			// dsr
			// irq has occurred
			return uart->status;
			break;
		case 2:
			// Command Register
			return uart->command;
			break;
		case 3:
			// Control Register
			return uart->control;
		default:
			return 0;
	}
}

void
w6551_update_word_size(uart_6551_t *uart)
{
	// min 1 start 1 stop
	uart->base.total_word_size = 2;

	switch ((uart->control & CR_WL) >> 5) {
		case 0: uart->base.total_word_size += 8; break;
		case 1: uart->base.total_word_size += 7; break;
		case 2: uart->base.total_word_size += 6; break;
		case 3: uart->base.total_word_size += 5; break;
	}

	if (uart->control & CR_STOP) {
		// if parity is on, there isn't actually a second
		// stop bit
		uart->base.total_word_size += 1;
	}
}

void
w6551_write(uart_6551_t *uart, uint8_t reg, uint8_t value)
{
	switch (reg) {
		case 0: {
			// write
			uart->base.tx_sr           = value;
			uart->base.tx_sr_has_value = 1;
		} break;
		case 1:
			// Programmed Reset
			// TODO
			break;
		case 2: {
			// Command Register
			uart->command = value;

			struct termios tty;
			tcgetattr(uart->base.fd, &tty);
			tty.c_cflag &= ~(CSTOPB | CSIZE);

			tcsetattr(uart->base.fd, TCSANOW, &tty);
			w6551_update_word_size(uart);
		} break;
		case 3: {
			// Control Register
			uart->control = value;

			static uint32_t divisors[] = {
			    1, 2304, 1536, 1048, 856, 768,
			    384, 192, 96, 64, 48, 32, 24, 16, 12, 6};
			uart->base.baud_rate_divisor = divisors[uart->control & CR_BAUD];

			// we just force RXCLK to 0, since we're not going
			// to implement that.
			uart->control &= ~CR_RXCLK;

			struct termios tty;
			tcgetattr(uart->base.fd, &tty);
			tty.c_cflag &= ~(CSTOPB | CSIZE);

			// NOTE these are not in the obvious order
			// that is intentional.
			switch ((uart->control & CR_WL) >> 5) {
				case 0: tty.c_cflag |= CS8; break;
				case 1: tty.c_cflag |= CS7; break;
				case 2: tty.c_cflag |= CS6; break;
				default: tty.c_cflag |= CS5; break;
			}

			if (uart->control & CR_STOP)
				tty.c_cflag |= CSTOPB;

			tcsetattr(uart->base.fd, TCSANOW, &tty);

			w6551_update_word_size(uart);
		}
	}
}

bool
w6551_irq(uart_6551_t *uart)
{
	return uart->status & STATUS_INT;
}

void
w6551_io_update(uart_6551_t *uart, bool rx_io, bool tx_io)
{
	// TODO check for various error conditions and update status with those

	// TODO DCD & DSR need to be reflected
	//      also changes in them cause an unconditional IRQ

	// NOTE W65C51 transmit IRQ is actually busted.
	if ((((uart->command & CMD_TXCTRL) >> 2) == 1) && (!uart->base.tx_sr_has_value) && tx_io)
		uart->status |= STATUS_INT;
	if ((uart->command & CMD_RXINT) && (!uart->base.rx_sr_has_value) && rx_io)
		uart->status |= STATUS_INT;
}

uart_t *
make_uart(int fd, uart_type_t type)
{
	uart_t *uart;
	switch (type) {
		case w6551:
			uart = (uart_t *)malloc(sizeof(uart_6551_t));
			break;
		case pc16550:
		case sc16752:
			uart = (uart_t *)malloc(sizeof(uart_16550_t));
			pc16550_init((uart_16550_t *)uart);
			break;
	}
	uart->fd       = fd;
	uart->type     = type;
	uart->clock_hz = 1843200;

	uart->ns_accumulated  = 0;
	uart->total_word_size = 10; // most common, just in case

	// neither is occupied on startup.
	uart->rx_sr_has_value = 0;
	uart->tx_sr_has_value = 0;

	// we maybe want to store here all the things that we'll
	// detect from the file descriptor, like CD/DSR/CTS, as
	// we'll need to scan them at a frequency which isn't consistent
	// with accesses to the UART registers. we could even imagine
	// setting up a thread to watch them and update these values.
	uart->overrun = 0;

	// ensure file descriptor is non-blocking
	int flags = fcntl(uart->fd, F_GETFL, 0);
	fcntl(uart->fd, F_SETFL, flags | O_NONBLOCK);

	return uart;
}

bool
uart_irq(uart_t *uart)
{
	switch (uart->type) {
		case w6551: return w6551_irq((uart_6551_t *)uart);
		default: return pc16550_irq((uart_16550_t *)uart);
	}
}

uint8_t
uart_read(uart_t *uart, uint8_t reg, bool debug)
{
	switch (uart->type) {
		case w6551: return w6551_read((uart_6551_t *)uart, reg, debug);
		default: return pc16550_read((uart_16550_t *)uart, reg, debug);
	}
}

void
uart_write(uart_t *uart, uint8_t reg, uint8_t value)
{
	switch (uart->type) {
		case w6551: return w6551_write((uart_6551_t *)uart, reg, value);
		default: return pc16550_write((uart_16550_t *)uart, reg, value);
	}
}

void
uart_step(uart_t *uart, uint8_t MHZ, unsigned clocks)
{
	bool tx_io = false, rx_io = false;

	// worth trying to optimize some of these calculations?
	uint64_t ns_elapsed = clocks * 1000 / MHZ;
	uart->ns_accumulated += ns_elapsed;
	uint64_t ns_per_byte = (10000000000 * uart->total_word_size) / uart->baud_rate;
	if (uart->ns_accumulated >= ns_per_byte) {
		if (uart->tx_sr_has_value) {
			int ret = write(uart->fd, &(uart->tx_sr), 1);
			if (ret) {
				// error
			}
			uart->tx_sr_has_value = 0;
			tx_io                 = true;
		}

		// we try to read a byte regardless, and it
		// replaces whatever is in the rx shift register
		uint8_t tmp;
		int     ret = read(uart->fd, &(tmp), 1);
		if (ret == 1) {
			uart->rx_sr = tmp;
			if (uart->rx_sr_has_value)
				uart->overrun = 1;
			else
				uart->rx_sr_has_value = 1;
			rx_io = true;
		}

		uart->ns_accumulated -= ns_per_byte;
	}

	// TODO maybe check for the various CD/DSR/CTS signal
	// changes here, every millisecond of system time, or so

	// some amount of IO has occurred, so it becomes worthwhile
	// to run the the specialized handlers
	if (rx_io || tx_io) {
		switch (uart->type) {
			case w6551:
				w6551_io_update((uart_6551_t *)uart, rx_io, tx_io);
				break;
			default:
				pc16550_io_update((uart_16550_t *)uart, rx_io, tx_io);
				break;
		}
	}
}
