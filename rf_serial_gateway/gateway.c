/****************************************************************************
 *   apps/rf_sub1G/simple/main.c
 *
 * sub1G_module support code - USB version
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *************************************************************************** */

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"

#define NET_ID 105 // set this same value on every sensors μController (range is 0-255)
#define SERIAL_EOL "\n\r"
#define RF_BUFF_LEN  64
#define UART_HEADER_SIZE 3

#define MODULE_VERSION	0x03
#define MODULE_NAME "RF Sub1G - USB"

#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif

#define DEBUG 1

#define SELECTED_FREQ  FREQ_SEL_48MHz
#define DEVICE_ADDRESS  0xFE /* Addresses 0x00 and 0xFF are broadcast */
/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_6;
const struct pio cc1101_gdo2 = LPC_GPIO_0_7;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;

const struct pio button = LPC_GPIO_0_12; /* ISP button */
 
/***************************************************************************** */
static volatile uint8_t uart_decode_buffer = 0; // bool for handle_uart_cmd()
static volatile uint8_t uart_process_payload = 0; // bool for handle_uart_cmd()
static volatile uint8_t uart_remaining_payload_size = 0; // int for handle_uart_cmd()
static volatile uint8_t uart_sensor_address = 0x00; // set by handle_uart_cmd(), read by send_on_rf()
void uart_reset_handle(); // reset these 4 var + cc_ptr ^^^^^^^


/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART0, name);
	while (1);
}

static volatile int check_rx = 0;
void rf_rx_calback(uint32_t gpio)
{
	check_rx = 1;
}

static uint8_t rf_specific_settings[] = {
	CC1101_REGS(gdo_config[2]), 0x07, /* GDO_0 - Assert on CRC OK | Disable temp sensor */
	CC1101_REGS(gdo_config[0]), 0x2E, /* GDO_2 - FIXME : do something usefull with it for tests */
	CC1101_REGS(pkt_ctrl[0]), 0x0F, /* Accept all sync, CRC err auto flush, Append, Addr check and Bcast */
#if (RF_915MHz == 1)
	/* FIXME : Add here a define protected list of seSALTttings for 915MHz configuration */
#endif
};

/* RF config */
void rf_config(void)
{
	config_gpio(&cc1101_gdo0, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	cc1101_init(0, &cc1101_cs_pin, &cc1101_miso_pin); /* ssp_num, cs_pin, miso_pin */
	/* Set default config */
	cc1101_config();
	/* And change application specific settings */
	cc1101_update_config(rf_specific_settings, sizeof(rf_specific_settings));
	set_gpio_callback(rf_rx_calback, &cc1101_gdo0, EDGE_RISING);
    cc1101_set_address(DEVICE_ADDRESS);
#if DEBUG
	uprintf(UART0, "CC1101 RF link init done.%s", (SERIAL_EOL));
#endif
}

int isValidNetId(uint8_t id){
	if(id == (NET_ID)){
        return 1;
    } else {
        return 0;
    }
}

uint8_t chenillard_active = 1;
void handle_rf_rx_data(void)
{
	uint8_t data[RF_BUFF_LEN];
	// int8_t ret = 0;
	uint8_t status = 0;

	/* Check for received packet (and get it if any) */
	cc1101_receive_packet(data, RF_BUFF_LEN, &status);
	/* Go back to RX mode */
	cc1101_enter_rx_mode();

    // RF packet look like this
    //      0       1       2      3     4 ...63
    // [ length | @dest | @src | netID | data ... ]

    if(status != 0 || isValidNetId(data[3]) == 0){
        #if DEBUG
            uprintf(UART0, "Something invalid received...%s", (SERIAL_EOL));
    		uprintf(UART0, "RF: send: packet: %d|Ox%02x|Ox%02x|Ox%02x|%s%s\n\r", data[0], data[1], data[2], data[3],  data+4, (SERIAL_EOL));
        #endif
        // skip this packet if it does not belong to us (or if invalid)
        return;
    }

	#if DEBUG
		uprintf(UART0, "DATA received %s", (SERIAL_EOL));
	#endif

	// Serial packet look like this
	//     0      1       2        3      4 ... 63
	// [ "#" | length | @src | checksum | data ... ]
	// send on UART
	uint8_t checksum = data[0] + data[2];
	uprintf(UART0, "#%d%d%d%s", data[0], data[2], checksum, data+4);
}

static volatile uint32_t cc_tx = 0;
static volatile uint32_t update_display = 0;
static volatile uint8_t cc_tx_buff[RF_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;

void send_on_rf(void)
{
	uint8_t serial_packet[RF_BUFF_LEN]; // to copy uart buffer 
	uint8_t senso_address = uart_sensor_address; // copy destination
	uint8_t tx_len = cc_ptr;
	// int ret = 0;

	/* Create a local copy */
	memcpy((char*)&(serial_packet[4]), (char*)cc_tx_buff, tx_len);
	/* "Free" the rx buffer as soon as possible */
	uart_reset_handle();

	// RF packet look like this
    //      0       1       2      3     4 ...63
    // [ length | @dest | @src | netID | data ... ]

	/* Prepare buffer for sending */
	serial_packet[0] = tx_len;
	serial_packet[1] = senso_address;
	serial_packet[2] = (DEVICE_ADDRESS);
	serial_packet[3] = (NET_ID);

	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	cc1101_send_packet(serial_packet, (tx_len + 4));
}

void handle_uart_cmd(uint8_t c)
{	
	if(uart_decode_buffer == 1){
		// copy byte in buffer
		cc_tx_buff[cc_ptr] = c;
		// increment buffer pointer
		cc_ptr++;

		if(uart_process_payload == 1){
			uart_remaining_payload_size--;
			if(uart_remaining_payload_size <= 0){
				// on send et on reset quand tout le payload est dans le buffer
					// NOP it's done by send_on_rf()
						// uart_decode_buffer = 0;
						// uart_process_payload = 0;
						// cc_ptr = 0; // "empty" buffer 
				cc_tx = 1; // ready to send on rf
			}
		}else{
			// when header is in buffer, we can process it
			if(cc_ptr >= (UART_HEADER_SIZE)){
				// header looks like this :
				//      0       1        2
				// [ length | @src | checksum ]
				if((cc_tx_buff[0]+cc_tx_buff[1]) == cc_tx_buff[2]){
					// checksum is valid
					// so we can process the payload
					uart_remaining_payload_size = cc_tx_buff[0]; // save the payload size
					uart_sensor_address = cc_tx_buff[1]; // save the sensor address
					uart_process_payload = 1;
					cc_ptr = 0; // "erase" buffer, we dont need it anymore
				}else{
					// checksum is NOT valid
					// reset
					uart_reset_handle();
				}
			}
		}

	}else{
		// detect new uart packet
		if(c == '#'){
			uart_decode_buffer = 1;
		}
	}
}

void uart_reset_handle(){
	uart_remaining_payload_size = 0;
	uart_sensor_address = 0x00;
	uart_decode_buffer = 0;
	uart_process_payload = 0;
	cc_ptr = 0;
}

int main(void)
{
	system_init();
	uart_on(UART0, 115200, handle_uart_cmd);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	status_led_config(&status_led_green, &status_led_red);

	/* Radio */
	rf_config();

    /* Activate the chenillard on Rising edge (button release) */
	// set_gpio_callback(sendResetScreenConfig_DEBUG, &button, EDGE_RISING);

#if DEBUG
	uprintf(UART0, "App started%s", (SERIAL_EOL));
#endif

	while (1) {
		uint8_t status = 0;

		/* Tell we are alive :) */
		chenillard(250);
        
        // When a serial packet is received, we send it on rf
		if (cc_tx == 1) {
			send_on_rf();
			cc_tx = 0;
		}

		/* Do not leave radio in an unknown or unwated state */
		do {
			status = (cc1101_read_status() & CC1101_STATE_MASK);
		} while (status == CC1101_STATE_TX);

		if (status != CC1101_STATE_RX) {
			static uint8_t loop = 0;
			loop++;
			if (loop > 10) {
				if (cc1101_rx_fifo_state() != 0) {
					cc1101_flush_rx_fifo();
				}
				cc1101_enter_rx_mode();
				loop = 0;
			}
		}

		if (check_rx == 1) {
			check_rx = 0;
			handle_rf_rx_data();
		}

		
	}
	return 0;
}