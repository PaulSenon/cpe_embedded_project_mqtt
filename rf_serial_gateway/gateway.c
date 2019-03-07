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

#define NET_ID 105
#define ACK "ACK"
#define SERIAL_EOL "\n\r"

#define MODULE_VERSION	0x03
#define MODULE_NAME "RF Sub1G - USB"

#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif

#define DEBUG 1
#define RF_BUFF_LEN  64
#define META_LEN 5
#define DATA_LEN (RF_BUFF_LEN)-(META_LEN)

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

/**
 * PacketId generator, for the UDPverified packet
 */
uint8_t packet_id = 0;
uint8_t getNewPacketId(){
    if(packet_id == 255){
        packet_id = 0;
    }else{
        packet_id ++;
    }
    return packet_id;
}

int isValidNetId(uint8_t id){
	if(id == (NET_ID)){
        return 1;
    } else {
        return 0;
    }
}

int isAck(char* message){
    int i;
	char* ack = ACK;
    // if too much data => false
    if(sizeof(message) > sizeof(ack)){
        return 0;
    }
    // if not start by "ACK" => false
	for(i=0; i<sizeof(ack); i++){
		if(message[i] != ack[i]){
			return 0;
		}
	}
	return 1;
}

uint8_t chenillard_active = 1;
void handle_rf_rx_data(void)
{
	uint8_t data[RF_BUFF_LEN];
	int8_t ret = 0;
	uint8_t status = 0;

	/* Check for received packet (and get it if any) */
	ret = cc1101_receive_packet(data, RF_BUFF_LEN, &status);
	/* Go back to RX mode */
	cc1101_enter_rx_mode();

    // RF packet look like this
    //      0       1      2      3     4     5 ... 63
    // [ length | @src | @dest | id | netID | data ... ]

    if(status != 0 || isValidNetId(data[4]) == 0){
        #if DEBUG
            uprintf(UART0, "Something invalid received...%s", (SERIAL_EOL));
        #endif
        // skip this packet if it does not belong to us (or if invalid)
        return;
    }

    if(isAck(data+5)){
        #if DEBUG
            uprintf(UART0, "ACK received for data packet n°%d%s", data[3], (SERIAL_EOL));
        #endif
        // remove_packet_from_ack_list(data[3]);
    }else{
        #if DEBUG
            uprintf(UART0, "DATA received (packet n°%d)%s", data[3], (SERIAL_EOL));
        #endif
        // Serial packet look like this
        //     0     1  ...  ?
        // [ @src | data .... ]

        // send on UART
        uprintf(UART0, "%c%s%s", data[1], data+4, (SERIAL_EOL));

        // send ACK back to @src
        send_ack(data[3], data[1]);
    }
}

static volatile uint32_t cc_tx = 0;
static volatile uint32_t update_display = 0;
static volatile uint8_t cc_tx_buff[RF_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;

// void sendResetScreenConfig_DEBUG(uint32_t gpio){
// 	char message[50];
// 	int len = 50;
// 	snprintf ( message, 50, "%s:LHT",SALT);
// 	memcpy((char*)cc_tx_buff, message, len);
// 	cc_ptr = len;
//     cc_tx=1;
// }

void send_ack(uint8_t packetId, uint8_t dest)
{
    // Creat our custom ACK packet
    //      0       1      2      3     4      5 6 7
    // [ length | @src | @dest | id | netID | "ACK" ]
    uint8_t packet[RF_BUFF_LEN];

    // set the one byte fields
    packet[0] = sizeof((ACK)) + (META_LEN); // 3 + 5
    packet[1] = (DEVICE_ADDRESS); // @src
    packet[2] = dest; // @dest
    packet[3] = packetId; // id
    packet[4] = (NET_ID); // netId

    // set the "ACK" message
    ackMessage = (ACK);
    memcpy((char*)&(packet[5]), (char*)ackMessage, sizeof(ackMessage));


    // Now we can do the stuff required by cc1101 to send some data :
    uint8_t cc_tx_data[RF_BUFF_LEN + 2];
	uint8_t tx_len = packet[0];
	int ret = 0;

	/* Create a local copy */
	memcpy((char*)&(cc_tx_data[2]), (char*)packet, tx_len);

	/* Prepare buffer for sending */
	cc_tx_data[0] = tx_len + 1;
	cc_tx_data[1] = dest; 

	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	ret = cc1101_send_packet(cc_tx_data, (tx_len + 2));
}

void send_uart_packet_on_rf(void)
{
	uint8_t serial_packet = [RF_BUFF_LEN];
    uint8_t tx_len = cc_ptr;

    /* Create a local copy */
	memcpy((char*)&(serial_packet[0]), (char*)cc_tx_buff, tx_len);
	/* "Free" the rx buffer as soon as possible */
	cc_ptr = 0;

    // first create our RF Data packet :
    //      0       1      2      3     4     5 ... 63
    // [ length | @src | @dest | id | netID | data ... ]
    // 
    // from the Serial packet received : (cc_tx_data from [2])
    //     0     1  ...  ?
    // [ @dest | data .... ]

	uint8_t packet = [RF_BUFF_LEN];

    // set the one byte fields
    packet[0] = tx_len + (META_LEN); // 3 + 5 // NOTE: not sure of tx_len, it might be ±1
    packet[1] = (DEVICE_ADDRESS); // @src
    packet[2] = serial_packet[0]; // @dest
    packet[3] = getNewPacketId(); // id
    packet[4] = (NET_ID); // netId

    // then set the data
    memcpy((char*)&(packet[5]), (char*)(serial_packet+1), packet[0]);


    // Now we can do the stuff required by cc1101 to send some data :
    uint8_t cc_tx_data[RF_BUFF_LEN + 2];
	tx_len = packet[0];
	int ret = 0;

	/* Create a local copy */
	memcpy((char*)&(cc_tx_data[2]), (char*)cc_tx_buff, tx_len);

	/* Prepare buffer for sending */
	cc_tx_data[0] = tx_len + 1;
	cc_tx_data[1] = serial_packet[0];

	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	ret = cc1101_send_packet(cc_tx_data, (tx_len + 2));
}


void handle_uart_cmd(uint8_t c)
{
	if (cc_ptr < RF_BUFF_LEN) {
		cc_tx_buff[cc_ptr++] = c;
	} else {
		cc_ptr = 0;
	}
	if ((c == '\n') || (c == '\r')) {
		cc_tx = 1;
	}
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
	set_gpio_callback(sendResetScreenConfig_DEBUG, &button, EDGE_RISING);

#if DEBUG
	uprintf(UART0, "App started%s", (SERIAL_EOL));
#endif

	while (1) {
		uint8_t status = 0;

		/* Tell we are alive :) */
		chenillard(250);
        
        // When a serial packet is received, we send it on rf
		if (cc_tx == 1) {
			send_uart_packet_on_rf();
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