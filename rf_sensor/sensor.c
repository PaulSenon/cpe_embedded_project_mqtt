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
#include "drivers/i2c.h"
#include "drivers/adc.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"

#define NET_ID 105 // set this same value on every μController (range is 0-255)
#define RF_BUFF_LEN  64
#define SENSOR_ID 1 // must be different for each sensors
// mind the sensor id
#define WEATHER_DATA_TOPIC "S/1/T" // "Temperature" from "Sensor" n° "1"  
#define TEMP_RLV_MS 2000 // interval de relevé de la température (en ms)
#define SIZE_RELEVES_BUFFER 1// taille du buffer (bytes) pour les relevés des sensors
#define ACK_MSG "ACK" // ACK message pour notre pseudo TCP

#define MODULE_VERSION	0x03
#define MODULE_NAME "RF Sub1G - USB"


#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif

#define DEVICE_ADDRESS  0x34 /* Addresses 0x00 and 0xFF are broadcast */
#define GATEWAY_ADDRESS 0xFE /* Address of the associated device */

#define DEBUG 1
#define BUFF_LEN 60

#define SELECTED_FREQ  FREQ_SEL_48MHz

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
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* ADC */
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_6;
const struct pio cc1101_gdo2 = LPC_GPIO_0_7;

#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};
const struct pio temp_alert = LPC_GPIO_0_3;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;

const struct pio button = LPC_GPIO_0_12; /* ISP button */


#define ADC_VBAT  LPC_ADC(0)
#define ADC_EXT1  LPC_ADC(1)
#define ADC_EXT2  LPC_ADC(2)


void send_ack();
static uint8_t rlv_buff_size = sizeof(int);
static volatile uint8_t rlv_buffer[sizeof(int)];


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



/***************************************************************************** */
/* Temperature */
/* The Temperature Alert pin is on GPIO Port 0, pin 7 (PIO0_7) */
/* The I2C Temperature sensor is at address 0x94 */
void WAKEUP_Handler(void)
{
}

void temp_config()
{
	int ret = 0;

	/* Temp Alert */
	config_gpio(&temp_alert, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	/* FIXME : add a callback on temp_alert edge */

	/* Temp sensor */
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(UART0, "Temp config error: %d\n\r", ret);
	}
}

/******************************************************************************/
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
	/* FIXME : Add here a define protected list of settings for 915MHz configuration */
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

#ifdef DEBUG
	uprintf(UART0, "CC1101 RF link init done.\n\r");
#endif
}

int isValidNetId(uint8_t id){
	if(id == (NET_ID)){
        return 1;
    } else {
        return 0;
    }
}

uint8_t isAck(uint8_t* message){
    uint8_t i;

	char* ack = (ACK_MSG);
    // if too much data => false
    if(sizeof(message) > sizeof(ack)){
        return 0;
    }
    // if not start by "ACK" => false
	for(i=0; i<sizeof(ack); i++){
		if((char)message[i] != ack[i]){
			return 0;
		}
	}
	return 1;
}

static volatile uint32_t cc_tx = 0;
static volatile uint8_t waitForACK = 0;
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
    //      0       1       2      3     4 ...63
    // [ length | @dest | @src | netID | data ... ]
#ifdef DEBUG
    uprintf(UART0, "RF: receive packet: %s\n\r", data);
#endif

    if(status != 0 || isValidNetId(data[3]) == 0){
        #ifdef DEBUG
            uprintf(UART0, "RF: receive: Something invalid received...\n\r");
        #endif
        // skip this packet if it does not belong to us (or if invalid)
        return;
    }



    if(isAck(data+4) == 1){
        uprintf(UART0, "RF: receive: \"ACK\"\n\r");
        waitForACK = 0;
    }else{
        uprintf(UART0, "RF: receive: data: %s\n\r", data+4);
        uprintf(UART0, "RF: receive: TODO mqtt stuff...\n\r");
        	// TODO Mqtt stuff...
		send_ack();
    }

#ifdef DEBUG
	uprintf(UART0, "RF: receive: ret:%d, st: %d.\n\r", ret, status);
#endif

}

int releve_temp(){
    uint16_t val = 0;
    int deci_degrees;
    if (tmp101_sensor_read(&tmp101_sensor, &val, &deci_degrees) != 0) {
        uprintf(UART0, "Temp read error\n\r");
		return 0;
    } else {
        uprintf(UART0, "Temp read: %d,%d - raw: 0x%04x - hex: 0x%08x - full: %d - chars: %s\n\r",
                (deci_degrees/10), (deci_degrees%10), val, deci_degrees, deci_degrees, deci_degrees);
        return deci_degrees;
    }
}

void releves(){
    // on update pas les relevé si on est en attente d'ACK
    if(waitForACK == 0){
        // Tous les relevés :
		int test = releve_temp();
		memcpy((char*)&(rlv_buffer[0]), (char*)(&test), rlv_buff_size);

        // send on rf
        cc_tx = 1;
#ifdef DEBUG
	uprintf(UART0, "releve\n\r");
#endif
    }
}

static volatile uint32_t releves_flag = 0;
void turn_releves_flag_on(uint32_t gpio){
	releves_flag = 1;
}

void send_ack(){
	uint8_t cc_tx_data[(3+4)];
	uint8_t tx_len = 3;
	int ret = 0;

	/* Create a local copy */
	memcpy((char*)&(cc_tx_data[4]), (char*)("ACK"), tx_len);

	// RF packet look like this
    //      0       1       2      3     4 ...63
    // [ length | @dest | @src | netID | data ... ]

	/* Prepare buffer for sending */
	cc_tx_data[0] = tx_len;
	cc_tx_data[1] = (GATEWAY_ADDRESS);
	cc_tx_data[2] = (DEVICE_ADDRESS);
	cc_tx_data[3] = (NET_ID);

	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	ret = cc1101_send_packet(cc_tx_data, (tx_len + 4));

#ifdef DEBUG
	uprintf(UART0, "RF: send: Tx ret: %d\n\r", ret);
#endif
}

void send_on_rf(void)
{
	uint8_t cc_tx_data[rlv_buff_size + 4];
	uint8_t tx_len = (rlv_buff_size);
	int ret = 0;

	/* Create a local copy */
	memcpy((char*)&(cc_tx_data[4]), (char*)rlv_buffer, tx_len);

	// RF packet look like this
    //      0       1       2      3     4 ...63
    // [ length | @dest | @src | netID | data ... ]

	/* Prepare buffer for sending */
	cc_tx_data[0] = tx_len;
	cc_tx_data[1] = (GATEWAY_ADDRESS);
	cc_tx_data[2] = (DEVICE_ADDRESS);
	cc_tx_data[3] = (NET_ID);

#ifdef DEBUG
    uprintf(UART0, "RF: send: packet: %d | Ox%02x | Ox%02x | Ox%02x | Ox", cc_tx_data[0], cc_tx_data[1], cc_tx_data[2], cc_tx_data[3]);
	int i;
	for(i = tx_len-1; i>=0; i++){
		uprintf(UART0, "%02x", cc_tx_data[4+i]);
	}
	uprintf(UART0, "\r\n");
#endif

	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}
	ret = cc1101_send_packet(cc_tx_data, (tx_len + 4));

    // wait for ACK
    waitForACK = 1;

#ifdef DEBUG
	uprintf(UART0, "RF: send: %d bytes sent\n\r", ret);
#endif
}

static volatile uint32_t wait_for_ack_flag_gpio = 0;
void toggle_toggle_wait_for_ack_flag(uint32_t gpio){
	wait_for_ack_flag_gpio = 1;
}
void toggle_wait_for_ack_DEBUG(){
    if(waitForACK == 1){
        waitForACK = 0;
    }else{
        waitForACK = 1;
    }
    uprintf(UART0, "waitforACK: %d\n\r", waitForACK);
}


/***************************************************************************** */
int main(void)
{
	system_init();
	uart_on(UART0, 115200, NULL);
	uprintf(UART0, "Hello\n\r");
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	adc_on(NULL);
	status_led_config(&status_led_green, &status_led_red);

	/* Radio */
	rf_config();

	/* Temperature sensor */
	temp_config();

    add_systick_callback(turn_releves_flag_on, (TEMP_RLV_MS));

    set_gpio_callback(toggle_toggle_wait_for_ack_flag, &button, EDGE_RISING);

	while (1) {
		uint8_t status = 0;
		/* Request a Temp conversion on I2C TMP101 temperature sensor */
		tmp101_sensor_start_conversion(&tmp101_sensor); /* A conversion takes about 40ms */
		/* Start an ADC conversion to get battery voltage */
		adc_start_convertion_once(ADC_VBAT, LPC_ADC_SEQ(0), 0);

		/* Tell we are alive :) */
		chenillard(250);

		/* DEBUG */
		if (wait_for_ack_flag_gpio == 1) {
			toggle_wait_for_ack_DEBUG();
			wait_for_ack_flag_gpio = 0;
		}

		/* get sensors values */
		if (releves_flag == 1) {
			releves();
			releves_flag = 0;
		}

		/* RF */
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




