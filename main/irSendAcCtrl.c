/* RMT receive example, displaying raw RMT data
 * 
 * This code was adapted from example: rmt_nec_tx_rx
 * That software is distributed under Public Domain (or CC0 licensed, at your option.)
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"

// FreeRTOS function
#define INCLUDE_vTaskDelay 1

#define rrmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value */
static const char* LOG_TAG = "TEST_LOG";

// LED on Huzzah32 board
const int LED_BUILTIN = 13;
#define DataLength 228    						//Length of powerOn (or powerOff, It has the same size)
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  26     /*!< GPIO number for receiver */
#define RMT_CLK_DIV      80    /*!< RMT counter clock divider */
const rmt_channel_t TX_CHANNEL = 2;
const gpio_num_t IR_PIN = GPIO_NUM_27;			//Using GPIO 5 of ESP32
const int  RMT_TX_CARRIER_EN = 1;   
// structure used to initialize RMT inputs
// NOTE: tag is used on the monitor outputs to distinguish between channels

// initialize RMT receive channels
void rx_channels_init() {

    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = 50000;
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
	
}

// initialize visible LED on ESP32 board
/*static void visible_led_init() {
    gpio_pad_select_gpio(LED_BUILTIN);
	gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);
}*/

/* Converts the RMT level, duration into a positive or negative integer
 * Compatible with the ESP32-RMT-server application
 * Note: most IR receivers have active-low outputs, where the
 *   ESP32-RMT-server application has active-high oututs
 * This function inverts the RMT receive level so the text output is
 *   compatible with ESP32-RMT-server application
 */
int dur( uint32_t level, uint32_t duration ) {
	if ( level == 0 ) { return duration; }
	else { return 1.0 * duration; }
}

// RMT receiver task
static void rmt_example_nec_rx_task() {
	size_t num_channels = 1; // sizeof(rx_inputs) / sizeof( rx_inputs[0] );
	size_t c, i;
    size_t rx_size = 0;
    rmt_item32_t* items = NULL;
	
    // define ringbuffer handle
    RingbufHandle_t rb;
	
    // start receiving IR data
    for ( c=0; c<num_channels; c++ ) {
		rmt_rx_start(0, 1);
	}
    
    // loop forever
    while (1) {
		// check each receive channel
		for ( c=0; c<num_channels; c++ ) {
			// get the ring buffer handle
			rmt_get_ringbuf_handle(0, &rb);
			
			// get items, if there are any
			items = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 10);
			if(items) {
				// turn on visible led
				gpio_set_level(LED_BUILTIN, 1);
				
				// print the RMT received durations to the monitor
				printf( "  %s received %i items\n", "ch_0", rx_size/4 );
				for ( i=0; i<rx_size/4; i++ ) {
					if ( i>0 ) { printf(","); }
					printf( "%i", dur( items[i].level0, items[i].duration0 ) );
					printf(",%i", dur( items[i].level1, items[i].duration1 ) );
				}
				printf("\n");
				
				// turn off visible led
				gpio_set_level(LED_BUILTIN, 0);
				
				// free up data space
				vRingbufferReturnItem(rb, (void*) items);
			}
		}
		// delay 100 milliseconds. No need to overheat the processor
		vTaskDelay( 100 / portTICK_PERIOD_MS );
	}
}

uint16_t powerOn[]= 
{  3125,9887,457,1636,482,560,487,563,483,560,
483,1603,482,552,483,551,484,559,484,558,483,553,
486,564,483,559,483,560,484,1602,482,552,483,551,
484,1625,460,559,484,551,484,1594,483,551,483,1609,461,
552,483,558,483,561,483,558,484,551,484,1625,460,0
};

void setup_rmt_config() 
{
	//put your setup code here, to run once:
	rmt_config_t rmtConfig;

	rmtConfig.rmt_mode = 0;  								//transmit mode
	rmtConfig.channel = TX_CHANNEL;  								//channel to use 0 - 7
	rmtConfig.clk_div = 80;  										//clock divider 1 - 255. source clock is 80MHz -> 80MHz/80 = 1MHz -> 1 tick = 1 us
	rmtConfig.gpio_num = IR_PIN; 									//pin to use
	rmtConfig.mem_block_num = 1; 									//memory block size

	rmtConfig.tx_config.loop_en = 0; 								//no loop
	rmtConfig.tx_config.carrier_freq_hz = 36000;  					//36kHz carrier frequency
	rmtConfig.tx_config.carrier_duty_percent = 33; 					//duty
	rmtConfig.tx_config.carrier_level =  RMT_CARRIER_LEVEL_HIGH; 	//carrier level
	rmtConfig.tx_config.carrier_en =RMT_TX_CARRIER_EN ;  			//carrier enable
	rmtConfig.tx_config.idle_level =  RMT_IDLE_LEVEL_LOW ; 			//signal level at idle
	rmtConfig.tx_config.idle_output_en = true;  					//output if idle

	rmt_config(&rmtConfig);
	rmt_driver_install(rmtConfig.channel, 0, 0);
	

}
static void rmtAcCtrlTask()
{
    vTaskDelay(10);
    setup_rmt_config();
    esp_log_level_set(LOG_TAG, ESP_LOG_INFO);

	const int sendDataLength = DataLength/2;        //sendDataLength is the range of the array powerOn and powerOff It is divided by two
													//because each index of object sendDataOn holds High and Low values and the powerOn/powerOff
													//array uses an index for High and other for Low
													
	rmt_item32_t sendDataOn[sendDataLength]; 		//Data to send the Ac turn On 

	for(int i = 0, t = 0; i < (sendDataLength*2)-1; i += 2, t++)
	{
								//Odd bit High
		sendDataOn[t].duration0 = powerOn[i];		//The patern is odd bits to High on IR LED and even bits to Low  
		sendDataOn[t].level0 = 1;					//looking at the powerOn and Off array index. So this is mapped on 
								//Even bit Low		//sendDataOn for every index
		sendDataOn[t].duration1 = powerOn[i+1];
		sendDataOn[t].level1 = 0;
	}
	
	/*rmt_item32_t sendDataOff[sendDataLength]; 		//Data to send the Ac turn Off 

	for(int i = 0, t = 0; i < (sendDataLength*2)-1; i += 2, t++)
	{
								//Odd bit High
		sendDataOff[t].duration0 = poweroff[i];
		sendDataOff[t].level0 = 1;
								//Even bit Low
		sendDataOff[t].duration1 = poweroff[i+1];
		sendDataOff[t].level1 = 0;
	}*/

    while(1) 
	{
		ESP_LOGI(LOG_TAG, "SEND RMT DATA");
		
		//Just to see it working I set this task to every 5 seconds send the AC turn On and Off
		rmt_write_items(TX_CHANNEL, sendDataOn, sendDataLength, false);
		vTaskDelay(10000 / portTICK_PERIOD_MS); //5 sec delay
		
		/*rmt_write_items(RMT_CHANNEL, sendDataOff, sendDataLength, false);
		vTaskDelay(5000 / portTICK_PERIOD_MS); //5 sec delay*/
    }
    vTaskDelete(NULL);
}
void app_main() {
	// initialize hardware
    rx_channels_init();
	setup_rmt_config();

    //visible_led_init();
	//rmt_example_nec_rx_task();
	// start receive processing task
	xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
	xTaskCreate(rmtAcCtrlTask, "SENDINNNG", 2048, NULL, 10, NULL);

	//while(true)
	//{
			//rmtAcCtrlTask();

	//}
	vTaskDelay(500);
	//rmtAcCtrlTask();
	//rmtAcCtrlTask();
}