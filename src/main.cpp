/********************************* Libraries *********************************/
#include "mbed.h"
#include "LSM6DSLSensor.h"

#if !defined(DATA_LOGGING) && !defined(NEAI_LIB)
#define DATA_LOGGING
#endif

#ifdef NEAI_LIB
#include "NanoEdgeAI.h"
#endif

/* Defines -------------------------------------------------------------------*/
#ifdef DATA_LOGGING
#define DATA_INPUT_USER 		1024
#define AXIS_NUMBER 			3
#else
#define LEARNING_NUMBER 		30 		/* Number of learning signals */
#endif

#define MINI 					5
#define THRESH					1.4
#define NOISE					0.15
#define THRESH_SIMILARITY 		90

/* Objects -------------------------------------------------------------------*/

Serial pc (USBTX, USBRX);
SPI spi(D11, D12, D13); // mosi, miso, sclk
DigitalOut cs(A3, 0);
LSM6DSLSensor *lsm6dsl = new LSM6DSLSensor(&spi, A3);
DigitalOut led(LED1);



/********************************* Prototypes *********************************/
void init(void);
#ifdef DATA_LOGGING
void data_logging_mode(void);
#endif
#ifdef NEAI_LIB
void neai_library_test_mode(void);
#endif
void get_values(void);
void fill_acc_array(void);
bool strum_trigger(void);

/* Variables -----------------------------------------------------------------*/
int32_t raw_values[AXIS_NUMBER], last_raw_values[AXIS_NUMBER] = {0};
float values[AXIS_NUMBER], data_user[AXIS_NUMBER * DATA_INPUT_USER] = {0};

/********************************* Main *********************************/
int main() 
{
	/* Initialization */
	init();
		
#ifdef DATA_LOGGING
		/* Data logging mode */
		/* Compiler flag: -DDATA_LOGGING */
		data_logging_mode();
#endif
#ifdef NEAI_LIB
		/* Smart sensor mode with NanoEdge AI Library*/
		/* Compiler flag -DNEAI_LIB */
		neai_library_test_mode();
#endif
}

/********************************* Functions *********************************/
void init ()
{
	pc.baud(115200);
    lsm6dsl->set_x_odr(3330.0f);
	lsm6dsl->set_x_fs(4.0f);
	lsm6dsl->enable_x();
	thread_sleep_for(100);
#ifdef NEAI_LIB
	NanoEdgeAI_initialize();
#endif
}

#ifdef DATA_LOGGING
/**
 * @brief  Data logging process
 *
 * @param  None
 * @retval None
 */
void data_logging_mode()
{
	while(1) {
		// Here we are polling in order to detect strumming vibration.
		// Depending on your setup and instrument, edit the trigger function as needed. 
		if (strum_trigger()) {
			fill_acc_array();
		}
	}
}
#endif

#ifdef NEAI_LIB
/**
 * @brief  Testing process with NanoEdge AI library
 *
 * @param  None
 * @retval None
 */
void neai_library_test_mode()
{
	uint16_t learn_cpt = 0;
	do {
		// Here we are polling in order to detect strumming vibration.
		// Depending on your setup and instrument, edit the trigger function as needed. 
		if (strum_trigger()) {
			fill_acc_array();
			NanoEdgeAI_learn(data_user);
			pc.printf("%d\n", (int)(learn_cpt * 100) / LEARNING_NUMBER);
			learn_cpt++;
		}
	} while (learn_cpt < LEARNING_NUMBER)

	led_blink_learning_over()

	while(1) {
		if (strum_trigger()) {
			fill_acc_array();
			similarity = NanoEdgeAI_detect(data_user);
			pc.printf("%d\n", similarity);

			if (similarity < THRESH_SIMILARITY) {
				led_blink_burst_ko();
			} else {
				led_blink_ok();
			}
		}
	}
}
#endif

bool strum_trigger () 
{
	/* Continuously monitor the average x, y or z accelerations. 
	   If 2 consecutive mini-buffers differ by more than a threshold %,
	   Then the trigger returns True, otherwise False. */

	float ref_sum_x = 0.0, ref_sum_y = 0.0, ref_sum_z = 0.0;
	float new_sum_x = 0.0, new_sum_y = 0.0, new_sum_z = 0.0;
	float ref_avg_x = 0.0, ref_avg_y = 0.0, ref_avg_z = 0.0;
	float new_avg_x = 0.0, new_avg_y = 0.0, new_avg_z = 0.0;

	// First we get some values and create a sum
	// This will be our reference
	for (uint16_t i = 0; i < MINI ; i++) {
		get_values();
		ref_sum_x += values[0];
		ref_sum_y += values[1];
		ref_sum_z += values[2];
	}

	// Next, we repeat the operation 
	for (uint16_t i = 0; i < MINI ; i++) {
		get_values();
		new_sum_x += values[0];
		new_sum_y += values[1];
		new_sum_z += values[2];
	}
	
	// We average both samples
	new_avg_x = abs(new_sum_x/MINI);
	new_avg_y = abs(new_sum_y/MINI);
	new_avg_z = abs(new_sum_z/MINI);
	ref_avg_x = abs(ref_sum_x/MINI);
	ref_avg_y = abs(ref_sum_y/MINI);
	ref_avg_z = abs(ref_sum_z/MINI);
	
	// If we are in a really small range, get out, it might just be noise
	if (new_avg_x > NOISE && new_avg_y > NOISE && new_avg_z > NOISE && new_avg_x > NOISE && new_avg_y > NOISE && new_avg_z > NOISE) {
		// Then we compare our new sample with the reference one
		if (new_avg_x > ref_avg_x*THRESH || new_avg_y > ref_avg_y*THRESH || new_avg_z > ref_avg_z*THRESH) {
    		return true; 
		} else {
    		return false;
		}
	} else {
		return false;
	}
}

void fill_acc_array ()
{
	/* Fill a buffer with accelerometer values 
	   Print output to serial port */

	for (uint16_t i = 0; i < DATA_INPUT_USER ; i++) {
		get_values();
		data_user[AXIS_NUMBER * i] = values[0];
		data_user[(AXIS_NUMBER * i) + 1] = values[1];
		data_user[(AXIS_NUMBER * i) + 2] = values[2];
	}
	/* Print data in the serial */
#ifdef NEAI_DATALOGGING
	for (uint16_t i = 0; i < DATA_INPUT_USER * AXIS_NUMBERS; i++) {
		pc.printf("%.3f ", data_user[i]);
	}
	pc.printf("\n");
#endif
}

void get_values ()
{
	/* Get acceleration values,
	   and make sure they are not duplicates */

	do {
		lsm6dsl->get_x_axes(raw_values);
	}
	while ( last_raw_values[0] == raw_values[0] || last_raw_values[1] == raw_values[1] || last_raw_values[2] == raw_values[2] );
	/* Update values */
	for (uint8_t i = 0; i < AXIS_NUMBER; i++) {
		last_raw_values[i] = raw_values[i];
		values[i] = (float) raw_values[i] / 1000;
	}
}

void led_blink_burst_ko()
{
	for (uint8_t j = 0; j < 5; j++) {
		led = 1;
		thread_sleep_for(200);
		led = 0;
		thread_sleep_for(200);
	}

}
void led_blink_learning_over()
{
	for (uint8_t j = 0; j < 5; j++) {
		led = 1;
		thread_sleep_for(1000);
		led = 0;
		thread_sleep_for(250);
	}

}
void led_blink_ok()
{
	for (uint8_t j = 0; j < 2; j++) {
		led = 1;
		thread_sleep_for(1000);
		led = 0;
		thread_sleep_for(250);
	}

}
