#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
//#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

#include "pico_uart_transports.h"
#include "debug_uart.h"

#define LED_PIN 	25
#define ADC_PIN 	26
#define SAMPLE_RATE 16000
#define BUFFER_SIZE 128  // Dimensione del "batch" di campioni

#define P_AUDIO		5

#define NUM_BUFFERS 2
int16_t buffer0[BUFFER_SIZE];
int16_t buffer1[BUFFER_SIZE];
int16_t *buffers[NUM_BUFFERS] = {buffer0, buffer1};

volatile int active_buf_idx = 0;
volatile int sample_count = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){uart_printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

QueueHandle_t xBufferQueue;
rcl_publisher_t audio_pub;

// --- 1. CALLBACK DI CAMPIONAMENTO (Alta Priorità) ---
bool adc_callback(struct repeating_timer *t) {
    // Leggi e centra il segnale (Bias 2048)
    buffers[active_buf_idx][sample_count++] = (int16_t)adc_read() - 2048;

    if (sample_count >= BUFFER_SIZE) {
        int16_t *full_ptr = buffers[active_buf_idx];
        
        // Switch immediato del buffer
        active_buf_idx = 1 - active_buf_idx;
        sample_count = 0;

		// CONTATORE PER IL DEBUG
        static int debug_count = 0;
        debug_count++;
        if (debug_count >= 125) { // Circa una volta al secondo a 16kHz/128
            static bool led_state = false;
            gpio_put(LED_PIN, (led_state = !led_state));
            debug_count = 0;
        }

        // Invia il PUNTATORE alla coda
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(xBufferQueue, &full_ptr, &woken);
        portYIELD_FROM_ISR(woken);
    }
    return true;
}

// --- 2. TASK MICRO-ROS (Consumer) ---
void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
	rclc_executor_t executor;
    
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "audio_node", "", &support));

    // Inizializzazione MultiArray (Attenzione alla memoria!)
    std_msgs__msg__Int16MultiArray msg;
    msg.data.capacity = BUFFER_SIZE;
    msg.data.size = BUFFER_SIZE;
    msg.data.data = (int16_t*) malloc(BUFFER_SIZE * sizeof(int16_t));

	msg.layout.dim.capacity = 0;
	msg.layout.dim.size = 0;
	msg.layout.dim.data = NULL;
	msg.layout.data_offset = 0;

	// Verifica che la memoria sia stata allocata
	if (msg.data.data == NULL) {
		uart_printf("Errore: Malloc fallita!\n");
		vTaskDelete(NULL);
	}

    RCCHECK(rclc_publisher_init_default(&audio_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "audio_raw"));

	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    int16_t *received_ptr;
    for (;;) {
        // Aspetta un buffer pronto dalla ISR
        if (xQueueReceive(xBufferQueue, &received_ptr, portMAX_DELAY)) {
            // Copia veloce dei dati nel messaggio
            memcpy(msg.data.data, received_ptr, BUFFER_SIZE * sizeof(int16_t));
            
            // Pubblica su ROS2
            RCSOFTCHECK(rcl_publish(&audio_pub, &msg, NULL));
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
		}
    }
}

// --- 3. MAIN ---
int main() {
    stdio_init_all();
	initialize_debug_uart();
	sleep_ms(3000);

	gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    // Trasporto USB Nativo (Assicurati che sia attivo nel CMakeLists.txt)
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // Coda di puntatori (contiene l'indirizzo dei buffer)
    xBufferQueue = xQueueCreate(NUM_BUFFERS, sizeof(int16_t*));

    // Timer Hardware per il campionamento a 16kHz
    struct repeating_timer timer;
    add_repeating_timer_us(-(1000000 / SAMPLE_RATE), adc_callback, NULL, &timer);

    // Task micro-ROS sul Core 1
    TaskHandle_t uros_handle;
    xTaskCreate(micro_ros_task, "uROS", 4096, NULL, 1, &uros_handle);
    vTaskCoreAffinitySet(uros_handle, (1 << 1)); // Core 1

    vTaskStartScheduler();

	while (1) {
        tight_loop_contents();
    }
}