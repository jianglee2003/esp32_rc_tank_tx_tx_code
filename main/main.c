#include <stdio.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_timer.h" // Driver for ESP Timer.

#include "esp_log.h"

#include "driver/gpio.h" // Driver for GPIO.
#include "driver/ledc.h" // Driver for Ledc PWM.

#include "esp_adc/adc_oneshot.h" // Library for ADC One Shot Ver 5.3.1

#include "nrf24l01.h"

#define LED_ON_BOARD_WEACT          GPIO_NUM_22
#define LED_ON_BOARD_DEVKIT         GPIO_NUM_2

// Program mode
#define TX_MODE                     1
#define RX_MODE                     0
#define PROGRAM_MODE                RX_MODE

// Macro for LEDC PWM.
#define LEDC_TIMER                  LEDC_TIMER_0
#define LEDC_MODE                   LEDC_LOW_SPEED_MODE

#define MOTOR_L_FORWARD_CHANNEL     LEDC_CHANNEL_1
#define MOTOR_L_BACKWARD_CHANNEL    LEDC_CHANNEL_2
#define MOTOR_R_FORWARD_CHANNEL     LEDC_CHANNEL_3
#define MOTOR_R_BACKWARD_CHANNEL    LEDC_CHANNEL_4

#define MOTOR_L_FORWARD_PIN         GPIO_NUM_13
#define MOTOR_L_BACKWARD_PIN        GPIO_NUM_12
#define MOTOR_R_FORWARD_PIN         GPIO_NUM_14
#define MOTOR_R_BACKWARD_PIN        GPIO_NUM_27

#define LEDC_DUTY_RESOLUTION        LEDC_TIMER_10_BIT // Set duty resolution to 10 bits
#define LEDC_FREQUENCY              (38000) // Frequency in Hertz. Set frequency at 38 kHz

/**
 * @note
 * - Due to high input voltage for the motor (5S - 18,5V with 12V DC Motor), 
 * we have to limit the duty cycle for the BTS7960 PWM Module.
 */
#define DUTY_CYCLE_LIMIT            35
#define LIMIT_ENABLE                1
#define LIMIT_DISABLE               0
#define LIMIT_MODE                  LIMIT_ENABLE

// pipeline address of NRF24L01 module. Both TX and RX must have the same addr.
uint8_t rf_tx_addr[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t rf_rx_addr[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};


// Macro for ADC channel - ADC pins.
#define ADC_PIN_1   ADC_CHANNEL_6
#define ADC_PIN_2   ADC_CHANNEL_7

// Create a struct to handle an ADC instance.
adc_oneshot_unit_handle_t adc_handle = NULL;

// Queue declaration.
QueueHandle_t tx_rx_queue = NULL;
char tx_buffer[30], rx_buffer[30];

QueueHandle_t motor_a_queue = NULL;
QueueHandle_t motor_b_queue = NULL;
char motor_a_buffer[10], motor_b_buffer[10], motor_buffer[30];

/**
 * @brief initialize ledc PWM driver.
 * @param[in] gpio_num gpio_num which will be used to generate PWM Pulse.
 * @param[in] channel ledc channel which will be used for the gpio pin.
 * @param[in] frequency frequency of PWM Pulse.
 * @note For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */
void ledc_init(int gpio_num, ledc_channel_t channel, uint32_t frequency) {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RESOLUTION,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = frequency,  // Set output frequency
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if(ret != ESP_OK) {
        ESP_LOGE("ledc_pwm", "Fail to initial LedC Timer.");
    }

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = channel,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = gpio_num,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if(ret != ESP_OK) {
        ESP_LOGE("ledc_pwm", "Fail to initial LedC Channel.");
    }  
}

/**
 * @brief Generate the PWM pulse.
 * @param[in] channel Channel - GPIO Pin which will be generated PWM Pulse.
 * @param[in] duty_val duty cycle for PWM pulse.
 * @note This function assume user use the Ledc Timer with low speed mode, user need to change this.
 */
void set_duty_cycle(ledc_channel_t channel, uint16_t duty_val) {
    //do LEDC_TIMER_10_BIT nen duty 100% khi duty_val o gia tri 1023
    duty_val = (duty_val * 1024) / 100;
    ledc_set_duty(LEDC_MODE, channel, duty_val);

    // Update duty to apply the new value
    ledc_update_duty(LEDC_MODE, channel);
}

// flag to check if TX was send stop command or not.
bool stop_command = false;

// Read ADC Raw value, if condition reached, send data to Queue.
void adc_one_shot_task (void * parameter) {
    for(;;) {
        int raw_data_joystick_x = 0, raw_data_joystick_y = 0;
        /**
         * Get one ADC conversion raw result.
         * If use multiple channel, this function needs to be called for each channel.
         */
        adc_oneshot_read(adc_handle, ADC_PIN_1, &raw_data_joystick_y);
        adc_oneshot_read(adc_handle, ADC_PIN_2, &raw_data_joystick_x);

        // When the condition of ADC values are reached, read them and send to Queue, then pass to RF Transaction.
        if(raw_data_joystick_x < 1800 || raw_data_joystick_x > 2000 
            || raw_data_joystick_y < 1800 || raw_data_joystick_y > 2000) {
            // printf("channel 6: %d, channel 7: %d\n", raw_data[0], raw_data[1]);
            
            sprintf(tx_buffer, "/%d/%d/", raw_data_joystick_x, raw_data_joystick_y);
            xQueueSend(tx_rx_queue, tx_buffer, 0);

            // because the adc values are sent, so this flag must be reset.
            stop_command = false;
        } 

        // when conditions are wrong, send the stop command once.
        else {
            if(stop_command == false) {
                // Send command to stop the motors.
                xQueueSend(tx_rx_queue, "/stop/", 0);

                // We just need to send stop command once, so this flag will be set, to skip further loop.
                stop_command = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(70));
    }
}

// Read data from Queue and send to NRF module.Used in TX Mode.
void send_rf_data_task(void *parameter) {
    for(;;) {
        xQueueReceive(tx_rx_queue, rx_buffer, portMAX_DELAY); // Rx buffer is used as buffer to transmit RF Data in Tx mode.
        // printf("%s\n", rx_buffer);
                
        uint8_t result = NRF24_Transmit((uint8_t *) rx_buffer);
        if (result == SUCCESS) {
            gpio_set_level(LED_ON_BOARD_WEACT, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_ON_BOARD_WEACT, 1);
        } else {
            printf("Transmit failed!\n");
        }
    }
}

// Flag to indicate there is new value coming in from RF Task.
bool new_command_coming_in = false, motor_already_turn_off = false;

// ESP-Timer Configuration
esp_timer_handle_t esp_timer_handle = NULL;

void esp_timer_callback(void *parameter) {
    // Set the flag to indicate this is a stop command.
    new_command_coming_in = false;

    // reset flag for further use to turn off the motor when there is no incomming RF data.
    motor_already_turn_off = false;
}

// Read received RF Data. Only used in RX Mode.
void receive_rf_data_task(void * parameter) {
    for(;;) {
        if (isDataAvailable(1)) {   // check if data arrived on pipe 1
            NRF24_Receive((uint8_t *) tx_buffer); // Tx buffer is used to receive RF data in RX Mode.

            gpio_set_level(LED_ON_BOARD_DEVKIT, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_ON_BOARD_DEVKIT, 0);

            // when received RF data is a stop command.
            if(strstr(tx_buffer, "stop") != NULL) {
                // Set the flag to indicate this is a stop command.
                new_command_coming_in = false;

                // reset flag for further use to turn off the motor when there is no incomming RF data.
                motor_already_turn_off = false;
            }

            // When it is a normal command. Standard format is: "/raw_data_x/raw_data_y/"
            else {
                // Send data to Queue for motor controllers.
                xQueueSend(tx_rx_queue, tx_buffer, 0);

                // Set the flag to indicate new incoming command.
                new_command_coming_in = true;

                // reset flag for further use to turn off the motor when there is no incomming RF data.
                motor_already_turn_off = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Helper function to convert raw ADC Value to PWM percentage (0 - 100).
 * 
 * @param[in] *x pointer to address of actual input value.
 * @param[in] in_min minimum input range.
 * @param[in] in_max maximum input range.
 * @param[in] reverse flag to reverse value conversion or not. read more in function definition.
 * @return converted value, mapped from input range -> output range (0 - 100).
 * 
 * @note The ADC raw value range: 
 * @note [0, 1800]: percentage 0 - 100% for forward direction.
 * @note [2000, 4095]: percentage 0 - 100% for backward direction.
 */
int map_range_100_int_format(int *x, int in_min, int in_max, bool reverse) {
    // reverse for range from 0 - 1800 (backward direction, so the smaller adc value, the bigger PWM Pulse for backward).
    if (reverse) {
        // in_min -> 100, in_max -> 0
        return (in_max - *x) * 100 / (in_max - in_min);
    } 
    
    // not reverse for range from 2000 - 4095
    else {
        // in_min -> 0, in_max -> 100
        return (*x - in_min) * 100 / (in_max - in_min);
    }
}

// === HELPER FUNCTIONS ===
static inline int clamp_percent(int val) {
    if (val < 0) return abs(val);
    if (val > 100) return 100;
    return val;
}

#define ADC_BALANCE_VALUE_MIN 1920
#define ADC_BALANCE_VALUE_MAX 2050

// Read data from tx_rx_Queue Buffer, then Pass to each motor controller code. Used in RX mode.
void get_raw_adc_from_rf(void *parameter) {
    for(;;) {
        xQueueReceive(tx_rx_queue, rx_buffer, portMAX_DELAY);

        char joystick_x[10], joystick_y[10];
        // %[^/] means "read until '/'"
        sscanf(rx_buffer, "/%[^/]/%[^/]/", joystick_x, joystick_y);

        xQueueSend(motor_a_queue, joystick_x, 0);
        xQueueSend(motor_b_queue, joystick_y, 0);

        // printf("First: %s, Second: %s\n", joystick_x, joystick_y);
    }
}

/**
 * @brief Task to control motors speed and direction.
 * @note This is version 2. idff
 */
void motor_controller_task(void *parameter) {
    for (;;) {
        xQueueReceive(tx_rx_queue, rx_buffer, portMAX_DELAY);

        int joystick_x, joystick_y;
        sscanf(rx_buffer, "/%d/%d/", &joystick_x, &joystick_y);

        bool forward = false, backward = false;
        bool rotate_left = false, rotate_right = false;

        int y_percent = 0, x_percent = 0;

        // --- Y-axis (forward / backward) ---
        if (joystick_y > ADC_BALANCE_VALUE_MAX) {
            y_percent = map_range_100_int_format(&joystick_y, ADC_BALANCE_VALUE_MAX, 4095, false);
            forward = true;
        } else if (joystick_y < ADC_BALANCE_VALUE_MIN) {
            y_percent = map_range_100_int_format(&joystick_y, 0, ADC_BALANCE_VALUE_MIN, true);
            backward = true;
        } else {
            y_percent = 0;
        }

        // --- Determine motion mode ---
        int left_pwm = y_percent;
        int right_pwm = y_percent;

        // --- X-axis (turning) --- steering factor.
        if (joystick_x > ADC_BALANCE_VALUE_MAX || joystick_x < ADC_BALANCE_VALUE_MIN) {
            if(joystick_x > ADC_BALANCE_VALUE_MAX) {
                x_percent = map_range_100_int_format(&joystick_x, ADC_BALANCE_VALUE_MAX, 4095, false); // left
                rotate_left = true;
                left_pwm = abs(x_percent - y_percent);
            }
            else if (joystick_x < ADC_BALANCE_VALUE_MIN) {
                x_percent = map_range_100_int_format(&joystick_x, 0, ADC_BALANCE_VALUE_MIN, true); // right
                rotate_right = true;
                right_pwm = abs(x_percent - y_percent);
            }
        } 

        #if LIMIT_MODE == LIMIT_ENABLE
            if(left_pwm > DUTY_CYCLE_LIMIT) {
                left_pwm = DUTY_CYCLE_LIMIT;
            }
            if(right_pwm > DUTY_CYCLE_LIMIT) {
                right_pwm = DUTY_CYCLE_LIMIT;
            }
        #endif

        // --- OUTPUT TO MOTORS ---
        if (forward) {
            // Forward motion
            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, left_pwm);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, right_pwm);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, 0);
        } 

        else if (backward) {
            // Backward motion
            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, left_pwm);
            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, right_pwm);
        } 

        else if (rotate_left) {
            // 2 motors must be spinning in the opposite direction.
            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, 0); 
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, left_pwm);
            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, left_pwm);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, 0);
        } 

        else if (rotate_right) {
            // 2 motors must be spinning in the opposite direction.
            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, right_pwm);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, right_pwm);
        } 

        esp_timer_start_once(esp_timer_handle, 500000);

        // printf("percent_x: %d, percent_y: %d, left_pwm: %d, right_pwm: %d | raw_X=%d raw_Y=%d | %s%s%s%s\n",
        //        x_percent, y_percent,
        //        left_pwm, right_pwm, joystick_x, joystick_y,
        //        forward ? "FWD " : "", backward ? "REV " : "",
        //        rotate_left ? "ROT_L " : "", rotate_right ? "ROT_R " : "");
    }
}

/**
 * @brief Task to control the speed and direction of Motor A. 
 * @note This task receive raw adc value from Queue, then convert it to percentage to control the PWM Pulse.
 */
void motor_a_controller_task(void * parameter) {
    for(;;) {
        xQueueReceive(motor_a_queue, motor_a_buffer, portMAX_DELAY);

        // format received data from string to integer.
        int joystick_x = 0;
        sscanf(motor_a_buffer, "%d", &joystick_x);

        // Value from 0 - 1800 is backward direction.
        if(joystick_x <= 1800) {
            int percent = map_range_100_int_format(&joystick_x, 0, 1800, true);

            #if LIMIT_MODE == LIMIT_ENABLE
                if(percent > DUTY_CYCLE_LIMIT) {
                    percent = DUTY_CYCLE_LIMIT;
                }
            #endif

            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, percent);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, 0);
            esp_timer_start_once(esp_timer_handle, 500000);
        } 

        // Value from 2000 - 4096 is forward direction.
        else if (joystick_x >= 2000) {
            int percent = map_range_100_int_format(&joystick_x, 2000, 4095, false);

            #if LIMIT_MODE == LIMIT_ENABLE
                if(percent > DUTY_CYCLE_LIMIT) {
                    percent = DUTY_CYCLE_LIMIT;
                }
            #endif

            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, percent);
            esp_timer_start_once(esp_timer_handle, 500000);
        } 

        // Otherwise is turn off the motor driver.
        else {
            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, 0);
        }
    }
}

/**
 * @brief Task to control the speed and direction of Motor B. 
 * @note This task has the same process as the motor_b_controller_task() task.
 */
void motor_b_controller_task(void * parameter) {
    for(;;) {
        xQueueReceive(motor_b_queue, motor_b_buffer, portMAX_DELAY);

        int joystick_y = 0;
        sscanf(motor_b_buffer, "%d", &joystick_y);

        if(joystick_y <= 1800) {
            int percent = map_range_100_int_format(&joystick_y, 0, 1800, true);

            #if LIMIT_MODE == LIMIT_ENABLE
                if(percent > DUTY_CYCLE_LIMIT) {
                    percent = DUTY_CYCLE_LIMIT;
                }
            #endif

            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, percent);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, 0);
            esp_timer_start_once(esp_timer_handle, 500000);
        } 
        
        else if (joystick_y >= 2000) {
            int percent = map_range_100_int_format(&joystick_y, 2000, 4095, false);

            #if LIMIT_MODE == LIMIT_ENABLE
                if(percent > DUTY_CYCLE_LIMIT) {
                    percent = DUTY_CYCLE_LIMIT;
                }
            #endif

            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, percent);
            esp_timer_start_once(esp_timer_handle, 500000);
        } 
        
        else {
            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, 0);
        }
    }
}

/**
 * @brief Task to turn off all the motors when there is no incomming RF data.
 * @note 
 * 2 conditions must be reached at the same time:
 * - First: there is no incomming RF data.
 * - Second: motors have not turned off when no incomming data yet.
 */
void turn_off_motor_task(void * parameter) {
    for(;;) {
        if(new_command_coming_in == false && motor_already_turn_off == false) {
            set_duty_cycle(MOTOR_L_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_L_BACKWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_FORWARD_CHANNEL, 0);
            set_duty_cycle(MOTOR_R_BACKWARD_CHANNEL, 0);

            // Set the flag, to skip this condition in next further loop.
            motor_already_turn_off = true;
            printf("all motors are stopped\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief initialize the system in TX Mode.
 */
void program_tx_mode(void) {
    printf("NRF24 TX Mode Example\n");

    // Init NRF24L01 at TX mode.
    NRF24_TxMode(rf_tx_addr, 10);  // channel 10

    xTaskCreate(adc_one_shot_task, "adc_one_shot_task", 2048, NULL, 4, NULL);
    xTaskCreate(send_rf_data_task, "send_rf_data_task", 2048, NULL, 4, NULL);
}

/**
 * @brief initialize the system in RX Mode.
 */
void program_rx_mode(void) {
    printf("NRF24 RX Mode Example\n");
    
    // Create Queue for motor controller, used only for RX mode.
    motor_a_queue = xQueueCreate(30, sizeof(motor_a_buffer));
    if (motor_a_queue == NULL) {
        ESP_LOGE("queue", "Failed to create motor_a_queue");
    }
    motor_b_queue = xQueueCreate(30, sizeof(motor_b_buffer));
    if (motor_b_queue == NULL) {
        ESP_LOGE("queue", "Failed to create motor_b_queue");
    }

    // Init NRF24L01 at RX mode.
    NRF24_RxMode(rf_rx_addr, 10);  // same channel as TX

    xTaskCreate(receive_rf_data_task, "receive_rf_data_task", 2048, NULL, 4, NULL);
    // xTaskCreate(get_raw_adc_from_rf, "get_raw_adc_from_rf", 2048, NULL, 4, NULL);

    // xTaskCreate(motor_a_controller_task, "motor_a_controller", 2048, NULL, 4, NULL);
    // xTaskCreate(motor_b_controller_task, "motor_b_controller", 2048, NULL, 4, NULL);

    xTaskCreate(motor_controller_task, "motor_controller_task", 2048, NULL, 4, NULL);

    xTaskCreate(turn_off_motor_task, 
                "turn_off_motor_when_no_incomming_rf_data_task", 
                2048, NULL, 4, NULL);    
}


void app_main(void) {
    gpio_reset_pin(LED_ON_BOARD_WEACT);
    gpio_set_direction(LED_ON_BOARD_WEACT, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ON_BOARD_WEACT, 1);

    gpio_reset_pin(LED_ON_BOARD_DEVKIT);
    gpio_set_direction(LED_ON_BOARD_DEVKIT, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ON_BOARD_DEVKIT, 0);

    // ESP - Timer initialization.
    esp_timer_create_args_t esp_timer_arg = {
        .callback = esp_timer_callback,
        .name = "esp_timer_callback",
    };
    esp_timer_create(&esp_timer_arg, &esp_timer_handle);

    // Create TX_RX_Queue, used for both TX and RX mode.
    tx_rx_queue = xQueueCreate(30, sizeof(tx_buffer));
    if (tx_rx_queue == NULL) {
        ESP_LOGE("queue", "Failed to create tx_rx_queue");
    }

    // Configuration structure for an ADC instance.
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1,              // Choose ADC unit. (ADC1)
        .clk_src = SOC_MOD_CLK_APB,         // Choose clk source (APB_CLK - Default)
        .ulp_mode = ADC_ULP_MODE_DISABLE,   // see "adc_ulp_mode_t".
    };

    // Configure ADC Channel IOs to measure analog signal.
    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,       // Attenuation of 12 dB (voltage range from 0 - 3,9V)
        .bitwidth = ADC_BITWIDTH_12,    // Resolution of 12 bit (raw data range from 0 - 4095)
    };

    // create an ADC unit handle.
    adc_oneshot_new_unit(&adc_cfg, &adc_handle);

    /*  Config channel for ADC One Shot.
        This function can be called multiple times to configure different ADC channels.
        IMPORTANT: This function should be define as a variadic function.
    */
    adc_oneshot_config_channel(adc_handle, ADC_PIN_1, &channel_cfg);
    adc_oneshot_config_channel(adc_handle, ADC_PIN_2, &channel_cfg);

    // Ledc init.
    ledc_init(MOTOR_L_FORWARD_PIN, MOTOR_L_FORWARD_CHANNEL, LEDC_FREQUENCY);
    ledc_init(MOTOR_L_BACKWARD_PIN, MOTOR_L_BACKWARD_CHANNEL, LEDC_FREQUENCY);
    ledc_init(MOTOR_R_FORWARD_PIN, MOTOR_R_FORWARD_CHANNEL, LEDC_FREQUENCY);
    ledc_init(MOTOR_R_BACKWARD_PIN, MOTOR_R_BACKWARD_CHANNEL, LEDC_FREQUENCY);

    // nrf24l01 setup.
    setup_nrf24l01_bus();
    NRF24_Init();

    #if PROGRAM_MODE == TX_MODE
        program_tx_mode();
    #elif PROGRAM_MODE == RX_MODE
        program_rx_mode();
    #endif
}