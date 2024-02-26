/**
 * @file
 * @brief Main C file for RF controlled outlet
 * @author Scott Stack 
 */

#include <xc.h>
#include "main.h"
#include "config.h"
#include "rfm69/RFM69.h"
#include "timer.h"
#include "remote_actions.h"


// ========== GLOBAL DATA ===========
uint32_t SYSTEM_TICKS = 0;      ///< Global system tick count
global_data_t global_data;      ///< All other global variables stored in structure for clarity             

// ========== LOCAL FUNCTIONS ===========
bool button_is_pressed(void);
void handle_button_input(void);
void process_new_commands(void);
void initialize_global_data(void);
void handle_periodic_msg_send(void);


// ========== MAIN FUNCTION ===========
void main(void)
{   
    int status = NO_ERROR;

    // Init global data structure
    initialize_global_data();

    // Initialize the device
    initialize_peripherals();

    //enable interrupts
    enable_interrupts();

    // Wait to ensure power stabilizes before continuing
    timer_wait_ms(INIT_TIME_WAIT_MS);
    CLRWDT();

    // Switch the attached outlet on right after power is applied
    switch_load(LOAD_ID_OUTLET_0, SWITCH_ON);

    // Initialize the wireless radio
    RFM69_init(RF69_915MHZ, 0, RFM69_DEFAULT_NETWORK_ID);

    // Read the channel code and set it in RFM driver
    global_data.current_channel = read_channel_code_input();
    RFM69_setNodeId(global_data.current_channel);

    // Send the on command to turn on the other switch(s)
    for (int8_t i = 0; i < NUM_INITIAL_RETRIES; i++) {
        status = send_load_switch_cmd(LOAD_ID_OUTLET_0, SWITCH_ON); 
        if (status != NO_ERROR) {
            LED_blinkErrorCode(status);
        }

        // Wait a random amount of time between 1 and 16 ms before sending again
        timer_wait_ms((uint32_t)rand_lfsr16() & 0xF);
    }

    // Set the RF module into receive mode by default
    RFM69_enableReceiveMode();

    // Ok to enable power loss interrupts (PLI) now that everything is initialized
    enable_pli_int();

    // Indicate aliveness by flashing DEBUG LED
    // LED_resetIndication();
    
    while (1)
    {
        // Handle all button inputs
        handle_button_input();

        // Check for new commands and process them 
        process_new_commands();

        // If it's time, send a wakeup packet to the other endpoint(s)
        handle_periodic_msg_send();

        // Clear the watchdog timer
        CLRWDT();
    }
}


/**
 * @brief Initialize the global data structure
*/
void initialize_global_data(void) {
    global_data.time_last_pwron_received = 0;
    global_data.time_last_pwron_sent = 0;
    global_data.poweron_msg_received = false;
    global_data.current_channel = 0;
    global_data.current_switch_val = SWITCH_OFF;
}


/**
 * @brief Function called from main loop to handle all button input
*/
void handle_button_input(void) {
    int status = NO_ERROR;

    // Send a message if a button is pressed
    if (button_is_pressed())
    {
        // Command switch to opposite on whatever we switched it to last time
        global_data.current_switch_val = (global_data.current_switch_val == SWITCH_ON) ? SWITCH_OFF : SWITCH_ON;

        // Set our local switch to new value
        switch_load(LOAD_ID_OUTLET_0, global_data.current_switch_val);

        // Send command to remote to switch states
        status = send_load_switch_cmd(LOAD_ID_OUTLET_0, global_data.current_switch_val); 
        if (status != NO_ERROR)
        {
            LED_blinkErrorCode(status);
        }

    }

    // Update the channel setting switches
    uint8_t temp_chan = read_channel_code_input();
    if (temp_chan != global_data.current_channel)
    {
        global_data.current_channel = temp_chan;
        RFM69_setNodeId(global_data.current_channel);
    }
}


/**
 * @brief Called from the main function to check for new messages and process them
*/
void process_new_commands(void) {
    uint8_t received_pkt_size = 0;

    // Check to see if there's a received packet, if so handle it
    if(RFM69_receivePacket(global_data.rf_receive_buf, &received_pkt_size, sizeof(global_data.rf_receive_buf)))
    {
        int status = handle_received_command(global_data.rf_receive_buf, received_pkt_size);
        if (status != NO_ERROR) {
            LED_blinkErrorCode(status);
        }
    }
}

/**
 * @brief Called from the main function to handle sending periodic wakeup packets
*/
void handle_periodic_msg_send(void) {
    // NOTE: don't do periodic sending... it isn't necessary and messes things up
    // If it's time to send a new periodic switch update command, do it. Add some jitter to time this is sent
    // if (timer_has_time_elapsed(global_data.time_last_pwron_sent, WAKEUP_POLL_PERIOD_MS + (rand_lfsr16() & 0x1FF))) {
    //     send_load_switch_cmd(LOAD_ID_OUTLET_0, global_data.current_switch_val);
    //     global_data.next_wakeup_time = (uint32_t)(WAKEUP_POLL_PERIOD_MS + (rand_lfsr16() & 0xFF));
    // }
}



// ========== SUPPORT FUNCTIONS ===========
uint8_t last_button_input = HIGH;
uint8_t button_state = HIGH;
uint32_t last_debounce_time;
const uint32_t debounce_delay_ms = 50;

bool button_is_pressed(void)
{
    uint8_t reading = BUTTON_PIN;
    bool button_pressed = false;

    // If the button input has changed at all, reset the debounce timer
    if (reading != last_button_input) {
        last_debounce_time = timer_get_ticks();
    }

    // Only sample again if the debounce_delay has elapsed
    if (timer_has_time_elapsed(last_debounce_time, debounce_delay_ms)) {

        if (button_state != reading) {
            // Button input has officially changed!
            button_state = reading;

            // Button has been pressed down if current reading is LOW
            if (reading == LOW) {
                button_pressed = true;
            }
        }
    }

    // Update the last button input with the current reading
    last_button_input = reading;

    return button_pressed;
}
 


/**
 * @brief Read and return the input channel number that we should use
*/
uint8_t read_channel_code_input(void)
{
    // Return the settings on the DIP switch represented as a bitfield
    return ((CHAN_SEL3_PIN << 3) | (CHAN_SEL2_PIN << 2) | (CHAN_SEL1_PIN << 1) | (CHAN_SEL0_PIN));
}

