#include "utilities.h"

// Define the global mode variable
volatile mode_t mode = IDLE;  // Initialize to a default state

// Function to set the mode
void set_mode(mode_t new_mode) {
    mode = new_mode;
}

// Function to get the mode
mode_t get_mode(void) {
    return mode;
}
