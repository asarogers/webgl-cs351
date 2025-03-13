#ifndef UTILITIES_H
#define UTILITIES_H

// Define the enum here
typedef enum {
    IDLE, 
    PWM, 
    ITEST, 
    HOLD, 
    TRACK
} mode_t;

// Declare the global variable as extern
extern volatile mode_t mode;

// Function prototypes for getting and setting the mode
void set_mode(mode_t new_mode);
mode_t get_mode(void);

#endif // UTILITIES_H
