#include "nu32dip.h"  // constants, funcs for startup and UART

#define CORE_TICKS_PER_SECOND 40000000  // Core clock ticks per second (assuming 40MHz)

static unsigned int startTime = 0;
static unsigned int stopTime = 0;
static unsigned int elapsedTime = 0;
static enum { WAITING, TIMING } timerState = WAITING;  // Stopwatch states

void __ISR(_EXTERNAL_2_VECTOR, IPL6SOFT) Ext2ISR(void) {  // INT2 interrupt service routine
  if (timerState == WAITING) {
    // Start timing: Reset the core timer and set the state to TIMING
    _CP0_SET_COUNT(0);  // Reset core timer
    startTime = _CP0_GET_COUNT();  // Store the start time
    timerState = TIMING;
    NU32_WriteUART1("Press the USER button again to stop the timer.\r\n");
  } else {
    // Stop timing: Get the current time and calculate elapsed time
    stopTime = _CP0_GET_COUNT();  // Read the current timer count
    elapsedTime = (stopTime - startTime) / (CORE_TICKS_PER_SECOND);  // Calculate elapsed time in seconds
    char msg[128];
    sprintf(msg, "Elapsed time: %.3f seconds\r\n", (float)elapsedTime);
    NU32_WriteUART1(msg);
    timerState = WAITING;  // Reset to waiting state
    NU32_WriteUART1("Press the USER button to start the timer.\r\n");
  }
  IFS0bits.INT2IF = 0;  // Clear the interrupt flag
}

int main(void) {
  NU32_Startup();  // Startup sequence (cache, interrupts, UART, etc.)
  
  __builtin_disable_interrupts();  // Disable interrupts initially
  
  INTCONbits.INT2EP = 0;  // INT2 triggers on falling edge (button press)
  IPC2bits.INT2IP = 6;  // Set INT2 priority to 6
  IPC2bits.INT2IS = 0;  // Set sub-priority to 0
  IFS0bits.INT2IF = 0;  // Clear INT2 interrupt flag
  IEC0bits.INT2IE = 1;  // Enable INT2 interrupt
  
  __builtin_enable_interrupts();  // Enable interrupts globally
  
  NU32_WriteUART1("Press the USER button to start the timer.\r\n");

  while (1) {
    // In "TIMING" state, you can periodically print the current elapsed time (optional)
    if (timerState == TIMING) {
      unsigned int currentTime = _CP0_GET_COUNT();
      unsigned int runningTime = (currentTime - startTime) / CORE_TICKS_PER_SECOND;
      char msg[128];
      sprintf(msg, "Running time: %d seconds\r\n", runningTime);
      NU32_WriteUART1(msg);
      __builtin_disable_interrupts();  // Disable interrupts while updating time
      delay(1000000);  // Delay for 1 second to avoid excessive UART output
      __builtin_enable_interrupts();  // Re-enable interrupts
    }
  }

  return 0;
}

void delay(unsigned int delayTime) {
  _CP0_SET_COUNT(0);
  while (_CP0_GET_COUNT() < delayTime) {
    ;  // Delay loop
  }
}
