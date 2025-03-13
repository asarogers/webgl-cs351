#include "nu32dip.h"
// config bits, constants, funcs for startup and UART
// include other header files here
#define BUF_SIZE 200
int main()
{
    char buffer[BUF_SIZE];
    NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
    NU32DIP_YELLOW = 1; // turn off the LEDs
    NU32DIP_GREEN = 1;
    __builtin_disable_interrupts();
    // in future, initialize modules or peripherals here
    __builtin_enable_interrupts();
    while(1)
    {
        UART2_Startup();
        NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
        NU32DIP_YELLOW = 1;
        int n = 0;
        int n2 = 0;
        // clear the error LED
        switch (buffer[0]) {
            case 'a':{
               WriteUART2("a");
               while(!get_encoder_flag()){}
                set_encoder_flag(0);
                char m[50];
                int p = get_encoder_count();
             
                sprintf(m, "%d\r\n",p);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'b':{
                 WriteUART2("b");
                 break;
             }
            // case 'd':{
            //     n = 0;
            //     NU32DIP_ReadUART1(buffer,BUF_SIZE);
            //     sscanf(buffer, "%d", &n);
            //     sprintf(buffer,"%d\r\n", n + 1); // return the number + 1
            //     NU32DIP_WriteUART1(buffer);
            //     break;
            // }
            // case 'x':{
            //     n = 0;
            //     n2 = 0;
            //     NU32DIP_ReadUART1(buffer,BUF_SIZE);
            //     sscanf(buffer, "%d %d", &n, &n2);
            //     sprintf(buffer,"%d\r\n", n + n2); // return the number + 1
            //     NU32DIP_WriteUART1(buffer);
            //     break;
            // }
            case 'q':{// handle q for quit. Later you may want to return to IDLE mode here.
                break;
            }
            default:{
                NU32DIP_YELLOW = 0; // turn on LED2 to indicate an error
                break;
            }
        }
    }
    return 0;
}