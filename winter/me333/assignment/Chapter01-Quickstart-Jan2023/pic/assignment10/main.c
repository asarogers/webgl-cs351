#include "nu32dip.h" // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"

#define BUF_SIZE 200
#define ENCODER_COUNTS_PER_REV 48

// Global variables
volatile float pwmDuty = 0;
volatile float kp_mA = -0.257;
volatile float ki_mA = 0.0004;
volatile float kp_deg = -0.2;
volatile float ki_deg = 0.00;
volatile float kd_deg = 0.0;
volatile float desired_deg = 0;
volatile static int integral = 0; 
volatile static float prevError = 0;
volatile float refCurrentFromPosition = 0;  // Global variablepwmDuty
static int actualCurrentArray[100];
static int refCurrentArray[100];
void timer2init();
void timer1init();
void startPWM();
void initHBridgeDir();
void sendDataToPython(int *refArray, int *actualArray, int length);

/**
 * Timer 2 ISR - Current Control Loop (5 kHz)
 */
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) {
    static int counter = 0;
    int actualCurrent = 0;
           
    static int refCurrent = -200;
    float test_current = INA219_read_current();
    // char debug_msg[100];
    // sprintf(debug_msg, "Manual current test: %f\r\n", test_current);
    // NU32DIP_WriteUART1(debug_msg);
    


    
    switch (get_mode()) {
        case IDLE:
            OC1RS = 0;
            LATBbits.LATB2 = 0; // Set direction to 0
            break;
        
        case PWM:
            OC1RS = (PR3 + 1) * (abs(pwmDuty) / 100.0);
            LATBbits.LATB2 = (pwmDuty >= 0) ? 1 : 0;
            break;
        
        case ITEST:
            if (counter % 25 == 0) { // Toggle reference at 0, 25, 50, 75 samples
                refCurrent = -refCurrent;
            }
            
            actualCurrent = INA219_read_current();
            float error = refCurrent - actualCurrent;
            integral = 0;
            integral += error;
            pwmDuty = (kp_mA * error) + (ki_mA * integral);

            // Apply saturation limits
            if (pwmDuty > 100) pwmDuty = 100;
            else if (pwmDuty < -100) pwmDuty = -100;
            
            OC1RS = (PR3 + 1) * (abs(pwmDuty) / 100.0);
            LATBbits.LATB2 = (pwmDuty >= 0) ? 1 : 0;

            refCurrentArray[counter] = refCurrent;
            actualCurrentArray[counter] = actualCurrent;
            
            counter++;
            if (counter >= 100) {
                set_mode(IDLE);
                sendDataToPython(refCurrentArray, actualCurrentArray, 100);
                counter = 0;
                integral = 0;
            }
            break;
        case HOLD:{
            
            actualCurrent = -INA219_read_current();
            float hold_error = refCurrentFromPosition - actualCurrent;
            // char msg[100];
            // sprintf(msg, "hold_error = %f refCurrentFromPosition = %f actualCurrent = %f\r\n", hold_error, refCurrentFromPosition, actualCurrent);
            // NU32DIP_WriteUART1(msg);

            integral += hold_error;
        
            pwmDuty = ((kp_mA * hold_error) + (ki_mA * integral));

        //     // char msg[50];char msg[50];
        // sprintf(msg,"position = %f\n\r", refCurrentFromPosition);
        // NU32DIP_WriteUART1(msg);
            // sprintf(msg,"pwmDuty = %d \n\r", pwmDuty);
            // NU32DIP_WriteUART1(msg);
        
            // Apply saturation limits
            if (pwmDuty > 100) pwmDuty = 100;
            else if (pwmDuty < -100) pwmDuty = -100;

            
        
            OC1RS = (PR3 + 1) * (abs(pwmDuty) / 100.0);
            LATBbits.LATB2 = (pwmDuty >= 0) ? 1 : 0;
            break;
        }

        default:
        
            break;
    }
    
    IFS0bits.T2IF = 0; // Clear interrupt flag
}

/**
 * Timer 1 ISR - position Control Loop (200 Hz)
 */
void __ISR(_TIMER_1_VECTOR, IPL6SOFT) PositionController(void)
{
    // if (get_mode() != 0){
    //     char msg[50];
    //     sprintf(msg,"Mode = %d\n\r", get_mode());
    //     NU32DIP_WriteUART1(msg);
    // }
    mode_t the_mode = get_mode();
    switch (the_mode)
    {
    case HOLD:
        {
            // char msg[50];
            // sprintf(msg,"BEFORE\n\r");
            // NU32DIP_WriteUART1(msg);

            // char debug_msg[100];
            // int encoder_count = get_encoder_count();
            // sprintf(debug_msg, "Encoder count: %d\n", encoder_count);
            // NU32DIP_WriteUART1(debug_msg);

            
            WriteUART2("a");
            while (!get_encoder_flag()){};
            set_encoder_flag(0);

            // sprintf(msg,"AFTER\n\r");
            // NU32DIP_WriteUART1(msg);

            float actualAngle = 360.0 * ((float)get_encoder_count() / (334.0 * 4.0));
            // PID Controller
            float errorAng = desired_deg - actualAngle;

            // char debug_msg[100];
            // int encoder_count = get_encoder_count();
            // sprintf(debug_msg, "desired_deg: %f actualAngle : %f\r\n", desired_deg, actualAngle);
            // NU32DIP_WriteUART1(debug_msg);

            if (!errorAng){
                errorAng = 0;
            }

            
            static int integralAngle = 0;
            integralAngle += errorAng;

            if (integralAngle > 5000) integralAngle = 5000;
            else if (integralAngle < -5000) integralAngle = -5000;


            refCurrentFromPosition = (kp_deg * errorAng) +
                            (ki_deg * integralAngle) +
                            (kd_deg * (errorAng - prevError));
            float cap = 500;
            if (refCurrentFromPosition > cap)
            {
                refCurrentFromPosition = cap; // upper bound
            }
            else if (refCurrentFromPosition < -cap)
            {
                refCurrentFromPosition = -cap; // lower bound
            }
            
            prevError = errorAng;
            
            
        }
    default:
        prevError = 0;
        break;
    }
    IFS0bits.T1IF = 0;
}


/**
 * Initialize Timer1 for Position Control ISR (200 Hz)
 */
void timer1init()
{
    T1CONbits.ON = 0;
    T1CONbits.TCS = 0;  
    T1CONbits.TCKPS = 2;
    int prescaler = 64;
    PR1 = (NU32DIP_SYS_FREQ / (200 * prescaler)) - 1;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    IPC1bits.T1IP = 6;
    IPC1bits.T1IS = 0;
    T1CONbits.ON = 1;
}



/**
 * Main function
 */
int main() {
    char buffer[BUF_SIZE];

    NU32DIP_Startup();
    UART2_Startup();
    INA219_Startup();

    set_mode(IDLE);
    NU32DIP_GREEN = 1;
    NU32DIP_YELLOW = 1;

    __builtin_disable_interrupts();
    startPWM();
    timer2init();
    timer1init();
    initHBridgeDir();
    __builtin_enable_interrupts();

    while (1) {
        NU32DIP_ReadUART1(buffer, BUF_SIZE);
        NU32DIP_YELLOW = 1;

        switch (buffer[0]) {
            case 'a': { // Read ADC (raw counts)
                signed short adcval = readINA219(0x04);
                char m[50];
                sprintf(m, "%hu\r\n", adcval);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'b': { // Read current sensor (mA)
                float adcval = INA219_read_current();
                char m[50];
                sprintf(m, "%f\r\n", adcval);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'c': { // Read encoder count
                WriteUART2("a");
                while (!get_encoder_flag()) {}
                set_encoder_flag(0);
                char m[50];
                int p = get_encoder_count();
                sprintf(m, "%d\r\n", p);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'd': { // Read encoder degrees
                WriteUART2("a");
                while (!get_encoder_flag()) {}
                set_encoder_flag(0);
                char m[50];
                int p = get_encoder_count();
                float deg = 360.0 * ((float)p / (334.0 * 4.0));
                sprintf(m, "%f\r\n", deg);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'e': { // Reset encoder
                WriteUART2("b");
                break;
            }
            case 'f': { // Set PWM duty cycle
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                int n;
                sscanf(buffer, "%d", &n);
                if (n >= -100 && n <= 100) {
                    pwmDuty = n;
                    set_mode(PWM);
                }
                break;
            }
            case 'g': { // Set current gains
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f %f", &kp_mA, &ki_mA);
                break;
            }
            case 'h': { // Get current gains
                char m[50];
                sprintf(m, "%f %f\r\n", kp_mA, ki_mA);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'i': { // Set Position Gains
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f %f %f", &kp_deg, &ki_deg, &kd_deg);
                break;
            }
            case 'j': { // Get Position Gains
                char m[50];
                sprintf(m, "%f %f %f\r\n", kp_deg, ki_deg, kd_deg);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'k': // Test ITEST mode
                set_mode(ITEST);
                break;
            case 'l': { // Go to target angle
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &desired_deg);
                set_mode(HOLD);
                char msg[50];
                sprintf(msg, "Mode: HOLD, Target Angle: %.2f deg\n", desired_deg);
                NU32DIP_WriteUART1(msg);
                break;
            }
            case 'p': // Unpower motor
                set_mode(IDLE);
                break;
            case 'q': // Quit
                set_mode(IDLE);
                break;
            case 'r': { // Read mode
                char m[50];
                switch (get_mode()) {
                    case IDLE: sprintf(m, "IDLE\n"); break;
                    case PWM: sprintf(m, "PWM\n"); break;
                    case ITEST: sprintf(m, "ITEST\n"); break;
                    case HOLD: sprintf(m, "HOLD\n"); break;
                    case TRACK: sprintf(m, "TRACK\n"); break;
                }
                NU32DIP_WriteUART1(m);
                break;
            }
            default:
                NU32DIP_YELLOW = 0; // Error indication
                break;
        }
    }
    return 0;
}

/**
 * Initialize Timer2 for Current Control ISR (5 kHz)
 */
void timer2init() {
    T2CONbits.ON = 0;
    int prescaler = 1;
    PR2 = (NU32DIP_SYS_FREQ / (5000 * prescaler)) - 1;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    IPC2bits.T2IP = 5;
    IPC2bits.T2IS = 0;
    T2CONbits.ON = 1;
}





/**
 * Initialize PWM output (20 kHz)
 */
void startPWM() {
    ANSELBbits.ANSB15 = 0;
    RPB15Rbits.RPB15R = 0b101;

    OC1CONbits.OCM = 0b110;
    OC1CONbits.OCTSEL = 1;
    TMR3 = 0;
    OC1RS = 0;
    OC1R = 0;
    int prescaler = 1;
    PR3 = (NU32DIP_SYS_FREQ / (20000 * prescaler)) - 1;
    T3CONbits.ON = 1;
    OC1CONbits.ON = 1;
}

/**
 * Initialize H-Bridge direction control pin
 */
void initHBridgeDir() {
    TRISBbits.TRISB2 = 0;
    LATBbits.LATB3 = 0;
}

/**
 * Send captured data to Python for plotting
 */
void sendDataToPython(int *refArray, int *actualArray, int length) {
    char buffer[50];
    sprintf(buffer, "%d\n", length);
    NU32DIP_WriteUART1(buffer);

    for (int i = 0; i < length; i++) {
        sprintf(buffer, "%d %d\n", refArray[i], actualArray[i]);
        NU32DIP_WriteUART1(buffer);
    }
}
