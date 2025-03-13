#include "nu32dip.h"


#define NUMSAMPS 1000
#define PLOTPTS 200
#define DECIMATION 10

// ADC information
#define MAXPLOTPTS 1000
#define SAMPLE_TIME 6 
#define NUMSAMPS 1000
#define PWM_FREQ 20000


static volatile int Waveform[NUMSAMPS];
static volatile int ADCarray[PLOTPTS];
static volatile int REFarray[PLOTPTS];
static volatile int StoringData = 0;
static volatile float Kp = 0, Ki = 0;


void ADC_Startup();
unsigned int adc_sample_convert(int);
void makeWaveform();

int main(void)
{
  NU32DIP_Startup();
  ADC_Startup();
  OC1_Startup();  
  makeWaveform();
  Timer2Setup();

  char message[100];
  char command;
  int datapoints;
  int index;
  short data[MAXPLOTPTS];
  
  float kptemp = 0, kitemp = 0;
  int i = 0;

  

  while (1)
  {
    
    // wait for oscope.py to send a command and number of data points to collect
    NU32DIP_ReadUART1(message, 100);
    sscanf(message, "%c %d", &command, &datapoints);
    if (datapoints > MAXPLOTPTS){
      datapoints = MAXPLOTPTS;
    }
    __builtin_disable_interrupts();
    Kp = kptemp;
    // copy local variables to globals used by ISR
    Ki = kitemp;
    __builtin_enable_interrupts();
    StoringData = 1;
    // message to ISR to start storing data
    while (StoringData) {
    // wait until ISR says data storing is done
    ; // do nothing
    }

    // print the data back
    for (i=0; i<PLOTPTS; i++)
    {
      sprintf(message, "%d %d %d\r\n", PLOTPTS-i, ADCarray[i], REFarray[i]);
      NU32DIP_WriteUART1(message);
    }
  }
  return 0;
}

void makeWaveform() {
  int i = 0, center = (PR3+1)/2, A = PR3; // square wave, fill in center value and amplitude
  for (i = 0; i < NUMSAMPS; ++i) {
    if ( i < NUMSAMPS/2) {
      Waveform[i] = center + A;
      } else {
      Waveform[i] = center - A;
      }
    }
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) {
  static int counter = 0; // initialize counter once
  static int plotind = 0; // index for data arrays; counts up to PLOTPTS
  static int decctr = 0; // counts to store data one every DECIMATION
  static int adcval = 0;
  // initialize counter once
  // insert line(s) to set OC1RS
  if (StoringData) {
    decctr++;
    if (decctr == DECIMATION) {
      // after DECIMATION control loops,
    decctr = 0;
    adcval = adc_sample_convert(1); // Assuming you're sampling from pin 1

    // reset decimation counter
    ADCarray[plotind] = adcval;
    // store data in global arrays
    REFarray[plotind] = Waveform[counter];
    plotind++;
    // increment plot data index
    }
    if (plotind == PLOTPTS) {
    // if max number of plot points plot is reached,
    plotind = 0;
    // reset the plot index
    StoringData = 0;
    // tell main data is ready to be sent to MATLAB
    }
    }

  counter++;
  // add one to counter every time ISR is entered
  if (counter == NUMSAMPS) {
    counter = 0;
  }
  OC1RS = Waveform[counter];
  IFS0bits.T2IF = 0; // insert line to clear interrupt flag
}

void Timer2Setup() {
    T2CONbits.TCKPS = 0;     
    PR3 = 23999;           
    TMR2 = 0;                
    IPC2bits.T2IP = 5;      
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;      
    T2CONbits.ON = 1;        
}

void OC1_Startup() {
    TRISBbits.TRISB15 = 0;     // Set RB15 as output for OC1
    RPB15R = 0b0101;           // Map OC1 to RB15
    
    OC1CONbits.OCM = 0b110;    // Set OC1 to PWM mode (OCM = 6 for PWM mode)
    OC1CONbits.OCTSEL = 1;     // Use Timer3 as the timer source
    
    // Timer 3 setup
    T3CONbits.TCS = 0;         
    T3CONbits.TCKPS = 0b00;    
    T3CONbits.TGATE = 0;     
    
    // Calculate period for 20 kHz PWM frequency
    PR3 = NU32DIP_SYS_FREQ / PWM_FREQ - 1;
    OC1RS = PR3 * 3 / 4;       // 75% duty cycle
    OC1R = OC1RS;              // Set OC1R to the duty cycle
    
    T3CONbits.TON = 1;         // Enable Timer 3
    OC1CONbits.ON = 1;         // Enable OC1
}

void ADC_Startup(){
  ANSELAbits.ANSA1 = 1; // AN1 is an adc pin
  AD1CON3bits.ADCS = 1; // ADC clock period is Tad = 2*(ADCS+1)*Tpb =2*2*(1/48000000Hz) = 83ns > 75ns
  AD1CON1bits.ADON = 1;
}

unsigned int adc_sample_convert(int pin)
{
  unsigned int elapsed = 0, finish_time = 0;
  AD1CHSbits.CH0SA = pin;
  AD1CON1bits.SAMP = 1;
  elapsed = _CP0_GET_COUNT();
  finish_time = elapsed + SAMPLE_TIME;
  while (_CP0_GET_COUNT() < finish_time)
  {
    ;
  }
  AD1CON1bits.SAMP = 0;
  while (!AD1CON1bits.DONE)
  {
    ;
  }
  return ADC1BUF0;
}