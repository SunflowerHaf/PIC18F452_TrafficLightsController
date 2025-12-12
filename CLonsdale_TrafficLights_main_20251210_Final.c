/*
 * Filename: main.c
 * Author: Caitlin Lonsdale
 * Date: 10/12/2025
 * Revision Number: 7
 * Target Device: PIC18F452
 * Compiler: XC8
 * Summary: Logbook Work - Traffic Lights Main Code Final Version
*/

//Configuration statements
//CONFIG1H
#pragma config OSC = HS         //Oscillator selection - high speed enabled (0b010)
#pragma config OSCS = OFF       //Oscillator system clock switch disabled (main oscillator is source)

//CONFIG2L
#pragma config PWRT = OFF       //Power-up timer disabled
#pragma config BOR = ON         //Brown-out reset enabled
#pragma config BORV = 20        //Brown-out reset voltage = 2.0V

//CONFIG2H
#pragma config WDT = OFF        //Watchdog timer disabled
#pragma config WDTPS = 128      //Watchdog timer postscale select bits (1:128)

//CONFIG3H
#pragma config CCP2MUX = OFF    //CCP2 I/O multiplexed with RC1 disabled

//CONFIG4L
#pragma config STVR = ON        //Stack full/underflow RESET enabled
#pragma config LVP = OFF        //Low voltage ICSP disabled (port B used for I/O)

//CONFIG5L
#pragma config CP0 = OFF        //Block 0 (000200-001FFFh) not code protected
#pragma config CP1 = OFF        //Block 1 (002000-003FFFh) not code protected
#pragma config CP2 = OFF        //Block 2 (004000-005FFFh) not code protected
#pragma config CP3 = OFF        //Block 3 (006000-007FFFh) not code protected

//CONFIG5H
#pragma config CPB = OFF        //Boot Block (000000-0001FFh) not code protected
#pragma config CPD = OFF        //Data EEPROM not code protected

//CONFIG6L
#pragma config WRT0 = OFF       //Block 0 (000200-001FFFh) not write protected
#pragma config WRT1 = OFF       //Block 1 (002000-003FFFh) not write protected
#pragma config WRT2 = OFF       //Block 2 (004000-005FFFh) not write protected
#pragma config WRT3 = OFF       //Block 3 (006000-007FFFh) not write protected

//CONFIG6H
#pragma config WRTC = OFF       //Configuration registers (300000-3000FFh) not write protected
#pragma config WRTB = OFF       //Boot Block (000000-0001FFh) not write protected
#pragma config WRTD = OFF       //Data EEPROM not write protected

//CONFIG7L
#pragma config EBTR0 = OFF      //Block 0 (000200-001FFFh) not protected from Table Reads executed in other blocks
#pragma config EBTR1 = OFF      //Block 1 (002000-003FFFh) not protected from Table Reads executed in other blocks
#pragma config EBTR2 = OFF      //Block 2 (004000-005FFFh) not protected from Table Reads executed in other blocks
#pragma config EBTR3 = OFF      //Block 3 (006000-007FFFh) not protected from Table Reads executed in other blocks

//CONFIG7H
#pragma config EBTRB = OFF      //Boot Block (000000-0001FFh) not protected from Table Reads executed in other blocks


//Included files
#include <xc.h>                 //XC8 compiler specifics
#include <stdio.h>              //Standard C I/O library
#include <stdlib.h>             //Standard C library


//Definitions
#define _XTAL_FREQ 24000000     //Oscillator frequency 

#define SENS_CFG TRISA          //Direction setting for port A (traffic sensors)
#define PED_CFG TRISB           //Direction setting for port B (pedestrian I/O)
#define LED_CFG1 TRISD          //Direction setting for port D (traffic lights)
#define LED_CFG2 TRISE          //Direction setting for port E (additional pedestrian lights)

#define PED_BUT PORTBbits.RB0                                           //Pin RB0 for pedestrian button input
#define LED_OUT PORTD                                                   //LED outputs for pedestrian (1 and 2), main road, side road lights on port D
#define PED_GRN(val) {LATBbits.LATB2 = (val); LATEbits.LATE0 = (val);}  //Address RB2 and RE0 for pedestrian green lights 3 and 4
#define PED_RED(val) {LATBbits.LATB3 = (val); LATEbits.LATE1 = (val);}  //Address RB3 and RE1 for pedestrian red lights 3 and 4
#define WAIT_LED(val) LATB = (LATB & 0x0F) | ((val) ? 0xF0 : 0x00)      //Address upper nibble of port B (RB4-RB7) with 1111 or 0000


//TMR0 = 65536-(24e6/(4*1*1000))+2 = 59538 = 0xE892 for 16 bit operation, 1ms tick, and pre-scale 1:1
#define TMR0_HIPL 0xE8                  //High preload value for 1ms count on timer 0
#define TMR0_LOPL 0x92                  //Low preload value for 1ms count on timer 0

//ADC = 1.4V threshold / 5V Vref * 255 (8-bit operation) = 71 (floor)
#define ADC_THRESH  71                  //Threshold value for opto-resistor road sensor ADC output
#define AVG_LENGTH 4                    //Total number of samples for moving average summation (ADC smoothing)

//TMR1 = 65536-(24e6/(4*1*40000)) = 65386 = 0xFF68 for 16 bit operation, 25us tick (t>>Tad), and pre-scale 1:1
#define TMR1_HIPL 0xFF                  //High pre-load value for 25us count on timer 1
#define TMR1_LOPL 0x68                  //Low pre-load value for 25us count on timer 1

//TMR2 = 0, PR2 = (24e6/(4*16*15*100))-1 = 249 = 0xF9 for 10ms tick (accumulated for 20ms), pre-scale 1:16, post-scale 1:15
#define TMR2_PR2 0xF9                   //Timeout value for 10ms (accumulated for 20ms) count on timer 2
#define TMR2_OF 2                       //Accumulation value for timer 2 to generate a 20ms count


//Prototype functions
void setup(void);                       //Hardware initialisation function
void main_road_grn(void);               //Main Green function (state zero)
void main_road_yel(void);               //Main Yellow function (state one)
void main_road_red(void);               //Main Red function (state two)
void main_road_redyel(void);            //Main Red Yellow function (state three)
void side_road_grn(void);               //Side Green function (state four)
void side_road_yel(void);               //Side Yellow function (state five)
void side_road_red(void);               //Side Red function (state six)
void side_road_redyel(void);            //Side Red Yellow function (state seven)
void ped_cross_grn(void);               //Pedestrian Green function (state eight)
void ped_cross_red(void);               //Pedestrian Red function (state nine)


//Global variables                        
typedef enum {
    state_zero,                         //Main Green state - main green, side red, pedestrian red
    state_one,                          //Main Yellow state - main yellow, side red, pedestrian red
    state_two,                          //Main Red state - main red, side red, pedestrian red
    state_three,                        //Main Red Yellow state - main red+yellow, side red, pedestrian red
    state_four,                         //Side Green state - main red, side green, pedestrian red
    state_five,                         //Side Yellow state - main red, side yellow, pedestrian red
    state_six,                          //Side Red state - main red, side red, pedestrian red
    state_seven,                        //Side Red Yellow state - main red, side red+yellow, pedestrian red
    state_eight,                        //Pedestrian Green state - main red, side red, pedestrian green
    state_nine                          //Pedestrian Red state - main red, side red, pedestrian red
} state_machine_value;
state_machine_value state = state_two;  //Global declaration of state for switch case

typedef union {                         //Create new union data type for bit-field
    struct {                            //Create bit-field for four single-bit flags (1 byte format)
        unsigned one_second:1;          //Flag for 1s overflow
        unsigned ped_button:1;          //Flag for pedestrian button - high if pressed
        unsigned side_sensor:1;         //Flag for side road traffic sensor (west OR east)
        unsigned main_sensor:1;         //Flag for main road traffic sensor (north OR south)
        unsigned ped_request:1;         //Flag for latched pedestrian crossing request
        unsigned side_request:1;        //Flag for latched side road request
        unsigned main_request:1;        //Flag for latched main road request
        unsigned adc_request:1;         //Flag from ADC ISR to request processing in main
    };
    uint8_t inputs;                     //Full byte access to union
} input_flags;                          
input_flags flags = {0};                //Global declaration of flags union 

uint16_t ms_counter = 0;                //Counter for 1 millisecond tick
uint8_t state_timer = 0;                //Counter for elapsed seconds in current state
uint8_t tmr2_count = 0;                 //Counter for timer 2 overflow

uint8_t adc_sample = 0;                     //Holds newest ADC output value (reads from SFR)
uint8_t adc_channel = 0;                    //ADC channels index(RA)0 to (RA)3 for road traffic sensors 
uint8_t adc_count[4] = {0,0,0,0};           //Summation counter for each ADC channel, initialised to zero
uint8_t adc_samples[4][AVG_LENGTH] = {0};   //Four most recent samples for each ADC channel, used as a circular buffer
uint8_t adc_index[4] = {0};                 //Indexing array for the ADC samples
uint16_t adc_sum[4] = {0};                  //Summation value for the ADC channel moving average
uint16_t adc_avg[4] = {0};                  //Calculated ADC channel average value



/* Low priority interrupt function
 * This interrupt writes the ADC register value into a local sample, and sets a flag request
 * to handle the ADC post-processing in the main loop, before incrementing to the next channel (0-3).
 * The interrupt uses a 20us overflow on timer 1 for acquisition delay between switching ADC 
 * channels and the new readings. The interrupt also handles a falling-edge trigger on RB0
 * (active low) for the pedestrian button. A 20ms overflow on timer 2 is used for the button
 * de-bounce, and flags are set to be read in the FSM. 
 * Note: Request flags and the disabling of the external interrupt during de-bounce time are used as 
 * methods to mitigate against the error or multiple clicks or sticking of the button.
*/
void __interrupt(low_priority) LowInt(void) {   //Low priority interrupt for ADC, RB0 button input, and associated timers
    //ADC interrupt
    if(PIE1bits.ADIE && PIR1bits.ADIF){         //Test for enabled and high on ADC interrupt flag
        PIR1bits.ADIF = 0;                      //Reset ADC interrupt flag
        adc_sample = ADRESH;                    //Write value in ADC high result register to sample
        flags.adc_request = 1;                  //Set request flag to process ADC output in main function
        
        adc_channel++;                  //Increment to next ADC channel
        if (adc_channel > 3) {
            adc_channel = 0;            //If channel (RA)3 completed, return to channel (RA)0
        }
        ADCON0bits.GO = 0;              //Stop ADC running to switch channel
        ADCON0bits.CHS = adc_channel;   //Set new ADC channel in register (3-bit value)
        
        PIR1bits.TMR1IF = 0;            //Reset timer 1 interrupt flag
        TMR1H = TMR1_HIPL;              //Set high pre-load value for 20us count
        TMR1L = TMR1_LOPL;              //Set low pre-load value for 20us count
        T1CONbits.TMR1ON = 1;           //Start timer 1 for acquisition time 25us (ADC restarted in timer 1 interrupt)
    }
    
    //Timer 1 interrupt
    if(PIE1bits.TMR1IE && PIR1bits.TMR1IF) {    //Test for enabled and high on timer 1 interrupt flag
        PIR1bits.TMR1IF = 0;                    //Reset timer 1 interrupt flag
        T1CONbits.TMR1ON = 0;                   //Stop timer 1 running
        ADCON0bits.GO = 1;                      //Start ADC conversion (new channel, acquisition time complete)
    }
    
    //Button (RB0) interrupt
    if(INTCONbits.INT0IE && INTCONbits.INT0IF) {    //Test for enabled and high on external 0 (pedestrian button) interrupt flag
        INTCONbits.INT0IE = 0;                      //Disable external interrupt while button value read (avoids re-trigger by pressing or switch-bounce)
        INTCONbits.INT0IF = 0;                      //Reset external 0 (pedestrian button) interrupt flag
        PIR1bits.TMR2IF = 0;                        //Reset timer 2 interrupt flag
        tmr2_count = 0;                             //Initialise counter for timer 2 overflow
        TMR2 = 0;                                   //Set pre-load value to zero (minimum))
        PR2 = TMR2_PR2;                             //Set timeout value for 10ms count
        T2CONbits.TMR2ON = 1;                       //Start timer 2 for de-bounce time 20ms (pedestrian flag set in timer 2 interrupt)
    }

    //Timer 2 interrupt
    if(PIE1bits.TMR2IE && PIR1bits.TMR2IF) {    //Test for enabled and high on timer 2 interrupt flag
        PIR1bits.TMR2IF = 0;                    //Reset timer 2 interrupt flag
        tmr2_count++;                           //Increment 2x counter by 1
        
        if (tmr2_count >= TMR2_OF) {    //If button de-bounce time (2*10ms = 20ms) has passed
            T2CONbits.TMR2ON = 0;       //Stop timer 1 running to check button
            tmr2_count = 0;             //Reset counter for timer 2 overflow
            if (!PED_BUT) {             //Active low button input
                flags.ped_button = 1;   //Set pedestrian button flag if button pressed
                flags.ped_request = 1;  //If pedestrian button flag (true) set latched pedestrian request also
            }
            INTCONbits.INT0IE = 1;      //Restart external interrupt on pedestrian button
        }
    }
}

/* High priority interrupt function
 * This interrupt sets timer 0 to overflow every millisecond. This is used to generate
 * a 1 second count, used for the FSM timings. 
*/
void __interrupt(high_priority) HighInt(void) {     //High priority interrupt for timer 0 
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {    //Test for enabled and high on timer 0 interrupt flag
        TMR0H = TMR0_HIPL;                          //Set high pre-load value for 1ms count
        TMR0L = TMR0_LOPL;                          //Set low pre-load value for 1ms count
        INTCONbits.TMR0IF = 0;                      //Reset timer 0 interrupt flag
        
        ms_counter++;                   //Increment 1ms counter by 1
        if (ms_counter >= 1000) {       //1 second elapsed
            ms_counter = 0;             //Reset counter to 0
            state_timer++;              //Increment elapsed seconds in current state by 1
            flags.one_second = 1;       //Set 1s counter flag - REVIEW
        }
        T0CONbits.TMR0ON = 1;           //Restart timer 0
    }
    LowInt();                           //Start low-priority ISR
    return;                             //Return from high priority interrupt
}

 

//Main function
void main(void){
    setup();                            //Call setup function to initialise SFRs and port values
    T0CONbits.TMR0ON = 1;               //Start timer 0
    ADCON0bits.GO = 1;                  //Start ADC

    while(1){
        /* This program for PIC 18 hardware implements a traffic lights finite state machine.
         * The FSM controls the FSM controls main road, side road, and pedestrian traffic lights, 
         * based on timed state transitions and external inputs from traffic sensors (through the ADC)
         * and a pedestrian activated button. The program also carries out smoothing of the ADC output
         * (from the traffic sensor input) in the form of a 4-point moving average, before comparing 
         * the result to the vehicle voltage in order to set the program flags read by the FSM.
        */
        PED_GRN(LATDbits.LATD0);        //Mirror pedestrian green lights 1 and 2 (RD0) on 3 (RB2) and 4 (RE0)
        PED_RED(LATDbits.LATD1);        //Mirror pedestrian red lights 1 and 2 (RD1) on 3 (RB3) and 4 (RE1)
        
        if (flags.adc_request) {        //Run if ADC has output a value
            flags.adc_request = 0;      //Reset ADC processing request flag
            
            adc_sum[adc_channel] -= adc_samples[adc_channel][adc_index[adc_channel]];   //Subtract the oldest ADC channel sample from the averaging summation
            adc_samples[adc_channel][adc_index[adc_channel]] = adc_sample;              //Overwrite the oldest sample in index (4) with the newest sample
            adc_sum[adc_channel] += adc_sample;                                         //Add the newest ADC channel sample to the averaging summation
        
            adc_index[adc_channel]++;                       //Increment the index of the averaging summation
            if (adc_index[adc_channel] >= AVG_LENGTH) {     //Compare index value to maximum length
                adc_index[adc_channel] = 0;                 //If index 3 reached, return to index 0
            }
            if (adc_count[adc_channel] < AVG_LENGTH){       //Compare number of valid samples to required length
                adc_count[adc_channel]++;                   //Increment sample counter until required length
            }
            adc_avg[adc_channel] = adc_sum[adc_channel] / 4;    //Calculate the moving average using the ADC samples
        
            if (adc_channel == 0 || adc_channel == 2) {
                flags.main_sensor |= (adc_avg[adc_channel] >= ADC_THRESH);  //Set main road sensor flag to true/false for vehicle detection based on threshold comparison
            }
            if (flags.main_sensor) {flags.main_request = 1;}                //If vehicle detected on main road (true) set latched main road request also
            if (adc_channel == 1 || adc_channel == 3) {
                flags.side_sensor |= (adc_avg[adc_channel] >= ADC_THRESH);  //Set side road sensor flag to true/false for vehicle detection based on threshold comparison
            }
            if (flags.side_sensor) {flags.side_request = 1;}                //If vehicle detected on side road (true) set latched side road request also
        }
        
        if (flags.one_second) {         //Run FSM on each 1s tick
            flags.one_second = 0;       //Reset 1 second flag
            
            switch(state) {                                 //Switch case for FSM implementation
            case state_zero: main_road_grn(); break;        //Main Green function (state zero)
            case state_one: main_road_yel(); break;         //Main Yellow function (state one)
            case state_two: main_road_red(); break;         //Main Red function (state two)
            case state_three: main_road_redyel(); break;    //Main Red Yellow function (state three)
            case state_four: side_road_grn(); break;        //Side Green function (state four) 
            case state_five: side_road_yel(); break;        //Side Yellow function (state five)
            case state_six: side_road_red(); break;         //Side Red function (state six)
            case state_seven: side_road_redyel(); break;    //Side Red Yellow function (state seven)
            case state_eight: ped_cross_grn(); break;       //Pedestrian Green function (state eight)
            case state_nine: ped_cross_red(); break;        //Pedestrian Red function (state nine)
            default: 
                LED_OUT = 0b00000000;   //All LEDs off for error state
                break;                  //Exit switch statement
            }                           //End of the switch statement
        }                               //End of flag if statement
    }                                   //End of the while loop
}                                       //End of the main function
            

/* Setup function
     * This setup function initialises all the PIC hardware to run the FSM.
     * The function sets the I/O direction of pins using TRIS registers, and writes
     * initial values to output pins. It also writes the SFR values for global interrupts,
     * a port B external interrupt (pedestrian button), timer 0 interrupt (state machine timer),
     * the ADC (for converting analogue sensor inputs), timer 1 (ADC acquisition delay),
     * and timer 2 (button de-bounce).
     */
void setup(void) {
    //Ports setup
    SENS_CFG = 0b1111111;               //Set all inputs for opto-resistor traffic sensors
    PED_CFG = 0b00000001;               //Set RB0 input for pedestrian button, RB2-RB3 outputs for pedestrian lights 3, RB4-RB7 outputs for wait lights
    LATB &= 0x0F;                       //Initialise pedestrian wait lights to off by clearing port B upper nibble  
    LED_CFG1 = 0b00000000;              //Set all outputs for LED traffic lights (including pedestrian lights 1 and 2)
    LED_OUT = 0b10010010;               //Initialise LED traffic lights to state 2 outputs
    LED_CFG2 = 0b00;                    //Set RE0-RE1 outputs for pedestrian lights 4
    PED_GRN(0);                         //Initialise mirrored (from LED_OUT) pedestrian green lights to off
    PED_RED(0);                         //Initialise mirrored (from LED_OUT) pedestrian red lights to off
    
    //Interrupts setup
    INTCON = 0b11110000;                //Set interrupts active - global enable (GIE), peripheral enable (PEIE), timer 0 enable (TMR0IF) and external 0 enable (INT0IF))
    RCONbits.IPEN = 1;                  //Enable interrupt prioritisation
    INTCON2bits.RBIP = 0;               //Set port B interrupt as low priority
    INTCON2bits.INTEDG0 = 0;            //Set external interrupt 0 as falling edge triggered (for active low RB0)
    INTCON2bits.RBPU = 0;               //Activate port B weak global pull-up (RB0 active low input for pedestrian button)
    INTCONbits.INT0IF = 0;              //Initialise external interrupt flag to zero
    
    //Timer 0 setup
    T0CON = 0b00011000;                 //Set timer 0 16-bit mode, internal clock, rising edge, pre-scale 1:1
    TMR0H = TMR0_HIPL;                  //Set high pre-load value for 1ms count
    TMR0L = TMR0_LOPL;                  //Set low pre-load value for 1ms count
    INTCON2bits.TMR0IP = 1;             //Set timer 0 interrupt as high priority
    INTCONbits.TMR0IF = 0;              //Initialise timer 0 interrupt flag to zero
    
    //ADC setup
    ADCON0 = 0b10000001;                //Set ADC power up (not yet running), selects AN0 channel, and Tosc=64
    ADCON1 = 0b01000010;                //Set all port A analogue channels (not port E), left-justified (read ADRESH), and Tosc=64
    PIE1bits.ADIE = 1;                  //Enable ADC interrupt
    IPR1bits.ADIP = 0;                  //Set ADC interrupt as low priority
    PIR1bits.ADIF = 0;                  //Initialise ADC interrupt flag to zero 
    
    //Timer 1 setup
    T1CON = 0b10000000;                 //Set timer 1 16-bit mode, internal clock (Tosc=4), pre-scale 1:1
    TMR1H = TMR1_HIPL;                  //Set high pre-load value for 25us count
    TMR1L = TMR1_LOPL;                  //Set low pre-load value for 25us count
    PIE1bits.TMR1IE = 1;                //Enable timer 1 interrupt
    IPR1bits.TMR1IP = 0;                //Set timer 1 interrupt as low priority
    PIR1bits.TMR1IF = 0;                //Initialise timer 1 interrupt flag to zero

    //Timer 2 setup
    T2CON = 0b01110011;                 //Set timer 2 pre-scale 1:16, post-scale 1:15
    PR2 = TMR2_PR2;                     //Set timeout value for 10ms count
    PIE1bits.TMR2IE = 1;                //Enable timer 2 interrupt
    IPR1bits.TMR2IP = 0;                //Set timer 2 interrupt as low priority
    PIR1bits.TMR2IF = 0;                //Initialise timer 2 interrupt flag to zero 
}
 
/* Main Green function (state zero)
 * This function turns on the pedestrian crossing red LED, the side road red LED, 
 * the main road green LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 10s
 * then sets the next state transition.
 */
void main_road_grn(void) {      
    LED_OUT = 0b00110010;                   //Traffic LEDs output values for main road green
    WAIT_LED(flags.ped_request);            //Set the wait LEDs to the boolean value set by the pedestrian button input
    LATB = flags.ped_request ? 0xf0 : 0x00; //Error mitigation case
    
    if (state_timer >= 10) {
        if (flags.side_request || flags.ped_request) {
            state = state_one;          //If 10s passed and side road or pedestrian flags set move to state one
        }
        else {
            state = state_zero;         //If 10s passed and no or only main road flag stay at state zero
        }
        flags.main_request = 0;         //Reset main road latched request as action completed
        state_timer = 0;                //Reset state timer for moving to next state
    }
}       

/* Main Yellow function (state one)
 * This function turns on the pedestrian crossing red LED, the side road red LED, 
 * the main road yellow LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 3s
 * then sets the next state transition.
 */
void main_road_yel(void) {
    LED_OUT = 0b01010010;               //Traffic LEDs output values for main road yellow
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 3) {
        state = state_two;              //If 3s passed move to state two
        state_timer = 0;                //Reset state timer for moving to next state
    }
}       

/* Main Red function (state two) 
 * This function turns on the pedestrian crossing red LED, the side road red LED, 
 * the main road red LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 3s
 * then sets the next state transition.
 */
void main_road_red(void) {
    LED_OUT = 0b10010010;               //Traffic LEDs output values for main road red
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 2) {
        if (flags.side_request) {
            state = state_seven;        //If 2s passed and side road flag set move to state seven 
        }
        else if (flags.ped_request) {
            state = state_eight;        //If 2s passed and pedestrian flag set move to state eight
        }
        else {
            state = state_three;        //If 2s passed and no or only main road flag set move to state three
        }
    state_timer = 0;                    //Reset state timer for moving to next state
    }
}       

/* Main Red Yellow function (state three)
 * This function turns on the pedestrian crossing red LED, the side road red LED, 
 * the main road red and yellow LEDs, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 2s
 * then sets the next state transition.
 */
void main_road_redyel(void) {
    LED_OUT = 0b11010010;               //Traffic LEDs output values for main road red and yellow
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 2) {
        state = state_zero;             //If 2s passed move to state zero
        state_timer = 0;                //Reset state timer for moving to next state
    }
}    

/* Side Green function (state four)
 * This function turns on the pedestrian crossing red LED, the side road green LED, 
 * the main road red LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 10s
 * then sets the next state transition.
 */
void side_road_grn(void) {
    LED_OUT = 0b10000110;               //Traffic LEDs output values for side road green
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >=10) {
        if (flags.main_request || !flags.side_request || flags.ped_request) {
            state = state_five;         //If 10s passed and main, pedestrian or no flags set move to state five
        }
        else {
            state = state_four;         //If 10s passed but only side sensor flag set stay at state four  
        }
        flags.side_request = 0;         //Reset side road latched request as action completed
        state_timer = 0;                //Reset state timer for moving to next state
    }
}       

/* Side Yellow function (state five) 
 * This function turns on the pedestrian crossing red LED, the side road yellow LED, 
 * the main road red LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 3s
 * then sets the next state transition.
 */
void side_road_yel(void) {
    LED_OUT = 0b10001010;               //Traffic LEDs output values for side road yellow
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 3) {
        state = state_six;              //If 3s passed move to state six
        state_timer = 0;                //Reset state timer for moving to next state
    }
}       

/* Side Red function (state six)
 * This function turns on the pedestrian crossing red LED, the side road red LED, 
 * the main road red LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 2s
 * then sets the next state transition.
 */
void side_road_red(void) {
    LED_OUT = 0b10010010;               //Traffic LEDs output values for side road red
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 2) {             
        if (flags.ped_button) {     
            state = state_eight;        //If 2s passed and pedestrian flag set move to state eight
        }
        else if (!flags.main_sensor && flags.side_sensor) {
            state = state_seven;        //If 2s passed and no main and side road flags set move to state seven
        }
        else {                      
            state = state_three;        //If 2s passed and main road (or no), no side road, and no pedestrian flags set move to state three
        }
        state_timer = 0;                //Reset state timer for moving to next state
    }
}      

/* Side Red Yellow function (state seven)
 * This function turns on the pedestrian crossing red LED, the side road red and yellow LEDs, 
 * the main road red LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 2s
 * then sets the next state transition.
 */
void side_road_redyel(void) {
    LED_OUT = 0b10011010;               //Traffic LEDs output values for side road red and yellow
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 2) {
        state = state_four;             //If 2s passed move to state four
        state_timer = 0;                //Reset state timer for moving to next state
    }
}    

/* Pedestrian Green function (state eight)
 * This function turns on the pedestrian crossing green LED, the side road yellow LED, 
 * the main road red LED, and sets the pedestrian WAIT light off to tell pedestrians to
 * cross (all other LEDs off). The function runs for 10s then sets the next state transition.
 */
void ped_cross_grn(void) {
    LED_OUT = 0b10010001;               //Traffic LEDs output values for pedestrian green
    LATB &= 0x0F;                       //Set pedestrian wait lights off by clearing port B upper nibble
    
    if (state_timer >= 10) {
        state = state_nine;             //If 10s passed move to state nine
        flags.ped_button = 0;           //Reset pedestrian flag to zero as pedestrians released on crossing
        flags.ped_request = 0;          //Reset pedestrian latched request as action completed
        state_timer = 0;                //Reset state timer for moving to next state
    }
}       

/* Pedestrian Red function (state nine)
 * This function turns on the pedestrian crossing red LED, the side road red LED, 
 * the main road red LED, and sets the pedestrian WAIT light according to the
 * pedestrian button reading (all other LEDs off). The function runs for 7s, for safety,
 * then sets the next state transition. 
 */
void ped_cross_red(void) {
    LED_OUT = 0b10010010;               //Traffic LEDs output values for pedestrian red
    WAIT_LED(flags.ped_request);        //Set the wait LEDs to the boolean value set by the pedestrian button input
    
    if (state_timer >= 7) {
        state = state_three;            //If 7s passed move to state three
        state_timer = 0;                //Reset state timer for moving to next state
    }
}       


