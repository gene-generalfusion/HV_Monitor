/*
 * File:   17-0323 HV Monitor Board.c
 * Author: jake.hobbis
 *
 * Created on January 6, 2023, 11:25 AM
 */

#include <xc.h>
#include "stdint.h"
//#include "stdio.h"



// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT    // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF               // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF              // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON               // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF                 // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF              // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF               // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF              // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)    

// Constants
#define _XTAL_FREQ    4000000           // Oscillator frequency for _delay()
//#define _XTAL_FREQ    1000000           // Oscillator frequency for _delay()

#define TMR1_HIGH_LOAD  0x85            // Timer 1 load for ~250ms
#define TMR1_LOW_LOAD   0xED    

#define PROBE_LOW_THRESHOLD 200        // Capacitor charged threshold for status LEDs

// Variables
uint8_t Probe_Volt_Low = 0;
uint8_t Probe_Volt_High = 0;

uint8_t Data_Out_Low = 0;
uint8_t Data_Out_High = 0;

uint8_t Low_Battery_Flag = 0;
uint8_t Sleep_Flag = 0;

uint8_t RXData = 0;

// IO Shortcuts
#define GRN_LED_ON       PORTCbits.RC0 = 1
#define GRN_LED_OFF      PORTCbits.RC0 = 0

#define YEL_LED_ON       PORTAbits.RA6 = 1
#define YEL_LED_OFF      PORTAbits.RA6 = 0

#define RED_LED_ON       PORTAbits.RA7 = 1
#define RED_LED_OFF      PORTAbits.RA7 = 0

#define EN_AMP_ON        PORTCbits.RC6 = 1
#define EN_AMP_OFF       PORTCbits.RC6 = 0

#define LVD_EN_ON        PORTAbits.RA3 = 1
#define LVD_EN_OFF       PORTAbits.RA3 = 0

#define OPT_OUT_ON       PORTCbits.RC5 = 1
#define OPT_OUT_OFF      PORTCbits.RC5 = 0

// Perform an analog conversion on a channel
// Saves the 10bit result
void ADC_Read(uint8_t channel)
{
    
    // Make an ADC conversion
    ADCON0bits.CHS = (0x0F&channel);            // Select ADC channel, only a 4 bit number
    __delay_ms(5);                              // Acquisition delay, Required after selecting channel
    ADCON0bits.GO_DONE = 1;                     // Start ADC conversion
                        
    while( ADCON0bits.GO_DONE == 1 ){}          // Wait for conversion to finish
    
    Probe_Volt_Low = ADRESL;                    // Get lower result
    Probe_Volt_High = ADRESH;                   // Get the upper result
            
}

// Initialize
void Init_PIC(void)
{
    // Initialize IO Ports
    ANSEL = 0b00000011;         // Analog inputs 0,1
    TRISA = 0b00000011;         // Set RA0, RA1 as inputs, rest as outputs
    ANSELH = 0b00000000;        // No analog pins on portB
    TRISB = 0b00000000;         // Set all of portB to outputs
    OPTION_REGbits.nRBPU = 1;   // Disable pull ups          
    TRISC = 0b00010000;         // Set RC4(SCK) to input, rest as outputs    
    
    // Initialize IO State, set low to save power
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    LVD_EN_ON;                  // Disable battery check
    
    // Initialize Internal Clock
    OSCCONbits.SCS = 0;         // Using internal oscillator
    OSCCONbits.IRCF = 0b110;    // Setting clock frequency to 4Mhz
    //OSCCONbits.IRCF = 0b100;    // Setting clock frequency to 1Mhz
    
    // Initialize ADC Module
    ADCON1bits.ADFM = 1;        // Right justify ADC conversion result
    ADCON1bits.VCFG1 = 0;       // Set references to VSS and VDD
    ADCON1bits.VCFG0 = 0;
    ADCON0bits.ADCS = 1;        // Set the ADC conversion clock to Fosc/8
    //ADCON0bits.ADCS = 0b10;        // Set the ADC conversion clock to Fosc/32
    ADCON0bits.ADON = 0;        // Disable ADC to save power
            
    // Initialize MSSP Module for Master SPI
    SSPCON = 0;                 // Clear control register, disable serial port
    SSPCONbits.CKP = 0;         // Clock idle state is low
    SSPCONbits.SSPM = 0b0011;   // Set to SPI master mode, clock = TMR2 output/2
    //SSPCONbits.SSPM = 0b0000;   // Set to SPI master mode, clock = Fosc/4
    //SSPCONbits.SSPM = 0b0001;   // Set to SPI master mode, clock = Fosc/16
    SSPSTATbits.SMP = 0;        // Input data sampled at middle of data output time
    SSPSTATbits.CKE = 1;        // Transmit data on falling edge of SCK
       
    // Initialize Timer 2
    T2CON = 0;
    TMR2 = 0;
    T2CONbits.T2CKPS = 0b10;    // 1:16 pre scaler
    
    PR2 = 0xF8;                 // Load the timer compare register, (1/(1Mhz) * 16 * 255)*2 = 8.16ms
    //PR2 = 0x177;                 // Load the timer compare register, (1/(1Mhz) * 32 * 255)*2 = 8.16ms
    //T2CONbits.TOUTPS = 0b10;    //added by GL for post scaling factor
    T2CONbits.TMR2ON = 1;       // Turn timer 2 on   
    
    // Initialize comparator 1 module
    CM1CON0bits.C1ON = 0;       // Disable comparator
    CM1CON0bits.C1OE = 0;       // Comparator output is internal only
    CM1CON0bits.C1POL = 0;      // Comparator logic is not inverted
    CM1CON0bits.C1R = 1;        // Comparator reference C1Vref, C1Vref connected to C1Vin+
    CM1CON0bits.C1CH = 0b01;    // Comparator channel set to C12IN1-, conected to C1Vin-
    CM2CON1bits.C1RSEL = 0;     // Set comparator ref to the fixed Vref
    SRCONbits.FVREN = 1;        // Enable the fixed Vref
    
    // Initialize Timer 1
    T1CON = 0;                  // Timer 1 set to internal clock (FOSC/4), timer 1 stopped
    T1CONbits.T1CKPS=0b11;      // Timer 1 prescale 1:8
    TMR1H = TMR1_HIGH_LOAD;     // Load Timer 1
    TMR1L = TMR1_LOW_LOAD;
    
    // Initialize Watchdog timer (31kHz)
    WDTCONbits.SWDTEN = 0;      // Software disable the watchdog timer
    WDTCONbits.WDTPS = 0b0010;  // Set watchdog timer prescale to 1:256
    OPTION_REGbits.PSA = 1;     // Assign second prescaler to WDT
    OPTION_REGbits.PS = 0b111;  // Set second prescaler to 1:128 for WDT
    asm("CLRWDT");              // Clear the watchdog timer    
    
    // Initialize Interrupts
    INTCON = 0;                 // Disable interrupts, clear all flags
    INTCONbits.PEIE = 1;        // Enable peripheral interrupts
    PIE1 = 0;
    PIE1bits.TMR1IE = 1;        // Enable Timer 1 overflow interrupt
    PIR1 = 0;                   // Clear timer 1 overflow flag
    INTCONbits.GIE = 1;         // Enable all interrupts    
 
}

void main(void) 
{

    Init_PIC();
                   
    // Main Loop
    while(1)
    {         
        WDTCONbits.SWDTEN = 0;  // Software disable the watchdog timer
        Sleep_Flag = 0;         // Reset sleep flag
        
        // Check Battery Voltage
        LVD_EN_OFF;             // Enable low battery check
        CM1CON0bits.C1ON = 1;   // Enable comparator
        __delay_us(110);        // Wait settling time
        if( CM1CON0bits.C1OUT == 0 ) // Check comparator output
            Low_Battery_Flag = 0;
        else
            Low_Battery_Flag = 1;
        LVD_EN_ON;             // Disable low battery check
        CM1CON0bits.C1ON = 0;  // Disable comparator
        
        // Measure probe voltage
        EN_AMP_ON;              // Enable analog power supply
        ADCON0bits.ADON = 1;    // Enable ADC 
        __delay_ms(15);         // Wait settling time
        ADC_Read(0);            // Read analog channel 0, probe voltage
        EN_AMP_OFF;             // Disable analog power supply
        ADCON0bits.ADON = 0;    // Disable ADC
        
        // Set status LEDs
        GRN_LED_ON;             // Turn heartbeat status lED on
        //YEL_LED_ON; //test 
        if( Low_Battery_Flag )
            YEL_LED_ON;         // Turn low battery status LED on if battery is low
        if( Probe_Volt_Low > PROBE_LOW_THRESHOLD )
            RED_LED_ON;             // Turn the charge status LED on
        T1CONbits.TMR1ON = 1;   // Start Timer 1
        
        // Format the data to send
        Data_Out_Low = 0x00;    
        Data_Out_High = 0x80;
        if( Low_Battery_Flag )
            Data_Out_High = Data_Out_High | 0x20;
        Data_Out_Low = ( 0xFC & ( Probe_Volt_Low << 2 ) ) | 0x01;      
        Data_Out_High  = Data_Out_High | ( 0x0C & ( Probe_Volt_High << 2) ) | ( 0x03 & ( Probe_Volt_Low >> 6 ) );
        
        // Send the data using SPI
        
//        // Send the start bit
//        OPT_OUT_ON;
//        __delay_ms(8);        
        
        // Send the first byte
        SSPCONbits.SSPEN = 1;               // Enable serial port
        SSPSTATbits.BF = 0;
        RXData = SSPBUF;
        T2CONbits.TMR2ON = 0;               // Turn timer 2 off, per errata data sheet   
        TMR2 = 0;                           // Clear timer 2, per errata data sheet
        SSPBUF = Data_Out_High;            // Send the first byte
        T2CONbits.TMR2ON = 1;               // Turn timer 2 on, pere errata data sheet   
        while( SSPSTATbits.BF == 0 ){}      // Wait for transmission
        SSPCONbits.SSPEN = 0;               // Disable serial port
     
//        // Send the stop bit
//        OPT_OUT_OFF;
//        __delay_ms(8);
//        
//        // Send the start bit
//        OPT_OUT_ON;
//        __delay_ms(8);
        
        // Send the second byte
        SSPCONbits.SSPEN = 1;               // Enable serial port
        SSPSTATbits.BF = 0;
        RXData = SSPBUF;
        T2CONbits.TMR2ON = 0;               // Turn timer 2 off, per errata data sheet   
        TMR2 = 0;                           // Clear timer 2, per errata data sheet
        SSPBUF = Data_Out_Low;             // Send the second byte
        T2CONbits.TMR2ON = 1;               // Turn timer 2 on, pere errata data sheet   
        while( SSPSTATbits.BF == 0 ){}      // Wait for transmission  
        SSPCONbits.SSPEN = 0;               // Disable serial port
         
//        // Send the stop bit
//        OPT_OUT_OFF;
//        __delay_ms(8);
        
        while( Sleep_Flag == 0 ){}          // Wait for LEDs to turn off
        
        WDTCONbits.SWDTEN = 1;              // Software enable the watchdog timer
        
        asm("CLRWDT");                      // Clear the watchdog timer 
        asm("SLEEP");                       // Go into sleep
        asm("NOP");
                
    }
}

// Interrupt Service Routine
__interrupt() void ISR(void) 
{
    INTCONbits.GIE = 0;             // Disable all interrupts
        
    // Timer 1 Interrupt, Turn off the status LEDs
    if(PIR1bits.TMR1IF == 1)
    {
        T1CONbits.TMR1ON = 0;       // Stop Timer 1
        PIR1bits.TMR1IF = 0;        // Clear timer 1 overflow flag
        
        //GRN_LED_OFF;                // Turn the status LEDs off
        //YEL_LED_OFF;
        //RED_LED_OFF;
                        
        TMR1H = TMR1_HIGH_LOAD;     // Load Timer 1
        TMR1L = TMR1_LOW_LOAD;
        
        Sleep_Flag = 1;             // Set the sleep flag, can go into sleep
    }
        
    INTCONbits.GIE = 1;             // Enable all interrupts
    
    return;
}
