
// PIC12F629 Configuration Bit Settings

// 'C' source line config statements

// CONFIG
/*
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // GP3/MCLR pin function select (GP3/MCLR pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
*/

#pragma config WDTE=OFF , BOREN=OFF , PWRTE=ON , MCLRE=OFF , FOSC=INTRCIO

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 4e6
#define LED1 GP5

void main(void) {
    
    TRISIO = 0b00001100;
    GPIO = 0;   
    INTCON = 0x90;
    OPTION_REG = 0b01001000;        //revisaar ultimos 4
    
    
    
    while (1)
    {
       GP4 = !GP4;
       __delay_ms(500);
       
    }    
    return;
}

void __interrupt() ISR()
{
    if (INTCONbits.INTF == 1)
    {
        LED1 = !LED1;
                
    }
    INTCONbits.INTF =0;
}
