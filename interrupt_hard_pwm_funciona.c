
// PIC12F629 Configuration Bit Settings

// 'C' source line config statements

#pragma config WDTE=OFF , BOREN=OFF , PWRTE=ON , MCLRE=OFF , FOSC=INTRCIO

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 4e6
#define LED1 GP5


const unsigned char nivelesPWM[6] = {0, 10, 25, 50, 75, 100};
unsigned char nivelActual = 0;


//Generacion de PWM
void pwm(unsigned char dutyCycle){
    for (unsigned char i = 0; i < 100; i++){
        if (i < dutyCycle){
            LED1 = 1;
        }else{
            LED1 = 0;
        }
        
        __delay_us(100);
    }
}


void main(void) {
    
    TRISIO = 0b00001100;
    GPIO = 0;   
    INTCON = 0x90;
    CMCON = 0xFF;
    OPTION_REG = 0b01000000;        //revisaar ultimos 4
    
    
    
    while (1)
    {
        pwm(nivelesPWM[nivelActual]);
    }    
    return;
}


void __interrupt() ISR()
{
    if (INTCONbits.INTF == 1)
    {
        
        nivelActual = (nivelActual + 1) % 6;

    }
    INTCONbits.INTF =0;
}
