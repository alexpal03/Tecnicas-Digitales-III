
// PIC12F629 Configuration Bit Settings

// 'C' source line config statements

#pragma config WDTE=ON , BOREN=OFF , PWRTE=ON , MCLRE=OFF , FOSC=INTRCIO

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 4e6
#define LED1 GP0
#define LIMITE_WDT 120


const unsigned char nivelesPWM[6] = {0, 10, 25, 50, 75, 100};
unsigned char nivelActual = 0;

unsigned int nWDTReset = 0;      // Cuente la cantidad de veces que se reseteó el wdt
unsigned int msWDT = 0;          // Cuenta la cantidad de ms que contó el wdt



//Generacion de PWM
void pwm(unsigned char dutyCycle){
    
    for (unsigned char i = 0; i < 100; i++){
        if (i < dutyCycle){
            LED1 = 1;
        }else{
            LED1 = 0;
        }
        
        // 82us en vez de 100us para compensar el tiempo de ejecución del for
        __delay_us(82);
    }
}


void main(void) {
    
    TRISIO = 0b00001100;
    GPIO = 0;   
    CMCON = 0xFF;
    
    INTCON = 0;
    INTCONbits.GIE = 1;        // Enables all unmasked interrupts
    INTCONbits.INTE = 1;       // Enables the GP2/INT external interrupt
   
    OPTION_REG = 0b111;        // WDT Rate 1:128 --> 2.3 seconds
    OPTION_REGbits.INTEDG = 1; // Interrupt on rising edge of GP2/INT pin
    OPTION_REGbits.PSA = 1;    // Prescaler is assigned to the WDT
    
    
    while (1)
    {   
        
        // Revisar si el wdt contó hasta 1 seg
        if (msWDT == 1000) {
            
            // Revisar si se superaron 2 minutos de inactividad
            // Se le resta 2 segundos por el ciclo del wwdt (2.3s))
            if (nWDTReset == LIMITE_WDT - 2) {
                // No se hace nada, se espera a que se resetee el pic
            } else {
                msWDT = 10;
                nWDTReset++;
                CLRWDT();
            }
        } else {
            msWDT += 10;
        }
        
        
        
        // Ciclo de pwm (10ms)
        pwm(nivelesPWM[nivelActual]);
        
    }    
    return;
}


void __interrupt() ISR()
{
    if (INTCONbits.INTF == 1)
    {
        
        nivelActual = (nivelActual + 1) % 6;
        
        // Resetear cualquier conteo del wdt
        msWDT = 0;
        nWDTReset = 0;
        CLRWDT();

    }
    INTCONbits.INTF = 0;
}
