
// PIC12F629 Configuration Bit Settings

// 'C' source line config statements

#pragma config WDTE=ON , BOREN=OFF , PWRTE=ON , MCLRE=OFF , FOSC=INTRCIO

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 4e6
#define LED1 GP0
#define LIMITE_WDT 120
#define TX GP4 // Salida UART
#define RX GP5 // Entrada UART


const unsigned char nivelesPWM[6] = {0, 10, 25, 50, 75, 100};
unsigned char nivelActual = 0;

unsigned int nWDTReset = 0;      // Cuente la cantidad de veces que se reseteó el wdt
unsigned int msWDT = 0;          // Cuenta la cantidad de ms que contó el wdt



unsigned int rxPrioridad = 0;

struct tipo_serial {
    unsigned int dato;
    unsigned char indice; // Almacena la posicion del bit proximo a leer
}txDato, rxDato;


void uart_setup() {
    TX = 1; //Estado inactivo
    TMR0 = 200;  // Cargar valor inicial para desborde a 104us
    
    txDato.indice = 10;
    
    rxDato.indice = 10;
    rxDato.dato = 0;  // Bit de start y de stop;
}

void uart_tx(unsigned char dato){
    txDato.dato = 0b0000001000000000;  // Bit de start y de stop;;
    txDato.dato |= ((unsigned int)dato) << 1;
    txDato.indice = 0;
}


unsigned char uart_rx(){
    return (rxDato.dato >> 1) & 0xFF;
}

//Generacion de PWM
void pwm(unsigned char dutyCycle){
    
    for (unsigned char i = 0; i < 100; i++){
        if (i < dutyCycle){
            LED1 = 1;
        }else{
            LED1 = 0;
        }
        
        // 63us en vez de 100us para compensar el tiempo de ejecución del for
        __delay_us(5);
    }
}

void reset_conteo() {
    // Resetear cualquier conteo del wdt
    msWDT = 0;
    nWDTReset = 0;
    CLRWDT();
}

void main(void) {
    
    TRISIO = 0b00101100;
    GPIO = 0;   
    CMCON = 0xFF;
    
    INTCON = 0;
    INTCONbits.GIE = 1;        // Enables all unmasked interrupts
    INTCONbits.INTE = 1;       // Enables the GP2/INT external interrupt
    INTCONbits.GPIE = 1;       // Enables the GPIO changed state interrupt
    INTCONbits.T0IE = 1;       // Enables the TMR0 interrupt
   
    OPTION_REG = 0b111;        // WDT Rate 1:128 --> 2.3 seconds
    OPTION_REGbits.INTEDG = 1; // Interrupt on rising edge of GP2/INT pin
    OPTION_REGbits.PSA = 1;    // Prescaler is assigned to the WDT
    
    // Uart inicio
    uart_setup();
    

    
    while (1)
    {   
        
        // Revisar si el wdt contó hasta 1 seg
        if (msWDT == 1000) {
            
            // Revisar si se superaron 2 minutos de inactividad
            // Se le resta 2 segundos por el ciclo del wwdt (2.3s))
            unsigned int segFaltantes = LIMITE_WDT - 2 - nWDTReset;
            if (!segFaltantes) {
                // No se hace nada, se espera a que se resetee el pic
            } else {
                
                if (segFaltantes == 20) {
                    uart_tx(0xEE);
                }
                
                msWDT = 10;
                nWDTReset++;
                CLRWDT();
            }
        } else {
            msWDT += 10;
        }
        
        
        uart_tx(0b01010111);
        
        // Ciclo de pwm (10ms)
        if (rxPrioridad) {
            
            unsigned char pwmValor = (uart_rx() * (int)100)/255;
            
            pwm(pwmValor);   
            
            // Nuevo nivel actual
            if (pwmValor <= 10) nivelActual = 1;
            else if (pwmValor <= 25) nivelActual = 2;
            else if (pwmValor <= 50) nivelActual = 3;
            else if (pwmValor <= 75) nivelActual = 4;
            else if (pwmValor <= 100) nivelActual = 5;
            
        } else {
            pwm(nivelesPWM[nivelActual]);   
            
            // Compensar secuencia de if anterior
            __delay_us(800);
        }
    }    
    return;
}


void __interrupt() ISR()
{
    // Interrupcion de hardware
    if (INTCONbits.INTF == 1)
    {
        INTCONbits.INTF = 0; // Limpiar bandera
        
        nivelActual = (nivelActual + 1) % 6;
        
        // Dejar de darle prioridad al ultimo valor del rx
        rxPrioridad = 0;
        
        // Resetear cualquier conteo del wdt
        reset_conteo();
    }
    // Interrupcion de timer
    else if(INTCONbits.T0IF == 1)  
    {       
        INTCONbits.T0IF = 0; // Limpiar bandera
        
        // PROTEUS
        // TMR0 = 200;  // Cargar valor inicial para desborde a 104us     
        
        TMR0 = 160;
        
        // Envio por serial
        if (txDato.indice < 10) {
            
            // Envio del dato
            TX = (txDato.dato >> txDato.indice) & 0x01;
            
            // Se prepara para el siguiente bit
            ++txDato.indice;
        }
        
        // Detección de bit de start
        if (RX == 0 && rxDato.indice == 10) {
            // Reset parametros lectura
            rxDato.dato = 0;
            rxDato.indice = 0;
            
            rxPrioridad = 1;
            
            reset_conteo();
        }  
        
        // Lectura de bits
        if (rxDato.indice < 10) {
               
            rxDato.dato |= (0x01 & RX) << rxDato.indice;
            ++rxDato.indice;
        }
    }  
    
    
    
}
