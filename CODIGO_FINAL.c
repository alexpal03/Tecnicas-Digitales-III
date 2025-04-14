


// --------------------------------------------------------------
// Programa para PIC12F629
//
// Descripción:
// Implementa comunicación serial mediante pines TX (GP4) y RX (GP5), sin
// hardware UART, utilizando el temporizador TMR0 como base de tiempo para la
// sincronización del protocolo serial por software.
//
// TMR1 se emplea como temporizador auxiliar para controlar el Watchdog Timer (WDT).
//
// Configuración de pines:
//   - GP0: Salida para LED
//   - GP2: Entrada para pulsador
//   - GP4: Salida TX (transmisión serial)
//   - GP5: Entrada RX (recepción serial)
//
// --------------------------------------------------------------



#pragma config WDTE=ON , BOREN=OFF , PWRTE=ON , MCLRE=OFF , FOSC=INTRCIO


#include <xc.h>

#define _XTAL_FREQ      4e6
#define LED1            GP0
#define LIMITE_WDT      120
#define TX              GP4 // Salida UART
#define RX              GP5 // Entrada UART


const unsigned char nivelesPWM[6] = {0, 10, 25, 50, 75, 100};
unsigned char nivelActual = 0;

// Cuente la cantidad de veces que se reseteó el wdt
unsigned int nWDTReset = 0;
// Cuenta la cantidad de ms que contó el wdt
unsigned int msWDT = 0;          
// Cuando está en 1, el pwm se debe tomar por el último dato de la serial
// En 0, el pwm se toma por el indice de nivel determinado por el boton
unsigned int rxPrioridad = 0;

struct tipo_serial {
    unsigned int dato;    // Almacena el dato
    unsigned char indice; // Almacena la posicion del bit proximo a leer
}txDato, rxDato;


void uart_setup() {
    
    //Estado inactivo
    TX = 1;
    
    // PROTEUS
    // TMR0 = 200;  // Cargar valor inicial para desborde a 104us     

    // REAL EXPERIMENTAL
    TMR0 = 160;
    
    // Valor inicial (sin datos por enviar)
    txDato.indice = 10;
    
    // Valor inicial (sin datos por leer)
    rxDato.indice = 10;

    rxDato.dato = 0;
}

void uart_tx(unsigned char dato){
    // Setear bit de start y de stop
    txDato.dato = 0b0000001000000000;
    // Enmascarar bits de dato
    txDato.dato |= ((unsigned int)dato) << 1;
    txDato.indice = 0;
}


unsigned char uart_rx(){
    return (rxDato.dato >> 1) & 0xFF;
}

// Generacion de PWM
void pwm(unsigned char dutyCycle){
    
    for (unsigned char i = 0; i < 100; i++){
        if (i < dutyCycle){
            LED1 = 1;
        }else{
            LED1 = 0;
        }
        
        // Valor experimental para 100Hz
        __delay_us(45);
    }
}

void reset_conteo() {
    // Resetear cualquier conteo del wdt
    msWDT = 0;
    nWDTReset = 0;
    CLRWDT();
}

void main(void) {
    
    TRISIO = 0b00100100;
    GPIO = 0;   
    CMCON = 0xFF;
    
    INTCON = 0;
    INTCONbits.GIE = 1;        // Enables all unmasked interrupts
    INTCONbits.INTE = 1;       // Enables the GP2/INT external interrupt
    INTCONbits.GPIE = 1;       // Enables the GPIO changed state interrupt
    INTCONbits.T0IE = 1;       // Enables the TMR0 interrupt
    INTCONbits.PEIE = 1;       // Enables the peripheral interrupt
   
    OPTION_REG = 0b111;        // WDT Rate 1:128 --> 2.3 seconds
    OPTION_REGbits.INTEDG = 1; // Interrupt on rising edge of GP2/INT pin
    OPTION_REGbits.PSA = 1;    // Prescaler is assigned to the WDT
    
 
    // Configuracion timer1
    PIR1 = 0;
    PIE1 = 0;
    PIE1bits.TMR1IE = 1;       // Enables the TMR1 interrupt
    T1CON = 0b00000101;
    
    // Cada 10ms overflow
    TMR1H = 0xcf;
    TMR1L = 0xfc;
    
    // Uart inicio
    uart_setup();
    

    
    while (1)
    {    
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
    // Interrupcion de hardware, pulsador presionado
    if (INTCONbits.INTF == 1)
    {
        INTCONbits.INTF = 0; // Limpiar bandera
        
        nivelActual = (nivelActual + 1) % 6;
        
        // Dejar de darle prioridad al ultimo valor del rx
        rxPrioridad = 0;
        
        // Resetear cualquier conteo del wdt
        reset_conteo();
    }
    
    
    
    // Interrupcion de timer 0
    else if(INTCONbits.T0IF == 1)  
    {       
        INTCONbits.T0IF = 0; // Limpiar bandera
        
        // PROTEUS
        // TMR0 = 200;  // Cargar valor inicial para desborde a 104us     
        
        // REAL EXPERIMENTAL
        TMR0 = 160;

        
        // RECEPCION SERIAL
        // Detección de bit de start
        if (RX == 0 && rxDato.indice == 10) {
            // Reset parametros lectura
            rxDato.dato = 0;
            rxDato.indice = 0;
            
            rxPrioridad = 1;
            
            // Envío de ACK
            uart_tx(0x1b);
           
            reset_conteo();
        }  
        
        // Lectura de bits
        if (rxDato.indice < 10) {
               
            rxDato.dato |= (0x01 & RX) << rxDato.indice;
            ++rxDato.indice;
        }
        // Si no hay datos pendientes de lectura
        // TRANSMISION SERIAL
        // Se prioriza la recepcion antes que la transmision
        // La transmision puede esperar, la recepción es en tiempo real
        else if (txDato.indice < 10) {

            // Envio del dato
            TX = (txDato.dato >> txDato.indice) & 0x01;

            // Se prepara para el siguiente bit
            ++txDato.indice;
        }
    }  
    
    
    // Control del WDT para reset a 2 min
    else if (PIR1bits.TMR1IF == 1){
        PIR1bits.TMR1IF = 0;
        
        TMR1H = 0xcf;
        TMR1L = 0xfc;
        
        // Revisar si el wdt contó hasta 1 seg
        if (msWDT == 1000) {
                
            // Revisar si se superaron 2 minutos de inactividad
            // Se le resta 2 segundos por el ciclo restante del wdt (2.3s)
            unsigned int segFaltantes = LIMITE_WDT - 2 - nWDTReset;
            if (!segFaltantes) {
                // No se hace nada, se espera a que se resetee el pic
            } else {
                
                if (segFaltantes == 20) {
                    uart_tx(0xEE);
                }
                
                
                nWDTReset++;
                msWDT = 10;
                CLRWDT();
            }
        } else {
            msWDT += 10;
        }
        
    }
}
