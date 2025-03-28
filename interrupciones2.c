#include <xc.h>

// CONFIGURACIÓN DEL MICROCONTROLADOR
#pragma config FOSC = INTRCIO  // Oscilador interno
#pragma config WDTE = ON       // Activar Watchdog Timer (WDT)
#pragma config PWRTE = OFF     // Desactivar Power-Up Timer
#pragma config MCLRE = OFF     // MCLR como entrada digital
#pragma config BOREN = OFF     // Desactivar Brown-Out Reset
#pragma config CP = OFF        // Desactivar Protección de Código

#define _XTAL_FREQ 4000000  // Frecuencia del oscilador interno

// Definición de pines
#define LED   GP0  // Salida PWM
#define BOTON GP1  // Entrada del botón
#define TX    GP4  // Transmisión UART (Salida)
#define RX    GP5  // Recepción UART (Entrada)

// Variables globales
volatile unsigned char receivedData = 0;  // Dato recibido
volatile bit dataReady = 0;  // Bandera de dato recibido
unsigned int watchdogTimer = 0; // Contador para el WDT
unsigned char pwmLevels[] = {0, 10, 25, 50, 75, 100};
unsigned char pwmIndex = 0;

// Configurar UART por software
void uart_init() {
    TRISIO |= (1 << 5); // RX (GP5) como entrada
    TRISIO &= ~(1 << 4); // TX (GP4) como salida
    TX = 1;  // Estado inactivo de TX

    INTCONbits.GPIE = 1;   // Habilitar interrupciones en GPIO
    INTCONbits.GPIF = 0;   // Limpiar bandera de interrupción
    INTCONbits.GIE = 1;    // Habilitar interrupciones globales
}

// Enviar un byte por UART
void uart_tx(unsigned char data) {
    unsigned char i;
    TX = 0;  // Start bit
    __delay_us(104);
    
    for (i = 0; i < 8; i++) {
        TX = (data >> i) & 0x01;
        __delay_us(104);
    }
    
    TX = 1;  // Stop bit
    __delay_us(104);
}

// Interrupción de Recepción UART (GP5)
void __interrupt() isr(void) {
    if (INTCONbits.GPIF) {
        if (!RX) {
            __delay_us(52);
            receivedData = 0;
            
            for (char i = 0; i < 8; i++) {
                __delay_us(104);
                receivedData |= (RX << i);
            }

            __delay_us(104);
            dataReady = 1;
            watchdogTimer = 0;
        }
        INTCONbits.GPIF = 0;
    }
}

// Simulación de PWM por software
void pwmSoftware(unsigned char dutyCycle) {
    for (unsigned char i = 0; i < 100; i++) {
        LED = (i < dutyCycle) ? 1 : 0;
        __delay_us(100);
    }
}

void main() {
    TRISIO = 0b00100010; // GP1 (BOTON) y GP5 (RX) como entrada
    GPIO = 0;
    OPTION_REGbits.nGPPU = 0;
    WPUbits.WPU1 = 1; // Activar pull-up interno en GP1

    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;

    uart_init();

    while (1) {
        if (dataReady) {
            pwmSoftware(receivedData * 100 / 255);
            uart_tx(0x1B);
            dataReady = 0;
            watchdogTimer = 0;
        }

        if (!BOTON) {
            __delay_ms(50);
            if (!BOTON) {
                while (!BOTON);
                pwmIndex = (pwmIndex + 1) % 6;
                pwmSoftware(pwmLevels[pwmIndex]);
                watchdogTimer = 0;
            }
        }

        if (watchdogTimer > 765) {
            uart_tx(0xEE);
        }

        watchdogTimer++;
        if (watchdogTimer > 916) {
            pwmSoftware(0);
            CLRWDT();
        }

        CLRWDT();
    }
}
