#include <xc.h>

// 🔹 CONFIGURACIÓN DEL MICROCONTROLADOR
#pragma config FOSC = INTRCIO  // Oscilador interno
#pragma config WDTE = ON       // Activar Watchdog Timer (WDT)
#pragma config PWRTE = OFF     // Desactivar Power-Up Timer
#pragma config MCLRE = OFF     // MCLR como entrada digital
#pragma config BOREN = OFF     // Desactivar Brown-Out Reset
#pragma config CP = OFF        // Desactivar Protección de Código

#define _XTAL_FREQ 4000000  // Frecuencia del oscilador interno

// 🔹 Definición de pines
#define LED   GP0  // Salida PWM
#define BOTON GP1  // Entrada del botón
#define TX    GP4  // Transmisión UART (Salida)
#define RX    GP5  // Recepción UART (Entrada)

// 🔹 Variables Globales
volatile unsigned char receivedData = 0;  // Dato recibido
volatile bit dataReady = 0;  // Bandera de dato recibido
unsigned int watchdogTimer = 0; // Contador para el WDT

// 🔹 Configurar UART por software
void uart_init() {
    TRISIO |= (1 << 5); // RX (GP5) como entrada
    TRISIO &= ~(1 << 4); // TX (GP4) como salida
    TX = 1;  // Estado inactivo del UART TX
    
    // 🔹 Configurar interrupción en GP5 (RX)
    INTCONbits.GPIE = 1;   // Habilitar interrupciones en GP
    INTCONbits.GPIF = 0;   // Limpiar bandera de interrupción
    INTCONbits.GIE = 1;    // Habilitar interrupciones globales
}

// 🔹 Enviar un byte por UART (9600 baudios, 8N1)
void uart_tx(unsigned char data) {
    unsigned char i;
    TX = 0;  // Start bit
    __delay_us(104);  // 9600 baudios

    for (i = 0; i < 8; i++) {
        TX = (data >> i) & 0x01;  // Enviar cada bit
        __delay_us(104);
    }
    
    TX = 1;  // Stop bit
    __delay_us(104);
}

// 🔹 Interrupción de Recepción UART (GP5)
void __interrupt() isr(void) {
    if (INTCONbits.GPIF) {  // Si hubo interrupción en GPIO
        if (!RX) {  // Si la interrupción fue por RX (GP5)
            __delay_us(52);  // Mitad del bit de Start
            receivedData = 0;
            
            for (char i = 0; i < 8; i++) {
                __delay_us(104);
                receivedData |= (RX << i);
            }

            __delay_us(104);  // Bit de Stop
            dataReady = 1;  // Dato recibido
            watchdogTimer = 0;  // Reiniciar contador WDT
        }
        INTCONbits.GPIF = 0;  // Limpiar bandera de interrupción
    }
}

// 🔹 Simulación de PWM por software
void pwmSoftware(unsigned char dutyCycle) {
    for (unsigned char i = 0; i < 100; i++) {
        if (i < dutyCycle)
            LED = 1;
        else
            LED = 0;
        __delay_us(100);
    }
}

void main() {
    TRISIO = 0b00100010; // GP1 (BOTON) y GP5 (RX) como entrada, demás salida
    GPIO = 0;
    OPTION_REGbits.nGPPU = 0;  // Habilitar pull-ups internos
    
    // 🔹 Configurar el Prescaler del WDT (1:128 → 131ms por ciclo)
    OPTION_REGbits.PSA = 0;  
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;

    uart_init();  // Inicializar UART con interrupciones

    while (1) {
        // 🔹 Si hay un dato recibido por UART
        if (dataReady) {
            pwmSoftware(receivedData * 100 / 255);  // Aplicar PWM
            uart_tx(0x1B);  // Enviar ACK
            dataReady = 0;  // Resetear bandera
            watchdogTimer = 0;  // Resetear contador de WDT
        }

        // 🔹 Botón para cambiar el brillo del LED
        if (BOTON == 0) {
            __delay_ms(50); // Antirrebote
            if (BOTON == 0) {
                receivedData = (receivedData + 51) % 256;  // Incrementar brillo
                dataReady = 1; // Simular recepción UART con botón
                watchdogTimer = 0;  // Resetear contador de WDT
                while (BOTON == 0); // Esperar a que se suelte el botón
            }
        }

        // 🔹 Advertencia de WDT cuando faltan 20s para el reset
        if (watchdogTimer > 765) {
            uart_tx(0xEE);  // Enviar advertencia '0xEE'
        }

        // 🔹 Si pasan 2 minutos sin actividad, el PIC se reinicia
        watchdogTimer++;
        if (watchdogTimer > 916) {  
            pwmSoftware(0);  // Apagar LED
            CLRWDT();
        }

        CLRWDT();  // Resetear WDT periódicamente
    }
}
