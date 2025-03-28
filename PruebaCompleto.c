
#include <xc.h>

// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-Up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 4000000 //Oscilador a 4MHz
#define LED GP0
#define BOTON GP1
#define TX GP4 //Salida UART
#define RX GP5 //Entrada UART


const unsigned char nivelesPWM[6] = {0, 10, 25, 50, 75, 100};
unsigned char nivelActual = 0; //Brillo
unsigned int watchdogTimer = 0; //Contador WDT
unsigned char pwmValue = 0; //Brillo recibido UART

void uart_inicio(){
    TRISIO |= (1 << 5);// RX (GP5)
    TRISIO &= ~(1 << 4); //TX (GP4)
    TX = 1; //Estado inactivo
}

void uart_tx(unsigned char data){
    unsigned char i;
    TX = 0; //Bit de start
    __delay_us(104); //9600 baudios (1 bit = 104us)
    
    for(i = 0; i < 8; i++){
        TX = (data >> i) & 0x01;
        __delay_us(104);
    }
    
    TX = 1; //Bit de stop
    __delay_us(104);
}

unsigned char uart_rx(){
    unsigned char i, data = 0;
    
    while(RX); //Esperar bit de start en 0
    __delay_us(52);
    
    for (i = 0; i < 8; i++){
        __delay_us(104);
        data |= (RX << i); //Leer de a bit
    }
    
    __delay_us(104); //Bit de stop
    return data;
}

//Generacion de PWM
void pwm(unsigned char dutyCycle){
    for (unsigned char i = 0; i < 100; i++){
        if (i < dutyCycle){
            LED = 1;
        }else{
            LED = 0;
        }
        
        __delay_us(100);
    }
}


void main(void) {
    TRISIO = 0b00100010; //GP1 (BOTON) Y GP5 (RX) COMO ENTRADA, DEMAS SALIDAS
    GPIO = 0;
    OPTION_REGbits.nGPPU = 0; //Habilitar pullups internos
    
    // Configurar el Prescaler para el WDT (1:128 ? 131.072ms por ciclo)
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 1;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    
    uart_inicio();
    
    while (1){
        //Recepcion de datos RX
        if (!RX){
            pwmValue = uart_rx();
            uart_tx(0x1B);
            watchdogTimer = 0; //Resetear WDT
        }
        
        if (BOTON == 0){
            __delay_ms(50);
            if (BOTON == 0){
                nivelActual = (nivelActual + 1) % 6;
                pwmValue = (nivelesPWM[nivelActual] * 255) / 100;
                watchdogTimer = 0;
                while (BOTON == 0);
            }
        }
        
        pwm(pwmValue * 100 / 255);
        
        if (watchdogTimer > 765){ //100s/131ms = 765 ciclos
            uart_tx(0xEE); //Enviar Advertencia
        }
        
        //Si pasan 2 minutos
        watchdogTimer++;
        if(watchdogTimer > 916){
            pwmValue = 0;
            CLRWDT();
        }
        
        CLRWDT(); //Reseteo periodico
    }
    
    
    
    
    return;
}
