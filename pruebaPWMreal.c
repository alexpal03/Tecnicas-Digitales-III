// PIC12F629 Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = ON        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // GP3/MCLR pin function select (GP3/MCLR pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>


int num = 0;
int duty = 1000;

void main(void) 
{
    	TRISIO = 0x00;          


	INTCON = 0xA0;         //Activo la interrupción por Timer0
	OPTION_REG = 0xC0;     //Prescaler a 1:2

	while(1)
    	{	
	

	}

}



void interrupt isr()     //Función para la interrupción por Timer
{
    if(INTCONbits.T0IF == 1)  
    {       
        num++;
        
        if(num <= duty)     
        {
          GP4 = 1;      	
            
        }else
        {
		GP4 = 0;
	}
	

	if(num > 1000)
	{
		num = 0;
		duty = duty - 10;
		if(duty < 0) 
			duty = 1000;
	}
        INTCONbits.T0IF = 0; //Bajo la bandera del Timer0
    }    
}
