/*
 * Archivo:   lab09_main.c
 * Dispositivo: PIC16F887
 * Autor: Brandon Garrido 
 * 
 * Programa: Laboratorio 9 - modulo ccp
 * Hardware:Potenciómetros en PORTA y servos en PORTC
 * 
 * Creado: Abril 26, 2021
 * Última modificación: Abril 20, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------    
#include <xc.h>

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Para delay

//------------------------------------------------------------------------------
//                          Palabras de configuración
//------------------------------------------------------------------------------    
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)


// Prototipos
void setup();
void config_reloj();
void config_io();
void config_int_enable();
void config_iocb();
void IOCB_interrupt();

/*
 *  main
 */

void main(void) {
    
    setup(); // llamar función de configuraciones
    
    while(1){  //loop principal
    }
    
    return;
}

void setup(){
    
    config_reloj();// configuraciones principales del programa
    config_io();
    config_int_enable();
    config_iocb();
    
    return;
};




// interrupciones

void __interrupt() isr(void){

    if(INTCONbits.RBIF){
        IOCB_interrupt();
    }

}




void IOCB_interrupt(){ // se verifica el push presionado e incrementa o decrem..

    if (PORTBbits.RB0 == 0){ 
        PORTA = 2;
        __delay_ms(10);
        PORTA = 0;
    }
    if(PORTBbits.RB1 == 0) {
        PORTA = 1; 
        __delay_ms(10);
        PORTA = 0;
    }
    if(PORTBbits.RB2 == 0) {
        PORTA = 8; 
    }
    
   
    INTCONbits.RBIF = 0;
    
    return;
} 

// configuraciones

void config_reloj(){

    OSCCONbits.IRCF2 =1 ; // IRCF = 111 (8MHz) 
    OSCCONbits.IRCF1 =1 ;
    OSCCONbits.IRCF0 =1 ;
    OSCCONbits.SCS = 1; // Habilitar reloj interno
    
    return;
}

void config_io(){
   
    ANSELH = 0x00;
    ANSEL = 0x00;
    
    TRISB = 0x07; // habilitar pines RB0,RB1 y RB2 como inputs
    
    TRISA = 0x00;

  
    OPTION_REGbits.nRBPU =  0 ; // se habilita el pull up interno en PORTB
    WPUB = 0x07;  // se habilita los pull ups para los pines RB0, RB1 y RB2
    
    PORTA = 0x00;
    PORTB = 0x07; // se limpian las salidas de los puertos y valores iniciales
    
   
    
    return;
}

void config_int_enable(){
    
    INTCONbits.GIE = 1; // Se habilitan las interrupciones globales
    
    INTCONbits.RBIE = 1; // habilitar banderas de interrupción puertos B
    INTCONbits.RBIF = 0; 	
    
    return;
}
    

void config_iocb(){
    
    IOCB = 0x07; // setear interrupciones en los pines RB0, RB1 y RB2
    
    INTCONbits.RBIF = 0;  
    
    return;
} 
  
