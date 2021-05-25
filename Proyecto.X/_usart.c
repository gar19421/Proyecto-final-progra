/*
 * Archivo:   lab10_pte2_main.c
 * Dispositivo: PIC16F887
 * Autor: Brandon Garrido 
 * 
 * Programa: Laboratorio 10 - Comunicación serial
 * Hardware:
 * 
 * Creado: Abril 26, 2021
 * Última modificación: Abril 26, 2021
 */


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
#pragma config LVP = ON         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------    
#include <xc.h>
#include <string.h>
//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Para delay

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void);  //Configuración
void showString(char *var);// funcion para cadena de strings
//------------------------------------------------------------------------------
//                          Variables
//------------------------------------------------------------------------------

const char data = 97; //constante valor a
int flag = 1; //bandera de menu con valor inicial 1
char texto[11]; //texto de opcion 1
unsigned char opcion=0; // opcion ingresada por el usuario


//------------------------------------------------------------------------------
//                          Loop principal
//------------------------------------------------------------------------------
void main(void) {
    
    setup(); //Configuración
    strcpy(texto,"hola mundo! ");
    
      
    while(1)//loop forever
    {
        __delay_ms(200);
        
        if (PIR1bits.TXIF){
             
            if(flag){ // si la bandera esta encendida mostrara el menu
                showString("Que accion desea ejecutar?");
                showString("(1)Desplegar cadena de caracteres");
                showString("(2)Cambiar PORTA");
                showString("(3)Cambiar PORTB");
                flag = 0;
            }
            if(opcion==49){ // cuando seleccione opcion 1 mostrara el texto 
                showString(texto);
                flag = 1;
                opcion = 0;
            }
            if(opcion==50){ // opcion 2, modificar caracter porta
                showString("Ingrese el caracter a mostrar en PORTA");
                
                flag = 1;
                opcion = 0;
                
                while(!opcion){ // hasta que ingrese un valor para el porta 
                                //se mantiene en espera
                }
                
                PORTA = opcion;
                opcion = 0;
                
                
            }
            if (opcion==51){ //opcion 3 modificar caracter portb
                showString("Ingrese el caracter a mostrar en PORTB");
                
                flag = 1;
                opcion = 0;
                
                while(!opcion){// hasta que ingrese un valor en portb
                                //se mantiene en espera 
                }
                
                PORTB = opcion;
                opcion = 0;
            } 
            
            
        }
    }

}

void __interrupt() isr(void){
    
    if (PIR1bits.RCIF){//registra los caracteres ingresados
        opcion = RCREG;
        
    }

}

//------------------------------------------------------------------------------
//                          Configuración
//------------------------------------------------------------------------------
void setup(){
    
    //Configuracion reloj
    OSCCONbits.IRCF2 = 1; //Frecuencia a 1MHz
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    
    //Configurar entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0x00; //Primeros dos pines con entradas analógicas
    
    TRISA = 0x00;
    TRISB = 0x00;
    PORTA = 0x00;
    PORTB = 0x00;

    //Configurar la interrupcion
    INTCONbits.GIE = 1;  //Enable interrupciones globales
    INTCONbits.PEIE = 1; //Enable interrupciones periféricas
    PIE1bits.RCIE = 1;
    PIR1bits.RCIF = 0;
             
    
    //Configuración de TX y RX
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 207;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1;
    
 } 


void showString(char *var){ //subrutina de formacion de cadena de caracteres
    int i;
       
    for (i = 0; i < strlen(var); i++) { //bucle en donde lee el array de char y
        TXREG = var[i]; //lo mueve a la consola
    }
    TXREG = 13;
}




