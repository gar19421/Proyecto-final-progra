
/*
 * Archivo:   proyecto_final.c
 * Dispositivo: PIC16F887
 * Autor: Brandon Garrido 
 * 
 * Programa: Proyecto final 2- Carrito con comunicación USART y control con push
 * Hardware: Push en PORTB, Salidas para motores DC y leds PORTA, salida servos
 *           CCP1 y CCP2, comunicación serial en puertos TX y RX
 * Creado: Mayo 20, 2021
 * Última modificación: Mayo 3 , 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------    
#include <xc.h>
#include <string.h>
#include <pic16f887.h>

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Para delay a una frecuencia de (8MZ)
//#define addressEEPROM 0x10

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
#pragma config LVP = OFF        //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)



//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------

//prototipos de configuración pic e interrupciones
void setup();
void config_reloj();
void config_io();
void config_int_enable();
void config_iocb();
void IOCB_interrupt();

//prototipos funciones de programa
void showString(char *var);// funcion para cadena de strings
void writeToEEPROM(int data, int address);
int dec_to_ascii(int val);
//int readFromEEPROM(int address);

//------------------------------------------------------------------------------
//                          Variables
//------------------------------------------------------------------------------

int flag = 1; //bandera de menu con valor inicial 1
int flag2=0; //bandera para control de giro de motores DC

unsigned char opcion=0; // opcion ingresada por el usuario

unsigned char temporal_posicion1;//variables de posiciones guardadas en EEPROM
unsigned char temporal_posicion2;//de los servos para reproducir y guardar
unsigned char leer = 0x10;

int valor_pot;//variables para control y conversión de entrada analogica 
int var_temp; //del potenciometro y convertirlo a decimal para mostrar en USART
int valor_centenas;
int valor_decenas;
int valor_unidades;

int RB3_old; // variables de control para escritura en EEPROM
int eepromVal = 0;

int addressEEPROM = 0x10;// posición de inicio para escribir en EEPROM 

int parpadear = 0; // control de titileo leds



//------------------------------------------------------------------------------
//                          Main
//------------------------------------------------------------------------------

void main(void) {
    
    setup(); // llamar función de configuraciones  
    
    
    ADCON0bits.GO = 1; //La conversión ADC se ejecuta
    while(1)
    {
  
        // codigo del adc
        if(ADCON0bits.GO == 0){ //Si la conversión ya está terminada
            if (ADCON0bits.CHS == 5){ //Si está en el primer canal,
                ADCON0bits.CHS = 6;}  //pasa al segundo canal
            else if (ADCON0bits.CHS == 6){ //Si está en el primer canal,
                ADCON0bits.CHS = 5;}  //pasa al segundo canal
            else if (ADCON0bits.CHS == 7){ //Si está en el primer canal,
                ADCON0bits.CHS = 5;}  //pasa al segundo canal
            __delay_us(50); //Delay para el capacitor sample/hold
            ADCON0bits.GO = 1; //Se vuelve a ejecutar la conversión ADC
        }
        
        
        // codigo del menú para el USART
            
        if (PIR1bits.TXIF){ 
            //verifica banderas de inicio para mostrar menú de opciones en USART
            if(flag){
                showString("Ingrese 1 si desea ingresar a modo control USART");
                flag = 0;
            }
            
                if(opcion == 49){
                    flag = 1;
                    opcion = 0;
                    
                    //se mantiene en control usart hasta que el usuario salga
                    while(opcion != 53){ 

                        // si la bandera esta encendida mostrara el menu
                        if(flag){ 
                            showString("Bienvenido a nuestro programa");
                            showString("Que accion desea ejecutar?");
                            showString("(1)Controlar Brazo");
                            showString("(2)Controlar Carro");
                            showString("(3)Controlar LEDs");
                            showString("(4)Mostrar potenciometro");
                            showString("(5)Salir de control por USART");
                            flag = 0;
                        }
                        
                        // cuando seleccione opcion 1 mostrara el texto 
                        if(opcion==49){ 
                            showString("Elija la posicion del servo 1 (abajo)");
                            showString("Ingrese: 1-.0grados 2-.90grados 3-.180grados");

                            flag = 1;
                            opcion = 0;

                            while(!opcion){

                            } //espera hasta que se ingrese opcion 

                            //mueve servo 1 a traves el CCP1
                            if(opcion==49){
                                PORTD = (0);
                                //Swift y ajuste de señal
                                CCPR1L = (PORTD>>1) + 128; 
                                CCP1CONbits.DC1B1 = PORTDbits.RD0;
                                CCP1CONbits.DC1B0 = ADRESL>>7;
                            }
                            if(opcion==50){
                                PORTD = (128);
                                //Swift y ajuste de señal
                                CCPR1L = (PORTD>>1) + 128; 
                                CCP1CONbits.DC1B1 = PORTDbits.RD0;
                                CCP1CONbits.DC1B0 = ADRESL>>7;
                            }
                            if(opcion==51){
                                PORTD = (255);
                                //Swift y ajuste de señal
                                CCPR1L = (PORTD>>1) + 120; 
                                CCP1CONbits.DC1B1 = PORTDbits.RD0;
                                CCP1CONbits.DC1B0 = ADRESL>>7;
                            }

                            showString("Elija la posicion del servo 2 (arriba)");
                            showString("Ingrese: 1-.0grados 2-.90grados 3-.180grados");

                            flag = 1;
                            opcion = 0;

                            while(!opcion){

                            }//espera que ingrese opcion 

                            //mueve servo 2 a traves el CCP2
                            if(opcion==49){
                                PORTD = (0);
                                //Swift y ajuste de señal
                                CCPR2L = (PORTD>>1) + 128; 
                                CCP2CONbits.DC2B1 = PORTDbits.RD0;
                                CCP2CONbits.DC2B0 = ADRESL>>7;
                            }
                            if(opcion==50){
                                PORTD = (128);
                                //Swift y ajuste de señal
                                CCPR2L = (PORTD>>1) + 128; 
                                CCP2CONbits.DC2B1 = PORTDbits.RD0;
                                CCP2CONbits.DC2B0 = ADRESL>>7;
                            }
                            if(opcion==51){
                                PORTD = (255);
                                //Swift y ajuste de señal
                                CCPR2L = (PORTD>>1) + 120; 
                                CCP2CONbits.DC2B1 = PORTDbits.RD0;
                                CCP2CONbits.DC2B0 = ADRESL>>7;
                            }


                            opcion = 0; //limpia opciones para poder reingresar
                        }
                        
                        //controla las acciones a realizar con USAR a los DC
                        if(opcion==50){ 
                            
                            showString("Elija la accion que desea realizar para mover el carro");
                            showString("1.Forback 2.Forward 3.Turn rigth 4.Turn left 5.Stop");
                
                            flag = 1;
                            opcion = 0;

                            while(!opcion){

                            }
                            
                            //control de giro avanzar / retroceder
                            if(opcion==49){ 
                                PORTA = 8; 
                                flag =1;
                            }
                            if(opcion==50){

                                PORTA = 4; 
                                flag =2;
                            }
                            
                            //control de giro derecha / izquierda
                            if(opcion==51){
                                PORTA = 9;
                                __delay_ms(800);
                                if(flag ==1){
                                    PORTA = 8;
                                }
                                if(flag ==2){
                                    PORTA = 4;
                                }
                            }
                            if(opcion==52){
                                PORTA = 10;
                                __delay_ms(800);
                                if(flag ==1){
                                    PORTA = 8;
                                }
                                if(flag ==2){
                                    PORTA = 4;
                                }
                            }
                            
                            //detiene los motores
                            if(opcion==53){
                                PORTA = 0;
                            }


                            opcion = 0; // limpia para poder reingresar


                        }
                        
                        //control de los indicadores (LEDs)
                        if (opcion==51){
                            showString("Ingrese la accion a realizar con los leds");
                            showString("1.Encender leds delanteras 2.Encender leds traseras 3.Parpadear leds");
                            
                            flag = 1;
                            opcion = 0;

                            while(!opcion){// hasta que ingrese un valor 
                                            //se mantiene en espera 
                            }
                            
                            
                            //enciende luces delanteras
                            if(opcion==49){
                                PORTAbits.RA4 = 1;
                                PORTAbits.RA5 = 1;
                            }
                            
                            //enciendo luces traseras
                            if(opcion==50){
                                PORTAbits.RA6 = 1;
                                PORTAbits.RA7 = 1;
                            }
                            
                            // realiza el titileo de los leds y los apaga
                            if(opcion==51){
                                int i;
                                for (i = 0; i < 3; i++) {
                                   
                                   PORTAbits.RA4 = 1;
                                   PORTAbits.RA5 = 1;
                                   PORTAbits.RA6 = 1;
                                   PORTAbits.RA7 = 1; 
                                   __delay_ms(250);
                                   PORTAbits.RA4 = 0;
                                   PORTAbits.RA5 = 0;
                                   PORTAbits.RA6 = 0;
                                   PORTAbits.RA7 = 0;
                                   __delay_ms(250);
                                    
                                }

                            }


                            opcion = 0; //limpia para reingresar
                        } 

                        //muestra el valor del potenciometro de comunicacion 
                        if (opcion==52){ 
                            flag = 1;
                            
                            //habilita el canal y conversión del potenciometro
                            ADCON0bits.CHS = 7;
                            __delay_us(50);
                            //Se vuelve a ejecutar la conversión ADC
                            ADCON0bits.GO = 1;
                            __delay_us(50); //Delay para capacitor sample/hold
                            valor_pot = ADRESH;
                            __delay_us(50);
                            
                            //realiza la conversión a valor decimal
                            var_temp = valor_pot;
                            valor_centenas = var_temp / 100; 
                            var_temp = var_temp - valor_centenas*100;
                            valor_decenas = var_temp / 10;
                            var_temp = var_temp - valor_decenas*10;
                            valor_unidades = var_temp;
                            
                            //manda a traer el valor ASCII de la UDC
                            showString("El valor en decimal del pot es:");
                            TXREG = dec_to_ascii(valor_centenas);
                            __delay_ms(10);
                            TXREG = dec_to_ascii(valor_decenas);
                            __delay_ms(10);
                            TXREG = dec_to_ascii(valor_unidades);
                            __delay_ms(10);
                            
                            TXREG = 13;
                            __delay_ms(10);
                            TXREG = 11;// nueva linea + retorno de carro
                            
                            opcion = 0;
                        } 


                    }
                    
                    // al salir muestra de nuevo opcion de reingreso al control
                    if(opcion==53){
                       showString("Ingrese 1 si desea ingresar a modo control USART");
                       flag = 0;
                    }
                    
                }
            
        }
        
    
        // codigo de la EEPROM
   
        if (PORTBbits.RB4 == 0){
            RB3_old = 1;
        }
        //escritura de la EEPROM cuando se presiona RB4
        if(PORTBbits.RB4 == 1 && RB3_old==1){
            eepromVal = temporal_posicion1; //escribe la posición servo 1
            
            writeToEEPROM(eepromVal,addressEEPROM);
            
            //inicia escribiendo el valor desde posición 0x10 hasta 0x17
            if(addressEEPROM == 0x17){
                addressEEPROM = 0x10;
            }else{
                addressEEPROM = addressEEPROM + 1;
            }
            
            __delay_ms(10);
            eepromVal = temporal_posicion2;//escribe la posición servo 2
            
            writeToEEPROM(eepromVal,addressEEPROM);
            
            if(addressEEPROM == 0x17){
                addressEEPROM = 0x10;
            }else{
                addressEEPROM = addressEEPROM + 1;
            }
            
            RB3_old = 0;
        }
      
        
          
    }
    
    return;
}


//función para convertir valores decimales a su codigo ASCII
int dec_to_ascii(int val){
    if(val==0){
        return 48;
    }else if(val==1){
        return 49;
    }else if(val==2){
        return 50;
    }else if(val==3){
        return 51;
    }else if(val==4){
        return 52;
    }else if(val==5){
        return 53;
    }else if(val==6){
        return 54;
    }else if(val==7){
        return 55;
    }else if(val==8){
        return 56;
    }else if(val==9){
        return 57;
    }

}

void setup(){
    
    config_reloj();// configuraciones principales del programa
    config_io();
    config_int_enable();
    config_iocb();
    
    return;
};


//escritura de EEPROM
void writeToEEPROM(int data, int address){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    
    //deshabilitar interrupciones
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;
   
    //se termina la escritura y reenciende interrupciones
    EECON1bits.WREN = 0; 
    INTCONbits.GIE = 1;
    
    return;
}


//Lectura de EEPROM
int readFromEEPROM(int address){
 EEADR = address;
 EECON1bits.EEPGD = 0;
 EECON1bits.RD = 1;
 int data = EEDATA;
 
 return data;

}




// interrupciones
void __interrupt() isr(void){

    if(INTCONbits.RBIF){
        IOCB_interrupt(); //interrupcion puertos B
    }
    
    if (PIR1bits.RCIF){//registra los caracteres ingresados
        opcion = RCREG;
        
    }

    if (PIR1bits.ADIF){
               
        if(ADCON0bits.CHS == 5) { //Verifica el canal en l que se encuentra
            PORTD = ADRESH;
            temporal_posicion1 = PORTD;
            CCPR1L = (PORTD>>1) + 128; //Swift y ajuste de señal
            CCP1CONbits.DC1B1 = PORTDbits.RD0;
            CCP1CONbits.DC1B0 = ADRESL>>7;}
        else if(ADCON0bits.CHS == 7){
            valor_pot = ADRESH;
        }
        
        else if(ADCON0bits.CHS == 6){
            PORTD = ADRESH;
            temporal_posicion2 = PORTD;
            CCPR2L = (PORTD>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = PORTDbits.RD0;
            CCP2CONbits.DC2B0 = ADRESL>>7;}
        
        PIR1bits.ADIF = 0; //Se limpia la bandera de ADC
    }
}




void IOCB_interrupt(){ // se verifica el push presionado e incrementa o decrem..

    //control de giro hacia derecha
    if (PORTBbits.RB0 == 0){  
        if (flag2){
             
            PORTA = 10;
            PORTAbits.RA4 = 1;
            __delay_ms(800);
            PORTA = 8;
            
        }
        else {
            PORTA = 2;
            PORTAbits.RA4 = 1;
            __delay_ms(800);
            PORTA = 0;
        }
    }
    //contorl de giro hacia la izquierda
    if(PORTBbits.RB1 == 0) {
        if (flag2){
            PORTA = 9;
            PORTAbits.RA5 = 1;
            __delay_ms(800);
            PORTA = 8;
        }
        else {
            PORTA = 1;
            PORTAbits.RA5 = 1;
            __delay_ms(800);
            PORTA = 0;
        }
    }
    
    //control de avance
    if(PORTBbits.RB2 == 0) {
        PORTA = 8; 
        flag2 = 1;
        PORTAbits.RA4 = 1;//prende leds
        PORTAbits.RA5 = 1;
    }
    //control de retroceso
    if(PORTBbits.RB3 == 0) {
                      
       PORTA = 4; 
       flag2 = 1;
       PORTAbits.RA6 = 1;//prende leds
       PORTAbits.RA7 = 1;
        
    }

    if(PORTBbits.RB3 == 1 && PORTBbits.RB2 == 1) {
        PORTA = 0; 
        flag2 = 0;
    }//cuando se sueltan los push pone en cero todo
   
    
    //si se presiona RB5 Reproduce los valores guardados en EEPROM
    if(PORTBbits.RB5 == 0){
        
        PORTD = readFromEEPROM(leer);
        
        if(leer == 0x17){//lee desde la posición 0x10 a 0x12 en pares (2 servos)
                leer = 0x10;
        }else{
            leer++;
        }
       
        //reproduce valores en servo 1
        CCPR1L = (PORTD>>1) + 120;//Swift y ajuste de señal
        CCP1CONbits.DC1B1 = PORTDbits.RD0;
        CCP1CONbits.DC1B0 = ADRESL>>7;
        
        __delay_ms(10);
        
        PORTD = readFromEEPROM(leer);
        
        if(leer == 0x17){
                leer = 0x10;
        }else{
            leer++;
        }
        
        //reproduce valores en servo 2
        CCPR2L = (PORTD>>1) + 128;//Swift y ajuste de señal
        CCP2CONbits.DC2B1 = PORTDbits.RD0;
        CCP2CONbits.DC2B0 = ADRESL>>7;
        
        
        __delay_ms(5000);
        
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
   
    //Configurar entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0xE0; //Primeros dos pines con entradas analógicas
    
    TRISB = 0xFF; // habilitar pines RB0,RB1 y RB2 como inputs
    TRISA = 0x00;

    TRISE = 0x07; //Para entrada de los potenciometros
    TRISD = 0x00;
    TRISC = 0xB9; //Para servos
  
    OPTION_REGbits.nRBPU =  0 ; // se habilita el pull up interno en PORTB
    WPUB = 0xFF;  // se habilita los pull ups para los pines RB0, RB1 y RB2
    
    PORTA = 0x00;
    PORTB = 0xFF; // se limpian las salidas de los puertos y valores iniciales
    
    PORTE = 0x00; //Se limpian los puertos    
    PORTD = 0x00;
    PORTC = 0x00;
    
    
    
    //Configurar ADC
    ADCON1bits.ADFM = 0; //Justificar a la izquierda
    ADCON1bits.VCFG0 = 0; //Vss
    ADCON1bits.VCFG1 = 0; //VDD
    
    ADCON0bits.ADCS = 0b10; //ADC oscilador -> Fosc/32
    ADCON0bits.CHS = 5;     //Comenzar en primer canal
    __delay_us(50);        
    ADCON0bits.ADON = 1;    //Habilitar la conversión ADC
    
    //Configurar PWM
    PR2 = 250; //Valor inicial de PR2
    CCP1CONbits.P1M = 0; //PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; //Se habilita PWM   
    CCP2CONbits.CCP2M = 0b00001100;   
    
    CCPR1L = 0x0F; 
    CCPR2L = 0x0F;
    CCP1CONbits.DC1B = 0; //Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0; //Se limpia la bandera
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); //Se espera una interrupción
    PIR1bits.TMR2IF = 0;
    
    
    
    
    //Configuración de TX y RX
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 207;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;//configuraciones USART y tasa de baudios
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1;
    
    
    
    return;
}

void config_int_enable(){
    
    INTCONbits.GIE = 1; // Se habilitan las interrupciones globales
    
    INTCONbits.RBIE = 1; // habilitar banderas de interrupción puertos B
    INTCONbits.RBIF = 0; 	
    
    INTCONbits.PEIE = 1; //Enable interrupciones periféricas
    PIE1bits.ADIE = 1;   //Enable interrupción ADC
    PIR1bits.ADIF = 0;   //Se limpia bandera de interrupción ADC
    
    
    PIE1bits.RCIE = 1;// del eusart
    PIR1bits.RCIF = 0;
    
    return;
}
    

void config_iocb(){
    
    IOCB = 0xFF; // setear interrupciones en los pines RB0, RB1 y RB2
    
    INTCONbits.RBIF = 0;  
    
    return;
} 


void showString(char *var){ //subrutina de formacion de cadena de caracteres
    int i;
       
    for (i = 0; i < strlen(var); i++) { //bucle en donde lee el array de char y
        TXREG = var[i]; //lo mueve a la consola
        __delay_ms(5);
    }
    
    TXREG = 13; 
    __delay_ms(5);
    TXREG = 11;//nueva linea + retorno de carro para el salto de linea
   
}

  

