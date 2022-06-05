/* 
 * File:   P2P1maestro.c
 * Author: Aida Toloza, Brandon Cruz
 *
 * Created on 29 de mayo de 2022, 05:48 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config LVP = OFF

// CONFIG2
#pragma config BOR4V = BOR40V
#pragma config WRT = OFF

// librerias
#include <xc.h>
#include <stdint.h>

// constantes
#define _XTAL_FREQ  4000000             // Constante para __delay_us

//Variables 
uint8_t flag_EEPROM = 0;
uint8_t current_SERVO = 0;

uint8_t S1 = 0;
uint8_t s2 = 0;
uint8_t s3 = 0;
uint8_t s4 = 0;

uint8_t s3_READ = 0;
uint8_t s4_READ = 0;

//Funciones
void setup(void);                                   // En estas función se realizar configuraciones importantes 
                                                    // para que funcione el codigo, primeramente se realiza la Configuración del PIC
void reset_TMR0(void);                              // posteriormente se realiza el Resetear TMR0
void write_EEPROM(uint8_t address, uint8_t data);   // a su misma vez tenemos la escritura EEPROM

uint8_t read_EEPROM(uint8_t address);               // y por ultimo tenemos la Lectura EEPROM

//Interrupciones
void __interrupt() isr (void)           
{
    // utilizamos la confición if la cual nos va a ayudar a realizar la Interrupcion ADC de la mejor manera
    if (PIR1bits.ADIF)                  
    {
        // Posteriormente realizamos la configuración de Servo 1 el cual va a guardar, encender o apagar dependiendo de
        // que tipos de indicaciones le brindemos
        if (ADCON0bits.CHS == 0b0000)
        {
            S1 = ADRESH;
            data_SPI = ADRESH;          // Esta parte es muy importante dejarla clara debido a que es aquó donde va Guardar valor de la conversion
            PORTDbits.RD0 = 1;          // así mismo gracias a esto se va al Seleccionar Servo 1 el cual va a brindar la acción de encendido
            PORTDbits.RD1 = 0;          // asi mismo gracias a esto se va a seleccionar el servo 2 el cual va a brindar la acción de apagado
        }
        
       // Posteriormente realizamos la configuración de Servo 2 el cual va a guardar, encender o apagar dependiendo de
        // que tipos de indicaciones le brindemos
        else if (ADCON0bits.CHS == 0b0001)
        {
            s2 = ADRESH;
            data_SPI = ADRESH;          // Esta parte es muy importante dejarla clara debido a que es aquó donde va Guardar valor de la conversion
            PORTDbits.RD0 = 0;          // así mismo gracias a esto se va al Seleccionar Servo 1 el cual va a brindar la acción de apagado
            PORTDbits.RD1 = 1;          // / así mismo gracias a esto se va al Seleccionar Servo 2 el cual va a brindar la acción de encendido
        }
        
        // después de haber realizado al configuración de los dos primeros servos procedemos a la  configuracion Servo 3
        else if (ADCON0bits.CHS == 0b0010)
        {
            s3 = ADRESH;
            CCPR1L = (ADRESH>>1)+123;
            CCP1CONbits.DC1B = (ADRESH & 0b01);
            CCP1CONbits.DC1B0 = (ADRESH>>7);
        }
        
       // por ultimo al haber realizado al configuración de los tres primeros servos procedemos a la  configuracion Servo 4 para completar la configuración 
        else if (ADCON0bits.CHS == 0b0011)
        {
            s4 = ADRESH;
            CCPR2L = (ADRESH>>1)+123;
            CCP1CONbits.DC1B = (ADRESH & 0b01);
            CCP1CONbits.DC1B0 = (ADRESH>>7);
        }
        
        PIR1bits.ADIF = 0;      // Algo que es necesario para que funcionen es realizar la desactivación de las interrupciones    
                                // es por esto que se realiza esta configuración 
    }
    
    // Después de realizar la configuración de los sevomotores hay que realziar la configuracion de la Interrupcion TMR0
    //para que funcione correctamente. 
    
    if (INTCONbits.T0IF)                
    {
        reset_TMR0();                   // procedemos con lo aprendido en clase y empezamos con Resetear el TMR0
        INTCONbits.T0IF = 0;            // posteriormente se procede a desactivar la interrupcion
    }
    
    if (INTCONbits.RBIF)
    {
        // Ahora con respecto a las conficiones realizamos la configuración del Pushbutton 1 el cual 
        //tiene el modo de funcionamiento Escritura/Lectura para que funcione correctamente
        if (!PORTBbits.RB0)
        {
            flag_EEPROM = !flag_EEPROM;  // es aquí donde le decimos qe tiene Invertir valor de la bandera
            PORTEbits.RE0 = flag_EEPROM;
            ADCON0bits.ADON = flag_EEPROM;
        }
        
        // Posteriormente de haber realiza la configuración del Pushbottom, realizamos la configuración ddel Pushbutton 2
        // el cual realiza la Posicion 1 el cual tiene la función de registrar
        else if (!PORTBbits.RB1)
        {
            if (flag_EEPROM)             // Aqui se registra la posición 1
            {   
                write_EEPROM(0x00, S1);
                write_EEPROM(0x01, s2);
                write_EEPROM(0x02, s3);
                write_EEPROM(0x03, s4);
            }
            
            else
            {
                data_SPI = read_EEPROM(0x00);
                PORTDbits.RD0 = 1;
                PORTDbits.RD0 = 0;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                data_SPI = read_EEPROM(0x01);
                PORTDbits.RD0 = 0;
                PORTDbits.RD0 = 1;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                s3_READ = read_EEPROM(0x02);
                CCPR1L = (s3_READ>>1)+123;
                CCP1CONbits.DC1B = (s3_READ & 0b01);
                CCP1CONbits.DC1B0 = (s3_READ>>7);
                
                s4_READ = read_EEPROM(0x03);
                CCPR2L = (s4_READ>>1)+123;
                CCP1CONbits.DC1B = (s4_READ & 0b01);
                CCP1CONbits.DC1B0 = (s4_READ>>7);
            }
        }
        
        // en esta parte de la progrmación se realiza la confiduración de los pushbottom que tal y como se ve en la 
        // simulación es cuando el Pushbutton  el cual brinda tanto la Posicion 2 como lo mismo a registrar
        else if (!PORTBbits.RB2)
        {
            if (flag_EEPROM)            // primeramente realizamos la configuración 1 la cual realiza la acción de Registrar posicion 2
            {
                write_EEPROM(0x04, S1);
                write_EEPROM(0x05, s2);
                write_EEPROM(0x06, s3);
                write_EEPROM(0x07, s4);
            }
            
            else
            { 
                data_SPI = read_EEPROM(0x04);
                PORTDbits.RD0 = 1;
                PORTDbits.RD0 = 0;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                data_SPI = read_EEPROM(0x05);
                PORTDbits.RD0 = 0;
                PORTDbits.RD0 = 1;
                SSPBUF = data_SPI;
                
                __delay_ms(10);
                
                s3_READ = read_EEPROM(0x06);
                CCPR1L = (s3_READ>>1)+123;
                CCP1CONbits.DC1B = (s3_READ & 0b01);
                CCP1CONbits.DC1B0 = (s3_READ>>7);
                
                s4_READ = read_EEPROM(0x07);
                CCPR2L = (s4_READ>>1)+123;
                CCP1CONbits.DC1B = (s4_READ & 0b01);
                CCP1CONbits.DC1B0 = (s4_READ>>7);
            }
        }
        
        INTCONbits.RBIF = 0;            // interrupcion desactiva
    }
    
    // Configuración de la Interrupcion SPI
    if (PIR1bits.SSPIF)
    {
        PIR1bits.SSPIF = 0;             // interrupcion desactiva
    }
}

//MAIN
void main(void)                         
{
    setup();                            
    reset_TMR0();                       
    __delay_us(50);
    
    while (1)                         
    {
        // lo que tenemos que realizar primeramente en la configuración inicial es el 
        // control de canales e inicio de conversion ADC
        if (ADCON0bits.GO == 0)         
        {
            if (ADCON0bits.CHS == 0b0000)           // cambiamos de Canal 0000 -> 0001
            {
                ADCON0bits.CHS = 0b0001;
            }
            
            else if (ADCON0bits.CHS == 0b0001)      // cabiamos de Canal 0001 -> 0010
            {
                ADCON0bits.CHS = 0b0010;
            }
            
            else if (ADCON0bits.CHS == 0b0010)      // Cambiamos de Canal 0010 -> 0011
            {
                ADCON0bits.CHS = 0b0011;
            }
            
            else if (ADCON0bits.CHS == 0b0011)       // Cambiamos de Canal 0011 -> 0000
            {
                ADCON0bits.CHS = 0b0000;
            }
            
            __delay_us(50);             
            ADCON0bits.GO = 1;          
        }
        
        //Configuración para realizar la configuración de la Comunicacion Maestro-Esclavo (SPI) la cual
        // es necesario para poder realizar correctamente el funcionamente
        
        PORTAbits.RA7 = 0;              // primeramente realiamos la activación del esclavo 
        __delay_ms(10);                 // Programamos un delay
        
        SSPBUF = data_SPI;              // y procedemos a transmitir los datos
    }
    
    return;
}

// Acontinuación procedemos a realizar la configuración del PIC
void setup(void)                        
{
    // I/O
    ANSEL = 0x0F;                       //Definimos las Entradas analógicas
    ANSELH = 0;                         //a su misma vez las  I/O digitales
    
    // Oscilador
    OSCCONbits.IRCF = 0b0111;           // especificamos que el oscilador interno es de 4MHz
    OSCCONbits.SCS = 1;                 // Oscilador interno
    
    // Definimos cuales van a ser las Entradas
    TRISA = 0x0F;                       // Puerto A 
    PORTA = 0;                          
    
    TRISB = 0x07;                                        
    PORTB = 0;     
    
    TRISC = 0x10;                       // Pueto C 
    PORTC = 0;
    
    TRISD = 0x00;                          
    PORTD = 0;  
    
    TRISE = 0x00;
    PORTE = 0;
    
    // PORTB
    OPTION_REGbits.nRBPU = 0;           // Habilitar resistencias pull-up
    WPUBbits.WPUB0 = 1;                 //  habilitamos en RB0
    WPUBbits.WPUB1 = 1;                 // habilitamos en RB1
    WPUBbits.WPUB2 = 1;                 // habilitamos en RB2
    
    // ADC & PWMs
    ADCON0bits.ADCS = 0b01;             // utilizamos el  Reloj de conversión: Fosc/8
    ADCON0bits.CHS = 0b0000;            // donde definimos cual va a ser Canal para pin AN0
    ADCON1bits.VCFG0 = 0;               // Ref: VDD
    ADCON1bits.VCFG1 = 0;               // Ref: VSS
    ADCON1bits.ADFM = 0;                // lo Justificamos a la izquierda
    ADCON0bits.ADON = 1;                // Habilitamos el  ADC

    TRISCbits.TRISC2 = 1;               // Deshabilitar salida en CCP1
    CCP1CONbits.P1M = 0;                // Modo Single Output
    CCP1CONbits.CCP1M = 0b1100;         // PWM 1
    CCP2CONbits.CCP2M = 0b1111;         // PWM 2 
    
    CCPR1L = 0x0F;                    
    CCP1CONbits.DC1B = 0;     
    CCPR2L = 0x0F;
    PR2 = 250;               
    
    T2CONbits.T2CKPS = 0b11;
    T2CONbits.TMR2ON = 1;
    PIR1bits.TMR2IF = 0;

    while(PIR1bits.TMR2IF == 0);
        PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;               // y por ultimo realizamos la habilitación de la salida de PWM
    
    // Configuración del TMR0
    OPTION_REGbits.T0CS = 0;            //procedemos con el Reloj interno para el TMR0
    OPTION_REGbits.T0SE = 0;            // realizamos el Flanco de reloj ascendente
    OPTION_REGbits.PS2 = 1;             // 
    OPTION_REGbits.PS1 = 1;             //   utilizamso un preescaler de  1:256
    OPTION_REGbits.PS0 = 1;             //
    
    // SPI
    SSPCONbits.SSPM = 0b0000;           // SPI: Master mode, Clock: Fosc/4
    SSPCONbits.CKP = 0;                 // desactivamos el reloj al inicio
    SSPCONbits.SSPEN = 1;               // se realiza la habilitación de los pines SPI 
    SSPSTATbits.CKE = 1;                // Aqui realizamos la configuramos para que lo pueda transmitir en el  flanco positivo 
    SSPSTATbits.SMP = 1;                // y finalmente este tiene que Enviar al final del pulso de reloj
    
    // Interrupciones
    INTCONbits.GIE = 1;                 // configuración de interrupciones globales
    INTCONbits.PEIE = 1;                // configuración de interrupciones Perifericas
    
    PIE1bits.ADIE = 1;                  // ADC
    PIR1bits.ADIF = 0;                  // Bandera ADC
    
    INTCONbits.T0IF = 0;                // TMR0
    INTCONbits.T0IE = 1;                // Bandera TMR0
    
    PIE1bits.SSPIE = 1;                 // SPI
    PIR1bits.SSPIF = 0;                 // Bandera SPI
    
    INTCONbits.RBIE = 1;                // PORTB
    INTCONbits.RBIF = 0;                // Bandera PORTB    
    IOCBbits.IOCB0 = 1;                 // On-change RB0
    IOCBbits.IOCB1 = 1;                 // On-change RB1
    IOCBbits.IOCB2 = 1;                 // On-change RB2
    
    // configuración de Valores iniciales
    PORTAbits.RA4 = 1;                  // Indicador Maestro
    SSPBUF = 0x00;                      // Dato inicial a transmitir
}

void reset_TMR0(void)                   // Resetear TMR0
{
    TMR0 = 0;                           // Prescaler
    INTCONbits.T0IF = 0;                // Limpiar bandera de interrupcion
    return;
}

void write_EEPROM(uint8_t address, uint8_t data)
{
    EEADR = address;
    EEDAT = data;
    
    INTCONbits.GIE = 0;             // Deshabilitar interrupciones
    
    EECON1bits.EEPGD = 0;           // Escribir EEPROM
    EECON1bits.WREN = 1;            // Habilitar escritura EEPROM
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;              // Iniciar escritura
    
    while(PIR2bits.EEIF == 0);
    PIR2bits.EEIF = 0;
    
    EECON1bits.WREN = 0;            // Deshabilitar escritura EEPROM
    
    INTCONbits.RBIF = 0;            // Limpiar bandera de interrupcion
    INTCONbits.GIE = 1;             // Habilitar interrupciones
    
    return;
}

uint8_t read_EEPROM(uint8_t address)
{
    EEADR = address;
    
    EECON1bits.EEPGD = 0;           // Lectura EEPROM
    EECON1bits.RD = 1;              // Obtener dato de la EEPROM
    
    return EEDAT;                   // Retornar valor leido
}