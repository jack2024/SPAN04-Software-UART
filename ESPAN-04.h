#include <18F2525.h>
//#device adc=8
#FUSES WDT
#FUSES WDT64                    //Watch Dog Timer uses 1:64 Postscale
#FUSES HS                       //High speed Osc (> 4mhz)
#FUSES PROTECT                //Code not protected from reading
//#FUSES IESO                     //Internal External Switch Over mode enabled
#FUSES BROWNOUT                 //Reset when brownout detected
#FUSES BORV20                   //Brownout reset at 2.0V
#FUSES NOPUT                    //No Power Up Timer
#FUSES NOCPD                    //No EE protection
//#FUSES STVREN                   //Stack full/underflow will cause reset
#FUSES NODEBUG                  //No Debug mode for ICD
//#FUSES LVP                      //Low Voltage Programming on B3(PIC16) or B5(PIC18)
//#FUSES NOWRT                    //Program memory not write protected
//#FUSES NOWRTD                   //Data EEPROM not write protected
#FUSES NOEBTR                   //Memory not protected from table reads
#FUSES NOCPB                    //No Boot Block code protection
#FUSES NOEBTRB                  //Boot block not protected from table reads
#FUSES NOWRTC                   //configuration not registers write protected
#FUSES NOWRTB                   //Boot block not write protected
//#FUSES FCMEN                    //Fail-safe clock monitor enabled
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES PBADEN                   //PORTB pins are configured as analog input channels on RESET
#FUSES LPT1OSC                  //Timer1 configured for low-power operation
#FUSES MCLR                     //Master Clear pin enabled

#define CLOCK_SP   20000000   //Clock Speed 20MHz

//23S17 Define
#define IO_MULTIPLE_DEVICES
#define IO_CS_PIN PIN_A5

#use delay(clock=20000000,RESTART_WDT)
//#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8)

/*
#include <16F886.h>
#device adc=10

#FUSES WDT                      //Watch Dog Timer
#FUSES HS                       //High speed Osc (> 4mhz)
#FUSES NOPUT                    //No Power Up Timer
//#FUSES MCLR                     //Master Clear pin enabled
#FUSES PROTECT                  //Code protected from reads
//#FUSES CPD                      //Data EEPROM Code Protected
#FUSES NOBROWNOUT               //No brownout reset
//#FUSES IESO                     //Internal External Switch Over mode enabled
//#FUSES FCMEN                    //Fail-safe clock monitor enabled
//#FUSES LVP                      //Low Voltage Programming on B3(PIC16) or B5(PIC18)
#FUSES NODEBUG                  //No Debug mode for ICD
//#FUSES BORV21                   //Brownout reset at 2.1V
//#FUSES WRT                      //Program Memory Write Protected

#use delay(clock=20000000,RESTART_WDT)
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,restart_wdt)
*/

