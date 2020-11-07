/*
 * File:   rover.c
 * Author: guy
 *
 * Created on September 22, 2018, 8:46 AM
 */

#include <xc.h>
#include <stdio.h>
#include "HC05.h"
#include <pic18f4550.h>

//#pragma config FOSC = HS    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config FCMEN = ON
#pragma config PBADEN = OFF				// RB0, RB1, RB2, RB3, & RB4 are configured as digital I/O on reset
#pragma config WDT = OFF
#pragma config IESO = OFF
#pragma config LVP = OFF
#pragma config MCLRE = OFF

/*********************Definition of Ports********************************/

#define RS LATD0  /*PIN 0 of PORTB is assigned for register select Pin of LCD*/
#define EN LATD1  /*PIN 1 of PORTB is assigned for enable Pin of LCD */
#define ldata LATD  /*PORTB(PB4-PB7) is assigned for LCD Data Output*/ 
#define LCD_Port TRISD  /*define macros for PORTB Direction Register*/

/*********************Proto-Type Declaration*****************************/

void MSdelay(unsigned int );       /*Generate delay in ms*/
void LCD_Init();                   /*Initialize LCD*/
void LCD_Command(unsigned char );  /*Send command to LCD*/
void LCD_Char(unsigned char x);    /*Send data to LCD*/
void LCD_String(const char *);     /*Display data string on LCD*/
void LCD_String_xy(char, char , const char *);
void LCD_Clear();                  /*Clear LCD Screen*/


#define _XTAL_FREQ 8000000

// Bluetooth

/*****************************USART Initialization*******************************/
void USART_Init(long baud_rate)
{
    float temp;
    TRISC6=0;                       /*Make Tx pin as output*/
    TRISC7=1;                       /*Make Rx pin as input*/
    temp=Baud_value;     
    SPBRG=(int)temp;                /*baud rate=9600, SPBRG = (F_CPU /(64*9600))-1*/
    TXSTA=0x20;                     /*Transmit Enable(TX) enable*/ 
    RCSTA=0x90;                     /*Receive Enable(RX) enable and serial port enable */
}
/******************TRANSMIT FUNCTION*****************************************/ 
void USART_TransmitChar(char out)
{        
        while(TXIF==0);            /*wait for transmit interrupt flag*/
        TXREG=out;                 /*wait for transmit interrupt flag to set which indicates TXREG is ready
                                    for another transmission*/    
}
/*******************RECEIVE FUNCTION*****************************************/
char USART_ReceiveChar()
{

    while(RCIF==0);                 /*wait for receive interrupt flag*/
    return(RCREG);                  /*receive data is stored in RCREG register and return to main program */
}

void USART_SendString(const char *out)
{
   while(*out!='\0')
   {            
        USART_TransmitChar(*out);
        out++;
   }
}


void LCD_Init()
{
    LCD_Port = 0;       /*PORT as Output Port*/
    MSdelay(15);        /*15ms,16x2 LCD Power on delay*/
    LCD_Command(0x02);  /*send for initialization of LCD 
                          for nibble (4-bit) mode */
    LCD_Command(0x28);  /*use 2 line and 
                          initialize 5*8 matrix in (4-bit mode)*/
	LCD_Command(0x01);  /*clear display screen*/
    LCD_Command(0x0c);  /*display on cursor off*/
	LCD_Command(0x06);  /*increment cursor (shift cursor to right)*/	   
}

void LCD_Command(unsigned char cmd )
{
	ldata = (ldata & 0x0f) |(0xF0 & cmd);  /*Send higher nibble of command first to PORT*/ 
	RS = 0;  /*Command Register is selected i.e.RS=0*/ 
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/ 
	NOP();
	EN = 0;
	MSdelay(1);
    ldata = (ldata & 0x0f) | (cmd<<4);  /*Send lower nibble of command to PORT */
	EN = 1;
	NOP();
	EN = 0;
	MSdelay(3);
}

void LCD_Char(unsigned char dat)
{
	ldata = (ldata & 0x0f) | (0xF0 & dat);  /*Send higher nibble of data first to PORT*/
	RS = 1;  /*Data Register is selected*/
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/
	NOP();
	EN = 0;
	MSdelay(1);
    ldata = (ldata & 0x0f) | (dat<<4);  /*Send lower nibble of data to PORT*/
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/
	NOP();
	EN = 0;
	MSdelay(3);
}

void LCD_String(const char *msg)
{
	while((*msg)!=0)
	{		
	  LCD_Char(*msg);
	  msg++;	
    }
}

void LCD_String_xy(char row,char pos,const char *msg)
{
    char location=0;
    if(row<=1)
    {
        location=(0x80) | ((pos) & 0x0f);  /*Print message on 1st row and desired location*/
        LCD_Command(location);
    }
    else
    {
        location=(0xC0) | ((pos) & 0x0f);  /*Print message on 2nd row and desired location*/
        LCD_Command(location);    
    }  
    

    LCD_String(msg);

}
void LCD_Clear()
{
   	LCD_Command(0x01);  /*clear display screen*/
    MSdelay(3);
}

void MSdelay(unsigned int val)
{
 unsigned int i,j;
 for(i=0;i<val;i++)
     for(j=0;j<165;j++);  /*This count Provide delay of 1 ms for 8MHz Frequency */
 }

void BinToASCII(unsigned int value)
{
	unsigned int x,d1,d2,d3;
    x=value/10;
	d1=value%10;
	d2=x%10;
	d3=x/10;
    char buffer[3];
	char dg1 = 0x30 | d1;
	char dg2 = 0x30 | d2;
	char dg3 = 0x30 | d3;
    //char str[1] = {dg3};
    char str[3];

    sprintf(str, "Encoder: %u%u%u", d3, d2,d1);
    
    LCD_String_xy(1,1,str);  /*Display string on 1st row, 5th location*/

}



void right(){
    LCD_String_xy(2,1,"Right     ");
    __delay_ms(100);
    PORTB=0x35;
  //  int i;
   // for(i=0;i<times;i++)
   //     {__delay_ms(1000);}  //time to drive 50 cm
   // PORTB=0x30;
    int counter = 0;
    char temp = PORTDbits.RD3;
//    BinToASCII(123);
//    while(1){
//    if(temp != PORTDbits.RD3){
//        counter++;
//        BinToASCII(counter/2);
//        temp = PORTDbits.RD3;
//    }
//    if( counter / 2 > 22 && counter / 2 < 26){
//        PORTB = 0x30;
//        break;
//    }
//    }
     int value =0;
    while(1)
	{
		
			T0CONbits.TMR0ON = 1;	//turn on T0
			value = (TMR0H *256) + TMR0L ;
			BinToASCII(value);
            if((value) > 60){
               // BinToASCII(value);
                PORTB = 0x30;
              //  T0CONbits.TMR0ON = 0;
              //  INTCONbits.TMR0IF = 0;
                break;
            }
		//wait for TF0 to roll over
    }
	T0CONbits.TMR0ON = 0;		//turn off T0
	INTCONbits.TMR0IF = 0;
    TRISAbits.TRISA4 = 1;
    T0CON = 0x20;
    TMR0H = 0;
	TMR0L = 0;	
    
}

//Motor A Forward AND Motor B Backward
void left(){
    LCD_String_xy(2,1,"Left     ");
    __delay_ms(100);
    PORTB=0x3A;
  //  int i;
   // for(i=0;i<times;i++)
   //     {__delay_ms(1000);}  //time to drive 50 cm
   // PORTB=0x30;
    int counter = 0;
    char temp = PORTDbits.RD3;
//    BinToASCII(123);
//    while(1){
//    if(temp != PORTDbits.RD3){
//        counter++;
//        BinToASCII(counter/2);
//        temp = PORTDbits.RD3;
//    }
//    if( counter / 2 > 22 && counter / 2 < 26){
//        PORTB = 0x30;
//        break;
//    }
//    }
     int value =0;
    while(1)
	{
		
			T0CONbits.TMR0ON = 1;	//turn on T0
			value = (TMR0H *256) + TMR0L ;
			BinToASCII(value);
            if((value) > 60){
               // BinToASCII(value);
                PORTB = 0x30;
              //  T0CONbits.TMR0ON = 0;
              //  INTCONbits.TMR0IF = 0;
                break;
            }
		//wait for TF0 to roll over
    }
	T0CONbits.TMR0ON = 0;		//turn off T0
	INTCONbits.TMR0IF = 0;
    TRISAbits.TRISA4 = 1;
    T0CON = 0x20;
    TMR0H = 0;
	TMR0L = 0;	
    
}




//Motor A Forward AND Motor B Forward
void forward(int times){
    LCD_String_xy(2,1,"Forward   ");
    __delay_ms(100);
    PORTB=0x39;
  //  int i;
   // for(i=0;i<times;i++)
   //     {__delay_ms(1000);}  //time to drive 50 cm
   // PORTB=0x30;
    int counter = 0;
    char temp = PORTDbits.RD3;
//    BinToASCII(123);
//    while(1){
//    if(temp != PORTDbits.RD3){
//        counter++;
//        BinToASCII(counter/2);
//        temp = PORTDbits.RD3;
//    }
//    if( counter / 2 > 22 && counter / 2 < 26){
//        PORTB = 0x30;
//        break;
//    }
//    }
     int value =0;
    while(1)
	{
		
			T0CONbits.TMR0ON = 1;	//turn on T0
			value = (TMR0H *256) + TMR0L ;
			BinToASCII(value);
            if((value) > 140){
               // BinToASCII(value);
                PORTB = 0x30;
              //  T0CONbits.TMR0ON = 0;
              //  INTCONbits.TMR0IF = 0;
                break;
            }
		//wait for TF0 to roll over
    }
		T0CONbits.TMR0ON = 0;		//turn off T0
		INTCONbits.TMR0IF = 0;
    TRISAbits.TRISA4 = 1;
    T0CON = 0x20;
    TMR0H = 0;
	TMR0L = 0;	
    
    
    
}
void backward(int times){
    LCD_String_xy(2,1,"Backward  ");
    __delay_ms(100);
    PORTB=0x36;
  //  int i;
   // for(i=0;i<times;i++)
   //     {__delay_ms(1000);}  //time to drive 50 cm
   // PORTB=0x30;
    int counter = 0;
    char temp = PORTDbits.RD3;
//    BinToASCII(123);
//    while(1){
//    if(temp != PORTDbits.RD3){
//        counter++;
//        BinToASCII(counter/2);
//        temp = PORTDbits.RD3;
//    }
//    if( counter / 2 > 22 && counter / 2 < 26){
//        PORTB = 0x30;
//        break;
//    }
//    }
     int value =0;
    while(1)
	{
		
			T0CONbits.TMR0ON = 1;	//turn on T0
			value = (TMR0H *256) + TMR0L ;
			BinToASCII(value);
            if((value) > 140){
               // BinToASCII(value);
                PORTB = 0x30;
              //  T0CONbits.TMR0ON = 0;
              //  INTCONbits.TMR0IF = 0;
                break;
            }
		//wait for TF0 to roll over
    }
	T0CONbits.TMR0ON = 0;		//turn off T0
	INTCONbits.TMR0IF = 0;
    TRISAbits.TRISA4 = 1;
    T0CON = 0x20;
    TMR0H = 0;
	TMR0L = 0;	
    
    
}
void main()
{
    OSCCON = 0x72;
    TRISD = 0;
    TRISB = 0;
    PORTD = 0xFF;
    PORTB = 0X00;
    
    PORTC = 0xFF;
    TRISAbits.TRISA4 = 1;
    T0CON = 0x20;
    TMR0H = 0;
	TMR0L = 0;	
   
    char data_in =0;
    USART_Init(9600);
    LCD_Init();
    
    MSdelay(50);
    USART_SendString("Connected");
    while(1){
   
    //USART_SendString(" ");
    data_in=USART_ReceiveChar();
    if(data_in=='1'){   
        forward(1);
        PORTCbits.RC6 = 0;
        PORTCbits.RC7 = 0;
        //USART_SendString("Forward"); /* send LED ON status to terminal */
    }
    if(data_in=='2'){   
        backward(1);
        PORTCbits.RC6 = 0;
        PORTCbits.RC7 = 0;
        //USART_SendString("Backward"); /* send LED ON status to terminal */
    }
     if(data_in=='3'){   
        left();
        PORTCbits.RC6 = 0;
        PORTCbits.RC7 = 0;
        //USART_SendString("Left"); /* send LED ON status to terminal */
    }
    if(data_in=='4'){   
        right();
        PORTCbits.RC6 = 0;
        PORTCbits.RC7 = 0;
       // USART_SendString("Right"); /* send LED ON status to terminal */
    }
     MSdelay(100);
    }
    
   // forward(1);
   // left();
   // forward(1);
   // backward(1);
   // right();
   // backward(1);

    
	  /*Initialize LCD to 5*8 matrix in 4-bit mode*/    
//	LCD_String_xy(1,1,"Hello");  /*Display string on 1st row, 5th location*/
//    LCD_String_xy(2,1,"Guy");  /*Display string on 2nd row,1st location*/ 
   // MSdelay(100);
   
//    backward(1);
    
//    LCD_String_xy(1,1,"Ayman");  /*Display string on 1st row, 5th location*/
  
    
    
  while(1){
  }
}

