
#include <ioCC2530.h>
#include "hal_cc8051.h"
#include "board.h"
#include "delay.h"
#include "uart_intfc.h"
#include "hal_board_defs.h"
#include "hal_cc8051.h"

extern volatile uint8_t Timer1_flag;
extern volatile uint8_t Timer4_flag;
extern Msg_t edgeMsg;
extern volatile uint16_t time1_cntflag;
extern volatile uint8_t Uart_flag;
extern uint32_t LocalTime;
extern volatile uint8_t time4_cntflag;
extern uint8_t sizeofedgeMsg;
//*****************************************************************************
//InitClk
//*****************************************************************************    
void InitClk()
{
    CLKCONCMD = 0x0;           //时器计数时钟设定为32M Hz,  系统时钟设定为32 MHz 0010 1000 
    while(CLKCONSTA & 0x40);    //等晶振稳定 0100 0000
}

/****************************************************************
初始化SPI0
CC2530_SPI0_MISO         P0_2
CC2530_SPI0_MOSI         P0_3
CC2530_SPI0_SSN          P0_4
CC2530_SPI0_SCK          P0_5
------------------------------------------------------------------*/ 
void SPI_INIT()
{
    PERCFG &= ~0x01;                  // PERCFG.U0CFG = 0，Set SPI on UART 0 alternative 1 
    P0SEL |= 0x2C;                   // P0_2, P0_3, P0_5 are peripherals 
    P0SEL &= ~0x10;                  // P0_4 is GPIO (SSN) 
    P0DIR |= 0x10;                   // SSN is set as output 
    U0GCR = 11; 
    U0BAUD = 216;                    //115200
    U0CSR &= ~0xA0;                  // SPI Master Mode  
    U0GCR &= ~0x80;                  // CPOL  = 0 
    U0GCR |= 0x40;                   //  CPHA  = 1
    U0GCR |= 0x20;                   // ORDER = 1   MSB first  
}

/////SPI 写一个字节
void SPI_Write(unsigned char Command)
{
    U0DBUF=Command;
    while (!(U0CSR&0x02)){} 
    U0CSR &= 0xFD;//复位
}   
////SPI 读一个字节
unsigned char SPI_Read()
{
    U0DBUF = 0x00;//废数据，为了输出SCK时钟
    while (!(U0CSR&0x02)){} //等待发送完成
    U0CSR &= 0xFD;//复位
    unsigned char temp = U0DBUF;//读取UxBUFF时，硬件会自动清零
    return temp;  
}
void SPI_Write_buffer(unsigned char* buff, uint16_t len)
{
    P0_4=1;
    P0_4=0;
    uint8_t i;
    for(i=0;i++;i<len)
    {
        SPI_Write(buff[i]);
    }
    P0_4=1;
}
//*****************************************************************************
//LED   
//***************************************************************************** 
void InitLED()
{
    P1DIR|=BIT7|BIT6;        
}
void TurnOnLed1()
{
    LED1=0;       
}
void TurnOnLed2()
{
    LED2=0;       
}
void TurnOffLed1()
{
    LED1=1;       
}
void TurnOffLed2()
{
    LED2=1;       
}
void ToggleLed1()
{
    LED1^=1;       
}
void ToggleLed2()
{
    LED2^=1;       
}
//*****************************************************************************
//外部中断，超声接收
//***************************************************************************** 
void init_external_interrupt()
{                              
    init_us1();
    init_us2();
    init_us3();
    //init_us4();
    //init_us5();
    //init_us6();
    //init_us7();
    //init_us8();
    //init_us9();
    //init_us10();
    PICTL&=~0x0F;//rising edge tri
    //P0IFG=0;
    P1IFG=0;
    //IEN1|=BIT5;//enable port 0 interrupt
    IEN2|=BIT4;//enable port 1 interrupt
    EA=1;
}
void init_us1()
{
  P1SEL&=~BIT5;
  P1DIR&=~BIT5;
  P1IEN|=BIT5;
}
void init_us2()
{
  P1SEL&=~BIT4;
  P1DIR&=~BIT4;
  P1IEN|=BIT4;
}
void init_us3()
{
  P1SEL&=~BIT3;
  P1DIR&=~BIT3;
  P1IEN|=BIT3;
}
void init_us4()
{
  P1SEL&=~BIT2;
  P1DIR&=~BIT2;
  P1IEN|=BIT2;
}
void init_us5()
{
  P1SEL&=~BIT1;
  P1DIR&=~BIT1;
  P1IEN|=BIT1;
}
void init_us6()
{
  P1SEL&=~BIT0;
  P1DIR&=~BIT0;
  P1IEN|=BIT0;
}
void init_us7()
{
  P0SEL&=~BIT7;
  P0DIR&=~BIT7;
  P0IEN|=BIT7;
}
void init_us8()
{
  P0SEL&=~BIT6;
  P0DIR&=~BIT6;
  P0IEN|=BIT6;
}
void init_us9()
{
  P0SEL&=~BIT1;
  P0DIR&=~BIT1;
  P0IEN|=BIT1;
}
void init_us10()
{
  P0SEL&=~BIT0;
  P0DIR&=~BIT0;
  P0IEN|=BIT0;
}
//*****************************************************************************
//timer
//***************************************************************************** 
/*
void hal_start_timer1(){
    T1CCTL0 = 0x44;  //Compare Mode on Channel 0 
    T1CC0H = 0xff&(timeSlot>>8);      //count max value
    T1CC0L = 0xff&timeSlot;  

    T1CCTL1 = 0x44;  //Compare Mode on Channel 1
    T1CC1H = 0xff & (timeOut>>8);      //count max value
    T1CC1L = 0xff & timeOut;  

    T1CNTH = 0x00;
    T1CNTL = 0x00;
    EA = 0;
    T1IF = 1;  // clear Flag on Timer 1 
    T1STAT &= ~(BIT0|BIT1);  //Clear Flag on channel 0 and channel 1
    T1IE = 1; // init interrupt on Timer 1
    EA = 1;
    T1CTL = 0x0E;     //128 div and count to T1CC0
}
void hal_stop_timer1(){
    T1CTL &= 0x00;
}
*/   
void init_timer1()
{
    T1CCTL0 = 0x44;             
    T1CC0H = 0xff&(time1_cntflag>>8);
    T1CC0L = 0xff&time1_cntflag;     
    T1CNTH = 0x00;              
    T1CNTL = 0x00;              
    T1IF = 1;                   
    T1STAT &= ~BIT0;            
    T1IE = 1;                   
    T1CTL = 0x0E;               
}
void init_timer4()
{ 
    T4CCTL0=0x44;
    T4CC0=time4_cntflag;
    IRCON &= ~0x10;   
    T4CTL=0xf2;     //32分频, 每4us CNT++, 即1ms中断
    IEN1 |= 0x10;  
    EA=1;
}
/****************************
//中断处理函数
*****************************/

#pragma vector = T1_VECTOR 
__interrupt void TIMER1_ISR(void) 
{ 
    T1IF = 0;
    if(T1STAT & BIT0){ //Interrupt on Channel 0 
	Uart_flag = 1; //Trigger UART output
	T1STAT &= ~BIT0; //clear interrupt 
    }
    //T1STAT &= ~(BIT0|BIT1);
}
#pragma vector = T4_VECTOR 
__interrupt void TIMER4_ISR(void) 
{ 
  IRCON &= ~0x10;
  LocalTime++;
}
/*
#pragma vector = P0INT_VECTOR
__interrupt void P0_ISR(void)
{  
  if(P0IFG>0)
  {
    uint16_t TL,TH;
    uint16_t THL;
    TL=T1CNTL;
    TH=T1CNTH;
    THL = (TH<<8)+TL;
    if((P0IFG&BIT7) && (edgeMsg.risingEdge[6]==0))           
    {
	P0IFG&=~BIT7; 
        edgeMsg.risingEdge[6]=THL;
        edgeMsg.numRising ++;
    }
    if((P0IFG&BIT6)  && (edgeMsg.risingEdge[7]==0))           
    {
	P0IFG&=~BIT6; 
        edgeMsg.risingEdge[7]=THL;
        edgeMsg.numRising ++;
    }
    if((P0IFG&BIT1) && (edgeMsg.risingEdge[8]==0))           
    {
	P0IFG&=~BIT1; 
        edgeMsg.risingEdge[8]=THL;
        edgeMsg.numRising ++;
    }
    if((P0IFG&BIT0) && (edgeMsg.risingEdge[9]==0))           
    {
	P0IFG&=~BIT0; 
        edgeMsg.risingEdge[9]=THL;
        edgeMsg.numRising ++;
    }
    IRCON&=~BIT5;   
  }
}
*/
#pragma vector = P1INT_VECTOR
__interrupt void P1_ISR(void)
{
  if(P1IFG>0)
  {
    uint16_t TL,TH;
    uint16_t THL;
    TL=T1CNTL;
    TH=T1CNTH;
    THL = (TH<<8)+TL;
    if((P1IFG&BIT5) && (edgeMsg.risingEdge[0]==0))
    {
      edgeMsg.risingEdge[0]=THL;
      edgeMsg.numRising ++;
      P1IFG&=~BIT5;
    }
    if((P1IFG&BIT4) && (edgeMsg.risingEdge[1]==0))
    {
      edgeMsg.risingEdge[1]=THL;
      edgeMsg.numRising ++;
      P1IFG&=~BIT4;
    }
    if((P1IFG&BIT3) && (edgeMsg.risingEdge[2]==0))
    {
      edgeMsg.risingEdge[2]=THL;
      edgeMsg.numRising ++;
      P1IFG&=~BIT3;
    }
    /*if((P1IFG&BIT2) && (edgeMsg.risingEdge[3]==0))
    {
      edgeMsg.risingEdge[3]=THL;
      edgeMsg.numRising ++;
      P1IFG&=~BIT2;
    }
    if((P1IFG&BIT1) && (edgeMsg.risingEdge[4]==0))
    {
      edgeMsg.risingEdge[4]=THL;
      edgeMsg.numRising ++;
      P1IFG&=~BIT1;
    }
    if((P1IFG&BIT0) && (edgeMsg.risingEdge[5]==0))
    {
      edgeMsg.risingEdge[5]=THL;
      edgeMsg.numRising ++;
      P1IFG&=~BIT0;
    }*/
    IRCON2&=~BIT3; 
  }
}

//Catch interrupt vectors that are not initialized.

#ifdef __CCS__
#pragma vector=PORT1_VECTOR, WDT_VECTOR, TIMER1_A1_VECTOR, TIMER0_A1_VECTOR, TIMER0_A0_VECTOR, ADC10_VECTOR, UNMI_VECTOR,COMP_D_VECTOR,	DMA_VECTOR, PORT3_VECTOR, PORT4_VECTOR, RTC_VECTOR, TIMER0_B0_VECTOR, TIMER0_B1_VECTOR, TIMER1_B0_VECTOR, TIMER1_B1_VECTOR, TIMER2_B0_VECTOR, TIMER2_B1_VECTOR,SYSNMI_VECTOR, USCI_A1_VECTOR, USCI_B0_VECTOR
__interrupt void Trap_ISR(void)
{
  while(1);
}

#endif


