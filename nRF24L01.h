#ifndef __nRF24L01_H
#define __nRF24L01_H


#include <nRF24L01.h>
#include <stdio.h>
#include <delay.h>
#include "stm32f1xx_hal.h"
// SPI functions



#define CS_LOW       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define CS_HIGH      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define CSN_LOW      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define CSN_HIGH     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE 0x50
#define R_RX_PL_WID 0x60
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF


/* Memory Map */
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D


extern void debug_flash(const char *);
extern void debug( char *);
extern void debug_number( char *pointer,uint32_t number);

//#pragma used+
/* library function prototypes */
extern SPI_HandleTypeDef hspi1;
unsigned char Base_Addrs[5]={0xcc,0xcc,0xcc,0xcc,0xcc};
unsigned char Temp_Addrs[5]={0xcc,0xcc,0xcc,0xcc,0xcc};
unsigned char payload[33];
unsigned char Command_Reg = 0,Status_Reg = 0,State = 0;
char Opr_Mode;
char send_actived = 0;


void Set_Reg(unsigned char ins)
{
    int i;                              
    CSN_LOW;    
    //Status_Reg=spi(ins);
		uint8_t time_out=10;
		while(HAL_SPI_TransmitReceive(&hspi1,&ins,&Status_Reg,1,HAL_MAX_DELAY)!=HAL_OK && time_out)time_out--; 
    switch(ins & 0xE0)
    {
        case 0x00:
        {                         
            if((ins & 0x1F)==0x0A || (ins & 0x1F)==0x0B || (ins & 0x1F)==0x10) 
            {
               // for(i=4;i>=0;i--)
                {unsigned char txbuffer[4]={NOP};
						uint8_t time_out=100;
                    //Temp_Addrs[i]=spi(NOP);
										while(HAL_SPI_TransmitReceive(&hspi1,txbuffer,Temp_Addrs,4,HAL_MAX_DELAY)!=HAL_OK&& time_out)time_out--;
                }
            }
            else
            {
							unsigned char txbuffer=NOP;
							uint8_t time_out=10;
                //Command_Reg=spi(NOP);
								while(HAL_SPI_TransmitReceive(&hspi1,&txbuffer,&Command_Reg,1,HAL_MAX_DELAY)!=HAL_OK&& time_out)time_out--;
            }
            break;
        }
        case W_REGISTER:  
        {                         
            if((ins & 0x1F)==0x0A || (ins & 0x1F)==0x0B || (ins & 0x1F)==0x10)
            {
               
                { 
                    uint8_t time_out=10;
										while(HAL_SPI_Transmit(&hspi1,Base_Addrs,5,HAL_MAX_DELAY)!=HAL_OK && time_out)time_out--;
                } 
            }
            else
            { uint8_t time_out=10;
                //spi(Command_Reg);
								while(HAL_SPI_Transmit(&hspi1,&Command_Reg,1,HAL_MAX_DELAY)!=HAL_OK && time_out)time_out--;
								
            }
            break;
        }         
        case R_RX_PL_WID:
        {
            if((ins & 0x01)==1)
            {
                i=payload[0];
               // while(i!=0)
                {
                    //payload[i]=spi(NOP); 
								unsigned char txbuffer[30]={NOP};
								uint8_t time_out=10;
								while(HAL_SPI_TransmitReceive(&hspi1,txbuffer,payload,i,HAL_MAX_DELAY)!=HAL_OK && time_out)time_out--;
                 //   i--;
                }    
            }
            else 
            {
                //Command_Reg=spi(NOP); 
								unsigned char txbuffer=NOP;
								uint8_t time_out=10;
								while(HAL_SPI_TransmitReceive(&hspi1,&txbuffer,&Command_Reg,1,HAL_MAX_DELAY)!=HAL_OK && time_out)time_out--;
            }
            break;
        }
        case W_TX_PAYLOAD:
        {    
            i=payload[0];
         //   while(i!=0)
            {
               // spi(payload[i]); 
								while(HAL_SPI_Transmit(&hspi1,payload,i+1,HAL_MAX_DELAY)!=HAL_OK && time_out)time_out--;
                i--;
            }
            break;
        }
        
    }   
    CSN_HIGH;  
  //  delay_us(10);
}




// External Interrupt 2 service routine
void ext_int1_isr(void)
{
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
// Place your code here
    if(Opr_Mode==0)
    {
        Set_Reg(NOP); 
        if(Status_Reg & W_REGISTER)  
        {
            State = 2;
            Set_Reg(FIFO_STATUS);        
            if((Command_Reg & 0x01)==0)
            {
                Set_Reg(R_RX_PL_WID);  
                if(Command_Reg<=32)
                {
                    payload[0]=Command_Reg; 
                    Set_Reg(R_RX_PAYLOAD);    
                    State = 3;
                }
                else
                    Set_Reg(FLUSH_RX);
            }          
        }
        else 
        {
            State = 4;
        }
    }                                                     
    else
    {
        Set_Reg(R_RX_PL_WID); 
        if(Command_Reg>32)
        {
            Set_Reg(FLUSH_RX);
        }
        else
        {
            payload[0]=Command_Reg;
            Set_Reg(R_RX_PAYLOAD);
            State = 1;
        }
    }     
    Command_Reg = 0x7E; 
    Set_Reg(0x27);    
    Set_Reg(FLUSH_TX);        
}





void Send_Data(char num , char *data)
{ 
    int counter = 0; 
		CS_LOW; 
    Opr_Mode = 0; 
    Command_Reg = 0x4E;
    Set_Reg(W_REGISTER);   
    send_actived = 1;
    delay_ms(10);
   // CS_HIGH; 
       
    for(counter=0;counter<32;counter++)
        payload[counter] = 0;
		
		
    payload[0] = num;
    
    for(counter=0;counter<num;counter++)
        payload[counter+1] = *(data+counter);
 
//   if(send_actived)
//    {
        send_actived = 0;  
        if((Temp_Addrs[4]==Base_Addrs[4]) && (Temp_Addrs[3]==Base_Addrs[3]) && (Temp_Addrs[2]==Base_Addrs[2]) && (Temp_Addrs[1]==Base_Addrs[1]) && (Temp_Addrs[0]==Base_Addrs[0]))
        {
					//uint8_t time_out=10;
					//State=0;
				//	do{
            Set_Reg(FLUSH_TX); 
            Set_Reg(W_TX_PAYLOAD); 
            delay_us(1);
            CS_HIGH;
            delay_us(10);
            CS_LOW;
            delay_ms(10);
					//	time_out--;
					//}while(State != 2 && time_out);
        }
        else
            State = 5; 
 //   } 
    
    if(State!=0)
        {
        send_actived=1; //faal sazi ersal mojadad
        State=0;
        }

        CS_LOW; 
        Opr_Mode = 1; 
        Command_Reg = 0x3F;
        Set_Reg(W_REGISTER);  
				delay_ms(5);				
        CS_HIGH;
}



void nRF_Config(void)
{

    CSN_HIGH;
    CS_LOW;
    
              
    delay_ms(110);
       
    
    Command_Reg = 0x01;
    Set_Reg(0x21);   
    
    Command_Reg = 0x01;
    Set_Reg(0x22);   
      
    Command_Reg = 0x03;
    Set_Reg(0x23);   
    
    Command_Reg = 0x2f;
    Set_Reg(0x24);   
                       
    Command_Reg = 0x01;   
    Set_Reg(0x25);      
    
    Command_Reg = 0x06;   
    Set_Reg(0x26);      
    
    Set_Reg(0x2A);      
    
    Set_Reg(0x30);      
    
    Command_Reg = 0x01;   
    Set_Reg(0x3C);      
        
    Command_Reg = 0x07;   
    Set_Reg(0x3D);          
    
		Opr_Mode = 1; 
        Command_Reg = 0x3F;  //reciver
        Set_Reg(W_REGISTER);   
        delay_ms(5);
        CS_HIGH;

}


//#pragma used-


//#pragma library nRF24L01+.lib


#endif
