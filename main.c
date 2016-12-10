// STM32F100 and SI4032 RTTY transmitter
// released under GPL v.2 by anonymous developer
// enjoy and have a nice day
// ver 1.5a
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_rcc.h>
#include "stdlib.h"
#include "misc.h"
#include <stdio.h>
#include "f_rtty.h"
#include "init.h"
#include "fun.h"

//**************config**************
char  callsign[15]={"NO1LIC-1"}; // put your callsign here

//************band select******************
#define fb		1
#define fbsel	1
		//fb		fbsel 0			1
  	  	// 0		208,0000		415,9992
  	  	// 1		216,6675		433,3325
  	  	// 2		225,3342		450,6658

//*************frequency********************
#define freq	434.150 //Mhz middle frequency
//*****************************************************

//********* power definition**************************
#define Smoc	0 // PWR 0...7 0- MIN ... 7 - MAX
//***************************************************

//********** frame delay in msec**********
#define tx_delay	100    // 2500 ~2,5  w polu flaga wpisywany jest tx_delay/1000 modulo 16 czyl;i dla 16000 bedzie 0 póki co.

//**************end config**************

//************ do not touch bellow this line;) *********************
#define gen_div		3	//Sta³a nie zmieniac
#define gen 	((26.0/gen_div) *(fbsel+1)) //26 ->26MHZ kwarc napedzajacy nadajnik
#define fc		(((freq/gen) - fb - 24) * 64000)

///////////////////////////// test mode /////////////
unsigned char test =0; // 0 - normal, 1 - short frame only cunter, height, flag

#define WR 							0x8000
#define gps_RMC_dlugosc				5
#define gps_RMC_dlugosc_len			10
#define gps_RMC_dlugosc_kier		6
#define gps_RMC_dlugosc_kier_len	1
#define gps_RMC_szerokosc			3
#define gps_RMC_szerokosc_len		9
#define gps_RMC_szerokosc_kier		4
#define gps_RMC_szerokosc_kier_len	1
#define gps_RMC_status				2
#define gps_RMC_status_len			1
#define gps_RMC_data				9
#define gps_RMC_data_len			6
#define gps_RMC_czas				1
#define gps_RMC_czas_len			6
#define gps_RMC_predkosc			7
#define gps_RMC_predkosc_len		5
#define gps_RMC_kierunek			8
#define gps_RMC_kierunek_len		5
#define gps_GGA_wysokosc			9
#define gps_GGA_wysokosc_len		5 //*
#define gps_GGA_use_sat				7
#define gps_GGA_use_sat_len			2 //*
#define gps_VTG_predkosc			7
#define gps_VTG_predkosc_len		5
#define GREEN	GPIO_Pin_7
#define RED	GPIO_Pin_8
#define GPS_START	'$'
#define GPS_STOP	0x0a 			// LF
#define OFF	GPIO_Pin_12
unsigned int send_cun; 				//frame counter
char czas[7]={"000000"};
char status[2]={'N'};
char dlugosc[14]={"0.00000"};
char dlugosc_kier = 0;//'W';
char szerokosc[13]={"0.00000"};
char szerokosc_kier = 0;// 'S';s
char wysokosc[6]={"0"};
char predkosc[6]={"0"};
char kierunek[6]={"0"};
char temperatura;
char FixOk =0;
int napiecie;
unsigned int czest;
float  fdlugosc=0;
float fszerokosc=0;
float deg = 0;
float fbuf=0;
char use_sat[3]={'0'};
char flaga= ((((tx_delay/1000) & 0x0f)<< 3) | Smoc);
int32_t CRC_rtty=0x12ab;	//checksum
char buf[512];
char buf_rtty[200];
char menu[] =   "$$$$$$STM32 RTTY tracker by Blasiu, enjoy and see you on the HUB... \n\r";
char init_trx[] = "\n\rPowering up TX\n\r";
char powitanie[]= "greetings from earth";
unsigned char pun = 0;
unsigned int cun =10;
char temp;
unsigned char dev = 0;
unsigned char tx_on =0;
unsigned int  tx_on_delay;
unsigned char tx_enable=0;
char send_rtty_status=0;
unsigned char cun_rtty =0;
char *rtty_buf;
unsigned char GPS_temp;
char GPS_buf[200];
char *wsk_GPS_buf;
char GPS_rec = 0;
char new_GPS_msg = 0;
unsigned char crc_GPS = 0;
char crc_GPS_start =0 ;
char crc_GPS_rec = 0;
char crc_GPS_cun=0;
char confGPSNAV[]=	{0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char GPSon[] = 		{0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
char  GPSoff[] = 	{0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};//, 0x4D, 0x3B};
char  GPS_ZDA_OFF[]={0xb5, 0x62, 0x06,0x01, 0x08, 0x00, 0xf0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char  GPS_GLL_OFF[]={0xb5, 0x62, 0x06,0x01, 0x08, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char  GPS_GSA_OFF[]={0xb5, 0x62, 0x06,0x01, 0x08, 0x00, 0xf0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char  GPS_GSV_OFF[]={0xb5, 0x62, 0x06,0x01, 0x08, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int Button=0;
unsigned char cun_off = 0;
unsigned char bOFF=0;
unsigned char bCheckKay=0;
unsigned char GPSConf=0;

void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
	{

    	GPS_temp = USART_ReceiveData(USART1);
    	USART_SendData(USART3, GPS_temp);
    	if (GPS_temp == '*')
    	    		{
    	    		crc_GPS_start = 0;
    	    		crc_GPS_cun = 0;
    	    		crc_GPS_rec = 0;
    	    		}
    	if (crc_GPS_start)
   	   				{
    	   			crc_GPS ^= GPS_temp;
    	    	   	}
    		else
    				{
    				if (GPS_rec)
    					{
    					if (crc_GPS_cun < 3 && crc_GPS_cun > 0 )
    						{
    						crc_GPS_rec *= 16;
    						crc_GPS_rec += HexCharToInt(GPS_temp);
    						}
    					crc_GPS_cun++;
						}

    				}
    	if(GPS_temp == GPS_START)
    		{
    		wsk_GPS_buf = GPS_buf;
    		GPS_rec	=	1;
    		crc_GPS=0;
    		crc_GPS_start =1;
    		}
    	if (GPS_rec)
    	  	{
    	   	*(wsk_GPS_buf++) = GPS_temp;
    	  	}

    	if(GPS_temp == GPS_STOP)
    		{
    		GPS_rec = 0;
    		*(wsk_GPS_buf) = 0x00;

    		if(crc_GPS_rec == crc_GPS)
    			{
    		    new_GPS_msg =1;
    		    GPS_rec = 0;
    		    }
    		}
	}
}

void TIM2_IRQHandler(void){
	  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	  {
	    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	  }
	  if (tx_on && ++cun_rtty ==17)
	  {
		cun_rtty = 0;
		send_rtty_status = send_rtty(rtty_buf);
		if (send_rtty_status ==2)
		{
			GPIO_SetBits(GPIOB, RED);
			if (*(++rtty_buf) == 0)
				{
				tx_on=0;
				tx_on_delay =tx_delay ;//2500;
				tx_enable=0;
				temp = spi_sendrecv(0x0740 | WR);
				}
		}
		if (send_rtty_status ==1)
		{
			temp = spi_sendrecv(0x7302 | WR);
			GPIO_SetBits(GPIOB, RED);
		}
		if (send_rtty_status ==0)
		{
			temp = spi_sendrecv(0x7300 | WR);
			GPIO_ResetBits(GPIOB, RED);
		}
	  }
	  if(!tx_on && --tx_on_delay == 0)
	  {
		tx_enable=1;
		tx_on_delay--;
	  }
	  if(--cun == 0 )
	  	  {
		 cun_off++;
		  if (pun)
	    			{
	    			  GPIO_ResetBits(GPIOB, GREEN);
	    			pun =0;
	    			}
	    		else
	    			{
	    			if (flaga & 0x80)
	    			{
	    			GPIO_SetBits(GPIOB, GREEN);
	    			}
	    			pun = 1;
	    			}
		cun = 200;
	}
bCheckKay =1;
}

int main(void)
{
#ifdef DEBUG
  debug();
#endif
  	RCC_Conf();
  	NVIC_Conf();
  	init_port();

  	wsk_GPS_buf = GPS_buf;


  	GPIO_SetBits(GPIOB, RED);
  	USART_SendData(USART3, 0xc);
  	print(menu);
  	temp = spi_sendrecv(0x02ff);
  	//send_hex(temp);

  	temp = spi_sendrecv(0x03ff);
	temp = spi_sendrecv(0x04ff);
	temp = spi_sendrecv(0x0780 | WR);
	print(init_trx);
	   	// programowanie czestotliwosci nadawania
	napiecie = fc;
  		temp = spi_sendrecv(0x7561 | WR);
  	  	temp = spi_sendrecv(0x7600 | (((uint16_t)fc>>8) & 0x00ff) | WR);
  	  	temp = spi_sendrecv(0x7700 | ( (uint16_t)fc	  & 0x00ff) | WR);

  	// Programowanie mocy nadajnika
  	temp = spi_sendrecv(0x6D00 | (Smoc & 0x0007) | WR);
  	temp = spi_sendrecv(0x7100 | WR);
  	temp = spi_sendrecv(0x8708);
  	temp = spi_sendrecv(0x02ff);
    temp = spi_sendrecv(0x75ff);
  	temp = spi_sendrecv(0x76ff);
  	temp = spi_sendrecv(0x77ff);
  	temp = spi_sendrecv(0x1220 | WR);
  	temp = spi_sendrecv(0x1300 | WR);
  	temp = spi_sendrecv(0x1200 | WR);
  	temp = spi_sendrecv(0x0f80 | WR);
  	rtty_buf = buf_rtty;
  	tx_on = 0;
  	tx_enable =1;
  	//tx_enable =0;
  	Button = ADCVal[1];

while(1)
  	{
	if (status[0] == 'A')
		{
		flaga |= 0x80;
		}
	else
		{
		flaga &= ~0x80;
		}

	if (bCheckKay)
		{
		if ((ADCVal[1]- Button ) >= 100)
			{
			if (cun_off> 2)
				{
				bOFF =1;
				cun_off = 2;
				}
			}
		else
			{
			Button = ADCVal[1];
			cun_off = 0;
			if (bOFF)
				{
				GPIO_SetBits(GPIOA, OFF);
				}

			}
		bCheckKay = 0;
	}

	if (tx_on == 0 && tx_enable)
	{
		temperatura = spi_sendrecv(0x11ff); //odczyt ADC
		temp = spi_sendrecv(0x0f80 |WR);
		temperatura = -64 +( temperatura *5 /10) -16;
		napiecie =  srednia(ADCVal[0]*600/4096);

		fdlugosc = atof(dlugosc);
		deg = (int)(fdlugosc/100);
		fbuf = fdlugosc -(deg  * 100);
		fdlugosc = deg + fbuf/60;
		if (dlugosc_kier == 'W')
		{
			fdlugosc *= -1;
		}
		fszerokosc = atof(szerokosc);
		deg = (int)(fszerokosc/100);
		fbuf = fszerokosc -(deg  * 100);
		fszerokosc = deg + fbuf/60;
		if (szerokosc_kier == 'S')
				{
					fszerokosc *= -1;
				}
		if (test)
		{
			sprintf(buf_rtty,"$$$$$$$%d,%d,%02x",send_cun,atoi(wysokosc),flaga);
		}
		else
		{
		sprintf(buf_rtty,"$$$$$$$%s,%d,%s,%.5f,%.5f,%d,%d,%s,%d,%d,%d,%02x",callsign,send_cun,czas,fszerokosc,fdlugosc,atoi(wysokosc),atoi(predkosc),kierunek,temperatura,napiecie,atoi(use_sat),flaga);
		}
		CRC_rtty=0xffff;																							//napiecie      flaga
		CRC_rtty=gps_CRC16_checksum(buf_rtty+7);
		sprintf(buf_rtty,"%s*%04X\n",buf_rtty,CRC_rtty & 0xffff);
		rtty_buf = buf_rtty;
		tx_on = 1;
		temp = spi_sendrecv(0x0748 | WR);
  		send_cun++;
	}

	if (new_GPS_msg)
		{
		//print(GPS_buf);
  		new_GPS_msg = 0;
  		if (strncmp(GPS_buf,"$GPRMC",6)	==	0)
  				{
  			if (czytaj_GPS(gps_RMC_czas,gps_RMC_czas_len,GPS_buf,czas) == 0)
  						{
  						strcpy(czas,"000000");
  						}

  			if (czytaj_GPS(gps_RMC_kierunek,gps_RMC_kierunek_len,GPS_buf,kierunek) == 0)
  		  					{
  		  					strcpy(kierunek,"0");
  		  					}
  			if (czytaj_GPS(gps_RMC_szerokosc,gps_RMC_szerokosc_len,GPS_buf,szerokosc) == 0)
  					{
  					strcpy(szerokosc,"0.00000");
  					}

  			if (czytaj_GPS(gps_RMC_szerokosc_kier,gps_RMC_szerokosc_kier_len,GPS_buf,&szerokosc_kier) == 0)
  					{
  					szerokosc_kier=0;
  					}

  			if (czytaj_GPS(gps_RMC_dlugosc,gps_RMC_dlugosc_len,GPS_buf,dlugosc) == 0)
  					{
  					strcpy(dlugosc,"0.00000");
  					}

  			if (czytaj_GPS(gps_RMC_dlugosc_kier,gps_RMC_dlugosc_kier_len,GPS_buf,&dlugosc_kier) == 0)
  					{
  					dlugosc_kier=0;
  					}


  			if (czytaj_GPS(gps_RMC_status,gps_RMC_status_len,GPS_buf,status) == 0)
  							{
  							strcpy(status,"-");
  							}


  				}
  		if (strncmp(GPS_buf,"$GPGGA",6)	==	0)
  		{
  			if (czytaj_GPS(gps_GGA_wysokosc,gps_GGA_wysokosc_len,GPS_buf,wysokosc) == 0)
  			  					{
  			  					strcpy(wysokosc,"0");
  			  					}
  			if (czytaj_GPS(gps_GGA_use_sat,gps_GGA_use_sat_len,GPS_buf,use_sat) == 0)
  			  					{
  			  					strcpy(use_sat,"0");
  			  					}

  		}

  		if (strncmp(GPS_buf,"$GPVTG",6)	==	0)
  			 {
  	    	if (czytaj_GPS(gps_VTG_predkosc,gps_VTG_predkosc_len,GPS_buf,predkosc) == 0)
  	    	  					{
  	    	  					strcpy(predkosc,"0");
  	    	  					}
  			 }
  		if(strncmp(GPS_buf,"$GPZDA",6 )== 0 )
  			{
  			switch (GPSConf)
  				{
  			case 0:
  				sendtogps(confGPSNAV, sizeof(confGPSNAV)/sizeof(uint8_t));
  				break;
  			case 1:
  				sendtogps(GPS_GSV_OFF,sizeof(GPS_GSV_OFF)/sizeof(uint8_t));
  				break;
  			case 2:
  				sendtogps(GPS_GSA_OFF,sizeof(GPS_GSA_OFF)/sizeof(uint8_t));
  				break;
  			case 3:
  				sendtogps(GPS_GLL_OFF,sizeof(GPS_GLL_OFF)/sizeof(uint8_t));
  				break;
  			case 4:
  				sendtogps(GPS_ZDA_OFF,sizeof(GPS_ZDA_OFF)/sizeof(uint8_t));
  				break;
  				}
  			GPSConf++;
  			}
		}

  	}
}

#ifdef  DEBUG
void assert_failed(uint8_t* file, uint32_t line)
{
  	while (1);
}
#endif
