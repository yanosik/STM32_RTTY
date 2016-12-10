#include <stm32f10x_usart.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_gpio.h>
const char ascii[] = "0123456789ABCDEF";
int srednia_u[5]={0,0,0,0,0};

int HexCharToInt(char ch)
        {
            if (ch < 48 || (ch > 57 && ch < 65) || ch > 70) return 0;
            return (ch < 58) ? ch - 48 : ch - 55;
        }

void print( char* s)
{
while (*s)
	   {
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		        {
		        }
	USART_SendData(USART3, *(s++));
	   }
}

void sendtogps(char* s, unsigned char cun)
{
	unsigned char CK_A=0;
	unsigned char CK_B=0;
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			        {
			        }
	USART_SendData(USART1, *(s++));
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
				        {
				        }
	USART_SendData(USART1, *(s++));
		cun--;
		cun--;
	while (cun-- != 0)
	   {
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		        {
		        }
	CK_A += *(s);
	CK_B += CK_A;
	USART_SendData(USART1, *(s++));
	   }
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
					        {
					        }
			USART_SendData(USART1, CK_A);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
							        {
							        }
			USART_SendData(USART1, CK_B);
}

void send_hex(unsigned char data)
{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		{
		}
		USART_SendData(USART3, ascii[data >> 4 ]);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
				{
				}
				USART_SendData(USART3, ascii[data & 0x0f ]);
}

uint8_t spi_sendrecv(uint16_t byte)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
 // wait for tx buffer
 while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
 SPI_I2S_SendData(SPI2, byte);

 // wait for data in rx buffer
 while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
 	 GPIO_SetBits(GPIOC, GPIO_Pin_13);
 return SPI_I2S_ReceiveData(SPI2);
}

unsigned char czytaj_GPS(unsigned char pos,unsigned char len,  char *source, char * destination)
{
	char   *wyn;
	char   pet = 0;
    wyn=source;
    while( pos-- !=0)
     {
    	 while (*(wyn++) != ',');
     }
     if (*(wyn) == 0)
        {
        return 0;
        }
        else
        {
        while((*(wyn) !=',') && (*(wyn) != 0)&& (len-- !=0))
         {
          *(destination++) = *(wyn++);
         pet = 1;
		 }
		 if (pet == 0)
		 	{
			return 0;
			}
         *(destination) = 0;
        }
        return 1;
}

uint16_t gps_CRC16_checksum (char *string)
{
	uint16_t crc = 0xffff;
	char i;
	 while (*(string) != 0)
	 {
		         crc = crc ^ (*(string++) << 8);
		         for (i=0; i<8; i++)
		         {
		             if (crc & 0x8000)
		                 crc = (crc << 1) ^ 0x1021;
		             else
		                 crc <<= 1;
		         }
	  }
	  return crc;
}

int srednia (int dana)
{
volatile char nr_pom=0;
volatile char first=1;
char i;
int sr=0;
if(first)
{
	 for (i=0; i<5; i++)
	 	 {
		 srednia_u[i] = dana;
	 	 }
	 first =0;

}
srednia_u[nr_pom] = dana;
if (++nr_pom >4)
	{
	nr_pom=0;
	}
 for (i=0; i<5; i++)
 	 {
	 	sr += srednia_u[i];
 	 }
 sr = sr/5;
return sr;
}
