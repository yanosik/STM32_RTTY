int HexCharToInt(char ch);
void print( char* s);
void sendtogps(char* s, unsigned char cun);
void send_hex(unsigned char data);
uint8_t spi_sendrecv(uint16_t byte);
unsigned char czytaj_GPS(unsigned char pos,unsigned char len,  char *source, char * destination);
uint16_t gps_CRC16_checksum (char *string);
int srednia (int dana);
