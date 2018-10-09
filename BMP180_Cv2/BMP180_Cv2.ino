


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
//#include <Adafruit_BMP085.h>
#include <Wire.h>
#include <inttypes.h>
#include <compat/twi.h>
#include <math.h>

//#define F_CPU 16000000UL
#define SCL_CLOCK  100000L 
///

///
#define BMP085_ADDR (0x77<<1) //0x77 default I2C address
#define I2C_WRITE 1
#define I2C_READ 0

//registers
#define BMP085_REGAC1 0xAA
#define BMP085_REGAC2 0xAC
#define BMP085_REGAC3 0xAE
#define BMP085_REGAC4 0xB0
#define BMP085_REGAC5 0xB2
#define BMP085_REGAC6 0xB4
#define BMP085_REGB1 0xB6
#define BMP085_REGB2 0xB8
#define BMP085_REGMB 0xBA
#define BMP085_REGMC 0xBC
#define BMP085_REGMD 0xBE
#define BMP085_REGCONTROL 0xF4 //control
#define BMP085_REGCONTROLOUTPUT 0xF6 //output 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB
#define BMP085_REGREADTEMPERATURE 0x2E //read temperature
#define BMP085_REGREADPRESSURE 0x34 //read pressure

//modes
#define BMP085_MODEULTRALOWPOWER 0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define BMP085_MODESTANDARD 1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define BMP085_MODEHIGHRES 2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define BMP085_MODEULTRAHIGHRES 3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25

//autoupdate temperature enabled
#define BMP085_AUTOUPDATETEMP 1 //autoupdate temperature every read

//setup parameters
#define BMP085_MODE BMP085_MODEULTRAHIGHRES //define a mode
#define BMP085_UNITPAOFFSET 0 //define a unit offset (pa)
#define BMP085_UNITMOFFSET 0 //define a unit offset (m)

//avarage filter
#define BMP085_FILTERPRESSURE 1 //avarage filter for pressure

int bmp085_regac1, bmp085_regac2, bmp085_regac3, bmp085_regb1, bmp085_regb2, bmp085_regmb, bmp085_regmc, bmp085_regmd;
unsigned int bmp085_regac4, bmp085_regac5, bmp085_regac6;
long bmp085_rawtemperature, bmp085_rawpressure;
 
  long l;
  double d;
//  float d;
  char printbuff[10];
///
unsigned int long lido; //variavel que vai ler o valor do registrador
int x=0;

int flag=0;
unsigned int long tempo;
char comando='0';
unsigned int long ovf;
unsigned int long temperatura;
int mostrador = 0;

int dataDHTrh0=0;
int dataDHTrh1=0;
int dataDHTt0=0;
int dataDHTt1=0;
int databit=0;

// Adafruit_BMP085 bmp180;

// create a FILE structure to reference our UART output function
static FILE uartout = {0} ;


int main()
{ 
   UART_inicializa(9600);
       // fill in the UART file descriptor with pointer to writer.
   fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

   // The uart is the standard output device STDOUT.
   stdout = &uartout ;

   //i2c_init();
   bmp085_init();
   sei();

   
  
  while(1)
  {

    //get temperature
    d = bmp085_gettemperature();
    dtostrf(d, 10, 2, printbuff);
    //printf("t=%f\n",d);
    printf("t=%s\n",printbuff);

    //uart_puts("temperature: "); uart_puts(printbuff);  uart_puts(" C deg"); uart_puts("\r\n");

    //get pressure
    l = bmp085_getpressure();
    ltoa(l, printbuff, 10);
    //uart_puts("pressure: "); uart_puts(printbuff); uart_puts(" Pa"); uart_puts("\r\n");
   // printf("p=%E\n",l);
    printf("p=%s\n",printbuff);

    //get altitude
    d = bmp085_getaltitude();
    dtostrf(d, 10, 2, printbuff);
    //uart_puts("altitude: "); uart_puts(printbuff); uart_puts(" M"); uart_puts("\r\n");
   // printf("a=%lf\n\n",d);
    printf("a=%s\n\n",printbuff);
    _delay_ms(1500);           

  }
  return 0;
}

////////////////////////////////////////////////////
//////////////// UART /////////////////////////////
///////////////////////////////////////////////////

void UART_inicializa(unsigned int BAUD)
{
   unsigned int velocidade = F_CPU/16/BAUD-1;
    UCSR0A = 0X20; // reset
    UBRR0H = (unsigned char)(velocidade>>8);
    UBRR0L = (unsigned char)velocidade;
    UCSR0B = 0X98; //não vou habilitar nenhuma interrupção da porta serial
    /* 8 bits , 1 stop bit, assincrono , sem paridade,modo nornal */
    UCSR0C = 0x06; //(1<<USBS0)|(3<<UCSZ00);
    
}

////////////////////////////////////////////////////

static int uart_putchar (char c, FILE *stream)
{
    escreve_caracter(c);
    //return 0 ;
}

///////////////////////////////////////////////////

void escreve_caracter(unsigned char carac)
{  
    _delay_loop_1(1);//tempo para estabilizar tensão            
    while ( !( UCSR0A & (1<<UDRE0)) ); //espera transmitir para enviar um novo
    UDR0 = carac;    
    while ( !( UCSR0A & (1<<UDRE0)) ); //espera transmitir desabilita a transmissão
    _delay_loop_1(1);//tempo para garantir o envio
}

////////////////////////////////////////////////////

ISR(USART_RX_vect)
{
 /* Wait for data to be received */
 while ( !(UCSR0A & (1<<RXC0)) )
 ;
 /* Get and return received data from buffer */
 //comando = UDR0;
 //I2C_conf();
  
}

////////////////////////////////////////////////////
////////////////// I2C /////////////////////////////
///////////////////////////////////////////////////

void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no prescaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */

}/* i2c_init */


/*************************************************************************  
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
    uint8_t   twst;

  // send START condition
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

  // wait until transmission completed
  while(!(TWCR & (1<<TWINT)));

  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

  // send device address
  TWDR = address;
  TWCR = (1<<TWINT) | (1<<TWEN);

  // wail until transmission completed and ACK/NACK has been received
  while(!(TWCR & (1<<TWINT)));

  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

  return 0;

}/* i2c_start */
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
      // send START condition
      TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
      // wait until transmission completed
      while(!(TWCR & (1<<TWINT)));
    
      // check value of TWI Status Register. Mask prescaler bits.
      twst = TW_STATUS & 0xF8;
      if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
      // send device address
      TWDR = address;
      TWCR = (1<<TWINT) | (1<<TWEN);
    
      // wail until transmission completed
      while(!(TWCR & (1<<TWINT)));
    
      // check value of TWI Status Register. Mask prescaler bits.
      twst = TW_STATUS & 0xF8;
      if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
      {         
          /* device busy, send stop condition to terminate write operation */
          TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
          
          // wait until stop condition is executed and bus released
          while(TWCR & (1<<TWSTO));
          
          continue;
      }
      //if( twst != TW_MT_SLA_ACK) return 1;
      break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
  
  // wait until stop condition is executed and bus released
  while(TWCR & (1<<TWSTO));

}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{ 
    uint8_t   twst;
    
  // send data to the previously addressed device
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);

  // wait until transmission completed
  while(!(TWCR & (1<<TWINT)));

  // check value of TWI Status Register. Mask prescaler bits
  twst = TW_STATUS & 0xF8;
  if( twst != TW_MT_DATA_ACK) return 1;
  return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));    

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));
  
    return TWDR;

}/* i2c_readNak */


////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////

void bmp085_writemem(uint8_t reg, uint8_t value) {
  i2c_start_wait(BMP085_ADDR | I2C_WRITE);
  i2c_write(reg);
  i2c_write(value);
  i2c_stop();
}

/*
 * i2c read
 */
void bmp085_readmem(uint8_t reg, uint8_t buff[], uint8_t bytes) {
  uint8_t i =0;
  i2c_start_wait(BMP085_ADDR | I2C_WRITE);
  i2c_write(reg);
  i2c_rep_start(BMP085_ADDR | I2C_READ);
  for(i=0; i<bytes; i++) {
    if(i==bytes-1)
      buff[i] = i2c_readNak();
    else
      buff[i] = i2c_readAck();
  }
  i2c_stop();
}


#if BMP085_FILTERPRESSURE == 1
#define BMP085_AVARAGECOEF 21
static long k[BMP085_AVARAGECOEF];
long bmp085_avaragefilter(long input) {
  uint8_t i=0;
  long sum=0;
  for (i=0; i<BMP085_AVARAGECOEF; i++) {
    k[i] = k[i+1];
  }
  k[BMP085_AVARAGECOEF-1] = input;
  for (i=0; i<BMP085_AVARAGECOEF; i++) {
    sum += k[i];
  }
  return (sum /BMP085_AVARAGECOEF) ;
}
#endif

/*
 * read calibration registers
 */
void bmp085_getcalibration() {
  uint8_t buff[2];
  memset(buff, 0, sizeof(buff));

  bmp085_readmem(BMP085_REGAC1, buff, 2);
  bmp085_regac1 = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGAC2, buff, 2);
  bmp085_regac2 = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGAC3, buff, 2);
  bmp085_regac3 = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGAC4, buff, 2);
  bmp085_regac4 = ((unsigned int)buff[0] <<8 | ((unsigned int)buff[1]));
  bmp085_readmem(BMP085_REGAC5, buff, 2);
  bmp085_regac5 = ((unsigned int)buff[0] <<8 | ((unsigned int)buff[1]));
  bmp085_readmem(BMP085_REGAC6, buff, 2);
  bmp085_regac6 = ((unsigned int)buff[0] <<8 | ((unsigned int)buff[1]));
  bmp085_readmem(BMP085_REGB1, buff, 2);
  bmp085_regb1 = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGB2, buff, 2);
  bmp085_regb2 = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGMB, buff, 2);
  bmp085_regmb = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGMC, buff, 2);
  bmp085_regmc = ((int)buff[0] <<8 | ((int)buff[1]));
  bmp085_readmem(BMP085_REGMD, buff, 2);
  bmp085_regmd = ((int)buff[0] <<8 | ((int)buff[1]));
}

/*
 * get raw temperature as read by registers, and do some calculation to convert it
 */
void bmp085_getrawtemperature() {
  uint8_t buff[2];
  memset(buff, 0, sizeof(buff));
  long ut,x1,x2;

  //read raw temperature
  bmp085_writemem(BMP085_REGCONTROL, BMP085_REGREADTEMPERATURE);
  _delay_ms(5); // min. 4.5ms read Temp delay
  bmp085_readmem(BMP085_REGCONTROLOUTPUT, buff, 2);
  ut = ((long)buff[0] << 8 | ((long)buff[1])); //uncompensated temperature value

  //calculate raw temperature
  x1 = ((long)ut - bmp085_regac6) * bmp085_regac5 >> 15;
  x2 = ((long)bmp085_regmc << 11) / (x1 + bmp085_regmd);
  bmp085_rawtemperature = x1 + x2;
}

/*
 * get raw pressure as read by registers, and do some calculation to convert it
 */
void bmp085_getrawpressure() {
  uint8_t buff[3];
  memset(buff, 0, sizeof(buff));
  long up,x1,x2,x3,b3,b6,p;
  unsigned long b4,b7;

  #if BMP085_AUTOUPDATETEMP == 1
  bmp085_getrawtemperature();
  #endif

  //read raw pressure
  bmp085_writemem(BMP085_REGCONTROL, BMP085_REGREADPRESSURE+(BMP085_MODE << 6));
  _delay_ms(2 + (3<<BMP085_MODE));
  bmp085_readmem(BMP085_REGCONTROLOUTPUT, buff, 3);
  up = ((((long)buff[0] <<16) | ((long)buff[1] <<8) | ((long)buff[2])) >> (8-BMP085_MODE)); // uncompensated pressure value

  //calculate raw pressure
  b6 = bmp085_rawtemperature - 4000;
  x1 = (bmp085_regb2* (b6 * b6) >> 12) >> 11;
  x2 = (bmp085_regac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)bmp085_regac1) * 4 + x3) << BMP085_MODE) + 2) >> 2;
  x1 = (bmp085_regac3 * b6) >> 13;
  x2 = (bmp085_regb1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_regac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> BMP085_MODE);
  p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  bmp085_rawpressure = p + ((x1 + x2 + 3791) >> 4);

  #if BMP085_FILTERPRESSURE == 1
  bmp085_rawpressure = bmp085_avaragefilter(bmp085_rawpressure);
  #endif
}

/*
 * get celsius temperature
 */
double bmp085_gettemperature() {
  bmp085_getrawtemperature();
  double temperature = ((bmp085_rawtemperature + 8)>>4);
  temperature = temperature /10;
  return temperature;
}

/*
 * get pressure
 */
int32_t bmp085_getpressure() {
  bmp085_getrawpressure();
  return bmp085_rawpressure + BMP085_UNITPAOFFSET;
}

/*
 * get altitude
 */
double bmp085_getaltitude() {
  bmp085_getrawpressure();
  return ((1 - pow(bmp085_rawpressure/(double)101325, 0.1903 )) / 0.0000225577) + BMP085_UNITMOFFSET;
}

/*
 * init bmp085
 */
void bmp085_init() {
  #if BMP085_I2CINIT == 1
  //init i2c
  i2c_init();
  _delay_us(10);
  #endif

  bmp085_getcalibration(); //get calibration data
  bmp085_getrawtemperature(); //update raw temperature, at least the first time

  #if BMP085_FILTERPRESSURE == 1
  //initialize the avarage filter
  uint8_t i=0;
  for (i=0; i<BMP085_AVARAGECOEF; i++) {
    bmp085_getrawpressure();
  }
  #endif
}







