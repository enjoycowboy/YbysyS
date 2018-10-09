//#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

///
unsigned int long lido; //variavel que vai ler o valor do registrador
int x=0;

int flag=0;
unsigned int long tempo;
char comando='0';
unsigned int long ovf;

int dataDHTrh0=0;
int dataDHTrh1=0;
int dataDHTt0=0;
int dataDHTt1=0;
int databit=0;

// create a FILE structure to reference our UART output function
static FILE uartout = {0} ;


int main()
{ 
   UART_inicializa(9600);
       // fill in the UART file descriptor with pointer to writer.
   fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);

   // The uart is the standard output device STDOUT.
   stdout = &uartout ;
   
   TCCR1A = 0X00;//modo normal até 0xffff

   TCCR1C = 0X00; //modo normal até 0xffff
   TIMSK1 = 0x00; //interrupção off
   TCCR1B = 0x00; //clock interno=0

   sei();
  while(1)
  {
      //if(comando=='0') 
    //{
    //  UART_Receive();
    //}
     //escreve_caracter(teste);
     sei();
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
 comando = UDR0;
  if (comando=='d')
    {
      fazerleitura_som();
    }
  else if (comando=='f')
    {
      inicializa_modo_captura();
    }
   else if (comando=='u')
    {
      confDHT();
    }  
}

////////////////////////////////////////////////////
/////////// DISTÂNCIA /////////////////////////////
///////////////////////////////////////////////////

void fazerleitura_som(void)
{
   inicializa_Timer1_ovf();
   ovf=0;
   PORTD= 0x00;
   DDRD = 0x00;
   DDRD = 0x10;  
   
   PORTD= 0x10;
   _delay_us(10);
   PORTD= 0x00;
   
   inicializa_int_ext0_rising();
   //inicializa_Timer1_ovf();
   sei();

}

///////////////////////////////////////////////////

void inicializa_Timer1_ovf(void)
{
   TCCR1A = 0X00;//modo normal até 0xffff
   TCCR1B = 0x00;//clock interno/1
   TCCR1C = 0X00;//modo normal até 0xffff
   TIMSK1 = 0x01; //habilita interrupção de ovf
   flag=0;
   sei();
   //
}

////////////////////////////////////////////////////

ISR(TIMER1_OVF_vect)    
{             
    //printf("F");
    ovf = ovf + 65536;
    
}

////////////////////////////////////////////////////

void inicializa_int_ext0_rising(void)
{
  EIFR=0;
  EICRA = 0x03;
  EIMSK = 0x01;
  flag=1;
  
}

void inicializa_int_ext0_falling(void)
{
  EICRA = 0x02;
  EIMSK = 0x01;
  flag=2;

}

///////////////////////////////////////////////////

ISR (INT0_vect)
{ 
  cli();
  if (flag==0)
    {
      //printf("E");
      //printf("e");
    }
  else if (flag==1)
   {
    TCNT1 = 0;
    TCCR1B = 0x01;
    inicializa_int_ext0_falling();
    //printf("a");    
   }
  else if (flag==2)
   {
    tempo = TCNT1;
    TCCR1B = 0x00;
    tempo = tempo + ovf;
    flag=0;
    EIMSK = 0x00;
    unsigned int long dist = (17*tempo)/1600;
    printf("d = %d mm\n\n",dist); 
    //printf("d = %d mm\n\n",tempo);   
    flag=0;

   }
   sei();   
}

////////////////////////////////////////////////////
/////////// UMIDADE SOLO //////////////////////////
///////////////////////////////////////////////////

void inicializa_modo_captura(void)
{
   TCCR1A = 0X00;//não tem uso para o modo captura de eventos externos
   TCCR1B = 0x41;//filtro desabilitado de ruido habilitado,captura na borda de descida do sinal, clock interno/1
   TCCR1C = 0X00;//não tem uso para o modo captura de eventos externos
   TIMSK1 = 0x20; //habilita interrupção de captura, ovf off
   x=0;
   sei();
}


ISR(TIMER1_CAPT_vect)     //interrupção de captura de eventos externos.
{             
      cli();
    if (x<=1)
      { cli();
        TCNT1 = 0;//limpar o timer
        x++;
        TIFR1 = 0XF0; //limpa interrupcão pendente
        sei();
      }
    else
    {
      //int low = ICR1L;        //primeiro deve ser lido os byte mais baixo e depois os mais altos
      //lido = (ICR1H<<8) + low;
      cli();
      lido=ICR1;
      //TCNT1 = 0;//limpar o timer          
      
      //TIFR1 = 0XF0; //limpa interrupcão pendente
      TCCR1B = 0x00; 
      TIMSK1 = 0x00;
      lido = 16000000 / lido;
      printf("f = %d Hz\n\n",lido);
    }
    sei();
}

////////////////////////////////////////////////////
////////////////// DHT11 //////////////////////////
///////////////////////////////////////////////////

void confDHT(void)
{  
    databit=0;
    dataDHTrh0=0;
    dataDHTrh1=0;
    dataDHTt0=0;
    dataDHTt1=0;   
   
    TCCR1A = 0X00;//modo normal até 0xffff
    TCCR1B = 0x02;//clock interno/8
    TCCR1C = 0X00;//modo normal até 0xffff
    TIMSK1 = 0x00; // nao habilita interrupção de ovf
    TCNT1=0x0000;

    PORTD = 0X08;
    DDRD = 0x08;  //inicializa portd3 como saída
    PORTD = 0X00;
    _delay_ms(18);
    PORTD = 0X08;
    DDRD = 0x00;  //inicializa portd3 como entrada

    databit=0;

    EICRA = 0x04; // int1 mudança
    EIMSK = 0x02; // int1 on

    sei();
    
}

///////////////////////////////////////////////////

ISR(INT1_vect)
{
  cli();
  unsigned int tempobit=0;
  
  if (PIND & (1 << PD3)) 
    {
      TCNT1=0x0000;
    }
  else 
    {
      tempobit=TCNT1;
      if(databit<=1)
      {
       if (tempobit<=85)
        {
          //printf("0");
        }
        else
        {
          //printf("1");
        }
      }
      else if(databit<=9)
      {
        if (tempobit<=85)
        {
          //databit=0x00;
          dataDHTrh0= (dataDHTrh0<<1);
          //printf("0");
        }
        else
        {
          //databit=0x01;
          dataDHTrh0= (dataDHTrh0<<1) +1;
          //printf("1");
        }
      }
      else if(databit<=17)
      {
        if (tempobit<=85)
        {
          //databit=0x00;
          dataDHTrh1= (dataDHTrh1<<1);
          //printf("0");
        }
        else
        {
          //databit=0x01;
          dataDHTrh1= (dataDHTrh1<<1)+1;
          //printf("1");
        }
      }
      else if(databit<=25)
      {
        if (tempobit<=85)
        {
          //databit=0x00;
          dataDHTt0= (dataDHTt0<<1);
          //printf("0");
        }
        else
        {
          //databit=0x01;
          dataDHTt0= (dataDHTt0<<1)+1;
          //printf("1");
        }
      }
      else if(databit<=33)
      {
        if (tempobit<=85)
        {
          //databit=0x00;
          dataDHTt1= (dataDHTt1<<1);
          //printf("0");
        }
        else
        {
          //databit=0x01;
          dataDHTt1= (dataDHTt1<<1)+1;
          //printf("1");
        }
      }
      else
      {
        TCCR1B = 0x00;//clock interno=0
        EICRA = 0x00; // int1 off
        EIMSK = 0x00; // int1 off
        TCNT1=0x0000;
        printf("U= %d,%d \n",dataDHTrh0,dataDHTrh1);
        printf("T= %d,%d \n\n",dataDHTt0,dataDHTt1);
        
      }

    databit++;   
    }
    sei();  
}

