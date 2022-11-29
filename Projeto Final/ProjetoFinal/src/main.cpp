//https://www.tinkercad.com/things/hZqoNlrwGBy?sharecode=-vneJ2BGAe2DPWQsmVp1XpGunDUv6IlcUKnUxGI_pug
#include<Arduino.h>
#include<stdio.h>
#include<stdlib.h>


#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

int porcentagem = 0; // Potencia do motor
bool ligado = false; // Estado do sistema
bool botao2 = false; // Botao de emergencia pressionado ou nao
bool contador = false; // Contador para mostrar a mensagem LIGADO ou RELIGADO

                           
// --------CONFIGURACAO SERIAL---------

#define FOSC 16000000U
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1


char msg_tx[20];
char msg_rx[32];
int pos_msg_rx = 0;
int tamanho_msg_rx = 1;
unsigned int x = 0;

void UART_Init(unsigned int ubrr){
  
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); //BIT 7 + BIT 4 + BIT 3
  
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); //BIT2 + BIT 1
  
}

void UART_Transmit(char *dados){
  
  while(*dados != 0)
  {
    while(!(UCSR0A & (1 << UDRE0))); //BIT5
    
    UDR0 = *dados;
    
    dados++; 
  }
}

ISR(USART_RX_vect){
  
  msg_rx[pos_msg_rx++] = UDR0;
  if(pos_msg_rx == tamanho_msg_rx){
    pos_msg_rx = 0;
  }
}

// ------FIM DA CONFIGURACAO SERIAL-------




// -----------CONFIGURACAO ADC------------

void ADC_init(void){
  
  ADMUX = (1 << REFS0); //DETERMINA A TENSAO DE REFERENCIA
  
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  
  //ADCSRB = 0;
  //ADEN = HABILITA ADC
  //ADPS CONFIGURA O PRESCALER //PRESCALER = 128
}

int ADC_read(){
  
  unsigned int leitura_AD;
  float tensao;
  int pino = 1;

  ADMUX = ((ADMUX & 0xF8) | pino);

  unsigned int SomaLeitura = 0, MediaLeitura;
  for (int i = 0; i < 10; i++)
  {
    ADCSRA |= BIT6;
    while ((ADCSRA & BIT6) == BIT6)
      ;

    leitura_AD = (ADCL | (ADCH << 8)); // Ou leitura_AD = ADC;
    SomaLeitura += leitura_AD;
  }
  MediaLeitura = SomaLeitura / 10;
  tensao = (MediaLeitura * 5) / 1023.0; // Variável auxiliar para o cálculo
  
} 

// -------FIM DA CONFIGURACAO ADC---------




// ----------CONFIGURACAO DO TIMER--------

void setupTimer(){
  TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1); //PWM RAPIDO //OC0A É LIMPO NA IGUALDADE DE COMPARACAO
  TCCR0B |= (1 << CS00); //FAZ A SELEÇAO DA FONTE DO CLOCK //PRESCALER = 1 OU SEJA SEM PRESCALER
  DDRD = BIT3 + BIT4 + BIT5 + BIT6; //DECLARA OS LEDS COMO SAIDA
}

// -----FIM DA CONFIGURACAO DO TIMER------




// CONFIGURACAO DA INTERRUPCAO EXTERNA (SENSOR)

void setupSensor(){
  
  EICRA = 0b00000010;
  
  EIMSK = 0b00000011;
  
  PORTD |= (1 << PD2);
}

// FIM DA CONFIGURACAO DA INTERRUPCAO EXTERNA



// Interrupcoes externas
ISR(INT0_vect){
  
  if(ligado){
    
    itoa(porcentagem, msg_tx, 10);
    UART_Transmit("Potencia: ");
    UART_Transmit(msg_tx);
    UART_Transmit("%");
    UART_Transmit("\n");
  }
  
}


// Configuracoes iniciais
void setup(){
  
  ADC_init();
  UART_Init(MYUBRR);
  
  setupTimer();   
  setupSensor();
  
  PORTD &= ~(BIT5); //LIGA
  PORTD &= ~(BIT4); //DESLIGA
  PORTD &= ~(BIT3); //EMERGENCIA
  PORTD &= ~(BIT6); //MOTOR
  
  sei();
  
  DDRB |= (1 << PB0);

}
  // Logica do problema
void loop(){
    
  	int interruptor = (PINB & BIT0);
    int rotacao =  ADC_read()/4;
    porcentagem = (100*rotacao)/255;

    if((interruptor) == BIT0){

      ligado = false;
      botao2 = true;
      contador = true;
    }
    else if(msg_rx[0] == 'D'){
      ligado = false;
      contador = true;
    }
    else if((!(interruptor)== BIT0) && (msg_rx[0] == 'L')){

      ligado = true;
      botao2 = false;
    }
  	

    if(ligado){

      PORTD |= BIT5;
      PORTD &= ~(BIT4);
      PORTD &= ~(BIT3);
      OCR0A = rotacao;
    }
    else{
      
      if(botao2){
        PORTD |= BIT3;
        PORTD &= ~(BIT5);
        PORTD &= ~(BIT4);
        if((PINB & BIT0) == 0){
          PORTD &= ~(BIT3);
          PORTD |= BIT4;
        }
        else{
          PORTD |= BIT3;
        }
      }
      else{
        PORTD |= BIT4;
        PORTD &= ~(BIT5);
        PORTD &= ~(BIT3);
      }
      OCR0A = 0;
    }

    msg_rx[0] = ' ';
    _delay_ms(100);
  }