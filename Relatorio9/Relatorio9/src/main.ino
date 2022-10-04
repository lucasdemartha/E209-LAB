#include <Arduino.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000
#define FECHADO 0
#define ABRINDO 1
#define PARADO_ABRINDO 2
#define FECHANDO 3
#define PARADO_FECHANDO 4
#define ABERTO 5
char state = FECHADO;


ISR(INT0_vect){
    
    switch(state){
      
        case FECHADO:
            PORTD &= ~(BIT5);
            PORTD |= BIT7;
            PORTD &= ~(BIT4);
            PORTD &= ~(BIT6);
            state = ABRINDO;
            break;
        case ABRINDO:
            PORTD &= ~(BIT5);
            PORTD &= ~(BIT7);
            PORTD &= ~(BIT4);
            PORTD &= ~(BIT6);
            state = PARADO_ABRINDO;
            break;
        case PARADO_ABRINDO:
            PORTD |= BIT5;
            PORTD &= ~(BIT7);
            PORTD &= ~(BIT4);
            PORTD &= ~(BIT6);
            state = FECHANDO;
            break;
        case FECHANDO:
            PORTD &= ~(BIT5);
            PORTD &= ~(BIT7);
            PORTD &= ~(BIT4);
            PORTD &= ~(BIT6);
            state = PARADO_FECHANDO;
            break;
        case PARADO_FECHANDO:
            PORTD &= ~(BIT5);
            PORTD |= BIT7;
            PORTD &= ~(BIT4);
            PORTD &= ~(BIT6);
            state = ABRINDO;
            break;
        case ABERTO:
            PORTD |= BIT5;
            PORTD &= ~(BIT7);
            PORTD &= ~(BIT4);
            PORTD &= ~(BIT6);
            state = FECHANDO;
            break;
      }
}


int main (void){

  DDRD = BIT5 + BIT7 + BIT6 + BIT4; 
  

  EICRA |= BIT1 + BIT0;
  EIMSK |= BIT0;
  
  //desligando os leds
  PORTD &= ~(BIT5);
  PORTD &= ~(BIT7);
  PORTD &= ~(BIT6);
  PORTD &= ~(BIT4);
  
  sei();

  for(;;){
    if(((PIND & BIT2) == BIT2) && ((PIND & BIT3) == BIT3) && state == FECHANDO){
        state = FECHADO;
    }
    else if(((PIND & BIT2) == 0) && ((PIND & BIT3) == 0) && state == ABRINDO){
        state = ABERTO;
    }
  } 
}