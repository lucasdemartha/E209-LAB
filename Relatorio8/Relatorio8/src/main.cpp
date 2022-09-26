#include <Arduino.h>

//RELATORIO 8!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000


ISR(PCINT0_vect){   
    PORTD |= BIT7;
    _delay_ms(1000);
    PORTD &= ~(BIT7);
}

ISR(PCINT1_vect){
    PORTD |= BIT6;
    _delay_ms(500);
    PORTD &= ~(BIT6);
}

ISR(PCINT2_vect){
    PORTB |= BIT1;
    _delay_ms(2000);
    PORTB &= ~(BIT1);
}

int main(){


  DDRD = BIT5 + BIT7 + BIT6;
  DDRB = BIT1;

  PORTD |= BIT4;
  PORTC |= BIT1;
  PORTB |= BIT0;

  PCICR |= BIT2 + BIT1 + BIT0;
  PCMSK2 |= BIT4;
  PCMSK0 |= BIT0;
  PCMSK1 |= BIT1;

  PORTD &= ~(BIT5);
  PORTD &= ~(BIT7);
  PORTD &= ~(BIT6);
  PORTB &= ~(BIT1);

  sei();
  for(;;){
      PORTD |= BIT5;
      _delay_ms(250);
      PORTD &= ~(BIT5);
      _delay_ms(250);
    }
}