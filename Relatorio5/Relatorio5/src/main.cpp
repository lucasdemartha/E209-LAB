#include <Arduino.h>

//RELATORIO 6!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

bool flag = false;

ISR(INT0_vect){
        PORTD |= BIT4;
        _delay_ms(1000);
        PORTD &= ~(BIT4);
}

ISR(INT1_vect){
    if(flag == false){
      EIMSK &= ~(BIT0);
      flag = true;
    }
    else{
      EIMSK |= BIT0;
      flag = false;
    }
}

int main(){


  DDRD = BIT4 + BIT5;

  EIMSK = BIT0+BIT1;
  EICRA = BIT3+BIT1;

  PORTD &= ~(BIT4);
  PORTD &= ~(BIT5);

  sei();
  for(;;){
      PORTD |= BIT5;
      _delay_ms(500);
      PORTD &= ~(BIT5);
      _delay_ms(500);
    }
}