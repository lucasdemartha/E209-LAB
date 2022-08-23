#include <Arduino.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

int main(){

DDRD = 0b00001110;
PORTD = 0b00000000;
int state = 0;

  for(;;){

    /* ex2
    PORTD = 0b00000000;
    _delay_ms(500);
    PORTD = 0b00000010;
    _delay_ms(500);
    PORTD = 0b00000100;
    _delay_ms(500);
    PORTD = 0b00000110;
    _delay_ms(500);
  */
   //ex3 abaixo

    if((PIND&BIT4) == BIT4){
      switch(state){
        case 0:
          PORTD = 0b00000010;
          _delay_ms(500);
          state++;
          break;

        case 1:
          PORTD = 0b00000100;
          _delay_ms(500);
          state++;
          break;

        case 2:
          PORTD = 0b00000000;
          _delay_ms(500);
          state++;
          break;
          
        default:
          state = 0;
          break;
      }
      
    }
  }
}