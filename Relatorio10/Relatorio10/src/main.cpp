#include <Arduino.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

int cont = 0;

ISR(TIMER0_COMPA_vect){
  cont++;
  if(cont >= 10000){
    PORTD ^= BIT0;
  }
}

void configTimer(void){
  TCCR0B|= (1<<CS01);
  TCCR0A|= (1<<WGM01);
  OCR0A = 200;
  TIMSK0|= (1<<OCIE0A);
}

void disableTimer(void){
  TCCR0B = 0;
}

int main (void){

  DDRD = BIT0 + BIT1;

  configTimer();

  sei();

  for(;;){

  } 
}