#include <Arduino.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

ISR(ADC_vect){
  unsigned int leitura = ADC;
  float temperatura = ((leitura*100)/1023);

  Serial.begin(9600);
  Serial.println(temperatura);

  if(temperatura <= 20){
    OCR0A = 64;
    OCR0B = 191;
  }
  else if(temperatura <= 50){
    OCR0A = 127;
    OCR0B = 127;
  }
  else if(temperatura <= 80){
    OCR0A = 191;
    OCR0B = 64;
  }
  else{
    OCR0A = 255;
    OCR0B = 0;
    PORTD |= BIT7;
  }
  ADCSRA = (1 << ADSC);
}

int main (void){
  DDRD = BIT5 + BIT6 + BIT7;

  PORTD &= ~(BIT5);
  PORTD &= ~(BIT6);
  PORTD &= ~(BIT7);

  PORTD |= BIT2;

  EICRA |= BIT1 + BIT0;
  EIMSK |= BIT0;
  
  ADMUX = BIT6;
  ADCSRA |= (1<< ADPS2) | (1<< ADPS1) | (1<< ADPS0) | (1<< ADIE) | (1<< ADEN);
  ADCSRA = (1 << ADSC);
  ADCSRB = 0;

  TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1);
  TCCR0B = (1 << CS00 | 1 << CS02);


  sei();

  for(;;){
    
  } 
}