#include <Arduino.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000
#define pwm_out (1 << PD6)

int aux = 0;
int valor = 0;
int tensao = 0;

int leitura_AD;
int brightness = 0;

ISR(ADC_vect){
  leitura_AD = (ADCL | (ADCH << 8)); //Ou leitura_AD = ADC;
  ADCSRA = (1 << ADSC);
}

ISR(INT0_vect){
  if(brightness < 255){
    brightness += leitura_AD;
  }
  else
    brightness = 0;

  OCR0A = brightness;
}

int main (void){

  Serial.begin(9600);

  aux = valor*5000;
  aux = aux/1023;
  tensao = (int)aux;

  DDRD |= pwm_out;

  PORTD &= ~(pwm_out);

  EICRA |= BIT1 + BIT0;
  EIMSK |= BIT0;

  TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);
  TCCR0B = (1 << CS00);

  ADMUX = BIT6;
  ADCSRA |= (1<< ADPS2) | (1<< ADPS1) | (1<< ADPS0) | (1<< ADIE);
  ADCSRA = (1 << ADSC);
  ADCSRB = 0;

  sei();
  for(;;){
    Serial.println(tensao);
  }
}