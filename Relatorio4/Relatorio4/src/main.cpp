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

  DDRD |= BIT7; //pino pd7 definido como saida //LED2
  DDRD |= BIT6; //pino pd6 definido como saida //LED1
  DDRD |= BIT3; //pino pd3 definido como saida //LED3

  PORTD |= BIT5; //habilitar pull-up no pd5 //BOTAO2
  PORTD |= BIT4; //habilitar pull-up no pd6 //BOTAO1

  //PORTD &= ~(BIT7); //desliga a saida do pd7
  PORTD |= BIT7;
  PORTD &= ~(BIT6); //desliga a saida do pd6
  PORTD &= ~(BIT3); //desliga a saida do pd6

  int state = 0;

  for(;;){
    /* EX1
    int botao1 = PIND & BIT4; //LE O ESTADO
    int botao2 = PIND & BIT5; //LE O ESTADO

    while((PIND & BIT5)==0){ //botao pressionado?
      PORTD |= BIT7; //pd7 -> high
      _delay_ms(1000);
      PORTD &= ~(BIT7);
      PORTD |= BIT6;
      _delay_ms(1000);
      PORTD &= ~(BIT6);
    }
    while((PIND & BIT4)==0){
      PORTD |= BIT7; //pd7 -> high
      _delay_ms(100);
      PORTD &= ~(BIT7);
      PORTD |= BIT6;
      _delay_ms(100);
      PORTD &= ~(BIT6);
    }*/
   /* if(botao2==BIT5){
      PORTD &= ~(BIT7); //pd7 -> high
      PORTD &= ~(BIT6);
    }
    if(botao1==BIT4){
      PORTD &= ~(BIT7); //pd7 -> high
      PORTD &= ~(BIT6);
    } */
    
    int botao1 = PIND & BIT4; //LE O ESTADO
    
    
    while((PIND & BIT4)==0){ //botao pressionado?
      switch(state){
        case 0:
          PORTD &= ~(BIT7); //apaga o vermelho
          PORTD |= BIT6; //pd7 -> high
          state++;
          break;
        case 1:
          _delay_ms(4000);
          PORTD |= BIT3;
          _delay_ms(2000);
          PORTD &= ~(BIT3);
          PORTD &= ~(BIT6);
          PORTD |= BIT7;
          state++;
          break;
        default:
          state =0;
          break;
    }
    }

    
  }
}