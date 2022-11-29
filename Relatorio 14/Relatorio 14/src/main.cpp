#include <Arduino.h>

#include <stdio.h>
#include <stdlib.h>

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

#define FOSC 16000000U
#define BAUD 9600
#define MYUBRR FOSC / 16 / (BAUD - 1)
#define botao (1 << PD4)

char msg_tx[20];
char msg_rx[32];
int pos_msg_rx = 0;
int tamanho_msg_rx = 3;
unsigned int x = 0, valor = 0;

void UART_Init(unsigned int ubrr);
void UART_Transmit(char *dados);


int main(void)
{
  UART_Init(MYUBRR);
  sei();
  PORTD |= botao;

  UART_Transmit("Digite 'ola':\n");

  x = 0;

  while (x == 0)
  {
    if ((msg_rx[0] == 'o') &&
    (msg_rx[1] == 'l') &&
    (msg_rx[2] == 'a'))
    {
      x=1;
    }
  } 
  UART_Transmit("Digite '250':\n");

  
  x = 0;
  valor = 0;
  while (x == 0)
  {
    valor = (msg_rx[0] - 48) * 100 +
    (msg_rx[1] - 48) * 10 +
    (msg_rx[2] - 48) * 1;

    if (valor == 250)
    x = 1;
  }

  x=0;
  
  UART_Transmit("Aperte o botao:\n");
  // Super-loop
  while (1) {
    if ((PIND & botao) == 0) // O botao foi pressionado?
    {
      // Se sim, envia mensagem
      UART_Transmit("Botão apertado, Parabéns!!\n");
      x++;

      UART_Transmit("num vezes botao press: ");
      itoa(x, msg_tx, 10);
      UART_Transmit(msg_tx);
      UART_Transmit("\n");

      _delay_ms(500); // Aguarda um tempo para evitar o bounce
    }
  }
}
ISR(USART_RX_vect)
{
  // Escreve o valor recebido pela UART na posição pos_msg_rx do buffer msg_rx
  msg_rx[pos_msg_rx++] = UDR0;
  if (pos_msg_rx == tamanho_msg_rx)
  pos_msg_rx = 0;
}

void UART_Transmit(char *dados)
{
  // Envia todos os caracteres do buffer dados ate chegar um final de linha
  while (*dados != 0)
  {
    while (!(UCSR0A & (1 << UDRE0))); // Aguarda a transmissão acabar
    // Escreve o caractere no registro de tranmissão
    UDR0 = *dados;
    // Passa para o próximo caractere do buffer dados
    dados++;
  }
}

void UART_Init(unsigned int ubrr)
{
  // Configura a baud rate */
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  // Habilita a recepcao, tranmissao e interrupcao na recepcao */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  // Configura o formato da mensagem: 8 bits de dados e 1 bits de stop */
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}