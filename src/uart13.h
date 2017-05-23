/* ���������� ����������� ���������� UART ��� ����������������� ATtiny */
/* �����: ������ ���������, 2015 �. */

#ifndef _UART13_H_
#define _UART13_H_ 1

#include <avr/io.h>
#include <avr/interrupt.h>

/*
*	���� ������������� ����� � ���� ������ ������� ����� ��������������
*	��� ���������� � �������� UART.
*/

#define TXPORT PORTB	// ��� ����� ��� ��������
#define RXPORT PINB		// ��� ����� �� �����
#define TXDDR DDRB		// ������� ����������� ����� �� ��������
#define RXDDR DDRB		// ������� ����������� ����� �� �����
#define TXD 0			// ����� ���� ����� ��� ������������� �� ��������
#define RXD 1			// ����� ���� ����� ��� ������������� �� �����

/*
*	���� �������� ���������, ������������ �������� �������� ������ (�������)
*	������ BAUD_DIV �������������� ��������� �������:
*	BAUD_DIV = (CPU_CLOCK / DIV) / BAUD_RATE
*	��� CPU_CLOCK - �������� ������� �����������, BAUD_RATE - �������� �������� UART,
*	� DIV - �������� �������� ������� �������, ���������� � �������� TCCR0B.
*	��������, �������� �� 8, �������� ����� 9600 ���:
*	BAUD_DIV = (9 600 000 / 8) / 9600 = 125 (0x7D).
*/

//#define T_DIV		0x01	// DIV = 1
#define T_DIV		0x02	// DIV = 8
//#define T_DIV		0x03	// DIV = 64
#define BAUD_DIV	0x7D	// �������� = 9600 ���.

/*
*	���� ���� ���������� ���������� ���������� � ������� ��� ������ UART
*/

volatile uint16_t txbyte;
volatile uint8_t rxbyte;
volatile uint8_t txbitcount;
volatile uint8_t rxbitcount;

void uart_init();
void uart_send(uint8_t tb);
int16_t uart_recieve(uint8_t* rb);

#endif /* _UART13_H_ */
