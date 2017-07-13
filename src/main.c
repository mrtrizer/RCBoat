/* Библиотека программной реализации UART для микроконтроллеров ATtiny */

#ifndef _UART13_H_
#define _UART13_H_ 1

#include <avr/io.h>
#include <avr/interrupt.h>

/*
*	Ниже настраиваются порты и пины портов которые будут использоваться
*	как передатчик и приемник UART.
*/

#define TXPORT PORTB		// Имя порта для передачи
#define RXPORT PINB		// Имя порта на прием
#define TXDDR DDRB		// Регистр направления порта на передачу
#define RXDDR DDRB		// Регистр направления порта на прием
#define TXD 3			// Номер бита порта для использования на передачу
#define RXD 1			// Номер бита порта для использования на прием

/*
*	Ниже задаются константы, определяющие скорость передачи данных (бодрейт)
*	расчет BAUD_DIV осуществляется следующим образом:
*	BAUD_DIV = (CPU_CLOCK / DIV) / BAUD_RATE
*	где CPU_CLOCK - тактовая частота контроллера, BAUD_RATE - желаемая скорость UART,
*	а DIV - значение делителя частоты таймера, задающееся регистром TCCR0B.
*	Например, тактовая частота 9.6 МГц, делитель на 8, скорость порта 9600 бод:
*	BAUD_DIV = (9 600 000 / 8) / 9600 = 125 (0x7D).
*/

//#define T_DIV		0x01	// DIV = 1
//#define T_DIV		0x02	// DIV = 8
#define T_DIV		0x03	// DIV = 64
#define BAUD_DIV	62	// Скорость = 2400 бод

#define bool unsigned char
#define true 1
#define false 0

/*
*	Ниже идут объявления глобальных переменных и функций для работы UART
*/

volatile uint16_t txbyte;
volatile uint8_t rxbyte;
volatile uint8_t txbitcount;
volatile uint8_t rxbitcount;
volatile uint16_t g_servoCounter = 0;
volatile uint16_t g_servoPos = 1;
volatile bool g_motorOn = false;

void uart_init();
void uart_send(uint8_t tb);
int16_t uart_recieve(uint8_t* rb);

#endif /* _UART13_H_ */

void uart_init()
{
	txbyte = 0xFFFF;		// Значение буфера на передачу - все единицы
	rxbyte = 0x00;			// Значение буфера на прием - все нули
	txbitcount = 0x00;		// Значение счетчика преедаваемых бит - ноль (ничего пока не передаем)
	rxbitcount = 0x09;		// Значение счетчика бит на прием - 9 (ожидаем возможного приема)

	TXDDR |= (1 << TXD);		// Задаем направление порта на передачу как выход
	RXDDR &= ~(1 << RXD);		// Задаем направление порта на прием как вход
	TXPORT |= (1 << TXD);		// Пишем единицу в выход TXD
	RXPORT |= (1 << RXD);		// Подтягиваем к единице вход RXD
	OCR0A = BAUD_DIV;		// Задаем значение регистра OCR0A в соответствии с бодрейтом
	TIMSK0 |= (1 << OCIE0A);	// Разрешаем прерывание TIM0_COMPA
	TCCR0A |= (1 << WGM01);		// Режим таймера CTC (очистка TCNT0 по достижению OCR0A)
	TCCR0B |= T_DIV;		// Задаем скорость счета таймера в соответствии с делителем
	MCUCR |= (1 << ISC01);		// Задаем прерывание INT0 по заднему фронту импульса
	GIMSK |= (1 << INT0);		// Разрешаем прерывание INT0
	sei();				// Разрешаем прерывания глобально
}

ISR(TIM0_COMPA_vect)
{
    if (g_servoCounter > g_servoPos) {
        PORTB &= ~(1 << PB2);
    }
    if (g_servoCounter > 50) {
        PORTB |= (1 << PB2);
        g_servoCounter = 0;
    }
    g_servoCounter++;

    static int on = 0;
    if (g_motorOn) {
        if (on > 2) {
            PORTB |= (1 << PB4);
            on = 0;
        } else {
            PORTB &= ~(1 << PB4);
        }
        on++;
    }

	TXPORT = (TXPORT & ~(1 << TXD)) | ((txbyte & 0x01) << TXD); // Выставляем в бит TXD младший бит txbyte
	txbyte = (txbyte >> 0x01) + 0x8000;	// Двигаем txbyte вправо на 1 и пишем 1 в старший разряд (0x8000)
	if(txbitcount > 0)			// Если идет передача (счетик бит больше нуля),
	{
		txbitcount--;			// то уменьшаем его на единицу.
	}
}


ISR(TIM0_COMPB_vect)
{
	if(RXPORT & (1 << RXD))			// Проверяем в каком состоянии вход RXD
		rxbyte |= 0x80;			// Если в 1, то пишем 1 в старший разряд rxbyte

	if(--rxbitcount == 0)			// Уменьшаем на 1 счетчик бит и проверяем не стал ли он нулем
	{
		TIMSK0 &= ~(1 << OCIE0B);	// Если да, запрещаем прерывание TIM0_COMPB
		TIFR0 |= (1 << OCF0B);		// Очищаем флаг прерывания TIM0_COMPB
		GIFR |= (1 << INTF0);		// Очищаем флаг прерывания по INT0
		GIMSK |= (1 << INT0);		// Разрешаем прерывание INT0
	}
	else
	{
		rxbyte >>= 0x01;		// Иначе сдвигаем rxbyte вправо на 1
	}
}

ISR(INT0_vect)
{
	rxbitcount = 0x09;			// 8 бит данных и 1 стартовый бит
	rxbyte = 0x00;				// Обнуляем содержимое rxbyte
	if(TCNT0 < (BAUD_DIV / 2))		// Если таймер не досчитал до середины текущего периода
	{
		OCR0B = TCNT0 + (BAUD_DIV / 2);	// То прерывание произойдет в текущем периоде спустя пол периода
	}
	else
	{
		OCR0B = TCNT0 - (BAUD_DIV / 2);	// Иначе прерывание произойдет уже в следующем периоде таймера
	}
	GIMSK &= ~(1 << INT0);			// Запрещаем прерывание по INT0
	TIFR0 |= (1 << OCF0A) | (1 << OCF0B);	// Очищаем флаги прерываний TIM0_COMPA (B)
	TIMSK0 |= (1 << OCIE0B);		// Разрешаем прерывание по OCR0B
}

void uart_send(uint8_t tb)
{
	while(txbitcount);		// Ждем пока закончится передача предыдущего байта
	txbyte = (tb + 0xFF00) << 0x01; // Пишем в младшие разряды txbyte данные для передачи и сдвигаем влево на 1
	txbitcount = 0x0A;		// Задаем счетчик байт равным 10
}

int16_t uart_recieve(uint8_t* rb)
{
	if(rxbitcount < 0x09)		// Если счетчик бит на прием меньше 9
	{
		while(rxbitcount);	// Ждем пока завершится текущий прием
		*rb = rxbyte;		// Пишем по адресу указателя принятый байт
		rxbitcount = 0x09;	// Восстанавливаем значение счетчика бит
		return (*rb);		// Возвращаемся
	}
	else
	{
		return (-1);		// Иначе возвращаем -1 (принимать нечего)
	}
}

int main(void)
{
	uint8_t b = 0;
	uart_init();
    // Servo
    DDRB |= (1 << DDB2);
    // Just light up a led
    DDRB |= (1 << DDB0);
    PORTB |= (1 << PB0);
    // Motor pin
    DDRB |= (1 << DDB4);
	while (1)
	{
		if(uart_recieve(&b) >= 0) {	// Если ничего не приняли, ничего и не передаем
			uart_send(b);		// А если приняли, передаем принятое

            if (b == 'b') {
                g_motorOn = true;
            }
            if (b == 'e') {
                g_motorOn = false;
                PORTB &= ~(1 << PB4);
            }
            if (b == 'l')
                g_servoPos = 1;
            if (b == 'c')
                g_servoPos = 2;
            if (b == 'r')
                g_servoPos = 3;
        }
	}
	return (0);
}
