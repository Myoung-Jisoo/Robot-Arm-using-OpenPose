#include "mcu_init.h"

void UART1_INIT(){
	DDRD = 0b00001000; // PD2에 있는 RX는 입력 PD3에 있는 TX는 출력
	
	UCSR1A = 0x00; // 전송속도 2배 설정안함
	UCSR1B = 0b00011000; // RX Complete Interrupt 사용 안하고 RX, TX Enable 및 8bit 데이터 사용
	UCSR1C = 0b00000110; // 비동기 모드, Non parity, Stop bit 1, 8bit 사용
	
	UBRR1H = 0;
	UBRR1L = 8; // Board rate 115200
}

void UART1_Transmit(unsigned char cData){
	while(!(UCSR1A & (1<<UDRE1)));
	UDR1 = cData;
}

unsigned char UART1_Receive(){
	while(!(UCSR1A & (1<<RXC1)));
	return UDR1;
}

void UART1_TransNum(int num) //숫자를 uart로 출력
{
	int j;
	if(num < 0)
	{
		UART1_Transmit('-');
		num = -num;
	}
	
	for(j = 1000 ; j > 0; j /= 10)
	{
		UART1_Transmit((num/j) + 48);
		num %= j;
	}
	UART1_Transmit(' ');
}

void Timer2_INIT(){
	TCCR2 = (0<<WGM21) | (0<<WGM20) | (0<<COM21) | (0<<COM20) | (1<<CS22) | (0<<CS21) | (0<<CS20);
	// Nomal, Normal port operation, Clear OC2 on compare match, 분주비 256
	TIMSK = (1<<TOIE2); // T/C2 Overflow Interrupt Enable
	TCNT2 = 131;
}

void BUTTON_INIT()
{
	DDRD = 0x00;
	
	EIMSK = (1<<INT0) | (1<<INT1);
	EICRA = (1<<ISC01) | (1<<ISC11);
}

void SERVO_MOTOR_INIT(){
	DDRE = 0b00110000; // PE5, PE4 출력
	PORTE = 0b00110000;
	TCCR3A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) |(1<<COM3C1) | (0<<COM3C0) | (1<<WGM11);
	TCCR3B = (1<<WGM13) | (1<<WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);
	// Fast PWM mode, 분주비 256
	ICR3 = 625-1; // 주기 10ms
}

void MOTOR_Direction(double a, double b){
	OCR3C = 1250 * a;
	OCR3B = 1250 * b;
}

void ADC_INIT(){
	DDRF = 0x00;
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC Enable, 분주비 128
}

int ADC_Receive(int i){ // adc값 받아온다.
	ADCSRA |= (1<<ADSC);
	ADMUX = i;
	
	while(!(ADCSRA & (1<<ADIF)));
	return ADC;
}