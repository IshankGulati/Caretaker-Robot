
#define F_CPU 14745600
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

volatile unsigned char data;
volatile unsigned long int left_shaft_count =0;   //to keep track of left position encoder
volatile unsigned long int right_shaft_count =0;  ////to keep track of right position encoder
volatile bool flag=1;

void move(unsigned char);


/*
* Function Name: motion_pin_config
* Input: None
* Output: None
* Logic: Initialize the pins for motion of bot
* Example Call: motion_pin_config();
*/
void motion_pin_config(void)
{
	DDRA |= 0X0F;
	DDRL |= 0X18;  //Setting PL3 and PL4 pins as output for PWM generation
	PORTA &= 0XF0;
	PORTL |= 0X18;  //PL3 and PL4 pins are for velocity control using PWM.
}

/*
* Function Name: left_encoder_pin_config
* Input: None
* Output: None
* Logic: Initialize the pins corresponding to left encoder of bot
* Example Call: left_encoder_pin_config();
*/

void left_encoder_pin_config(void)
{
	DDRE &= 0XEF;  //Set the direction of the PORTE 4 pin as input
	PORTE |=0x10;  //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name: right_encoder_pin_config
* Input: None
* Output: None
* Logic: Initialize the pins corresponding to right encoder of bot
* Example Call: right_encoder_pin_config();
*/

void right_encoder_pin_config(void)
{
	DDRE &= 0XDF;  //Set the direction of the PORTE 4 pin as input
	PORTE |=0x20;  //Enable internal pull-up for PORTE 4 pin
}

/*
* Function Name: buzzer_pin_config
* Input: None
* Output: None
* Logic: Initialize the pins corresponding to the buzzer
* Example Call: buzzer_pin_config();
*/
void buzzer_pin_config(void)
{
	DDRC |= 0x08;   //Setting PORTC 3 as output
	PORTC &= 0xF7;  //Setting PORTC 3 logic low to turnoff buzzer
}

/*
* Function Name: rgb_led_config
* Input: None
* Output: None
* Logic: Initialize the pins corresponding to RGB leds on PORT J(expansion slot)
* Example Call: rgb_led_config();
*/

void rgb_led_config(void)
{
	DDRJ = 0xFF;     //Setting PORTJ as output
	PORTJ &= 0xDD;   //Setting PORTJ logic low to turnoff all the LEDs initially
}

/*
* Function Name: port_init
* Input: None
* Output: None
* Logic: Initialize all the Ports
* Example Call: port_init();
*/

void port_init()
{
	motion_pin_config();   //robot motion pins config
	left_encoder_pin_config();   //left encoder pin config
	right_encoder_pin_config();  //right encoder pin config
	buzzer_pin_config();  //buzzer_pin_config
	rgb_led_config();   //RGB led config
}

/*
* Function Name: uart0_init
* Input: None
* Output: None
* Logic: Initialize the Universal Asynchronous Receiver Transmitter 0 of Atmega 2560
* Example Call: uart0_init();
*/

void uart0_init(void)
{
	// desired baud rate:9600
	// char size: 8 bit
	// parity: Disabled
	UCSR0A = 0X00;   //disable while setting baud rate
	UCSR0B = 0X00;
	UCSR0C = 0X06;
	UBRR0H = 0X00;   //set baud rate hi
	UBRR0L = 0X5F;   // 14745600 Hzset baud rate lo
	UCSR0B = 0X98;
}

/*
* Function Name: left_encoder_interrupt_init
* Input: None
* Output: None
* Logic: Initialize the Interrupts for Encoder of left motor
* Example Call: left_encoder_interrupt_init();
*/

void left_encoder_interrupt_init(void)
{
	cli();   //Clears the global interrupt
	EICRA |= 0X00;
	EICRB |= 0X02;   // INT4 is set to trigger with falling edge
	EIMSK |= 0X10;   // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

/*
* Function Name: right_encoder_interrupt_init
* Input: None
* Output: None
* Logic: Initialize the Interrupts for Encoder of right motor
* Example Call: right_encoder_interrupt_init();
*/

void right_encoder_interrupt_init(void)
{
	cli();  //Clears the global interrupt
	EICRA |= 0X00;
	EICRB |= 0X08;   // INT5 is set to trigger with falling edge
	EIMSK |= 0X20;   // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

/*
* Function Name: ISR
* Input: INT4_vect
* Output: None
* Logic: Interrupt service routine for INT4 corresponding to encoder of left motor
* Example Call: None
*/

ISR(INT4_vect)
{
	left_shaft_count++;   //increment left shaft position count
}

/*
* Function Name: ISR
* Input: INT5_vect
* Output: None
* Logic: Interrupt service routine for INT5 corresponding to encoder of right motor
* Example Call: None
*/

ISR(INT5_vect)
{
	right_shaft_count++;   //increment right shaft position count
}

/*
* Function Name: timer5_init
* Input: None
* Output: None
* Logic: Initialize timer 5 for PWM to control the speed of motors
* Example Call: timer5_init();
*/

void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
* Function Name: velocity
* Input: unsigned char left_motor, unsigned char right_motor
* Output: None
* Logic: To control the velocity of motors of robot
* Example Call: velocity(255,255);
*/

void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;   //left motor velocity control
	OCR5BL = (unsigned char)right_motor;  //right motor velocity control
}

/*
* Function Name: linear_distance
* Input: unsigned int distance_mm
* Output: None
* Logic: To control the linear distance robot will move
* Example Call: linear_distance(100);
*/

void linear_distance(unsigned int distance_mm)
{
	float reqd_shaft_count = 0;
	unsigned long int reqd_shaft_count_int = 0;
	
	reqd_shaft_count = distance_mm / 5.44;   //division by resolution to convert distance to shaft count
	reqd_shaft_count_int = (unsigned long int)reqd_shaft_count;   //typecasting
	
	right_shaft_count = 0;
	while(1)
	{
		if(right_shaft_count >= reqd_shaft_count_int)
		{
			break;
		}
	}
	move(0x00);   //stop the robot
}

/*
* Function Name: rotate_degrees
* Input: unsigned int degrees
* Output: None
* Logic: To control the degree of rotation of robot
* Example Call: rotate_degrees(90);
*/

void rotate_degrees(unsigned int degrees)
{
	float reqd_shaft_count = 0;
	unsigned long int reqd_shaft_count_int = 0;
	
	reqd_shaft_count = (float)degrees /4.09;  //division by resolution to convert degrees of rotation to shaft count
	reqd_shaft_count_int = (unsigned long int) reqd_shaft_count;  //typecasting
	
	right_shaft_count = 0;
	left_shaft_count = 0;
	while(1)
	{
		if((right_shaft_count >= reqd_shaft_count_int)|(left_shaft_count >= reqd_shaft_count_int))
		{
			break;
		}
	}
	
	move(0x00);  //stop robot
}

/*
* Function Name: buzzer_on
* Input: None
* Output: None
* Logic: To switch on the buzzer
* Example Call: buzzer_on();
*/

void buzzer_on(void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

/*
* Function Name: buzzer_off
* Input: None
* Output: None
* Logic: To switch off the buzzer
* Example Call: buzzer_off();
*/

void buzzer_off(void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

/*
* Function Name: move
* Input: unsigned char direction
* Output: None
* Logic: To control the direction of movement of robot
* Example Call: move(0x01);
*/

void move(unsigned char direction)
{
	volatile unsigned int distance_mm_1 =267;
	volatile unsigned int distance_mm_2 = 30;
	volatile unsigned int distance_mm_3 = 237;
	volatile unsigned int distance_mm_4 = 90;
	volatile unsigned int distance_mm_5 = 95;
	volatile unsigned int distance_mm_hor =140;
	volatile unsigned int distance_mm_back = 108;
	volatile unsigned int degrees_r = 90;
	volatile unsigned int degrees_l = 90;
	volatile unsigned int degrees_pi= 180;
	
	if(direction == 0x00)
	{
		PORTA = 0x00;  //stop
	}
	else if(direction == 0x01)
	{
		velocity(225,205);
		PORTA = 0x06; //forward
		linear_distance(distance_mm_1);
	}
	else if(direction == 0x02)
	{
		velocity(225,205);
		PORTA = 0x09;  //backward
		linear_distance(distance_mm_back);
	}
	else if(direction == 0x03)
	{
		velocity(160,150);
		PORTA = 0x06;  //forward
		linear_distance(distance_mm_2);
		PORTA = 0x0A;  //zero right
		rotate_degrees(degrees_r);
	}
	else if(direction == 0x04)
	{
		velocity(156,149);
		PORTA = 0x06;   //forward
		linear_distance(distance_mm_2);
		PORTA = 0x05;  //zero left
		rotate_degrees(degrees_l);
	}
	 else if(direction == 0x05)
	{
		velocity(225,205);
		PORTA = 0x06; //forward
		linear_distance(distance_mm_hor);
	}
	else if(direction == 0x13)
	{
		velocity(153,149);
		PORTA = 0x0A;  //zero right
		rotate_degrees(degrees_pi);
	}
	else if(direction == 0x14)
	{
		velocity(225,205);
		PORTA = 0x06; //forward
		linear_distance(distance_mm_3);		
	}
	else if(direction == 0x15)
	{
		velocity(154,149);
		PORTA = 0x05;  //zero left
		rotate_degrees(degrees_pi);
	}
	else if(direction == 0x16)
	{
		velocity(225,205);
		PORTA = 0x06;  //forward
		linear_distance(distance_mm_4);
	}
	else if(direction == 0x17)
	{
		velocity(225,205);
		PORTA = 0x06;  //forward
		linear_distance(distance_mm_5);
	}
}

/*
* Function Name: rgb_led_on
* Input: unsigned char color
* Output: None
* Logic: To switch on the LEDs
* Example Call: rgb_led_on(0x07);
*/

void rgb_led_on(unsigned char color)
{
	if(color == 0x07)
	{
		PORTJ |= 0x01;  // LED1 -> Red 
	}
	else if(color == 0x08)
	{
		PORTJ |= 0x08;  // LED1 -> Blue
	}
	else if(color == 0x09)
	{
		PORTJ |= 0x04;  // LED1 -> Yellow	
	}
	else if(color == 0x0A)
	{
		PORTJ |= 0x10;  // LED2 -> Red
	}
	else if(color == 0x0B)
	{
		PORTJ |= 0x80;  // LED2 -> Blue
	}
	else if(color == 0x0C)
	{
		PORTJ |= 0x40;  // LED2 -> Yellow
	}
}

/*
* Function Name: rgb_led_off
* Input: unsigned char color
* Output: None
* Logic: To switch off the LEDs
* Example Call: rgb_led_off(0x0D);
*/

void rgb_led_off(unsigned char color)
{
	if(color == 0x0D)
	{
		PORTJ &= 0xFE;  // LED1 -> Red
	}
	else if(color == 0x0E)
	{
		PORTJ &= 0xF7;  // LED1 -> Blue
	}
	else if(color == 0x0F)
	{
		PORTJ &= 0xFB;  // LED1 -> Yellow
	}
	else if(color == 0x10)
	{
		PORTJ &= 0xEF;  // LED2 -> Red
	}
	else if(color == 0x11)
	{
		PORTJ &= 0x7F;  // LED2 -> Blue
	}
	else if(color == 0x12)
	{
		PORTJ &= 0xBF;  // LED2 -> Yellow
	}
}

/*
* Function Name: SIGNAL
* Input: USART0_RX_vect
* Output: None
* Logic: ISR for receive complete interrupt through USART0
* Example Call: None
*/

SIGNAL(USART0_RX_vect)
{
	while(!(UCSR0A&(1<<RXC0)));
	if(flag == 1)
	{
		data = UDR0;   //store the received hex code in variable data
		flag =0;      //if any command is executing(or mcu is busy) flag->0 
					 //if mcu is free to receive further commands flag->1
	}
	sei();
	// Commands to control the movement of robot
	if(data == 0x00||data ==0x01||data ==0x02||data ==0x03||data ==0x04||data ==0x05 || data == 0x13 || data == 0x14 || data == 0x15 || data == 0x16 ||data == 0x17 )
	{
		move(data);
	}
	// Commands to switch the buzzer
	else if(data == 0x06)
	{
		buzzer_on();
		_delay_ms(7000);
		buzzer_off();	
	}
	// Commands to switch on the RGB LEDs
	else if(data == 0x07 ||data == 0x08 ||data == 0x09 || data == 0x0A ||data ==0x0B||data == 0x0C)
	{
		rgb_led_on(data);
	}
	// Commands to switch off the RGB LEDs
	else if(data == 0x0D ||data == 0x0E ||data == 0x0F||data == 0x10||data == 0x11||data == 0x12)
	{
		rgb_led_off(data);
	}
	flag = 1;    //flag is set after the completion of the command
	UDR0 = data; //echo data back
}

/*
* Function Name: init
* Input: USART0_RX_vect
* Output: None
* Logic: To initialize all the devices
* Example Call: init();
*/

void init()
{
	cli();   //Clears the global interrupts
	port_init();   //Initializes all the ports
	uart0_init();   //Initialize UART0 for serial communication
	left_encoder_interrupt_init();   //Initialize interrupts for left encoder
	right_encoder_interrupt_init();  //Initialize interrupts for right encoder
	timer5_init();   //Initialize TIMER5 for PWM
	sei();   //Enables the global interrupts
}

/*
* Function Name:main
* Input: None
* Output: None
* Logic: main function
* Example Call: None
*/

int main(void)
{
	init();
    while(1);
}
