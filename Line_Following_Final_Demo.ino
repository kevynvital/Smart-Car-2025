/*
  Final Line Following Demo 
  By: Diego Morales, Kevyn Vital, Jarren Hill
  Edited: May 5, 2025
  I/O Pins
  A0: Right LF
  A1: Middle LF
  A2: Left LF
  A3:
  A4:
  A5:
  D0:
  D1:
  D2:
  D3: Left Motor Backward Speed OC2B
  D4:
  D5: Right Motor Backward Speed OC0B
  D6: Right Motor Forward Speed OC0A
  D7:
  D8:
  D9:
  D10:
  D11: Left Motor Forward Speed OC2A
  D12: Left wheel encoder
  D13: Right wheel encoder
*/

void setup() {
  cli();

  Serial.begin(9600);

  // Setting D3, D5, D6, and D11 as outputs
  DDRD = 0x68;
  DDRB = 0x08;
  // Setting internal pull up resistors for D12 and D13
  PORTB |= 0x30;

  // Setting up TCNT0
  // OC0A and OC0B activated non-inverting mode
  TCCR0A = 0xA1;  // Phase correct PWM mode
  TCCR0B = 0x01;  // Prescaler of 1
  TIMSK0 = 0x00;  // No interrupts

  // Setting up TCNT2
  // OC2A and OC2B activated non-inverting mode
  TCCR2A = 0xA1;  // Phase correct PWM mode
  TCCR2B = 0x01;  // Prescaler of 1
  TIMSK2 = 0x00;  // No interrupts

  //setting up PIN CHANGE interrupts for D12, D13
  PCICR = 0x01;
  PCMSK0 = 0x30;

  // Setting up ADC autotrigger disabled, starting first conversion, enabling interrupt, 128 prescaler, free-running mode, and 8-bit mode
  ADCSRA = 0xCF;
  ADCSRB = 0x00;
  ADMUX = 0x60;

  sei();
}

// Line follower values
volatile unsigned char leftLF = 0, middleLF = 0, rightLF = 0;
volatile unsigned int rightTog = 0;  // total amount of toggles on right wheel
volatile unsigned int leftTog = 0;   // total amount of toggles on left wheel

// Global variables were we store the values that will be given to the OCRxn
volatile unsigned char rightPWM = 145;    // right wheel forward speed
volatile unsigned char leftPWM = 144;     // left wheel forward speed
volatile unsigned char turn = 125;        // turn wheel speed
volatile unsigned char turnSupport = 50;  // turn support wheel speed


void loop() {
  if (leftLF) {
    left();

  } else if (rightLF) {
    right();

  } else if (middleLF) {
    forward();
  }

  // Set all the PWM variables to 0 to stop the car
  if (((rightTog + leftTog) / 2) > 4600) {
    // Set all PWM global variables equal to 0
    rightPWM = 0;
    leftPWM = 0;
    turn = 0;
    turnSupport = 0;
  }
}

void forward() {
  // Turning OC2A on again
  TCCR2A |= 0x80;
  OCR0A = rightPWM;
  OCR0B = 0;
  OCR2A = leftPWM;
  OCR2B = 0;
}

void right() {
  // Turning OC2A on again
  TCCR2A |= 0x80;
  OCR0A = 0;
  OCR0B = rightPWM;
  OCR2A = leftPWM;
  OCR2B = 0;
}

void left() {
  // Turning OC2B on and OC2A off
  TCCR2A |= 0x20;
  TCCR2A &= ~0x80;
  OCR0A = rightPWM;
  OCR0B = 0;
  OCR2A = 0;
  OCR2B = leftPWM;
}

//This ISR is default toggle and we want it to be falling edge
volatile unsigned char statusVar;
ISR(PCINT0_vect) {
  statusVar = SREG;
  static unsigned char leftOld = 0;      // previous left encoder value
  unsigned char leftNew = PINB & 0x10;   // current left encoder value
  static unsigned char rightOld = 0;     // previous right encoder value
  unsigned char rightNew = PINB & 0x20;  // current right encoder value

  if (leftNew < leftOld) {
    leftTog++;
  }
  if (rightNew < rightOld) {
    rightTog++;
  }
  leftOld = leftNew;
  rightOld = rightNew;
  SREG = statusVar;
}

ISR(ADC_vect) {
  statusVar = SREG;
  switch (ADMUX & 0x0F) {
    case 0:
      // Read from right LF sensor
      if (ADCH > 180) {
        rightLF = 1;  // flag set to call right()
      } else {
        rightLF = 0;
      }
      // Next ISR read from pin A1 (Middle LF)
      ADMUX |= 0x01;
      break;

    case 1:
      // Read from middle LF sensor
      if (ADCH > 210) {
        middleLF = 1;  // flag set to call forward()
      } else {
        middleLF = 0;
      }
      // Next ISR read from pin A2 (Left LF)
      ADMUX |= 0x02;
      ADMUX &= ~0x01;
      break;

    case 2:
      // Read from left LF sensor
      if (ADCH > 180) {
        leftLF = 1;  // flag set to call left()
      } else {
        leftLF = 0;
      }
      // Next ISR read from pin A0 (Right LF)
      ADMUX &= 0xF0;
      break;
  }
  ADCSRA |= 0x40;  // Start new ADC conversion
  SREG = statusVar;
}