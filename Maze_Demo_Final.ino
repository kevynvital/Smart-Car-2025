/*
  Barrier Detection Demonstration Ultrasonic Sensor
  By: Diego Morales, Jarren Hill, Kevyn Vital
  I/O Pins
  A0:
  A1:
  A2:
  A3:
  A4: MUX Echo pins
  A5:
  D0:
  D1:
  D2:
  D3: Left Motor Backward Speed OC2B
  D4: 
  D5: Right Motor Backward Speed OC0B
  D6: Right Motor Forward Speed OC0A
  D7: 
  D8: [INPUT] ICU / ECHO on ICU
  D9:  Trigger front
  D10: Trigger right
  D11: Left Motor Forward Speed OC2A
  D12: Left wheel encoder
  D13: Right wheel encoder
*/

// Constants with the values that will be given to the motors
#define leftPWM 100
#define rightPWM 100

volatile unsigned long leftTog = 0;
volatile unsigned long rightTog = 0;

// Variables that are used to calculate the distance with the ultrasonic sensors in the front and on the right of the car
volatile unsigned long captFront[2] = {};
volatile unsigned long captRight[2] = {};
volatile unsigned long count = 0;  // adds 65536 every time TCNT1 overflows

void setup() {
  cli();
  // Setting D3, D5, D6, D9, D10 and D11 as outputs
  DDRD = 0x68;
  DDRB = 0x0E;
  DDRC = 0x10;  // Output on A4 for Trigger Ultrasonic

  // Setting internal pull up resistors for D12 and D13
  PORTB |= 0x30;

  // Setting up TCNT0
  // OC0A and OC0B activated non-inverting mode
  TCCR0A = 0xA1;  // Phase correct PWM mode
  TCCR0B = 0x01;  // Prescaler of 1
  TIMSK0 = 0x00;  // No interrupts

  // TCNT1 Normal mode, 256 prescaler, input capture enable
  TCCR1A = 0x00;
  TCCR1B = 0xC4;
  TCCR1C = 0x00;
  TIMSK1 = 0x21;  // Input capture interrupt enable and overflow interrupt enable

  // Setting up TCNT2
  // OC2A and OC2B activated non-inverting mode
  TCCR2A = 0xA1;  // Phase correct PWM mode
  TCCR2B = 0x01;  // Prescaler of 1
  TIMSK2 = 0x00;  // No interrupts

  //setting up PIN CHANGE interrupts for D12, D13
  PCICR = 0x01;
  PCMSK0 = 0x30;
  sei();
}

void loop() {
  // Depending on the MUX pin
  if (PINC & 0x10) {
    // Sending Right Trigger
    PORTB |= 0x04;
    _delay_us(10);
    PORTB &= ~0x04;

  } else {
    // Sending Front Trigger
    PORTB |= 0x02;
    _delay_us(10);
    PORTB &= ~0x02;
  }

  static unsigned long frontDistance = 0;
  static unsigned long rightDistance = 0;

  // Getting the distance in mm
  if (captRight[1] > captRight[0]) {
    rightDistance = ((captRight[1] - captRight[0]) * 340LL) / 125;
  }
  if (captFront[1] > captFront[0]) {
    frontDistance = ((captFront[1] - captFront[0]) * 340LL) / 125;
  }

  // Go forward until a barrier is less than 85mm away from the front sensor
  forward();
  if (frontDistance < 85) {
    // Reverse a little
    reverse();
    leftTog = 0;
    rightTog = 0;
    while (((leftTog + rightTog) / 2) < 4)  // create delay
      ;
    // If there is an object at less than 170mm, turn left
    if (rightDistance < 170) {
      left();
      leftTog = 0;
      while ((leftTog) < 17)
        ;
    } else {
      // If there isn't an object to the right, turn right
      right();
      rightTog = 0;
      while ((rightTog) < 23)
        ;
    }
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

void reverse() {
  // Turning OC2B on and OC2A off
  TCCR2A |= 0x20;
  TCCR2A &= ~0x80;
  OCR0A = 0;
  OCR0B = rightPWM;
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

// Getting the information from the ultrasonic sensors
ISR(TIMER1_CAPT_vect) {
  statusVar = SREG;
  // Receiving rising edge signal
  if (TCCR1B & (1 << ICES1)) {
    // If the MUX control pin A4 is 1, it will capture from the right ultrasonic sensor, else it will from the front one
    if (PINC & 0x10) {
      captRight[0] = count + ICR1;
    } else {
      captFront[0] = count + ICR1;
    }
    TCCR1B &= 0XBF;  // Clearing ICES1 to receive on falling edge
  }
  // Receiving falling edge signal
  else {
    // If the MUX control pin A4 is 1, it will capture from the right ultrasonic sensor, else it will from the front one
    if (PINC & 0x10) {
      captRight[1] = count + ICR1;
      PORTC &= ~0x10;  // Clearing MUX pin to read from the front Ultrasonic sensor
    } else {
      captFront[1] = count + ICR1;
      PORTC |= 0x10;  // Setting MUX pin to read from right Ultrasonic sensor
    }
    TCCR1B |= 0x40;  // Setting ICES1 to receive on rising edge
  }
  SREG = statusVar;
}

// Every time the TCNT1 overflows, it will add 65536 to the count variable
ISR(TIMER1_OVF_vect) {
  statusVar = SREG;
  count += 65536;
  SREG = statusVar;
}