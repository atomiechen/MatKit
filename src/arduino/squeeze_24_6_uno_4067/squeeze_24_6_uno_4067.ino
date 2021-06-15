#define reg_x PORTB
#define reg_y_high PORTD
#define ang_in A0

#define set_mux_reg(x, reg) (reg = (reg & 0xf0) + (x & 0xf))
#define set_mux_reg_high(x, reg) (reg = (reg & 0xf) + (x<<4))
#define sample(out) (out = (analogRead(ang_in) >> 2))
#define sample_delay(out, d) (delayMicroseconds(d),sample(out))

uint8_t pin_mux_x[4] = {8, 9, 10, 11};
uint8_t pin_mux_y[4] = {4, 5, 6, 7};
int buf[512];
volatile byte out_voltage;
volatile byte cnt = 0;

void set_ADC_prescaler(byte mode = 128) {
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
  
  switch (mode) {
    case 2:
      ADCSRA |= bit (ADPS0);                               //   2  
      break;
    case 4:
      ADCSRA |= bit (ADPS1);                               //   4  
      break;
    case 8:
      ADCSRA |= bit (ADPS0) | bit (ADPS1);                 //   8  
      break;
    case 16:
      ADCSRA |= bit (ADPS2);                               //  16 
      break;
    case 32:
      ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32 
      break;
    case 64:
      ADCSRA |= bit (ADPS1) | bit (ADPS2);                 //  64 
      break;
    case 128:
    default:
      ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128
  }
  
//  ADCSRA |= bit (ADPS0);                               //   2  
//  ADCSRA |= bit (ADPS1);                               //   4  
//  ADCSRA |= bit (ADPS0) | bit (ADPS1);                 //   8  
//  ADCSRA |= bit (ADPS2);                               //  16 
//  ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32 
//  ADCSRA |= bit (ADPS1) | bit (ADPS2);                 //  64 
//  ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128

}

void set_ADC_interrupt() {
  cli();

  // ADCSRA = 0; // clear ADCSRA register
  // ADCSRB = 0; // clear ADCSRB register
  ADMUX |= (0 & 0x07); // set A0 analog input pin
  ADMUX |= (1 << REFS0); // set reference voltage to AVcc
  ADMUX |= (1 << ADLAR); // left align ADC value to 8 bits from ADCH register
  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE); // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); // enable ADC
  ADCSRA |= (1 << ADSC); // start ADC measurements

  sei();
}

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(pin_mux_x[i], OUTPUT);
    pinMode(pin_mux_y[i], OUTPUT);
  }
  Serial.begin(500000);

  // set_ADC_interrupt(); // 必须在下面的set_ADC_prescaler之前，因为清零了ADCSRA
  set_ADC_prescaler(16);
}

void loop() {
  scan();
}

// 24*6 reshaped as 12*12 according to wiring
void scan() {
  uint8_t x = 0, y = 0, t = 0;
  Serial.write(0xff);
  for (x = 0; x < 12; x++) {
    set_mux_reg(x, reg_x);
    t = 0;
    for (y = 0; y < 12; y++) {
      set_mux_reg_high(t, reg_y_high);
      sample(out_voltage);
      Serial.write(out_voltage);

      if (t < 5) {
        t++;
      } else if (t == 5) {
        t = 11;
      } else {
        t--;
      }
    }
  }
}
