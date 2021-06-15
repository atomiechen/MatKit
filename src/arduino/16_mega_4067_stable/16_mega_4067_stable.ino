// useful macros
#define CLR(x, y) (x &= (~(1<<y))) // clears a bit by setting it to 0
#define SET(x, y) (x |= (1<<y)) // sets a bit by setting it to 1
#define CHK(x, y) (x & (1<<y)) // checks if a bit is set. It will result in true if that bit is 1
#define TOG(x, y) (x ^= (1<<y)) // toggles a bit. If it was 1 then now it is 0 and if it was 0 then it is set to 1

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // clear bit in special funciton register
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) // set bit in special funciton register

#define reg_x PORTB
#define reg_y PORTL
#define ang_in A0

#define set_mux_reg(x, reg) (reg = (reg & 0xf0) + (x & 0xf))
#define sample(out) (out = (analogRead(ang_in) >> 2))
#define sample_delay(out, d) (delayMicroseconds(d),sample(out))

uint8_t pin_mux_1[4] = {53, 52, 51, 50};
uint8_t pin_mux_2[4] = {49, 48, 47, 46};
uint32_t pin_out_vtg = A0;
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
    pinMode(pin_mux_1[i], OUTPUT);
    pinMode(pin_mux_2[i], OUTPUT);
  }
  Serial.begin(500000);

  // set_ADC_interrupt(); // 必须在下面的set_ADC_prescaler之前，因为清零了ADCSRA
  set_ADC_prescaler(16);
}

void loop() {
  scan();
//  scan_delay();
//  scan_opt();

//  scan_test_digital();
}

void scan() {
  uint8_t x = 0, y = 0;
  Serial.write(0xff);
  for (x = 0; x < 16; x++) {
    set_mux_reg(x, reg_x);
    for (y = 0; y < 16; y++) {
      set_mux_reg(y, reg_y);
      sample(out_voltage);
      Serial.write(out_voltage);
    }
  }
}

void scan_delay() {
  uint8_t delay = 50;
  uint8_t x = 0, y = 0;
  Serial.write(0xff);
  for (x = 0; x < 16; x++) {
    set_mux_reg(x, reg_x);
    for (y = 0; y < 16; y++) {
      set_mux_reg(y, reg_y);
      sample_delay(out_voltage, delay);
      Serial.write(out_voltage);
    }
  }
}

void scan_opt() {
  uint8_t d = 30;
  uint8_t x = 0, y = 0;
  set_mux_reg(x, reg_x);
  set_mux_reg(y, reg_y);
  Serial.write(0xff);
  sample_delay(out_voltage, d);

  for (y = 1; y < 16; y++) {
    set_mux_reg(y, reg_y);
    Serial.write(out_voltage);
    sample_delay(out_voltage, d);
  }

  for (x = 1; x < 16; x++) {
    set_mux_reg(x, reg_x);
    for (y = 0; y < 16; y++) {
      set_mux_reg(y, reg_y);
      Serial.write(out_voltage);
      sample_delay(out_voltage, d);
    }
  }
  Serial.write(out_voltage);
}

inline void set_mux(uint8_t x, uint8_t y) {
  PORTB = (PORTB & 0xf0) + (x & 0xf);
  PORTL = (PORTL & 0xf0) + (y & 0xf);
}

void scan_test_digital() {
  uint8_t x = 0, y = 0;
  Serial.write(0xff);
  for (x = 0; x < 16; x++) {
    digitalWrite(pin_mux_1[0], (x & 1? HIGH : LOW));
    digitalWrite(pin_mux_1[1], (x & 2? HIGH : LOW));
    digitalWrite(pin_mux_1[2], (x & 4? HIGH : LOW));
    digitalWrite(pin_mux_1[3], (x & 8? HIGH : LOW));
    for (y = 0; y < 16; y++) {
        digitalWrite(pin_mux_2[0], (y & 1? HIGH : LOW));
        digitalWrite(pin_mux_2[1], (y & 2? HIGH : LOW));
        digitalWrite(pin_mux_2[2], (y & 4? HIGH : LOW));
        digitalWrite(pin_mux_2[3], (y & 8? HIGH : LOW));
      sample(out_voltage);
      Serial.write(out_voltage);
    }
  }
}
