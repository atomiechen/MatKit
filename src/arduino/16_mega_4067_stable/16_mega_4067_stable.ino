// useful macros
#define CLR(x, y) (x &= (~(1<<y))) // clears a bit by setting it to 0
#define SET(x, y) (x |= (1<<y)) // sets a bit by setting it to 1
#define CHK(x, y) (x & (1<<y)) // checks if a bit is set. It will result in true if that bit is 1
#define TOG(x, y) (x ^= (1<<y)) // toggles a bit. If it was 1 then now it is 0 and if it was 0 then it is set to 1

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // clear bit in special funciton register
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit)) // set bit in special funciton register

#define pin_mux_1_reg PORTB
#define pin_mux_2_reg PORTL

uint8_t pin_mux_1[4] = {50, 51, 52, 53};
uint8_t pin_mux_2[4] = {46, 47, 48, 49};
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

// ISR(ADC_vect) {
//   out_voltage = ADCH; // read 8 bit value from ADC
//   cnt = 1;
// //  Serial.println("#");
// }

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
//   scan();
  // send();

  scan_interrupt();
}

void scan_interrupt() {
  Serial.write(0xff);
  int ptr = 0;
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      // cnt = 0;
      set_mux(x, y);
      // while (cnt == 0);  // 这里需要等待ADC结果
      int val = analogRead(A0);
      out_voltage = val >> 2;
      Serial.write(out_voltage);
    }
  }
}

void scan() {
  Serial.write(0xff);
  int ptr = 0;
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      int rx = test_resistance(x, y);
      // buf[ptr++] = rx;
      Serial.write((highByte(rx << 3)) & 31);
      Serial.write(lowByte(rx & 31));
    }
  }
}

void send() {
  Serial.write(0xff);
  for (int i = 0; i < 256; i++) {
    int rx = buf[i];
    Serial.write((highByte(rx << 3)) & 31);
    Serial.write(lowByte(rx & 31));
  }
}

inline void set_mux(uint8_t x, uint8_t y) {
  PORTB &= 0xf0;
  PORTB += x & 0xf;
  PORTL &= 0xf0;
  PORTL += y & 0xf;
}

int test_resistance(uint8_t x, uint8_t y) {
//   digitalWrite(pin_mux_1[0], (x & 1? HIGH : LOW));
//   digitalWrite(pin_mux_1[1], (x & 2? HIGH : LOW));
//   digitalWrite(pin_mux_1[2], (x & 4? HIGH : LOW));
//   digitalWrite(pin_mux_1[3], (x & 8? HIGH : LOW));
//   digitalWrite(pin_mux_2[0], (y & 1? HIGH : LOW));
//   digitalWrite(pin_mux_2[1], (y & 2? HIGH : LOW));
//   digitalWrite(pin_mux_2[2], (y & 4? HIGH : LOW));
//   digitalWrite(pin_mux_2[3], (y & 8? HIGH : LOW));

  // 调整寄存器的低4位，直接取为x、y的低4位。前提要求引脚必须刚好是某个寄存器的低4位。
  set_mux(x, y);

//  Serial.println(pin_mux_1_reg);
//  Serial.println(pin_mux_2_reg);
  
  int voltage = analogRead(pin_out_vtg);
  
  return voltage;
}
