#define SYNC 8
#define DATA 11
#define SCLK 13

#define CSA 6
#define CSB 5

byte clr;
volatile byte out_voltage;
volatile byte cnt = 0;

void set_ADC_interrupt() {
  cli();
  ADMUX |= (0 & 0x07); // set A0 analog input pin
  ADMUX |= (1 << REFS0); // set reference voltage to AVcc
  ADMUX |= (1 << ADLAR); // left align ADC value to 8 bits from ADCH register
  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE); // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); // enable ADC
  ADCSRA |= (1 << ADSC); // start ADC measurements
  sei();
}

ISR(ADC_vect) {
  out_voltage = ADCH; // read 8 bit value from ADC
  cnt = 1;
}

// ref: https://www.arduino.cc/en/Tutorial/SPIEEPROM
inline char spi_transfer(volatile char data) {
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)));    // Wait the end of the transmission
  return SPDR;                    // return the received byte
}

void setup() {
  Serial.begin(500000);

  pinMode(DATA, OUTPUT);
  pinMode(SYNC, OUTPUT);
  pinMode(SCLK, OUTPUT);
  // must set SS to HIGH to enable master mode
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  
  // SPCR = 01010100
  // | SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
  // interrupt disabled,spi enabled,msb 1st,master mode,clk low when idle,
  // sample on falling edge of clk,system clock/4 rate (fastest)
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPHA);

  clr=SPSR;
  clr=SPDR;
  // delay(10);

  set_ADC_interrupt(); // 必须在下面语句之前，因为清零了ADCSRA
  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2));  // clear prescaler bits
  ADCSRA |= bit (ADPS2);  // set prescaler 16
}

void loop() {
  scan_interrupt();
}

void scan_interrupt() {
  Serial.write(0xff);
  uint8_t reg_x = (1<<CSB);  // 固定CSB，设置CSA
  for (int x = 0; x < 16; x++) {
    set_mux(reg_x);
    uint8_t reg_y = (1<<CSA);  // 固定CSA，设置CSB
    for (int y = 0; y < 16; y++) {
      set_mux(reg_y);
      cnt = 0;
      while (cnt == 0);  // 这里需要等待ADC结果
      Serial.write(out_voltage);
      reg_y++;
    }
    reg_x++;
  }
}

inline void set_mux(uint8_t reg) {
  // set SYNC to LOW
  PORTB &= 0xfe;

  // | EN | CSA | CSB | X | A3 | A2 | A1 | A0 |
  spi_transfer(reg);

  // set SYNC to HIGH
  PORTB |= 0x1;
}
