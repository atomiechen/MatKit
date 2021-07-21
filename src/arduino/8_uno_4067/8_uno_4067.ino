bool digital_S0[] = {0, 1, 0, 1, 0, 1, 0, 1};
bool digital_S1[] = {0, 0, 1, 1, 0, 0, 1, 1};
bool digital_S2[] = {0, 0, 0, 0, 1, 1, 1, 1};

bool hori_digital_S0[] = {0, 1, 0, 1, 1, 0, 1, 0};
bool hori_digital_S1[] = {0, 0, 1, 1, 1, 1, 0, 0};
bool hori_digital_S2[] = {0, 0, 0, 0, 1, 1, 1, 1};

int vol = 0;

#define reg_x PORTB
#define reg_y_high PORTD
#define ang_in A0
#define set_mux_reg(x, reg) (reg = (reg & 0xf0) + (x & 0xf))
#define set_mux_reg_high(x, reg) (reg = (reg & 0xf) + (x<<4))
#define sample(out) (out = (analogRead(ang_in) >> 2))

uint8_t pin_mux_x[4] = {8, 9, 10, 11};
uint8_t pin_mux_y[4] = {4, 5, 6, 7};

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

void setup() {
  // put your setup code here, to run once:
//  Serial.begin(115200);
//  for (int i = 2; i <= 11; i++){
//    pinMode(i, OUTPUT);
//  }
//  digitalWrite(4, LOW);
//  digitalWrite(6, LOW);
//  digitalWrite(9, LOW);
//  digitalWrite(11, LOW);

  for (int i = 0; i < 4; i++) {
    pinMode(pin_mux_x[i], OUTPUT);
    pinMode(pin_mux_y[i], OUTPUT);
  }
  set_ADC_prescaler(16);
  Serial.begin(500000);
}

void loop() {
  // put your main code here, to run repeatedly:
  scan();
}

void scan_slow() {
  Serial.write(0xFF);
  for (int i = 0; i < 8; i++) {
    digitalWrite(3, digital_S0[i]);
    digitalWrite(2, digital_S1[i]);
    digitalWrite(5, digital_S2[i]);
    for (int j = 0; j < 8; j++) {
      digitalWrite(8, hori_digital_S0[j]);
      digitalWrite(7, hori_digital_S1[j]);
      digitalWrite(10, hori_digital_S2[j]);
      vol = analogRead(A0);
      int out = (vol >> 2);
      Serial.write(out);
    }
  }
}

void scan() {
  uint8_t x = 0, y = 0;
  Serial.write(0xff);
  for (x = 0; x < 8; x++) {
    set_mux_reg(x, reg_x);
    for (y = 0; y < 8; y++) {
      set_mux_reg_high(y, reg_y_high);
      sample(vol);
      Serial.write(vol);
    }
  }
}
