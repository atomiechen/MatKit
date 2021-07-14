bool digital_S0[] = {0, 1, 0, 1, 0, 1, 0, 1};
bool digital_S1[] = {0, 0, 1, 1, 0, 0, 1, 1};
bool digital_S2[] = {0, 0, 0, 0, 1, 1, 1, 1};

bool hori_digital_S0[] = {0, 1, 0, 1, 1, 0, 1, 0};
bool hori_digital_S1[] = {0, 0, 1, 1, 1, 1, 0, 0};
bool hori_digital_S2[] = {0, 0, 0, 0, 1, 1, 1, 1};

int vol = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i = 2; i <= 11; i++){
    pinMode(i, OUTPUT);
  }
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
  digitalWrite(9, LOW);
  digitalWrite(11, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
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
