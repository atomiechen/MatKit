bool digital_S0[] = {0, 1, 0, 1};
bool digital_S1[] = {0, 0, 1, 1};
int vol = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 2; i <= 11; i++){
    pinMode(i, OUTPUT);
  }
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.write(0xFF);
  for (int i = 0; i < 4; i++) {
    digitalWrite(3, digital_S0[i]);
    digitalWrite(2, digital_S1[i]);
    for (int j = 0; j < 4; j++) {
      digitalWrite(8, digital_S0[j]);
      digitalWrite(7, digital_S1[j]);
      vol = analogRead(A0);
      int out = (vol >> 2);
      Serial.write(out);
    }
  }
}
