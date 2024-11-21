int motorPin = 9; // PWMピン

void setup() {
  pinMode(motorPin, OUTPUT);
}

void loop() {
  analogWrite(motorPin, 128); // 振動モーターを半分の強さで回す（0~255）
  delay(1000); // 1秒間振動
  analogWrite(motorPin, 0); // 振動を止める
  delay(1000); // 1秒間停止
}