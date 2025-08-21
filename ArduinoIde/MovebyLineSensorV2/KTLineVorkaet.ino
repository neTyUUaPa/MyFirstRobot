#define IN1 5  // Направление левого двигателя
#define IN2 4
#define IN3 3  // Направление правого двигателя
#define IN4 2

#define OUT1 34
#define OUT2 32
#define OUT3 30
#define OUT4 28
#define OUT5 26

void readLineSensors(int sensorValues[5]) {
  // Предполагаем, что датчики подключены к аналоговым входам и
  // черная линия дает значение ниже порога.
  sensorValues[0] = digitalRead(OUT5);
  sensorValues[1] = digitalRead(OUT4);
  sensorValues[2] = digitalRead(OUT3);
  sensorValues[3] = digitalRead(OUT2);
  sensorValues[4] = digitalRead(OUT1);
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(OUT1, INPUT);
  pinMode(OUT2, INPUT);
  pinMode(OUT3, INPUT);
  pinMode(OUT4, INPUT);
  pinMode(OUT5, INPUT);
  Serial.begin(9600);
}

int PIDControl(float error) {
    static float kp = 15.0, ki = 1.0, kd = 7.0;  // Настроенные параметры PID
    static float integral = 0, previousError = 0;
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;  // Время в секундах
    lastTime = currentTime;

    if (dt <= 0) dt = 0.001;  // Минимальный dt для избежания деления на ноль

    float derivative = (error - previousError) / dt;
    integral += error * dt;  // Учитываем dt для корректного накопления

    float output = kp * error + ki * integral + kd * derivative;
    previousError = error;

    //Serial.print("PID - P: ");
    //Serial.print(kp * error, 2);
    //Serial.print(", I: ");
    //Serial.print(ki * integral, 2);
    //Serial.print(", D: ");
    //Serial.print(kd * derivative, 2);
    //Serial.print(", Output: ");
    //Serial.println(output, 2);

    return constrain(output, -100, 100);  // Увеличили диапазон для большей коррекции
}


void loop() {
  float speedK = 0;
  float error = 0;
  int sensorValues[5] = {1, 1, 1, 1, 1};
  readLineSensors(sensorValues);
  int normalValues[5] = {1, 0, 0, 0, 1};
  int weight[5] = {-2, -1, 0, 1, 2};
  int i = 0;
  for (i = 0; i < 6; i++)
  {
    if (sensorValues[i] != normalValues[i])
    { 
      error += weight[i];
    }
  }
  
  float motorSpeed = PIDControl(error);
  Serial.println(error);

  if (error < 0){
      analogWrite(IN1, (70 - motorSpeed));  // Вперед
      digitalWrite(IN2, LOW);
      analogWrite(IN3, (70 + motorSpeed));
      digitalWrite(IN4, LOW);
    }
  if (error > 0){
      analogWrite(IN1, (70 + motorSpeed));  // Вперед
      digitalWrite(IN2, LOW);
      analogWrite(IN3, (70 - motorSpeed));
      digitalWrite(IN4, LOW);
    }
    else{
      analogWrite(IN1, 70);  // Вперед
      digitalWrite(IN2, LOW);
      analogWrite(IN3, 70 );
      digitalWrite(IN4, LOW);
    }
}
