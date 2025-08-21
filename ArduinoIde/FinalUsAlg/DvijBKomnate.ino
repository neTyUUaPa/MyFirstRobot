#include <Wire.h> // библиотека для управления устройствами по I2C 
#include <math.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <MPU6050.h>
MPU6050 mpu;

// Константы
const float WHEEL_RADIUS = 0.0325;  // Радиус колеса (м)
const float WHEEL_BASE = 0.21;      // Расстояние между колесами (м)
const float MOTOR_RPM = 200;       // Обороты мотора (об/мин)
const int PWM_MAX = 255;            // Максимальное значение PWM
float TARGET_DISTANCE = 0.200;  // Середина коридора (750 мм / 2)

#define IN1 5  // Направление левого двигателя
#define IN2 4
#define IN3 3  // Направление правого двигателя
#define IN4 2


// УЗ-датчики для коридора
#define LEFT_TRIG_PIN 51   // Пин триггера для левого УЗ-датчика
#define LEFT_ECHO_PIN 49   // Пин эхо для левого УЗ-датчика
#define RIGHT_TRIG_PIN 41  // Пин триггера для правого УЗ-датчика
#define RIGHT_ECHO_PIN 39  // Пин эхо для правого УЗ-датчика
#define FRONT_TRIG_PIN 22  // Пин триггера
#define FRONT_ECHO_PIN 24 // Пин эхо

#define SOUND_SPEED 343.0


int robot_x = 2;
int robot_y = 4;
float robot_angle = 0.0;

String CurrentDir = "Up";

bool IsMoving = false;
bool IsTurned = false;
bool IsRotating = false;
float angleZ = 0; // Угол поворота по оси Z (из гироскопа)
float gzOffset = 0; // Смещение гироскопа для калибровки
unsigned long lastTime;



enum Direction {
    LEFT = 0,
    UP = 1,
    RIGHT = 2,
    DOWN = 3
};

String get_new_direction(int current_pos[2], int next_pos[2]){
    //"""Определяет новое направление движения"""
    int dx = next_pos[0] - current_pos[0];
    int dy = next_pos[1] - current_pos[1];
    Serial.print(current_pos[0]);
    Serial.print(",");
    Serial.print(current_pos[1]);
    Serial.println();
    Serial.print(next_pos[0]);
    Serial.print(",");
    Serial.print(next_pos[1]);
    Serial.println();
    Serial.print(dx);
    Serial.print(",");
    Serial.print(dy);
    Serial.println();

    if (dx > 0)
    {
      return "Right";
    }
    if (dx < 0)
    {
      return "Left";
    } 
    if (dy < 0)
    {
      return "Up";
    } 
    if (dy > 0)
    {
      return "Down";
    }
    return ::CurrentDir;  // Если остались на месте (ошибка)
}

bool needs_turning(int current_pos[2],int next_pos[2], String current_dir){
    //Проверяет, нужно ли менять направление движения"""
    String new_dir = get_new_direction(current_pos, next_pos);
    return new_dir != current_dir;  // Если направление изменилось — нужен поворот
}



// Функция для преобразования строки в Direction (для входных данных)
Direction stringToDirection(const char* value) {
    if (strcmp(value, "Left") == 0) return LEFT;
    if (strcmp(value, "Up") == 0) return UP;
    if (strcmp(value, "Right") == 0) return RIGHT;
    if (strcmp(value, "Down") == 0) return DOWN;
    return LEFT;  // Значение по умолчанию (можно настроить под нужды)
}

// Функция для преобразования Direction в строку (для отладки)
const char* directionToString(Direction dir) {
    switch (dir) {
        case LEFT: return "Left";
        case UP: return "Up";
        case RIGHT: return "Right";
        case DOWN: return "Down";
        default: return "Left";
    }
}

// Функция для получения индекса (теперь это просто значение enum)
int getIndex(const char* value) {
    Direction dir = stringToDirection(value);
    Serial.print("Ищем направление: ");
    Serial.println(directionToString(dir));
    return static_cast<int>(dir);  // Возвращаем индекс (0, 1, 2, 3)
}

// Функция для определения направления поворота (используем массив вместо map)
String get_turn_direction(String current_dir, String new_dir){
    //"Определяет, нужно ли повернуть налево или направо"""
    int a = getIndex(current_dir.c_str());
    int b = getIndex(new_dir.c_str());
    Serial.println(current_dir);
    Serial.println(new_dir);
    Serial.println(a);
    Serial.println(b);
    if ((a - b == -1) || (a - b == 3)){
      return "RIGHT";
    }
    if ((a - b == 1) || (a - b == -3)){
      return "LEFT";
    }
    if (abs(a - b) == 2){
      return "180";
    }
    return "PovorotNeTuda";
}

String update_movement(int path[][2], String current_dir){
    int current_pos[2] = {robot_x, robot_y};
    int next_pos[2] = {path[0][0],path[0][1]};
    Serial.println();

    if (needs_turning(current_pos, next_pos, current_dir))
    {
        String new_dir = get_new_direction(current_pos, next_pos);
        String turn = get_turn_direction(current_dir, new_dir);
        return new_dir + "," + turn;  // Возвращаем новое направление и поворот
    }

    return current_dir +",None"; // Если не поворачиваем, остаёмся в том же направлении
}

int PIDControl(float error) {
    static float kp = 50.0, ki = 0.5, kd = 10.0;  // Настроенные параметры PID
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



// Карта дома
const byte gameMap[5][4] = {
    {'@','@', 3,'@'},  // Y = 0 (строка 0)
    { 0,  0,  0, 2},   // Y = 1 (строка 1)
    { 4, '@', 0,'@'},   // Y = 2 (строка 2)
    {'@','@', 0, 1},   // Y = 3 (строка 3)
    {'@','@', 5,'@'}   // Y = 4
};


// Структура для узла пути
struct Node {
    int x, y;        // Координаты (X, Y)
    int parentX, parentY;  // Родительские координаты для построения пути
};

// Глобальный путь для поворотов
int path[16][2];  // Массив для пути (X, Y)
int pathLength = 0;  // Длина пути
int pathIndex = 0;  // Текущий индекс пути (начинаем с конца)

// Поиск пути BFS
void findPath(int startX, int startY, int targetX, int targetY, int path[][2], int& pathLength) {
    bool visited[5][4] = {false};  // Массив посещений
    int parentX[5][4] = {-1};      // Родительские X
    int parentY[5][4] = {-1};      // Родительские Y

    // Очередь для BFS (используем статический массив, чтобы не тратить динамическую память)
    Node queue[16];  // Максимум 16 узлов (4x4)
    int front = 0, rear = 0;

    // Добавляем стартовую точку в очередь
    queue[rear++] = {startX, startY, -1, -1};
    visited[startY][startX] = true;

    // Направления движения (вверх, вниз, лево, право)
    int dx[] = {0, 0, -1, 1};  // X-координаты (лево, право, вверх, вниз)
    int dy[] = {-1, 1, 0, 0};  // Y-координаты

    while (front < rear) {
        // Извлекаем текущий узел из очереди
        Node current = queue[front++];
        int currX = current.x, currY = current.y;

        // Если достигли цели
        if (currX == targetX && currY == targetY) {
            // Построим путь
            pathLength = 0;
            int x = currX, y = currY;
            while (x != -1 && y != -1) {
                path[pathLength][0] = x;
                path[pathLength][1] = y;
                pathLength++;
                int nextX = parentX[y][x], nextY = parentY[y][x];
                x = nextX; y = nextY;
            }

            // Удаляем все вхождения точек (2, 2) и (0, 0) из пути
            int newPathLength = 0;
            for (int i = 0; i < pathLength; i++) {
                if (!((path[i][0] == 2 && path[i][1] == 2) ||  // Удаляем (2, 2)
                      (path[i][0] == 0 && path[i][1] == 0) || (path[i][0] == robot_x && path[i][1] == robot_y) || (path[i][0] == 1 && path[i][1] == 1))) {  // Удаляем (0, 0)
                    path[newPathLength][0] = path[i][0];
                    path[newPathLength][1] = path[i][1];
                    newPathLength++;
                }
            }
            pathLength = newPathLength;  // Обновляем длину пути

            // Разворачиваем путь (от начала к концу)
            int tempPath[16][2];  // Временный массив для инверсии
            for (int i = 0; i < pathLength; i++) {
                tempPath[i][0] = path[pathLength - 1 - i][0];
                tempPath[i][1] = path[pathLength - 1 - i][1];
            }
            // Копируем развернутый путь обратно в path
            for (int i = 0; i < pathLength; i++) {
                path[i][0] = tempPath[i][0];
                path[i][1] = tempPath[i][1];
            }

            return;
        }

        // Проверяем соседние клетки
        for (int i = 0; i < 4; i++) {
            int newX = currX + dx[i];
            int newY = currY + dy[i];

            // Проверяем границы и стены
            if (newX < 0 || newX >= 4 || newY < 0 || newY >= 5 ||
                gameMap[newY][newX] == '@') {
                continue;
            }

            // Если клетка не посещена, добавляем в очередь
            if (!visited[newY][newX]) {
                visited[newY][newX] = true;
                parentX[newY][newX] = currX;
                parentY[newY][newX] = currY;
                queue[rear++] = {newX, newY, currX, currY};
            }
        }
    }

    // Если путь не найден
    pathLength = 0;
    Serial.println("Путь не найден!");
}

// Измерение расстояния с УЗ-датчика
float ultrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.000001 * SOUND_SPEED) / 2.0 * 100.0;  // Расстояние в см
    delay(10);
    return distance / 100.0;  // Конвертируем в метры
}

// Улучшенная проверка стабильности УЗ-датчика
bool isStableDistance(float target_distance, int samples) {
    float measurements[10];  // Буфер для хранения измерений (увеличиваем до 10 для надежности)
    const float STABILITY_THRESHOLD = 0.02;  // Порог стабильности ±2 см (можно настроить)

    // Считываем заданное количество измерений
    for (int i = 0; i < samples; i++) {
        // Считываем среднее значение с левого и правого датчиков
        float leftDist = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
        float rightDist = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
        measurements[i] = (leftDist + rightDist) / 2.0;  // Среднее значение

        // Выводим каждое измерение для отладки
        Serial.print("Измерение ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(measurements[i], 2);
        Serial.println(" м");

        delay(10);  // Задержка между измерениями
    }

    // Сортируем массив измерений для нахождения медианы
    for (int i = 0; i < samples - 1; i++) {
        for (int j = i + 1; j < samples; j++) {
            if (measurements[i] > measurements[j]) {
                float temp = measurements[i];
                measurements[i] = measurements[j];
                measurements[j] = temp;
            }
        }
    }
// Берем медиану (для нечетного числа samples — среднее центральных значений)
    float median;
    if (samples % 2 == 0) {
        median = (measurements[samples / 2 - 1] + measurements[samples / 2]) / 2.0;
    } 
    else {
        median = measurements[samples / 2];
    }

    // Выводим медиану для отладки
    Serial.print("Медиана расстояния: ");
    Serial.print(median, 2);
    Serial.println(" м");

    // Проверяем стабильность: разница между целевым расстоянием и медианой должна быть меньше порога
    bool is_stable = fabs(target_distance - median) < STABILITY_THRESHOLD;

    Serial.print("Стабильность (");
    Serial.print(target_distance, 2);
    Serial.print(" м vs ");
    Serial.print(median, 2);
    Serial.print(" м): ");
    Serial.println(is_stable ? "Стабильно" : "Не стабильно");

    return is_stable;
}

void moveForward(){
    analogWrite(IN1, 100);
    digitalWrite(IN2, LOW);
    analogWrite(IN3, 100);
    digitalWrite(IN4, LOW); 
}

void moveForward(float distance){
    analogWrite(IN1, 100);
    digitalWrite(IN2, LOW);
    analogWrite(IN3, 100);
    digitalWrite(IN4, LOW); 
    delay(300);
}

void moveBack(){
    analogWrite(IN1, 155 + 8);
    digitalWrite(IN2, HIGH);
    analogWrite(IN3, 155);
    digitalWrite(IN4, HIGH); 
}

void StopMoving(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void RotateRobot(String direction, float angle){ // в первую очередь надо переделать, чтобы я лишь указывал ему угол(в промежутке от -180до180), а робот сам считал насколько о сейчас не оч
  float startAngle = getAngleZ();
  if(direction == "LEFT"){
    startAngle = getAngleZ();
  }
  if(direction == "RIGHT" ||direction == "180"){
    startAngle = getAngleZ();
  }
  float currentAngle = getAngleZ();
  Serial.println(currentAngle);
  float normalizedAngle = fmod(startAngle + 360, 360);
  if(normalizedAngle > 180.0){
    normalizedAngle = 0 - (360 - normalizedAngle);
  }
  float normalizedRobotAngle = fmod(robot_angle + 360, 360);
  if(normalizedRobotAngle>180.0){
    if (normalizedRobotAngle == 180.0){
      normalizedRobotAngle = 0.0;
    }
    else{
      normalizedRobotAngle = 0 -(360.0 - normalizedRobotAngle);
    }
  }
  Serial.println(normalizedRobotAngle);
  Serial.println(normalizedAngle);
  float Aangle = angle;
  if(direction == "RIGHT" || direction == "180"){
    if(normalizedAngle >= normalizedRobotAngle && normalizedAngle > 0){
      angle = angle + abs(normalizedRobotAngle-normalizedAngle);
    }
    if(normalizedAngle >= normalizedRobotAngle && normalizedAngle < 0){
      angle = angle - abs(normalizedRobotAngle-normalizedAngle);
    }
    if(normalizedAngle < normalizedRobotAngle && normalizedAngle < 0){
      angle = angle - abs(normalizedRobotAngle-normalizedAngle);
    }
    if(normalizedAngle < normalizedRobotAngle && normalizedAngle > 0){
      angle = angle + abs(normalizedRobotAngle-normalizedAngle);
    }
  }
  if(direction == "LEFT"){
    if(normalizedAngle >= normalizedRobotAngle && normalizedAngle < 0){
      angle = angle + abs(normalizedRobotAngle-normalizedAngle);
    }
    if(normalizedAngle >= normalizedRobotAngle && normalizedAngle > 0){
      angle = angle - abs(normalizedRobotAngle-normalizedAngle);
    }
    if(normalizedAngle < normalizedRobotAngle && normalizedAngle > 0){
      angle = angle + abs(normalizedRobotAngle-normalizedAngle);
    }
    if(normalizedAngle < normalizedRobotAngle && normalizedAngle < 0){
      angle = angle + abs(normalizedRobotAngle-normalizedAngle);
    }
  }
  Serial.println(angle);
  
  if (direction == "RIGHT" or direction == "180"){
      Serial.println("TurningRight");
      while (abs(startAngle - currentAngle) <= angle){
          currentAngle = getAngleZ() - 7.4;
          if(abs(startAngle - currentAngle) >= angle){
            StopMoving();
            break;
          }
          analogWrite(IN1, 185);
          digitalWrite(IN2, HIGH);
          analogWrite(IN3, 70);
          digitalWrite(IN4, LOW);   
      }
      robot_angle -= Aangle;
  }
  if (direction == "LEFT"){
    Serial.print("TurningLeft");
      while (abs(startAngle - currentAngle) <= angle){
          currentAngle = getAngleZ() + 8.3;
          if(abs(startAngle - currentAngle) >= angle){
            StopMoving();
            break;
          }
          analogWrite(IN1, 70);
          digitalWrite(IN2, LOW);
          analogWrite(IN3, 185);
          digitalWrite(IN4, HIGH);   
      }
      robot_angle += Aangle;
  }
  Serial.println(startAngle);
  Serial.println(currentAngle);
  float RealAngle = getAngleZ();
  Serial.println(currentAngle);
  StopMoving();
}



bool IsInRoom = false;

void exitRoom()
{
 enterRoom();
 moveForward(0.1);
}

bool IsLeaved = false;

void enterRoom() {
    StopMoving();
    float leftDistance = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float rightDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    while(leftDistance > 0.3 && rightDistance > 0.3){
      leftDistance = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
      rightDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
      moveForward();
      //updatePosition(motorSpeed, motorSpeed);
    }
    //IsInRoom = true;
    Serial.print("Заезжаю в комнату ");
    Serial.println();
    IsLeaved = true;
    StopMoving();
    // Логика для комнаты (например, findAndGrabCan для комнаты 1/2)
}

bool IsDoorChecked = false;
// === Движение робота по пути ===
void move_to_target(int path[][2]) {
    
    float leftDistance = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    float rightDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    
    
    leftDistance = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    rightDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    if (rightDistance < 0.5 && leftDistance < 0.5){
            IsDoorChecked = false;
    }

    //if (pathLength <=2){
    //  float frontDistance = ultrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    //  if(isStableDistance(frontDistance, 5)){
    //    StopMoving();
    //    IsInRoom = true;
    //  }
    //}

    if (pathLength <=1){
      StopMoving();
      IsInRoom = true;
      robot_x = path[0][0];
      robot_y = path[0][1];
    }

    leftDistance = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
    rightDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
    Serial.println("Left:");
    Serial.print(leftDistance);
    Serial.print("   Right:");
    Serial.print(rightDistance);
    Serial.println();

    if (pathLength > 0 && (leftDistance >= 0.6 || rightDistance >= 0.6) && IsDoorChecked == false && IsInRoom == false) {
        IsDoorChecked = true;
        robot_x = path[0][0];
        robot_y = path[0][1];
        Serial.println("robotx:");
        Serial.print(path[0][0]);
        Serial.print("   roboty:");
        Serial.print(path[0][1]);
        Serial.println();

        for (int i = 0; i < pathLength - 1; i++) {
            path[i][0] = path[i + 1][0];
            path[i][1] = path[i + 1][1];
        }
        pathLength--;
        if (pathLength > 0) {
            path[pathLength][0] = -1;  // Очищаем последнюю точку
            path[pathLength][1] = -1;
        }
        String result = update_movement(path, CurrentDir);
        int commaIndex = result.indexOf(',');
        String dir = result.substring(0, commaIndex);
        String turn = result.substring(commaIndex + 1);
        Serial.println(dir);
        Serial.println(turn);
        if (turn != "None") {
            StopMoving();
            if (turn != "180") {
                moveForward(0.1);
                RotateRobot(turn, 90);  // Поворот на 90 градусов
                Serial.print("Поворачиваюсь на ");
                Serial.println(turn);
                enterRoom();
            } else {
                //robot_x = path[0][0];
                //robot_y = path[0][1];
                RotateRobot(turn, 180);  // Поворот на 180 градусов
                Serial.print("Поворачиваюсь на ");
                Serial.println(turn);
                moveForward();
            }

        } else {
            //robot_x = path[0][0];
            //robot_y = path[0][1];
            moveForward(0.1);
            Serial.print("Двигаюсь к (");
            Serial.print(path[0][0]);
            Serial.print(", ");
            Serial.print(path[0][1]);
            Serial.print(") в направлении ");
            Serial.println(dir);  // dir не определен в C++, см. ниже
        }
        ::CurrentDir = dir; 
    }
    float leftError = leftDistance - TARGET_DISTANCE;
    float rightError = rightDistance - TARGET_DISTANCE;
    float error = min(leftError, rightError);  // Средняя ошибка для центрирования
    float motorSpeed = PIDControl(error);
    if (leftDistance >= rightDistance){
      analogWrite(IN1, (100 - motorSpeed));  // Вперед
      digitalWrite(IN2, LOW);
      analogWrite(IN3, (100 + motorSpeed));
      digitalWrite(IN4, LOW);
    }
    else{
      analogWrite(IN1, (100 + motorSpeed));  // Вперед
      digitalWrite(IN2, LOW);
      analogWrite(IN3, (100 - motorSpeed));
      digitalWrite(IN4, LOW);
    }
}

float getAngelFromRasp(){
    if (Serial1.available() > 0) {
      String received = Serial1.readStringUntil('\n');
      received.trim();
      Serial.print("Получено от Raspberry Pi: ");
      Serial.println(received);
      return received
  }
  return 179.11
}

void found_barrel(){
  float angle = getAngleFromRasp();
  float leftDistance = ultrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  float rightDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  int dir = 0;
  if (leftDistance > rightDistance){
    dir = 1;
  }
  while (angle == 179.11){
    if (dir == 1){
      RotateRobot("LEFT", 30.0);
    }
    else{
      RotateRobot("RIGHT", 30.0);
    }
    angle = getAngleFromRasp();
  }
  if(angle > 0){
    RotateRobot("RIGHT", abs(angle));
  }
  else{
    RotateRobot("LEFT", abs(angle))
  }
  // Добавить проверку на то что угол банки щас 0град
}
void moveForwardPid(float Speed, int turn){
  
}

void TakeBarrel(){
  //взять код у Гриши
}

float findPathBack(){
  return (atan(min(room_robot_x, room_robot_y)/(max(room_robot_y, room_robot_x)*180.0))/M_PI) // тут надо понимать, что угол этот не учитывает, 
            //куда сейчас смотрит робот, поэтому потом надо еще будет учитывать это
}

uint16_t tmr;
void updateLocation(int Speed){
  float angle = getAngleZ();
  float speed = getSpeedEnc(); //  создать функцию, которая возвращает скорость по энкодерам наверно???
                               // Также если уж прям надо будет, надо делать x и y, каждого колеса и потом их усреднять
  if (millis() - tmr >= 100){
    room_robot_x = abs(room_robot_x + cos(angle)*(0.1*speed))
    room_robot_y = abs(room_robot_y + sin(angle)*(0.1*speed))
  }
}

void move_in_room(){
  float startAngle = getAngleZ() - 180.0; // его нужно нормализовать только
  float door_X = 0;
  float door_Y = 0;
  found_barrel();
  float barrelAngle = getAngleZ();
  float frontDistance = ultrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  while true{
    moveForwardPid(motorSpeed); // Тут должен быть PID + его надо новый писать, потому что I и D у каждого свой
    updateLocation(motorSpeed);
    if frontDistance <= 0.03{
      StopMoving();
      break;
    }
  }
  TakeBarrel();
  findPathBack();
  while true{
    moveBack();
    updateLocation();
    if (room_robot_x <= door_X && room_robot_y <= door_Y){
      StopMoving();
      break;
    }
  }
  RotateRobot();
}


// Инициализация датчика и калибровка
void setupMPU6050() {
  Wire.begin();
  mpu.initialize();
}


float getAngleZ() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz); // Получаем данные гироскопа

  // Время между измерениями в секундах
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Преобразование в °/с с учетом калибровки (±250°/с)
  float gyroZ = (gz - gzOffset) / 131.0;

  // Интеграция для вычисления угла
  angleZ += gyroZ * deltaTime;

  return angleZ; // Возвращаем текущий угол
}

// Вывод пути
void printPath(int path[][2], int pathLength) {
    Serial.print("Путь: ");
    for (int i = pathLength - 1; i >= 0; i--) {
        Serial.print("(");
        Serial.print(path[i][0]);
        Serial.print(", ");
        Serial.print(path[i][1]);
        Serial.print(") -> ");
    }
    Serial.println("Конец");
}

void getPath(int room){
  switch (room){
    case 1:
      findPath(robot_x, robot_y, 3, 3, path, pathLength);
    case 2:
      findPath(robot_x, robot_y, 3, 1, path, pathLength);
    case 3:
      findPath(robot_x, robot_y, 2, 0, path, pathLength);
    case 4:
      findPath(robot_x, robot_y, 0, 2, path, pathLength);
    case 5:  
      findPath(robot_x, robot_y, 2, 4, path, pathLength);
  }
}


void setup() {
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  setupMPU6050();
  getPath(4);
  float leftDistance = ultrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  Serial.println(leftDistance);
  TARGET_DISTANCE = 0.2;
  printPath(path, pathLength);
}
bool isCreate = false;
bool IsRotated = false;

void loop() 
{ 
  Serial.println(getAngleZ());
  if(!IsInRoom){
    move_to_target(path);
  }
  if(IsInRoom)
  {
    if (!IsRotated){
      moveForward(0.1);
      RotateRobot("RIGHT", 180);
      CurrentDir = "Up";
      IsRotated = true;
      findPath(0, 2, 2, 4, path, pathLength);
    }
    if(IsRotated){
      exitRoom();
      moveForward(0.1);
      IsInRoom = false;
  }

//RotateRobot("RIGHT", 90);
//delay(500);
  
   }
}