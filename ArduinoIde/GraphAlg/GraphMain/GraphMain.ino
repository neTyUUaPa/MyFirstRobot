#include <Arduino.h>

#define ENCODER_A_1 3  // Первый мотор - сигнал A
#define ENCODER_B_1 2  // Первый мотор - сигнал B
#define ENCODER_A_2 19  // Второй мотор - сигнал A
#define ENCODER_B_2 18 // Второй мотор - сигнал B

#define LMP 6
#define LM 7
#define RMP 4
#define RM 5

#define MAX_NODES 18  // Максимальное количество вершин в графе (по твоему графу)
#define MAX_QUEUE 40  // Максимальный размер очереди

#define pi 3.14159265358979323846

volatile long encoderCount1 = 0;  // Счетчик для первого мотора
volatile long encoderCount2 = 0;  // Счетчик для второго мотора

int speedRide = 0;
int speedStop = 255;

float SpeedK = 245.0/255.0;

float oneSm = 121.44;

int CurAngle = 90;

int lastLeftValueEncoder = 0;
int lastRightValueEncoder = 0;

bool leftFlag = false;
bool rightFlag = false;

// Очередь на основе массива (ручная реализация)
struct Queue {
    int items[MAX_QUEUE];
    int front, rear;

    Queue() { front = 0; rear = 0; }

    bool isEmpty() { return front == rear; }

    void push(int value) {
        if (rear < MAX_QUEUE) {
            items[rear++] = value;
        }
    }

    int pop() {
        if (!isEmpty()) {
            return items[front++];
        }
        return -1;  // Если очередь пуста
    }

    int peek() {
        if (!isEmpty()) {
            return items[front];
        }
        return -1;
    }
};

// Граф как список смежности (матрица смежности с массивами для distance и angle)
float graph[MAX_NODES][MAX_NODES][2];  // Матрица смежности, где [][0] - distance, [][1] - angle
bool visited[MAX_NODES] = {false};
int parents[MAX_NODES];
int path[MAX_NODES];

// Инициализация графа с несколькими вершинами
void initializeGraph() {
    // Пример твоего графа (связи для нескольких вершин)

    // C1 <-> C2
    graph[0][1][0] = 48.5;  // Расстояние
    graph[0][1][1] = 90.0;  // Угол
    graph[1][0][0] = 48.5;  // Симметричная связь
    graph[1][0][1] = 270.0; // Симметричный угол

    // C2 <-> R11
    graph[1][4][0] = 147.5;
    graph[1][4][1] = 0.0;
    graph[4][1][0] = 147.5;
    graph[4][1][1] = 180.0;

    // C2 <-> C3
    graph[1][2][0] = 126.5;
    graph[1][2][1] = 90.0;
    graph[2][1][0] = 126.5;
    graph[2][1][1] = 270.0;

    // R11 <-> R12
    graph[4][5][0] = 147.5;
    graph[4][5][1] = 0.0;
    graph[5][4][0] = 147.5;
    graph[5][4][1] = 180.0;

    // C3 <-> R21
    graph[2][6][0] = 160.0;
    graph[2][6][1] = 0.0;
    graph[6][2][0] = 160.0;
    graph[6][2][1] = 180.0;

    // R21 <-> R22
    graph[6][7][0] = 160.0;
    graph[6][7][1] = 0.0;
    graph[7][6][0] = 160.0;
    graph[7][6][1] = 180.0;

    // R22 <-> R23
    graph[7][8][0] = 160.0;
    graph[7][8][1] = 0.0;
    graph[8][7][0] = 160.0;
    graph[8][7][1] = 180.0;

    // R22 <-> R23
    graph[8][9][0] = 160.0;
    graph[8][9][1] = 0.0;
    graph[9][8][0] = 160.0;
    graph[9][8][1] = 180.0;

    // R23 <-> R24
    graph[9][10][0] = 160.0;
    graph[9][10][1] = 0.0;
    graph[10][9][0] = 160.0;
    graph[10][9][1] = 180.0;

    // C3 <-> C4
    graph[2][3][0] = 150.0;
    graph[2][3][1] = 180.0;
    graph[3][2][0] = 150.0;
    graph[3][2][1] = 0.0;

    // C4 <-> R41
    graph[3][11][0] = 158.5;
    graph[3][11][1] = 270.0;
    graph[11][3][0] = 158.5;
    graph[11][3][1] = 90.0;

    // R41 <-> R42
    graph[11][12][0] = 112.0;
    graph[11][12][1] = 180.0;
    graph[12][11][0] = 112.0;
    graph[12][11][1] = 0.0;

    // R42 <-> R43
    graph[12][13][0] = 47.0;
    graph[12][13][1] = 270.0;
    graph[13][12][0] = 6.0;
    graph[13][12][1] = 90.0;

    // R43 <-> R44
    graph[13][14][0] = 47.5;
    graph[13][14][1] = 180.0;
    graph[14][13][0] = 47.5;
    graph[14][13][1] = 0.0;

    // R44 <-> R45
    graph[14][15][0] = 65.5;
    graph[14][15][1] = 270.0;
    graph[15][14][0] = 65.5;
    graph[15][14][1] = 90.0;
    
    // R45 <-> R46
    graph[15][16][0] = 10.0;
    graph[15][16][1] = 270.0;
    graph[16][15][0] = 10.0;
    graph[16][15][1] = 90.0;

    // И так далее для всех связей...
}

// BFS для поиска пути
void bfs(int start, int target) {
    Queue q;
    for (int i = 0; i < MAX_NODES; i++) {
        visited[i] = false;
        parents[i] = -1;
    }

    q.push(start);
    visited[start] = true;

    while (!q.isEmpty()) {
        int current = q.pop();
        Serial.print("Visiting: ");
        Serial.println(current);

        if (current == target) break;

        for (int i = 0; i < MAX_NODES; i++) {
            if (graph[current][i][0] != 0 && !visited[i]) {  // Если существует связь
                visited[i] = true;
                parents[i] = current;
                q.push(i);
            }
        }
    }

    // Восстанавливаем путь
    Serial.print("Path: ");
    int node = target;
    path[MAX_NODES];
    int pathIndex = 0;

    while (node != -1) {
        path[pathIndex++] = node;
        node = parents[node];
    }

    // Печатаем путь в правильном порядке
    for (int i = pathIndex - 1; i >= 0; i--) {
        Serial.print(path[i]);
        Serial.print(" ");
    }
    
}

int CheckSign(int alpha){

  if (abs(alpha) == alpha){
    return 1;
  } else {
    return -1;
  }
}

void Ride(int speed1, int speed2 = 256, int dir1 = 1, int dir2 = 1){

  if (speed2 == 256){
    speed2 = speed1;
  }
  if (dir1 == dir2){
      if (dir1 == 1){
        speed1 = 255 - (255-speed2)*SpeedK;
        //Serial.println("Tut");
      }
  }
  //Serial.println(speed1);
  //Serial.println(speed2);

  digitalWrite(LM, dir1);
  analogWrite(LMP, speed1);
  digitalWrite(RM, dir2);
  analogWrite(RMP, speed2);
}

void RideForward(float dist){

  float distance = dist * oneSm;
  long* encodersData = getEncodersData();
  Serial.println(distance);
  while (abs(encodersData[0]) < distance || abs(encodersData[1]) < distance){
    Ride(0, 0, 1, 1);
    encodersData = getEncodersData();
    //Serial.println(encodersData[0]);
  } 
  
  Ride(255, 255, 1, 1);
  encoderCount1 = 0; //надо будет убрать, если будем пид делать, там проблема перед поворотом возникнет
  encoderCount2 = 0;
  
}

float TurnDistance(int alpha){
  int radius = 11;
  float length = ((2 * radius * pi) / 360) * alpha;
  return length;
}

void TurnAround(int tarAngle){

  int ang;
  if (tarAngle != CurAngle){
    ang = (tarAngle - CurAngle) % 360;
    if (ang > 180){
      ang -= 360;
    }
  } else{
    ang = 0;
  }

  Serial.println(ang);
  Serial.println(TurnDistance(ang));

  if (CheckSign(ang) == -1){
    if (leftFlag == false && rightFlag == false){
      long* encodersData = getEncodersData();
      lastLeftValueEncoder = encodersData[0];
      lastRightValueEncoder = encodersData[1];
      leftFlag = true;
      rightFlag = true;
    }
    if ((getEncodersData()[0] > lastLeftValueEncoder - (TurnDistance(ang) * oneSm)) && (getEncodersData()[1] < lastRightValueEncoder + (TurnDistance(ang) * oneSm))){
      Serial.print(getEncodersData()[0]);
      Serial.print("...");
      Serial.print(lastLeftValueEncoder);
      Serial.print("...");
      Serial.println((TurnDistance(ang) * oneSm));
      Ride(0, 255, 1, 0);
      Serial.println("lesha");
    } else {
      Serial.println("LOH");
      Ride(255, 0);
      CurAngle = tarAngle;
      
    }
    Serial.println("a");
  } 
  else{
    if (((getEncodersData()[0] < lastLeftValueEncoder + (TurnDistance(ang) * oneSm)) && (getEncodersData()[1] > lastRightValueEncoder - (TurnDistance(ang) * oneSm)))){
      Ride(255, 0, 0, 1);
    } 
    else {
      Ride(0, 255);
      CurAngle = tarAngle;
    }
    Serial.println("b");
  }

  leftFlag = false;
  rightFlag = false;
  encoderCount1 = 0;
  encoderCount2 = 0;
}

void MovePath(int target){
  bfs(0, target);
  Serial.print("Path: ");
    int node = target;
    path[MAX_NODES];
    int pathIndex = 0;

    while (node != -1) {
        path[pathIndex++] = node;
        node = parents[node];
    }

    // Печатаем путь в правильном порядке
    for (int i = pathIndex - 1; i >= 0; i--) {
        //Serial.println(path[i-1]);
        //Serial.println(path[i]);
        Serial.print("Угол");
        Serial.println(graph[path[i]][path[i-1]][1]);
        TurnAround(graph[path[i]][path[i-1]][1]);
        RideForward(graph[path[i]][path[i-1]][0]);
        Serial.print("Текущая точкаа");
        Serial.println(path[i-1]);
        
        if (i == 1){
          break;
        }
    }
    Serial.println();
}

void readEncoder1() {
    if (digitalRead(ENCODER_A_1) == digitalRead(ENCODER_B_1)) {
        encoderCount1++;
    } else {
        encoderCount1--;
    }
}

void readEncoder2() {
    if (digitalRead(ENCODER_A_2) == digitalRead(ENCODER_B_2)) {
        encoderCount2++;
    } else {
        encoderCount2--;
    }
}

long* getEncodersData() {
    static long encoderData[2];  // Статический массив, чтобы не терялись данные при выходе из функции
    encoderData[0] = encoderCount1;
    encoderData[1] = encoderCount2;
    return encoderData;
}

void setup() {
    Serial.begin(9600);

    pinMode(ENCODER_A_1, INPUT_PULLUP);
    pinMode(ENCODER_B_1, INPUT_PULLUP);
    pinMode(ENCODER_A_2, INPUT_PULLUP);
    pinMode(ENCODER_B_2, INPUT_PULLUP);

    pinMode(LMP, OUTPUT);
    pinMode(LM, OUTPUT);
    pinMode(RMP, OUTPUT);
    pinMode(RM, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), readEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), readEncoder2, CHANGE);

    // Инициализируем граф
    initializeGraph();

    // Запускаем BFS, например, от C1 (0) до R25 (10)
    MovePath(5);
    //Ride(0);
}

void loop() {
  
}