from controller import Robot, Camera, Lidar
import numpy as np
import cv2
import math
import heapq
import matplotlib.pyplot as plt  # 🔹 Добавляем импорт для отрисовки карты
import threading
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Инициализация сенсоров
camera = robot.getDevice("camera")
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()
center_x = width // 2  # Центр кадра по X

DSF = robot.getDevice("distance sensor")
DSF.enable(timestep)
DSL = robot.getDevice("distance sensor(1)")
DSL.enable(timestep)
DSB = robot.getDevice("distance sensor(2)")
DSB.enable(timestep)
DSR = robot.getDevice("distance sensor(3)")
DSR.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Колеса работают в режиме скорости, не фиксированная позиция
right_motor.setPosition(float('inf'))  # Колеса работают в режиме скорости, не
left_motor.setVelocity(0.0)  # Пример действия - движение вперед
right_motor.setVelocity(0.0)

    # Подключаем серву для камеры
servo = robot.getDevice("servo")
servo.setPosition(0.0)  # Начальное положение камеры

Obj_Angle = 0. 
servo_angle = 0.0
servo_step = 0.04  # Шаг поворота
dead_zone = width * 0.05  # Зона ±5% от центра кадра
TurnVelocity = (4 * 0.033)
LenghtWheels = 0.160

map_size = 120  # 10x10 метров в сетке 100x100
grid_map = np.full((map_size, map_size), 2)  # 0 - свободно, 1 - стена
robot_x, robot_y = 0, 0
robot_angle = 0  # Угол поворота

scan_state = "searching"
IsScanning = False


count = 0
rooms_count = 0

# Путь к вашему файлу
file_path = 'C:\\Users\\Alexei\\Desktop\\УмныйДом\\matrix1.txt'

# Загружаем карту


#IsFound = False
IsTurned = False
IsStartTurn = False
IsRotating = False
IsMoving = False
frame_count = 0
warehouse_position = 24, 55
IsUpdated = False
IsStartToObject = False

prohod_list = None
objects = None

CurrentDir = "DOWN"

def load_map_from_file(file_path):
    # Читаем файл и преобразуем его в numpy массив
    return np.loadtxt(file_path)



def RotateRobot(direction, angle):
    start_time = robot.getTime()
    global IsTurned, IsRotating, robot_angle
    Rad_Angle = angle*math.pi / 180
    RotationTime = (Rad_Angle*LenghtWheels) * 1.11/(TurnVelocity)
    print(RotationTime)
    #print(start_time)
    IsRotating = True
    if (direction == "RIGHT" or direction == "180"):
        print("TurningRight")
        while robot.step(timestep) != -1:
    # Получаем текущее время
            
            current_time = robot.getTime()
            #print(current_time)
    # Проверяем, прошло ли время для выполнения действия
            if current_time - start_time <= RotationTime:
                servo.setPosition(0.0)
        # Действие робота в пределах времени max_time
                left_motor.setVelocity(2.0)  # Пример действия - движение вперед
                right_motor.setVelocity(-2.0)
            else:
                IsTurned = True
                IsRotating = False
                left_motor.setVelocity(0.0)  # Пример действия - движение вперед
                right_motor.setVelocity(0.0)
                robot_angle -= angle
                break
                
    if (direction == "LEFT"):
        print("TurningLeft")
        while robot.step(timestep) != -1:
    # Получаем текущее время
            current_time = robot.getTime()

    # Проверяем, прошло ли время для выполнения действия
            if current_time - start_time <= RotationTime:
        # Действие робота в пределах времени max_time
                left_motor.setVelocity(-2.0)  # Пример действия - движение вперед
                right_motor.setVelocity(2.0)
                print("Turning")
            else:
                IsTurned = True            
                IsRotating = False
                left_motor.setVelocity(0.0)  # Пример действия - движение вперед
                right_motor.setVelocity(0.0)
                robot_angle += angle
                break
    IsTurned = True

def detect_red(frame):
    """Обнаружение красного цвета в кадре"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Диапазоны для красного цвета (можно настроить)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Фильтры по двум диапазонам
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    
    # Поиск контуров
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return (x + w // 2, y + h // 2)  # Центр объекта
    return None
    
def detect_green(frame):
    """Обнаружение зеленого цвета в кадре"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Диапазон для зеленого цвета (можно настроить)
    lower_green = np.array([35, 100, 100])  # Нижняя граница (H, S, V)
    upper_green = np.array([85, 255, 255])  # Верхняя граница (H, S, V)
    
    # Создание маски для зеленого цвета
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Поиск контуров
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return (x + w // 2, y + h // 2), w  # Центр объекта
    return None, None

    
def MoveForward():   
    left_motor.setVelocity(2.0)  # Пример действия - движение вперед
    right_motor.setVelocity(2.0)

def MoveForwardFor(distance):
    start_time = robot.getTime()
    MoveTime = distance / (2 * 0.033)
    while robot.step(timestep) != -1:
        current_time = robot.getTime();
        if current_time - start_time <= MoveTime:
            left_motor.setVelocity(2.0)  # Пример действия - движение вперед
            right_motor.setVelocity(2.0)
        else:
            StopMoving()
            IsMoving = False
            break

def StopMoving():
    left_motor.setVelocity(0.0)  # Пример действия - движение вперед
    right_motor.setVelocity(0.0)
         

def RotateCamera():
    global servo_angle, servo_step, count
    servo_angle += servo_step  # Двигаем камеру, пока не найдем объект
    count += 1
    if servo_angle > 1.57 or servo_angle < -1.57:
        servo_step = -servo_step  # Меняем направление сканирования
        count += 1
    servo.setPosition(servo_angle)

def CorrectingCamera(red_position):
    if red_position is None:
        return None  # Если объекта нет, выходим
    
    global servo_angle, servo_step
    obj_x = red_position[0]
    
    # Корректируем поворот камеры
    if obj_x < center_x - dead_zone:
        #servo_angle -= servo_step
        None
                
    elif obj_x > center_x + dead_zone:
        #servo_angle += servo_step
        None
                
    else:
        print("Объект в центре! Останавливаем камеру.") 
        return servo_angle
        
    #servo.setPosition(servo_angle)
    return None

def CameraShow():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    frame = np.frombuffer(image, np.uint8).reshape((height, width, 4))  # Webots использует (H, W, C)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Убираем альфа-канал
    cv2.imshow("Camera View", frame)
    cv2.waitKey(1)  # Добавляем задержку, чтобы OpenCV мог обновлять окно
    return frame


def scan_room(room):
    global scan_state, objects, IsScanning, count
    red_position, Obj_Angle = None, None
    if scan_state == "searching":
        IsScanning = True
        frame = CameraShow()
        RotateCamera()
        red_position, obj_width = detect_green(frame)
        
        if red_position:
            Obj_Angle = CorrectingCamera(red_position)
            if Obj_Angle is not None:
                scan_state = "done"
                count = 0
        if count >= 157:
            count = 0
            scan_state = "failed"
    elif scan_state == "failed":
        IsScanning = False
        return None
    elif scan_state == "done":
        print("✅ Объект найден!")
        objects = [Obj_Angle]
        IsScanning = False
        return objects  # Завершаем сканирование
        
    #objects_in_rooms[room] = objects
    #return objects
    objects = None
    return objects

def pick_up_object(obj_position):
    print(f"Захват объекта в {obj_position}...")
    time.sleep(1)

def drop_object():
    print("Опускание объекта на складе...")
    time.sleep(1)

def return_to_start():
    print("Возвращение на старт...")
    move_to((67, 113))

IsRoomFound = False
def moveInCorridor():
    global leftTargetDistance, rightTargetDistance, IsRoomFound
    MoveForward()
    leftDistance = DSL.getValue()
    rightDistance = DSR.getValue()
    if (rightDistance > rightTargetDistance + 0.05) or (leftDistance > leftTargetDistance + 0.05):
        StopMoving()
        MoveForwardFor(0.4)
        if(rightDistance > rightTargetDistance + 0.05):
            RotateRobot("RIGHT", 90)
            defineTheRoom()
            IsRoomFound = True
        else:
            RotateRobot("LEFT", 90)
            defineTheRoom()
            IsRoomFound = True
            
IsInRoom = False            
def moveInDoor():
    global IsInRoom
    leftDistance = DSL.getValue()
    rightDistance = DSR.getValue()
    MoveForward()
    if(leftDistance < 0.5 or rightDistance < 0.5):
        StopMoving()
        IsInRoom = True

def defineTheRoom():
    None

IsRoomScaned = False
robotRotateBack = 0
def scanTheRoom():
    global IsRoomScaned, robotRotateBack
    prevRes = DSF.getValue()
    count = 0
    while True:
        RotateRobot("RIGHT", 2)
        curRes = DSF.getValue()
        count += 1;
        if (curRes - prevRes > 0.6):
            break
        prevRes = curRes
    IsRoomScaned = True
    robotRotateBack = count*2
    objDistance = DSF.getValue()

def moveInRoom():
    objDistance = DSF.getValue()
    if (objDistance < 0.15):
        StopMoving()
        RotateRobot("LEFT", 180)
    MoveForward()
        


IsStart = True
while robot.step(timestep) != -1:
    if (IsStart):
        leftTargetDistance = DSL.getValue()
        rightTargetDistance = DSR.getValue()
        IsStart = False
    if (not IsRoomFound and not IsInRoom):
        moveInCorridor()
    if (IsRoomFound and not IsInRoom):
        moveInDoor()
    if (IsInRoom and not IsRoomScaned):
        scanTheRoom()
    if (IsRoomScaned):
        moveInRoom()
        
#plt.show()
cv2.destroyAllWindows()
