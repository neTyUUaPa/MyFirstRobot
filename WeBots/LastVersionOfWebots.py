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
robot_x, robot_y = 67, 113
robot_angle = 180.0  # Угол поворота

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



def RotationRobot(direction, angle):
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



def compute_cost_map(grid_map, wall_penalty=5, near_wall_penalty=2, penalty_radius=2):
    """ Создаёт карту штрафов, увеличивая стоимость передвижения рядом со стенами """
    cost_map = np.ones_like(grid_map)  # Изначально все клетки = 1

    # Перебираем всю карту
    for y in range(len(grid_map)):
        for x in range(len(grid_map[0])):
            if grid_map[y, x] == 1:
                cost_map[y, x] += wall_penalty  # Стена → большой штраф

                # Добавляем штраф только в радиусе `penalty_radius`
                for dx in range(-penalty_radius, penalty_radius + 1):
                    for dy in range(-penalty_radius, penalty_radius + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < len(grid_map[0]) and 0 <= ny < len(grid_map):
                            if grid_map[ny, nx] == 0:  # Только если клетка свободна
                                cost_map[ny, nx] += near_wall_penalty
                
    return cost_map

def astar(start, goal, grid, cost_map, lambda_factor=1.0):
    """A* поиск кратчайшего пути с учётом штрафов за стены"""
    
    def heuristic(a, b):
        """Манхэттенское расстояние"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Вверх, вниз, вправо, влево
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    open_set = []
    
    heapq.heappush(open_set, (fscore[start], start))
    
    if grid[goal[1]][goal[0]] == 1:
        return []  # Если цель в стене, пути нет

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Возвращаем путь в правильном порядке
        
        close_set.add(current)

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if neighbor[0] < 0 or neighbor[0] >= len(grid) or neighbor[1] < 0 or neighbor[1] >= len(grid[0]):
                continue  # Пропускаем выход за границы
            
            if grid[neighbor[1]][neighbor[0]] == 1 or neighbor in close_set:
                continue  # Пропускаем стены
            
            # Добавляем штраф из cost_map
            move_cost = cost_map[neighbor[1]][neighbor[0]]
            tentative_gscore = gscore[current] + move_cost  # Учитываем штраф
            
            if neighbor not in gscore or tentative_gscore < gscore[neighbor]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_gscore
                fscore[neighbor] = tentative_gscore + heuristic(neighbor, goal)
                heapq.heappush(open_set, (fscore[neighbor], neighbor))
    return []  # Если пути нет

    
def MoveForward():
    global IsMoving
    IsMoving = True
    start_time = robot.getTime()
    MoveTime = 0.05 / (2 * 0.033)
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

def MyOrientation():
    if robot_angle % 180 == 0 and robot_angle % 360 != 0: return "DOWN"
    elif robot_angle == 0 or robot_angle % 360 == 0: return "UP"
    elif (robot_angle % 90 == 0 and robot_angle > 0) or (robot_angle % 270 == 0 and robot_angle < 0): return "LEFT"
    elif (robot_angle % 90 == 0 and robot_angle < 0) or (robot_angle % 270 == 0 and robot_angle > 0): return "RIGHT"

def SetOrientation():
    global CurrentDir
    CurrentDir = MyOrientation()
    print(CurrentDir) 

def get_new_direction(current_pos, next_pos):
    """Определяет новое направление движения"""
    dx, dy = next_pos[0] - current_pos[0], next_pos[1] - current_pos[1]

    if dx < 0: return "RIGHT"
    if dx > 0: return "LEFT"
    if dy < 0: return "DOWN"
    if dy > 0: return "UP"

    return None  # Если остались на месте (ошибка)

def needs_turning(current_pos, next_pos, current_dir):
    """Проверяет, нужно ли менять направление движения"""
    new_dir = get_new_direction(current_pos, next_pos)
    return new_dir != current_dir  # Если направление изменилось — нужен поворот

def get_turn_direction(current_dir, new_dir):
    """Определяет, нужно ли повернуть налево или направо"""
    turn_map = {
        ("UP", "LEFT"): "LEFT",
        ("UP", "RIGHT"): "RIGHT",
        ("DOWN", "LEFT"): "RIGHT",
        ("DOWN", "RIGHT"): "LEFT",
        ("LEFT", "UP"): "RIGHT",
        ("LEFT", "DOWN"): "LEFT",
        ("RIGHT", "UP"): "LEFT",
        ("RIGHT", "DOWN"): "RIGHT",
        ("RIGHT", "LEFT"): "180",
        ("LEFT", "RIGHT"): "180",
        ("UP", "DOWN"): "180",
        ("DOWN", "UP"): "180",
    }
    return turn_map.get((current_dir, new_dir), None)  # None = нет поворота

def update_movement(path, current_dir):
    current_pos = (robot_x,robot_y)
    next_pos = path[0]

    # Проверяем, надо ли поворачивать
    if needs_turning(current_pos, next_pos, current_dir):
        new_dir = get_new_direction(current_pos, next_pos)
        turn = get_turn_direction(current_dir, new_dir)
        return new_dir, turn  # Возвращаем новое направление и поворот

    return current_dir, None  # Если не поворачиваем, остаёмся в том же направлении

moves_count = 0

# === Движение робота по пути ===
def move_to_target(path):
    """Перемещает робота по найденному маршруту"""
    global robot_x, robot_y, grid_map
    if path:
        dir, turn = update_movement(path, CurrentDir)
        if turn != None:
           StopMoving()
           if turn != "180":
               robot_x, robot_y = path[0]
               RotationRobot(turn, 90)
               print("Поворачиваюсь на", turn)
               MoveForward()
           else:
               robot_x, robot_y = path[0]
               RotationRobot(turn, 180)
               print("Поворачиваюсь на", turn)
               MoveForward()
           #UpdateMap()
        else:
            robot_x, robot_y = path[0]
            MoveForward()
            #UpdateMap()          
            print("Двигаюсь к ", path[0], dir)
        path.pop(0)
         
    

def draw_map(grid_map, robot_x, robot_y):
    """Отображает карту в виде таблицы с позицией робота"""
    map_size = len(grid_map)
    for y in range(map_size):
        row = ""
        for x in range(map_size):
            if x == robot_x and y == robot_y:
                row += "3 "  # Робот
            elif grid_map[y][x] == 2:
                row += "2 "  # Препятствие
            elif grid_map[y][x] == 1:
                row += "1 "  # Препятствие
            elif grid_map[y][x] == 5:
                row += "5 "  # Препятствие
            else:
                row += "0 "  # Пустая клетка
        print(row)
    print("\n" + "="*60 + "\n")  # Разделитель между кадрами

def UpdateMap():
    #lidar_data = lidar.getRangeImage()
    #update_map(grid_map, lidar_data, robot_x, robot_y, robot_angle)
    draw_map(grid_map, robot_x, robot_y)


def find_passages(grid_map):
    """
    Ищет проходы (любого размера) с учетом условия пустоты перед одной из стен.
    Поиск идет снизу вверх и справа налево.

    :param grid_map: 2D numpy массив (0 - свободно, 1 - стена)
    :return: Список координат [(x, y), ...] центральных точек проходов
    """
    passages = set()
    rows, cols = grid_map.shape

    for y in range(rows - 2, 0, -1):  # Проход снизу вверх (пропускаем границы)
        for x in range(cols - 2, 0, -1):  # Проход справа налево
            if grid_map[y, x] == 0:  # Нашли свободную клетку, возможно это проход
                # Определяем границы прохода
                left = x
                while left > 0 and grid_map[y, left - 1] == 0:
                    left -= 1

                right = x
                while right < cols - 1 and grid_map[y, right + 1] == 0:
                    right += 1

                passage_width = right - left + 1

                # Проверяем наличие стен слева и справа
                if left > 0 and right < cols - 1 and grid_map[y, left - 1] == 1 and grid_map[y, right + 1] == 1:
                    # Проверяем, что перед одной из стен (сверху или снизу) пусто
                    if ((y > 0 and grid_map[y - 1, left - 1] == 0) and (y < rows - 1 and grid_map[y + 1, left - 1] == 0)) or ((y > 0 and grid_map[y - 1, right + 1] == 0) and (y < rows - 1 and grid_map[y + 1, right + 1] == 0)):
                        # Находим центр прохода
                        center_x = (left + right) // 2
                        passages.add((center_x, y))
    # Поиск вертикальных проходов (слева направо, снизу вверх)
    for x in range(cols - 2, 0,-1):
        for y in range(rows - 2, 0, -1):
            if grid_map[y, x] == 0:
                top = y
                while top > 0 and grid_map[top - 1, x] == 0:
                    top -= 1

                bottom = y
                while bottom < rows - 1 and grid_map[bottom + 1, x] == 0:
                    bottom += 1

                if top > 0 and bottom < rows - 1 and grid_map[top - 1, x] == 1 and grid_map[bottom + 1, x] == 1:
                    if ((x > 0 and grid_map[top - 1, x - 1] == 0) and (x < cols - 1 and grid_map[top - 1, x + 1] == 0)) or ((x > 0 and grid_map[bottom + 1, x - 1] == 0) and (x < cols - 1 and grid_map[bottom + 1, x + 1] == 0)):
                        center_y = (top + bottom) // 2
                        passages.add((x, center_y))
    return passages

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

def move_to(position):
    #global robot_position
    cost_map = compute_cost_map(grid_map)  # Создаём карту штрафов
    path = astar((robot_x, robot_y), position, grid_map, cost_map)
    if path:
        print(f"Путь к цели найден: {path}")
        while (robot_x, robot_y) != position:
            SetOrientation()
            move_to_target(path)  # Двигаем робота по пути
    #robot_position = position
    #time.sleep(1)

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



IsRobotInRoom = False
IsStart = True
TimeToMove = 0
IsObjectTaken = False
StartScanning = False
IsObjectDelivering = False

while robot.step(timestep) != -1:
    # Получаем изображение с камеры
    
    #frame_count +=1
    sensor_data = [DSF.getValue(), DSB.getValue(), DSR.getValue(), DSL.getValue()]
    SetOrientation()
    
    if IsStart:
        grid_map = load_map_from_file(file_path)
        UpdateMap()
        print(sensor_data)
        #print(localize_robot(grid_map, sensor_data, (robot_x, robot_y), robot_angle))
        IsStart = False
        prohod = find_passages(grid_map)
        prohod_list = list(prohod)
        room = prohod_list[0]
        print(prohod)
        
    if not IsScanning:
        move_to(room)
        scan_state = "searching"
        
    if (robot_x, robot_y) == room:
        objects = scan_room(room)
        if scan_state == "failed":
            rooms_count += 1
            room = prohod_list[rooms_count]
        
    if not IsScanning and objects != None:
        for obj in objects:
                #move_to(obj)
            pick_up_object(obj)
            move_to(warehouse_position)
            drop_object()
            if len(objects) > 1:
                move_to(room)
            else:
                rooms_count += 1
                room = prohod_list[rooms_count]
      
    #cv2.imshow("Camera View", frame)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
        
#plt.show()
cv2.destroyAllWindows()
