from controller import Robot, Camera, Lidar
import numpy as np
import cv2
import math
import heapq
import matplotlib.pyplot as plt  # 🔹 Добавляем импорт для отрисовки карты


robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Инициализация сенсоров
camera = robot.getDevice("camera")
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()
center_x = width // 2  # Центр кадра по X

lidar = robot.getDevice("LDS-01")
lidar.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))  # Колеса работают в режиме скорости, не фиксированная позиция
right_motor.setPosition(float('inf'))  # Колеса работают в режиме скорости, не
left_motor.setVelocity(0.0)  # Пример действия - движение вперед
right_motor.setVelocity(0.0)

# Подключаем серву для камеры
servo = robot.getDevice("servo")
servo.setPosition(0.0)  # Начальное положение камеры

Obj_Angle = 0.0
servo_angle = 0.0
servo_step = 0.05  # Шаг поворота
dead_zone = width * 0.02  # Зона ±5% от центра кадра
TurnVelocity = (4 * 0.033)
LenghtWheels = 0.160

map_size = 100  # 10x10 метров в сетке 100x100
grid_map = np.full((map_size, map_size), 2)  # 0 - свободно, 1 - стена
robot_x, robot_y = 95, 30
robot_angle = 0.0  # Угол поворота

IsFound = False
IsTurned = False
IsStartTurn = False
IsRotating = False
IsMoving = False
frame_count = 0
target_position = 90, 40
IsUpdated = False
IsStartToObject = False

CurrentDir = "Right"

def RotationRobot(direction, angle):
    start_time = robot.getTime()
    global IsTurned, IsRotating, robot_angle
    Rad_Angle = angle*math.pi / 180
    RotationTime = (Rad_Angle*LenghtWheels) * 1.11/(TurnVelocity)
    print(RotationTime)
    #print(start_time)
    IsRotating = True
    if (direction == "RIGHT"):
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


def corner_penalty(node, grid, penalty=5):
    """
    Вычисляет штраф для узла node (кортеж (x, y)) на основе наличия препятствий,
    которые формируют угол.
    
    penalty - дополнительная стоимость, которую добавляем, если узел находится у угла.
    """
    x, y = node
    penalty_cost = 0

    # Проверяем 8 соседей
    # Для удобства определим относительные координаты соседей:
    neighbors = [(-1, -1), (0, -1), (1, -1),
                 (-1,  0),          (1,  0),
                 (-1,  1), (0,  1), (1,  1)]
    
    # Например, если слева и сверху ((-1,0) и (0,-1)) являются препятствиями,
    # считаем, что узел находится около угла.
    # Можно проверить все комбинации перпендикулярных направлений:
    directions = [((0, -1), (-1, 0)),  # сверху и слева
                  ((0, -1), (1, 0)),   # сверху и справа
                  ((0, 1), (-1, 0)),   # снизу и слева
                  ((0, 1), (1, 0))]    # снизу и справа
    
    for (dx1, dy1), (dx2, dy2) in directions:
        n1_x, n1_y = x + dx1, y + dy1
        n2_x, n2_y = x + dx2, y + dy2
        # Проверяем, что соседи внутри границ
        if (0 <= n1_x < len(grid)) and (0 <= n1_y < len(grid[0])) \
           and (0 <= n2_x < len(grid)) and (0 <= n2_y < len(grid[0])):
            if grid[n1_y][n1_x] == 1 and grid[n2_y][n2_x] == 1:
                penalty_cost += penalty  # Добавляем штраф за угол

    return penalty_cost

def astar(start, goal, grid, lambda_factor=1.0):
    """A* поиск кратчайшего пути на сетке"""
    
    def heuristic(a, b):
        """Манхэттенское расстояние"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Вверх, вниз, вправо, влево
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal) }
    open_set = []
    
    heapq.heappush(open_set, (fscore[start], start))
    
    if grid[goal[1]][goal[0]] == 1:
        return[]
    
    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]  # Возвращаем в правильном порядке
        
        close_set.add(current)

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if neighbor[0] < 0 or neighbor[0] >= len(grid) or neighbor[1] < 0 or neighbor[1] >= len(grid[0]):
                continue  # Пропускаем выход за границы
            
            if grid[neighbor[1]][neighbor[0]] == 1 or neighbor in close_set:
                continue  # Пропускаем стены
            
            tentative_gscore = gscore[current] + 1

            if neighbor not in gscore or tentative_gscore < gscore[neighbor]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_gscore
                fscore[neighbor] = tentative_gscore + heuristic(neighbor, goal)
                heapq.heappush(open_set, (fscore[neighbor], neighbor))

    return []  # Если пути нет


def check_for_unknown(path, grid):
    if len(path) < 2:
        return

    # Берем первые две точки после текущего положения робота
    next_positions = path[:2]
    unknown_found = any(grid[y][x] == 2 for x, y in next_positions)

    if unknown_found:
        print("Обнаружены неизвестные зоны, обновляем карту...")
        UpdateMap()
        
    elif grid[robot_y][robot_x - 1] == 1 or grid[robot_y][robot_x + 1] == 1:
        UpdateMap()
        
    elif grid[robot_y - 1][robot_x] == 1 or grid[robot_y + 1][robot_x] == 1:
        UpdateMap()
       
    
def MoveForward():
    global IsMoving
    IsMoving = True
    start_time = robot.getTime()
    MoveTime = 0.1 / (2 * 0.033)
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
    }
    return turn_map.get((current_dir, new_dir), None)  # None = нет поворота

def update_movement(path, current_dir):
    global CurrentDir
    current_pos, next_pos = path[0], path[1]

    # Проверяем, надо ли поворачивать
    if needs_turning(current_pos, next_pos, current_dir):
        new_dir = get_new_direction(current_pos, next_pos)
        turn = get_turn_direction(current_dir, new_dir)
        CurrentDir = new_dir
        return new_dir, turn  # Возвращаем новое направление и поворот

    return current_dir, None  # Если не поворачиваем, остаёмся в том же направлении

moves_count = 0

# === Движение робота по пути ===
def move_to_target(path):
    """Перемещает робота по найденному маршруту"""
    global robot_x, robot_y
    if path:
        dir, turn = update_movement(path, CurrentDir)
        if turn != None:
           robot_x, robot_y = path[0]
           RotationRobot(turn, 90)
           print("Поворачиваюсь на", turn)
           MoveForward()
           check_for_unknown(path, grid_map)
           #UpdateMap()
        else:
            robot_x, robot_y = path[0]
            MoveForward()
            check_for_unknown(path, grid_map)
            #UpdateMap()          
            print("Двигаюсь к ", path[0], dir)
            
          # Обновляем положение
        
def bresenham(x0, y0, x1, y1):
    """ Алгоритм Брезенхэма для построения линии между (x0, y0) и (x1, y1) """
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        
        points.append((x0, y0))  # Добавляем текущую точку
        
        if x0 == x1 and y0 == y1:
            break
                   
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

def is_area_free(grid_map, x, y, radius=30):
    """
    Проверяет, есть ли стена в радиусе (в ячейках). 
    Если стен нет, возвращает True.
    """
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(grid_map) and 0 <= ny < len(grid_map[0]):
                if grid_map[ny, nx] == 1:  # Найдена стена
                    return False
    return True
          
# === Функция обновления карты ===
def update_map(grid_map, lidar_data, robot_x, robot_y, robot_angle):
    robot_pos = robot_x, robot_y
    for i, distance in enumerate(lidar_data):
        if 0.1 < distance < 3.0:  # Если препятствие найдено
            angle = math.radians(i) + math.radians(robot_angle)
            x = int(robot_x + math.cos(angle) * distance * 10)
            y = int(robot_y + math.sin(angle) * distance * 10)

            # Проверяем границы
         
                
            if 0 <= x < map_size and 0 <= y < map_size:
                # Заполняем пустое пространство между роботом и точкой обнаружения
                
                for x1, y1 in bresenham(round(robot_pos[0]), round(robot_pos[1]), x, y):
                    if grid_map[y1, x1] == 2:  # Только если неизвестно
                        grid_map[y1, x1] = 0  # Свободное пространство
                
                if grid_map[y, x] != 1:  # Только если область еще неизвестна
                    grid_map[y, x] = 1  # Стена
                


#fig, ax = plt.subplots()
#img = ax.imshow(grid_map, cmap="gray")


    

def draw_map(grid_map, robot_x, robot_y):
    """Отображает карту в виде таблицы с позицией робота"""
    map_size = len(grid_map)
    for y in reversed(range(map_size)):
        row = ""
        for x in reversed(range(map_size)):
            if x == robot_x and y == robot_y:
                row += "3 "  # Робот
            elif grid_map[y][x] == 2:
                row += "2 "  # Препятствие
            elif grid_map[y][x] == 1:
                row += "1 "  # Препятствие
            else:
                row += "0 "  # Пустая клетка
        print(row)
    print("\n" + "="*30 + "\n")  # Разделитель между кадрами

def UpdateMap():
    lidar_data = lidar.getRangeImage()
    update_map(grid_map, lidar_data, robot_x, robot_y, robot_angle)
    draw_map(grid_map, robot_x, robot_y)


def find_object_position(grid_map, robot_x, robot_y, robot_angle, object_angle, max_distance=3.0):
    """Определяет координаты объекта по углу камеры"""
    
    # Угол объекта относительно карты
    absolute_angle = math.radians(robot_angle) + object_angle  # Глобальный угол
    x_end = int(robot_x + math.cos(absolute_angle) * max_distance * 10)
    y_end = int(robot_y + math.sin(absolute_angle) * max_distance * 10)

    # Поиск объекта на карте
    object_x, object_y = None, None
    for x, y in bresenham(robot_x, robot_y, x_end, y_end):
        if not (0 <= x < len(grid_map) and 0 <= y < len(grid_map[0])):
            break  # Если вышли за границы карты
        
        if grid_map[y, x] == 1:  # Если наткнулись на стену
            print(x, y)
            break
        
        object_x, object_y = x, y  # Запоминаем последнюю свободную точку

    if object_x is None or object_y is None:
        print("Объект не найден!")
        return None

    print(f"Объект найден в ({object_x}, {object_y})")
    return object_x, object_y  # Возвращаем найденные координаты

IsStart = True

while robot.step(timestep) != -1:
    # Получаем изображение с камеры
    frame_count +=1
    #img.set_data(grid_map)
    #plt.pause(0.01)  # Снизить частоту обновлений
    
    
    #plt.imshow(grid_map, cmap="gray")
    #plt.pause(0.1)
    if IsStart:
        UpdateMap()
        IsStart = False
        
    if target_position and frame_count % 20 == 0:
        
        if not IsMoving and not IsRotating:
            path = astar((robot_x, robot_y), target_position, grid_map)
            if path:
                print(f"Путь к цели найден: {path}")
                move_to_target(path)  # Двигаем робота по пути
                
    image = camera.getImage()
    if image:
        width = camera.getWidth()
        height = camera.getHeight()
        frame = np.frombuffer(image, np.uint8).reshape((height, width, 4))  # Webots использует (H, W, C)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Убираем альфа-канал

        # Поиск красного объекта
        red_position, obj_width = detect_green(frame)
        if red_position and not IsFound:
            obj_x = red_position[0]
            print(f"Обнаружен красный объект в координатах: {red_position}")
            # Корректируем поворот камеры
            if obj_x < center_x - dead_zone:
                None
                #IsFound = False
            elif obj_x > center_x + dead_zone:
                None
                #IsFound = False
            else:
                print("Объект в центре! Останавливаем камеру.")
                Obj_Angle = servo_angle
                IsFound = True
    
    if not IsFound:
        servo_angle += servo_step  # Двигаем камеру, пока не найдем объект
        if servo_angle > 1.57 or servo_angle < -1.57:
            servo_step = -servo_step  # Меняем направление сканирования
        servo.setPosition(servo_angle)
        
    if IsFound and not IsStartToObject:
        servo.setPosition(0.85)  # Фиксируем камеру на объекте
        print(Obj_Angle)
        target_position = find_object_position(grid_map, robot_x, robot_y, robot_angle, 3.14 - Obj_Angle)
        IsStartToObject = True
        print(robot_angle)
        draw_map(grid_map, robot_x, robot_y)          
    #if frame_count % 10 == 0 and not IsRotating:
    cv2.imshow("Camera View", frame)
    # Получаем данные с лидара
    #lidar_data = lidar.getRangeImage()
    #if lidar_data:
    #    min_distance = min(lidar_data)
    #    print(f"Минимальное расстояние до объекта: {min_distance:.2f} м")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
#plt.show()
cv2.destroyAllWindows()
