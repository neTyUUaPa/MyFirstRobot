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
servo_step = 0.05  # Шаг поворота
dead_zone = width * 0.05  # Зона ±5% от центра кадра
TurnVelocity = (4 * 0.033)
LenghtWheels = 0.160

map_size = 120  # 10x10 метров в сетке 100x100
grid_map = np.full((map_size, map_size), 2)  # 0 - свободно, 1 - стена
robot_x, robot_y = 67, 113
robot_angle = 0.0  # Угол поворота

count = 0

# Путь к вашему файлу
file_path = 'C:\\Users\\Alexei\\Desktop\\УмныйДом\\matrix1.txt'

# Загружаем карту


IsFound = False
IsTurned = False
IsStartTurn = False
IsRotating = False
IsMoving = False
frame_count = 0
target_position = 24, 55
IsUpdated = False
IsStartToObject = False

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
                robot_angle += angle
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
                robot_angle -= angle
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
    global CurrentDir
    current_pos = (robot_x,robot_y)
    next_pos = path[0]

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

def localize_robot(map_matrix, sensor_data, initial_position, initial_orientation):
    """
    Определяет наиболее вероятную позицию робота на основе показаний датчиков.
    
    :param map_matrix: 2D-массив карты (0 - свободно, 1 - стена)
    :param sensor_data: Измеренные расстояния с датчиков [front, back, left, right]
    :param initial_position: Начальная позиция (x, y) в ячейках
    :param initial_orientation: Начальный угол (в градусах)
    :return: (x, y, theta) - наилучшая оценка положения
    """
    min_error = float('inf')
    best_position = initial_position
    best_orientation = initial_orientation
    for x in range(len(sensor_data)):
        if sensor_data[x] == 1.0:
            sensor_data[x] = 10 
    rows, cols = map_matrix.shape
    search_range = 10  # Сколько ячеек вокруг начальной позиции проверять

    for x in range(max(0, initial_position[0] - search_range), min(cols, initial_position[0] + search_range)):
        for y in range(max(0, initial_position[1] - search_range), min(rows, initial_position[1] + search_range)):
            for theta in np.arange(0, 360, 15):  # Шаг 15° (в градусах)
                theta_rad = math.radians(theta)
                expected_distances = calculate_expected_distances(map_matrix, (x, y), theta_rad)

                # Рассчитать ошибку (квадрат разности между измеренными и ожидаемыми значениями)
                error = sum((measured - expected) ** 2 for measured, expected in zip(sensor_data, expected_distances))
                
                #print(f"Позиция: ({x}, {y}), Угол: {theta}, Ожидания: {expected_distances}, Ошибка: {error}")
                
                if error < min_error:
                    min_error = error
                    best_position = (x, y)
                    best_orientation = theta

    return best_position, best_orientation

    

def calculate_expected_distances(map_matrix, position, orientation):
    """
    Рассчитать ожидаемые расстояния от позиции робота до ближайших стен в направлениях датчиков.

    :param map_matrix: 2D-массив карты (0 - свободно, 1 - стена)
    :param position: Кортеж (x, y) текущей позиции робота в ЯЧЕЙКАХ МАТРИЦЫ
    :param orientation: Угол ориентации робота (в радианах)
    :return: Список расстояний [front, back, left, right] в ЯЧЕЙКАХ МАТРИЦЫ
    """
    distances = []
    directions = [
        0,                # Вперед (0 градусов от ориентации)
        math.pi,          # Назад (180 градусов от ориентации)
        -math.pi / 2,     # Влево (90 градусов против часовой стрелки)
        math.pi / 2       # Вправо (90 градусов по часовой стрелке)
    ]

    for direction in directions:
        # Учитываем ориентацию робота
        angle = orientation + direction  
        
        # Учитываем выход за границы карты
        if not (0 <= position[0] < map_matrix.shape[1] and 0 <= position[1] < map_matrix.shape[0]):
            distances.append(float('inf'))  # Если робот за пределами карты
            continue
        
        distance = trace_ray(map_matrix, position, angle)
        distances.append(distance)

    return distances


def trace_ray(map_matrix, position, angle):
    """
    Выполняет трассировку луча в направлении `angle` до ближайшей стены.
    
    :param map_matrix: 2D-массив карты (0 - свободно, 1 - стена)
    :param position: Кортеж (x, y) позиции робота в ЯЧЕЙКАХ МАТРИЦЫ
    :param angle: Угол направления (в радианах)
    :return: Расстояние до ближайшей стены в ЯЧЕЙКАХ
    """
    x, y = position
    step_size = 0.05  # Теперь 1 ячейка = 1 шаг
    max_distance = len(map_matrix)  # Максимальное расстояние в ячейках

    distance = 0
    while distance < max_distance:
        x += step_size * math.cos(angle)
        y += step_size * math.sin(angle)
        distance += step_size

        i, j = int(round(y)), int(round(x))  # Приводим координаты к индексам массива
        if i < 0 or j < 0 or i >= len(map_matrix) or j >= len(map_matrix[0]):
            return max_distance  # Вышли за пределы карты

        if map_matrix[i][j] == 1:  # Если наткнулись на стену
            return distance

    return max_distance

import numpy as np

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

# === A* с диагональными движениями ===
def astar_diagonal(start, goal, grid):
    """A* с учетом диагональных движений."""
    def heuristic(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy)  # Диагональная эвристика

    neighbors = [
        (0, 1), (0, -1), (1, 0), (-1, 0),  # Обычные перемещения
        (1, 1), (-1, -1), (1, -1), (-1, 1) # Диагональные перемещения
    ]
    
    open_set = []
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    
    heapq.heappush(open_set, (fscore[start], start))
    
    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]) and grid[neighbor[1]][neighbor[0]] != 1:
                tentative_gscore = gscore[current] + (1.4 if dx != 0 and dy != 0 else 1)
                
                if neighbor not in gscore or tentative_gscore < gscore[neighbor]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_gscore
                    fscore[neighbor] = tentative_gscore + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (fscore[neighbor], neighbor))
    
    return []


def find_target_position(robot_pos, angle, grid_map, max_distance=20):
    """
    Переводит угол в координаты сетки для движения к цели.
    :param robot_pos: (x, y) текущая позиция робота
    :param angle: угол в радианах
    :param grid_map: карта (numpy массив)
    :param max_distance: дальность поиска цели
    :return: координаты цели в сетке
    """
    x, y = robot_pos
    for _ in range(max_distance):
        x += math.cos(angle)
        y += math.sin(angle)
        i, j = int(round(y)), int(round(x))

        if 0 <= i < len(grid_map) and 0 <= j < len(grid_map[0]):
            if grid_map[i][j] == 1:  # Если наткнулись на стену, возвращаем предыдущую точку
                return int(round(y - math.sin(angle))), int(round(x - math.cos(angle)))
        else:
            break  # Вышли за границы

    return int(round(y)), int(round(x))  # Возвращаем конечную точку



IsRobotInRoom = False
IsStart = True
TimeToMove = 0
IsObjectTaken = False
StartScanning = False
IsObjectDelivering = False

while robot.step(timestep) != -1:
    # Получаем изображение с камеры
    
    
    frame_count +=1
    sensor_data = [DSF.getValue(), DSB.getValue(), DSR.getValue(), DSL.getValue()]
    if IsStart:
        grid_map = load_map_from_file(file_path)
        UpdateMap()
        print(sensor_data)
        #print(localize_robot(grid_map, sensor_data, (robot_x, robot_y), robot_angle))
        IsStart = False
        prohod = find_passages(grid_map)
        prohod_list = list(prohod)
        target_position = prohod_list[0]
        print(prohod)
    
    
    #print(DSF.getValue())
    #print(DSR.getValue())
    #print(DSL.getValue())
    #print(DSB.getValue())
    
    
    if target_position and frame_count % 20 == 0:
        
        if not IsMoving and not IsRotating:
            cost_map = compute_cost_map(grid_map)  # Создаём карту штрафов
            path = astar((robot_x, robot_y), target_position, grid_map, cost_map)
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
        if red_position and not IsFound and IsRobotInRoom:
            obj_x = red_position[0]
            print(f"Обнаружен красный объект в координатах: {red_position}")
            # Корректируем поворот камеры
            if obj_x < center_x - dead_zone:
                servo_angle -= servo_step
                
            elif obj_x > center_x + dead_zone:
                servo_angle += servo_step
                
            else:
                print("Объект в центре! Останавливаем камеру.")
                Obj_Angle = servo_angle
                IsFound = True
            servo.setPosition(servo_angle)
    #if robot_x == 90 and robot_y == 40:
    #    target_position = 95, 30
        
    if (robot_x, robot_y) == prohod_list[count] and IsObjectDelivering == False:
        grid_map[robot_y, robot_x] = 5
        IsRobotInRoom = True
        if count < 3:
            count += 1
        if count == 3:
            UpdateMap()
    if not IsFound and not red_position:
        servo_angle += servo_step  # Двигаем камеру, пока не найдем объект
        if servo_angle > 1.57 or servo_angle < -1.57:
            servo_step = -servo_step  # Меняем направление сканирования
        servo.setPosition(servo_angle)
    if IsFound and IsRobotInRoom:
        IsObjectDelivering = True
        print(math.degrees(Obj_Angle))
        RotationRobot("RIGHT", -math.degrees(Obj_Angle))
        x = DSF.getValue()
        while x > 0.1:
            MoveForward()
            TimeToMove += 1
            x = DSF.getValue()
        StopMoving()
        IsObjectTaken = True
        if IsObjectTaken:
            RotationRobot("LEFT", 180)
            while TimeToMove > 0:
                MoveForward()
                TimeToMove -= 1;
            RotationRobot("LEFT", -math.degrees(Obj_Angle))
            target_position = (67, 113)
        IsRobotInRoom = False
        CurrentDir = "RIGHT"
        Robot_Angle = -90
        
    if IsObjectDelivering and (robot_x, robot_y) == (37, 113):
        IsObjectDelivering = False
      
    cv2.imshow("Camera View", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
#plt.show()
cv2.destroyAllWindows()
