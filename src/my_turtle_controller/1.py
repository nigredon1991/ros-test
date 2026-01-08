import math
x, y, angle = [ float(i) for i in input().split()]
x_goal, y_goal = [ float(i) for i in input().split()]

len_path = math.sqrt((x - x_goal)**2 + (y - y_goal)**2)

# Вычисляем угол к цели в радианах (через арктангенс)
delta_x = x_goal - x
delta_y = y_goal - y
angle_to_goal_rad = math.atan2(delta_y, delta_x)

# Переводим угол в градусы
angle_to_goal_deg = math.degrees(angle_to_goal_rad)

# Приводим текущий угол черепашки к диапазону [0, 360)
current_angle = angle % 360

# Вычисляем требуемый относительный поворот
turn_angle = angle_to_goal_deg - current_angle

# Нормализуем угол поворота к диапазону [-180, 180]
if turn_angle > 180:
    turn_angle -= 360
elif turn_angle <= -180:
    turn_angle += 360

# # Округляем угол поворота до сотых
# turn_angle_rounded = round(turn_angle, 2)
print("{:.2f}".format(len_path))
print("{:.2f}".format(turn_angle))
