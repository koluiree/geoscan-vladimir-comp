import sys
import time

import numpy as np
import path_finder
from PioneerControl import PioneerControl  # ИЗМЕНЕНИЕ
from shapely.geometry import Polygon
from shapely.ops import unary_union

FLIGHT_ALTITUDE = 0.1  # Увеличим высоту для безопасности
FLIGHT_SPEED = 2.0
ZONE_PENALTY_PER_SECOND = 1.0
OUTSIDE_PENALTY_PER_SECOND = 12.0
ZONE_FULL_CROSS_MARGIN = 0.25

def fly_mission(pioneer: PioneerControl, waypoints: list):
    print("Дрон подключен. Начинаем миссию.")
    pioneer.arm()
    pioneer.takeoff()
    
    for i, point in enumerate(waypoints):
        x, y = point
        print(f"\nОтправка на точку #{i+1}: X={x:.2f}, Y={y:.2f} со скоростью {FLIGHT_SPEED} м/с")
        pioneer.set_point_with_speed(target_x=x, target_y=y, target_z=FLIGHT_ALTITUDE, speed=FLIGHT_SPEED)
        
        while not pioneer.point_reached():
            current_pos = pioneer.get_local_pose()
            if current_pos:
                print(f"\rТекущая позиция: X={current_pos[0]:.2f}, Y={current_pos[1]:.2f}, Z={current_pos[2]:.2f}", end="")
            time.sleep(0.05)
        
        print(f"\nТочка #{i+1} достигнута.")
        time.sleep(0.05)

    print("\nВсе точки пройдены. Посадка...")
    pioneer.land()
    
    return True

def main():
    if len(sys.argv) < 4:
        print("Использование: python main.py <ip> <port> <zones_file>")
        sys.exit(1)
        
    ip = sys.argv[1]
    port = int(sys.argv[2])
    zones_file = sys.argv[3]


    try:
        with open(zones_file, "r", encoding="utf-8") as f:
            text = f.read()
        start, finishes, zones_pts = path_finder.parse_data(text)
    except FileNotFoundError:
        print(f"Ошибка: Файл с зонами не найден по пути: {zones_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Ошибка при чтении или парсинге файла с зонами: {e}")
        sys.exit(1)

    res = 0.6

    all_x = [start[0]] + [finish[0] for finish in finishes] + [
        x for poly in zones_pts for (x, y) in poly
    ]
    all_y = [start[1]] + [finish[1] for finish in finishes] + [
        y for poly in zones_pts for (x, y) in poly
    ]
    min_x, max_x = min(all_x) - 1.0, max(all_x) + 1.0
    min_y, max_y = min(all_y) - 1.0, max(all_y) + 1.0

    shapely_polys = [Polygon(poly) for poly in zones_pts if len(poly) >= 3]
    permitted_zones_union = unary_union(shapely_polys) if shapely_polys else Polygon()
    effective_permitted_zones_union = (
        permitted_zones_union.buffer(ZONE_FULL_CROSS_MARGIN, join_style=2)
        if ZONE_FULL_CROSS_MARGIN > 0
        else permitted_zones_union
    )

    xs = np.arange(min_x, max_x + 1e-9, res)
    ys = np.arange(min_y, max_y + 1e-9, res)
    width, height = len(xs), len(ys)

    print("Запускаем поиск лучшего пути A*...")
    best_result = path_finder.find_best_path(
        start,
        finishes,
        min_x,
        min_y,
        res,
        xs,
        ys,
        width,
        height,
        effective_permitted_zones_union,
        flight_speed=FLIGHT_SPEED,
        cost_inside_zone_per_second=ZONE_PENALTY_PER_SECOND,
        cost_outside_zone_per_second=OUTSIDE_PENALTY_PER_SECOND,
    )

    if best_result is None:
        print("Путь не найден! Проверьте параметры или расположение зон.")
        sys.exit(1)

    finish_index, best_finish, optimized_path, total_penalty = best_result

    print(
        f"Выбран финиш #{finish_index}: X={best_finish[0]:.3f}, Y={best_finish[1]:.3f}, "
        f"ожидаемый штраф={total_penalty:.3f}"
    )
    
    print("\nОптимизированный путь (точки для дрона):")
    for p in optimized_path:
        print(f"X: {p[0]:.3f}, Y: {p[1]:.3f}")
    pioneer_drone = PioneerControl(ip_addr=ip, pioneer_port=port)
    time.sleep(0.3)

    try:
        success = fly_mission(pioneer_drone, optimized_path)
        if success:
            print("\nМиссия успешно завершена!")
        else:
            print("\nМиссия прервана из-за ошибки.")
    except Exception as e:
        print(f"\nПроизошла критическая ошибка во время полета: {e}")


if __name__ == "__main__":
    main()
