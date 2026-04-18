import sys
import time

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

FLIGHT_ALTITUDE = 0.1  # Увеличим высоту для безопасности
FLIGHT_SPEED = 7.0
ZONE_PENALTY_PER_SECOND = 1.0
OUTSIDE_PENALTY_PER_SECOND = 12.0
ZONE_FULL_CROSS_MARGIN = 0.25


def fly_mission(pioneer: PioneerControl, waypoints: list):
    print("Дрон подключен. Начинаем миссию.")
    pioneer.arm()
    pioneer.takeoff()

    for i, point in enumerate(waypoints):
        x, y = point
        print(
            f"\nОтправка на точку #{i + 1}: X={x:.2f}, Y={y:.2f} со скоростью {FLIGHT_SPEED} м/с"
        )
        pioneer.set_point_with_speed(
            target_x=x, target_y=y, target_z=FLIGHT_ALTITUDE, speed=FLIGHT_SPEED
        )

        while not pioneer.point_reached():
            current_pos = pioneer.get_local_pose()
            if current_pos:
                print(
                    f"\rТекущая позиция: X={current_pos[0]:.2f}, Y={current_pos[1]:.2f}, Z={current_pos[2]:.2f}",
                    end="",
                )
            time.sleep(0.05)

        print(f"\nТочка #{i + 1} достигнута.")
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

    all_x = (
        [start[0]]
        + [finish[0] for finish in finishes]
        + [x for poly in zones_pts for (x, y) in poly]
    )
    all_y = (
        [start[1]]
        + [finish[1] for finish in finishes]
        + [y for poly in zones_pts for (x, y) in poly]
    )
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


import heapq
import math

from shapely.geometry import LineString

try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon
except ModuleNotFoundError:
    plt = None
    MplPolygon = None


def _edge_cache_key(
    a,
    b,
    permitted_zones_union,
    flight_speed,
    cost_inside_zone_per_second,
    cost_outside_zone_per_second,
):
    left, right = (a, b) if a <= b else (b, a)
    return (
        left,
        right,
        id(permitted_zones_union),
        flight_speed,
        cost_inside_zone_per_second,
        cost_outside_zone_per_second,
    )


def _parse_point_line(line, label):
    parts = list(map(float, line.split()))
    if len(parts) != 2:
        raise ValueError(f"Координаты {label} должны содержать ровно 2 числа")
    return tuple(parts)


def _parse_zone_line(line):
    parts = list(map(float, line.split()))
    if len(parts) < 6 or len(parts) % 2 != 0:
        raise ValueError("Каждая зона должна содержать не менее 3 точек")
    return [(parts[i], parts[i + 1]) for i in range(0, len(parts), 2)]


def parse_data(text):
    lines = [line.strip() for line in text.strip().splitlines() if line.strip()]
    if len(lines) < 3:
        raise ValueError("Неверный формат файла")

    first_parts = lines[0].split()
    zones = []

    if len(first_parts) == 1:
        point_count = int(first_parts[0])
        if point_count < 2:
            raise ValueError("Должен быть как минимум один старт и один финиш")

        point_lines = lines[1 : 1 + point_count]
        if len(point_lines) != point_count:
            raise ValueError("Несовпадение количества строк с точками")

        points = [_parse_point_line(point_line, "точки") for point_line in point_lines]
        start = points[0]
        finishes = points[1:]

        zone_count_index = 1 + point_count
        if zone_count_index >= len(lines):
            raise ValueError("Не найдено количество разрешённых зон")
        zone_count = int(lines[zone_count_index])
        zone_lines = lines[zone_count_index + 1 : zone_count_index + 1 + zone_count]
    else:
        start = _parse_point_line(lines[0], "старта")
        line_index = 1

        second_parts = lines[line_index].split()
        if len(second_parts) == 1:
            finish_count = int(second_parts[0])
            if finish_count <= 0:
                raise ValueError("Количество финишей должно быть больше нуля")
            line_index += 1
            finish_lines = lines[line_index : line_index + finish_count]
            if len(finish_lines) != finish_count:
                raise ValueError("Несовпадение количества финишей")
            finishes = [
                _parse_point_line(finish_line, "финиша") for finish_line in finish_lines
            ]
            line_index += finish_count
        elif len(second_parts) == 2:
            finishes = [_parse_point_line(lines[line_index], "финиша")]
            line_index += 1
        else:
            raise ValueError("Неверный блок финишей")

        if line_index >= len(lines):
            raise ValueError("Не найдено количество разрешённых зон")
        zone_count = int(lines[line_index])
        zone_lines = lines[line_index + 1 : line_index + 1 + zone_count]

    if len(zone_lines) != zone_count:
        raise ValueError("Несовпадение количества зон")

    for zone_line in zone_lines:
        zones.append(_parse_zone_line(zone_line))

    return start, finishes, zones


def build_grid(min_x, max_x, min_y, max_y, res):
    xs = np.arange(min_x, max_x + 1e-9, res)
    ys = np.arange(min_y, max_y + 1e-9, res)
    nodes = []
    index = {}
    i = 0
    for yi, y in enumerate(ys):
        for xi, x in enumerate(xs):
            nodes.append((x, y))
            index[(xi, yi)] = i
            i += 1
    width = len(xs)
    height = len(ys)
    return nodes, index, xs, ys, width, height


def coord_to_grid_index(coord, min_x, min_y, res):
    x, y = coord
    gx = int(round((x - min_x) / res))
    gy = int(round((y - min_y) / res))
    return gx, gy


def grid_index_to_coord(gx, gy, min_x, min_y, res):
    return (min_x + gx * res, min_y + gy * res)


def edge_cost(
    a,
    b,
    permitted_zones_union,
    flight_speed=1.0,
    cost_inside_zone_per_second=1.0,
    cost_outside_zone_per_second=12.0,
    edge_cost_cache=None,
):
    cache_key = None
    if edge_cost_cache is not None:
        cache_key = _edge_cache_key(
            a,
            b,
            permitted_zones_union,
            flight_speed,
            cost_inside_zone_per_second,
            cost_outside_zone_per_second,
        )
        if cache_key in edge_cost_cache:
            return edge_cost_cache[cache_key]

    line = LineString([a, b])
    total_len = line.length
    if total_len == 0:
        return 0.0

    inter = line.intersection(permitted_zones_union)
    len_in_zone = 0.0
    if not inter.is_empty:
        len_in_zone = inter.length
    len_in_zone = min(len_in_zone, total_len)
    len_out = max(0.0, total_len - len_in_zone)

    time_in_zone = len_in_zone / flight_speed
    time_out_zone = len_out / flight_speed
    cost = (
        time_in_zone * cost_inside_zone_per_second
        + time_out_zone * cost_outside_zone_per_second
    )

    if cache_key is not None:
        edge_cost_cache[cache_key] = cost

    return cost


def path_cost(
    path,
    permitted_zones_union,
    flight_speed=1.0,
    cost_inside_zone_per_second=1.0,
    cost_outside_zone_per_second=12.0,
    edge_cost_cache=None,
):
    if not path or len(path) < 2:
        return 0.0
    return sum(
        edge_cost(
            path[i - 1],
            path[i],
            permitted_zones_union,
            flight_speed=flight_speed,
            cost_inside_zone_per_second=cost_inside_zone_per_second,
            cost_outside_zone_per_second=cost_outside_zone_per_second,
            edge_cost_cache=edge_cost_cache,
        )
        for i in range(1, len(path))
    )


def astar_grid_with_cost(
    start,
    goal,
    min_x,
    min_y,
    res,
    xs,
    ys,
    width,
    height,
    permitted_zones_union,
    flight_speed=1.0,
    cost_inside_zone_per_second=1.0,
    cost_outside_zone_per_second=12.0,
    edge_cost_cache=None,
):
    sx = (start[0] - min_x) / res
    sy = (start[1] - min_y) / res
    gx = (goal[0] - min_x) / res
    gy = (goal[1] - min_y) / res

    start_gx = int(round(sx))
    start_gy = int(round(sy))
    goal_gx = int(round(gx))
    goal_gy = int(round(gy))

    def in_bounds(gx, gy):
        return 0 <= gx < width and 0 <= gy < height

    start_idx = (start_gx, start_gy)
    goal_idx = (goal_gx, goal_gy)
    start_coord = grid_index_to_coord(start_gx, start_gy, min_x, min_y, res)
    goal_coord = grid_index_to_coord(goal_gx, goal_gy, min_x, min_y, res)

    if start_idx == goal_idx:
        return [start, goal], edge_cost(
            start,
            goal,
            permitted_zones_union,
            flight_speed=flight_speed,
            cost_inside_zone_per_second=cost_inside_zone_per_second,
            cost_outside_zone_per_second=cost_outside_zone_per_second,
            edge_cost_cache=edge_cost_cache,
        )

    min_penalty_per_second = min(
        cost_inside_zone_per_second, cost_outside_zone_per_second
    )

    def heuristic(coord):
        return (
            math.hypot(goal[0] - coord[0], goal[1] - coord[1])
            / flight_speed
            * min_penalty_per_second
        )

    open_heap = []
    start_cost = edge_cost(
        start,
        start_coord,
        permitted_zones_union,
        flight_speed=flight_speed,
        cost_inside_zone_per_second=cost_inside_zone_per_second,
        cost_outside_zone_per_second=cost_outside_zone_per_second,
        edge_cost_cache=edge_cost_cache,
    )
    heapq.heappush(open_heap, (start_cost + heuristic(start_coord), start_idx))
    g_score = {start_idx: start_cost}
    came_from = {}
    closed_nodes = set()

    neighbor_offsets = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    ]

    while open_heap:
        f, current = heapq.heappop(open_heap)
        if current in closed_nodes:
            continue

        current_coord = grid_index_to_coord(current[0], current[1], min_x, min_y, res)
        expected_f = g_score[current] + heuristic(current_coord)
        if f > expected_f + 1e-9:
            continue

        closed_nodes.add(current)

        if current == goal_idx:
            path = []
            cur = current
            while cur in came_from:
                coord = grid_index_to_coord(cur[0], cur[1], min_x, min_y, res)
                path.append(coord)
                cur = came_from[cur]
            path.append(start_coord)
            path.reverse()

            if path[0] != start:
                path.insert(0, start)
            else:
                path[0] = start

            if path[-1] != goal:
                path.append(goal)
            else:
                path[-1] = goal

            total_cost = g_score[current] + edge_cost(
                goal_coord,
                goal,
                permitted_zones_union,
                flight_speed=flight_speed,
                cost_inside_zone_per_second=cost_inside_zone_per_second,
                cost_outside_zone_per_second=cost_outside_zone_per_second,
                edge_cost_cache=edge_cost_cache,
            )
            return path, total_cost

        for dx, dy in neighbor_offsets:
            nx, ny = current[0] + dx, current[1] + dy
            if not in_bounds(nx, ny):
                continue
            neighbor = (nx, ny)
            if neighbor in closed_nodes:
                continue
            a = grid_index_to_coord(current[0], current[1], min_x, min_y, res)
            b = grid_index_to_coord(nx, ny, min_x, min_y, res)
            tentative_g = g_score[current] + edge_cost(
                a,
                b,
                permitted_zones_union,
                flight_speed=flight_speed,
                cost_inside_zone_per_second=cost_inside_zone_per_second,
                cost_outside_zone_per_second=cost_outside_zone_per_second,
                edge_cost_cache=edge_cost_cache,
            )
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                nb_coord = grid_index_to_coord(nx, ny, min_x, min_y, res)
                f_score = tentative_g + heuristic(nb_coord)
                heapq.heappush(open_heap, (f_score, neighbor))
                came_from[neighbor] = current

    return None, math.inf


def astar_grid(
    start,
    goal,
    min_x,
    min_y,
    res,
    xs,
    ys,
    width,
    height,
    permitted_zones_union,
    flight_speed=1.0,
    cost_inside_zone_per_second=1.0,
    cost_outside_zone_per_second=12.0,
    edge_cost_cache=None,
):
    path, _ = astar_grid_with_cost(
        start,
        goal,
        min_x,
        min_y,
        res,
        xs,
        ys,
        width,
        height,
        permitted_zones_union,
        flight_speed=flight_speed,
        cost_inside_zone_per_second=cost_inside_zone_per_second,
        cost_outside_zone_per_second=cost_outside_zone_per_second,
        edge_cost_cache=edge_cost_cache,
    )
    return path


def path_distance(path):
    if not path or len(path) < 2:
        return 0.0
    return sum(
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )


def _simplify_collinear_path(path, tol=1e-9):
    if not path or len(path) < 3:
        return path[:]

    simplified = [path[0]]

    for i in range(1, len(path) - 1):
        prev = simplified[-1]
        curr = path[i]
        nxt = path[i + 1]

        if curr == prev:
            continue

        v1x = curr[0] - prev[0]
        v1y = curr[1] - prev[1]
        v2x = nxt[0] - curr[0]
        v2y = nxt[1] - curr[1]
        cross = v1x * v2y - v1y * v2x
        dot = v1x * v2x + v1y * v2y

        if abs(cross) <= tol and dot >= -tol:
            continue

        simplified.append(curr)

    if simplified[-1] != path[-1]:
        simplified.append(path[-1])

    return simplified


def find_best_path(
    start,
    goals,
    min_x,
    min_y,
    res,
    xs,
    ys,
    width,
    height,
    permitted_zones_union,
    flight_speed=1.0,
    cost_inside_zone_per_second=1.0,
    cost_outside_zone_per_second=12.0,
):
    best_result = None
    shared_edge_cost_cache = {}

    for goal_index, goal in enumerate(goals, start=1):
        path, _ = astar_grid_with_cost(
            start,
            goal,
            min_x,
            min_y,
            res,
            xs,
            ys,
            width,
            height,
            permitted_zones_union,
            flight_speed=flight_speed,
            cost_inside_zone_per_second=cost_inside_zone_per_second,
            cost_outside_zone_per_second=cost_outside_zone_per_second,
            edge_cost_cache=shared_edge_cost_cache,
        )
        if path is None:
            continue

        optimized_path = simplify_path(
            path,
            permitted_zones_union=permitted_zones_union,
            flight_speed=flight_speed,
            cost_inside_zone_per_second=cost_inside_zone_per_second,
            cost_outside_zone_per_second=cost_outside_zone_per_second,
            edge_cost_cache=shared_edge_cost_cache,
        )
        total_cost = path_cost(
            optimized_path,
            permitted_zones_union,
            flight_speed=flight_speed,
            cost_inside_zone_per_second=cost_inside_zone_per_second,
            cost_outside_zone_per_second=cost_outside_zone_per_second,
            edge_cost_cache=shared_edge_cost_cache,
        )

        if best_result is None:
            best_result = (goal_index, goal, optimized_path, total_cost)
            continue

        _, _, best_path, best_cost = best_result
        same_cost = math.isclose(total_cost, best_cost, rel_tol=1e-9, abs_tol=1e-9)
        if total_cost < best_cost or (
            same_cost and path_distance(optimized_path) < path_distance(best_path)
        ):
            best_result = (goal_index, goal, optimized_path, total_cost)

    return best_result


def simplify_path(
    path,
    permitted_zones_union=None,
    flight_speed=1.0,
    cost_inside_zone_per_second=1.0,
    cost_outside_zone_per_second=12.0,
    edge_cost_cache=None,
    shortcut_cost_tolerance=0.1,
    tol=1e-9,
):
    base_path = _simplify_collinear_path(path, tol=tol)
    if permitted_zones_union is None or not base_path or len(base_path) < 3:
        return base_path

    prefix_costs = [0.0]
    for i in range(1, len(base_path)):
        prefix_costs.append(
            prefix_costs[-1]
            + edge_cost(
                base_path[i - 1],
                base_path[i],
                permitted_zones_union,
                flight_speed=flight_speed,
                cost_inside_zone_per_second=cost_inside_zone_per_second,
                cost_outside_zone_per_second=cost_outside_zone_per_second,
                edge_cost_cache=edge_cost_cache,
            )
        )

    optimized_path = [base_path[0]]
    i = 0
    while i < len(base_path) - 1:
        best_next = i + 1
        for j in range(len(base_path) - 1, i + 1, -1):
            direct_cost = edge_cost(
                base_path[i],
                base_path[j],
                permitted_zones_union,
                flight_speed=flight_speed,
                cost_inside_zone_per_second=cost_inside_zone_per_second,
                cost_outside_zone_per_second=cost_outside_zone_per_second,
                edge_cost_cache=edge_cost_cache,
            )
            original_cost = prefix_costs[j] - prefix_costs[i]
            if direct_cost <= original_cost + shortcut_cost_tolerance:
                best_next = j
                break

        optimized_path.append(base_path[best_next])
        i = best_next

    return _simplify_collinear_path(optimized_path, tol=tol)


def plot_result(
    start,
    goals,
    zones,
    path,
    res,
    min_x,
    max_x,
    min_y,
    max_y,
    selected_goal=None,
):
    if plt is None or MplPolygon is None:
        raise RuntimeError("Для визуализации установите matplotlib")

    fig, ax = plt.subplots(figsize=(10, 10))

    for pts in zones:
        mpl_poly = MplPolygon(
            np.array(pts),
            closed=True,
            facecolor="limegreen",
            alpha=0.35,
            edgecolor="darkgreen",
            zorder=2,
        )
        ax.add_patch(mpl_poly)

    if path:
        px, py = zip(*path)
        ax.plot(
            px,
            py,
            marker="o",
            linewidth=2,
            markersize=4,
            label="Лучший путь",
            zorder=5,
        )
    ax.plot(start[0], start[1], "go", markersize=10, label="Старт", zorder=6)

    for index, goal in enumerate(goals, start=1):
        is_selected = selected_goal is not None and goal == selected_goal
        marker = "ro" if is_selected else "bo"
        label = "Выбранный финиш" if is_selected else "Финиш"
        ax.plot(goal[0], goal[1], marker, markersize=9, label=label, zorder=6)
        ax.text(goal[0], goal[1], f" {index}", fontsize=9, zorder=7)

    ax.set_xlim(min_x - res, max_x + res)
    ax.set_ylim(min_y - res, max_y + res)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Прокладка пути по разрешённым зонам (A*, res = {:.3f})".format(res))
    plt.show()


def main():
    try:
        with open("./zones1.txt", "r") as f:
            text = f.read()
    except FileNotFoundError:
        print(
            "Ошибка: Файл 'zones1.txt' не найден. Убедитесь, что он находится в той же директории."
        )
        return

    start, goals, zones_pts = parse_data(text)

    res = 0.5
    flight_speed = 2.0
    cost_inside_zone_per_second = 1.0
    cost_outside_zone_per_second = 12.0

    all_x = (
        [start[0]]
        + [goal[0] for goal in goals]
        + [x for poly in zones_pts for (x, y) in poly]
    )
    all_y = (
        [start[1]]
        + [goal[1] for goal in goals]
        + [y for poly in zones_pts for (x, y) in poly]
    )
    min_x = min(all_x) - 1.0
    max_x = max(all_x) + 1.0
    min_y = min(all_y) - 1.0
    max_y = max(all_y) + 1.0

    shapely_polys = [Polygon(poly) for poly in zones_pts if len(poly) >= 3]
    if shapely_polys:
        zones_union = unary_union(shapely_polys)
    else:
        zones_union = Polygon()

    xs = np.arange(min_x, max_x + 1e-9, res)
    ys = np.arange(min_y, max_y + 1e-9, res)
    width = len(xs)
    height = len(ys)

    best_result = find_best_path(
        start,
        goals,
        min_x,
        min_y,
        res,
        xs,
        ys,
        width,
        height,
        zones_union,
        flight_speed=flight_speed,
        cost_inside_zone_per_second=cost_inside_zone_per_second,
        cost_outside_zone_per_second=cost_outside_zone_per_second,
    )
    if best_result is None:
        print("Путь не найден. Попробуйте увеличить область или изменить разрешение.")
        plot_result(
            start,
            goals,
            zones_pts,
            None,
            res,
            min_x,
            max_x,
            min_y,
            max_y,
        )
        return

    goal_index, goal, path, total_cost = best_result

    print("Полный путь (узлы сетки, затем подставленный точный старт/финиш):")
    for p in path:
        print(f"{p[0]:.3f} {p[1]:.3f}")

    print(
        f"\nЛучший финиш: #{goal_index} ({goal[0]:.3f}, {goal[1]:.3f}), "
        f"ожидаемый штраф: {total_cost:.3f}"
    )

    simplified = simplify_path(path)
    print("\nУпрощённый путь (точки смены направления):")
    for p in simplified:
        print(f"{p[0]:.3f} {p[1]:.3f}")

    plot_result(
        start,
        goals,
        zones_pts,
        simplified,
        res,
        min_x,
        max_x,
        min_y,
        max_y,
        selected_goal=goal,
    )


import logging
import threading

from pioneer_sdk import Pioneer

MAX_FLIGHT_SPEED = 2.0
SLOWDOWN_DISTANCE = 1.2
MIN_APPROACH_SPEED = 0.6


class PioneerControl:
    def __init__(self, ip_addr, pioneer_port):
        logging.basicConfig(
            level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
        )
        self.logger = logging.getLogger(__name__)

        self.pioneer: Pioneer | None = None
        self.target_speed = 0  # для управления скоростью в функции __compute_speed
        self.low_distance = False

        self.__current_pos: tuple[float, float, float] = (
            0.0,
            0.0,
            0.0,
        )  # последняя полученная позиция дрона
        self.__command_pos: tuple[int] | None = None  # позиция для команды
        self.__yaw: float = 0.0  # рысканье
        self.__yaw_rate: float = 0.0  # скорость поворота рысканья
        self.__speed: tuple[float, float, float] = (
            0.0,
            0.0,
            0.0,
        )  # задание скоростей по x, y, z
        self.__coords_system: str = (
            "GLOBAL_COORDS"  # GLOBAL_COORDS / SPEED / SPEED_FIXED / SPEED_POINT
        )
        self.__armed: bool = False
        self.__takeoffed: bool = False
        self.__target_yaw: int | None = None

        self.__connect(ip_addr=str(ip_addr), port=int(pioneer_port))

        # --- Изменения для корректного завершения ---
        self.__stop_event = threading.Event()
        self.__command_thread = threading.Thread(
            target=self.__command_loop, daemon=True
        )
        self.__command_thread.start()

    def __command_loop(self):
        self.logger.info("Command loop started")
        prev_cmd = None
        # --- Цикл теперь проверяет событие остановки ---
        while not self.__stop_event.is_set():
            if not self.pioneer.connected():
                self.logger.error("PIONEER NOT CONNECTED")
                time.sleep(1)  # Пауза перед новой попыткой
                continue

            if self.__coords_system == "GLOBAL_COORDS":
                if self.__command_pos is not None and self.__armed and self.__takeoffed:
                    if prev_cmd != self.__command_pos:
                        self.pioneer.go_to_local_point(
                            x=self.__command_pos[0],
                            y=self.__command_pos[1],
                            z=self.__command_pos[2],
                            yaw=self.__yaw,
                        )
                        self.logger.info(f"Command sent {self.__command_pos}")
                        prev_cmd = self.__command_pos
            elif self.__coords_system == "SPEED":
                if self.__armed and self.__takeoffed:
                    if prev_cmd != (*self.__speed, self.__yaw_rate):
                        self.pioneer.set_manual_speed(
                            vx=self.__speed[0],
                            vy=self.__speed[1],
                            vz=self.__speed[2],
                            yaw_rate=self.__yaw_rate,
                        )
                        self.logger.info(
                            f"Command sent {self.__speed}, yaw_rate: {self.__yaw_rate}"
                        )
                        prev_cmd = (*self.__speed, self.__yaw_rate)
            elif self.__coords_system == "SPEED_FIXED":
                if self.__armed and self.__takeoffed:
                    if self.__target_yaw is not None and self.__is_yaw_reached():
                        self.__target_yaw = None
                        self.__yaw_rate = 0
                    if prev_cmd != (*self.__speed, self.__yaw_rate):
                        self.pioneer.set_manual_speed_body_fixed(
                            vx=self.__speed[0],
                            vy=self.__speed[1],
                            vz=self.__speed[2],
                            yaw_rate=self.__yaw_rate,
                        )
                        self.logger.info(
                            f"Command sent {self.__speed}, yaw_rate: {self.__yaw_rate}"
                        )
                        prev_cmd = (*self.__speed, self.__yaw_rate)
            elif self.__coords_system == "SPEED_POINT":
                if self.__command_pos is not None:
                    self.__speed = self.__compute_speed(
                        *self.__command_pos, wanted_speed=self.target_speed
                    )
                if self.__armed and self.__takeoffed:
                    self.pioneer.set_manual_speed_body_fixed(
                        vx=self.__speed[0],
                        vy=self.__speed[1],
                        vz=self.__speed[2],
                        yaw_rate=self.__yaw_rate,
                    )

            self.__update_position()
            # self.__update_yaw()
            time.sleep(0.05)
        self.logger.info("Command loop stopped.")

    def stop(self):
        """Останавливает поток управления и отключается от дрона."""
        self.logger.info("Stopping...")
        self.__stop_event.set()
        self.__command_thread.join()  # Ожидаем завершения потока
        if self.pioneer and self.pioneer.connected():
            self.pioneer.close()  # Закрываем соединение
            self.logger.info("Pioneer connection closed.")

    def __connect(self, ip_addr, port: str):
        """Подключение к пионеру"""
        try:
            self.pioneer = Pioneer(ip=ip_addr, mavlink_port=port, logger=False)
            if self.pioneer.connected():
                self.logger.info("Pioneer connected!")
            else:
                self.logger.warning("Can't connect to Pioneer!")
        except Exception as e:
            self.logger.error(f"Failed to connect to Pioneer: {e}")

    def __is_yaw_reached(self, dead_zone: int = 10) -> bool:
        if self.__coords_system == "SPEED_FIXED" and self.__target_yaw is not None:
            return abs(math.degrees(self.__yaw) - self.__target_yaw) < dead_zone
        return False

    def __is_point_reached(self, dead_zone: float = 0.7) -> bool:
        """Проверка на расстояние до точки"""
        if self.__command_pos is None:
            return False
        if self.__coords_system in ("GLOBAL_COORDS", "SPEED_POINT"):
            if self.__current_pos:
                distance = math.sqrt(
                    (self.__current_pos[0] - self.__command_pos[0]) ** 2
                    + (self.__current_pos[1] - self.__command_pos[1]) ** 2
                    + (self.__current_pos[2] - self.__command_pos[2]) ** 2
                )
                return distance < dead_zone
            return False
        else:
            self.logger.warning(
                "Can't check if point is reached in current coordinate system"
            )
            return True

    # def __update_yaw(self):
    #     yaw = self.pioneer.get_yaw(get_last_received=True)
    #     if yaw is not None:
    #         self.__yaw = yaw

    def __update_position(self):
        pos = self.pioneer.get_local_position_lps(get_last_received=True)
        if pos is not None:
            self.__current_pos = pos

    def __compute_speed(
        self, target_x: float, target_y: float, target_z: float, wanted_speed: float
    ) -> tuple[float, float, float]:
        """Вычисляет скорость, с которой дрону нужно лететь по x, y, z чтобы добраться до точки"""
        if self.__current_pos is None or self.__command_pos is None:
            return 0.0, 0.0, 0.0

        diff_x = target_x - self.__current_pos[0]
        diff_y = target_y - self.__current_pos[1]
        diff_z = target_z - self.__current_pos[2]

        distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

        if distance < 0.05:  # Если мы уже в точке, скорость равна нулю
            return 0.0, 0.0, 0.0

        wanted_speed = max(0.0, min(float(wanted_speed), MAX_FLIGHT_SPEED))

        # Плавное замедление при приближении к цели
        current_speed = wanted_speed
        if distance < SLOWDOWN_DISTANCE:
            current_speed = max(
                MIN_APPROACH_SPEED,
                wanted_speed * (distance / SLOWDOWN_DISTANCE),
            )

        speed_scale = current_speed / distance
        vx = diff_x * speed_scale
        vy = diff_y * speed_scale
        vz = diff_z * speed_scale

        return vx, vy, vz

    def point_reached(self):
        if self.pioneer.connected():
            return self.__is_point_reached()
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR POINT REACHED")
            return False

    def set_local_pose(self, x, y, z, yaw=None):
        """Ставит координаты для command_loop, yaw задается в градусах"""
        self.__coords_system = "GLOBAL_COORDS"
        self.__yaw_rate = 0.0
        self.__speed = (0.0, 0.0, 0.0)
        self.__command_pos = (x, y, z)
        if yaw is not None:
            self.__yaw = math.radians(yaw)
        self.logger.info(f"Coords set: {self.__command_pos}, yaw: {self.__yaw}")

    def set_speed(self, vx, vy, vz, yaw_rate=None):
        """Устанавливает скорость в глобальной системе координат"""
        self.__coords_system = "SPEED"
        self.__command_pos = None
        self.__speed = (vx, vy, vz)
        if yaw_rate is not None:
            self.__yaw_rate = math.radians(yaw_rate)
        self.logger.info(f"Speed set: {self.__speed}, yaw rate: {self.__yaw_rate}")

    def set_point_with_speed(self, target_x, target_y, target_z, speed):
        """Лететь к точке с заданной скоростью в связанной с телом системе координат"""
        self.__coords_system = "SPEED_POINT"
        self.__command_pos = (target_x, target_y, target_z)
        self.target_speed = speed

    def set_speed_fixed(self, vx: float, vy: float, vz: float, yaw_rate=None):
        """Устанавливает скорость в связанной с телом системе координат"""
        self.__coords_system = "SPEED_FIXED"
        self.__command_pos = None
        self.__speed = (vx, vy, vz)
        if yaw_rate is not None:
            self.__yaw_rate = math.radians(yaw_rate)
        self.logger.info(
            f"<<FIXED>> Speed set: {self.__speed}, yaw rate: {self.__yaw_rate}"
        )

    def get_local_pose(self):
        return self.__current_pos

    def get_yaw(self, degrees=False):
        return math.degrees(self.__yaw) if degrees else self.__yaw

    def arm(self):
        if self.pioneer.connected():
            self.pioneer.arm()
            self.__armed = True
            self.logger.info("ARMED")
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR ARM")

    def takeoff(self):
        if self.pioneer.connected():
            self.pioneer.takeoff()
            self.__takeoffed = True
            self.logger.info("TAKEOFF")
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR TAKEOFF")

    def connection(self):
        return self.pioneer and self.pioneer.connected()

    def land(self):
        if self.pioneer.connected():
            if self.__takeoffed:
                self.__command_pos = None
                self.__speed = (0.0, 0.0, 0.0)
                self.__yaw_rate = 0.0
                self.pioneer.land()
                self.__takeoffed = False
                self.__armed = False
                self.logger.info("LANDING")
            else:
                self.logger.warning("PIONEER NOT TAKEOFF FOR LANDING")
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR LANDING")

    def set_yaw(self, yaw, yaw_rate=30):
        if self.__coords_system == "SPEED_FIXED":
            self.__target_yaw = yaw
            self.__yaw_rate = math.radians(
                yaw_rate if self.get_yaw(degrees=True) < yaw else -yaw_rate
            )
            self.logger.info(f"Set yaw: {yaw} with rate: {yaw_rate}")
