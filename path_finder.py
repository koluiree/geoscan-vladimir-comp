import heapq
import math

import numpy as np
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon
except ModuleNotFoundError:
    plt = None
    MplPolygon = None


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

        points = [
            _parse_point_line(point_line, "точки")
            for point_line in point_lines
        ]
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
                _parse_point_line(finish_line, "финиша")
                for finish_line in finish_lines
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
):
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
    return (
        time_in_zone * cost_inside_zone_per_second
        + time_out_zone * cost_outside_zone_per_second
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
        )

    min_penalty_per_second = min(
        cost_inside_zone_per_second, cost_outside_zone_per_second
    )

    def heuristic(coord):
        return (
            math.hypot(goal[0] - coord[0], goal[1] - coord[1]) / flight_speed
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
    )
    heapq.heappush(open_heap, (start_cost + heuristic(start_coord), start_idx))
    g_score = {start_idx: start_cost}
    came_from = {}

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
            )
            return path, total_cost

        for dx, dy in neighbor_offsets:
            nx, ny = current[0] + dx, current[1] + dy
            if not in_bounds(nx, ny):
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
            )
            neighbor = (nx, ny)
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
    )
    return path


def path_distance(path):
    if not path or len(path) < 2:
        return 0.0
    return sum(
        math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        for i in range(1, len(path))
    )


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

    for goal_index, goal in enumerate(goals, start=1):
        path, total_cost = astar_grid_with_cost(
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
        )
        if path is None:
            continue

        if best_result is None:
            best_result = (goal_index, goal, path, total_cost)
            continue

        _, _, best_path, best_cost = best_result
        same_cost = math.isclose(total_cost, best_cost, rel_tol=1e-9, abs_tol=1e-9)
        if total_cost < best_cost or (
            same_cost and path_distance(path) < path_distance(best_path)
        ):
            best_result = (goal_index, goal, path, total_cost)

    return best_result


def simplify_path(path, tol=1e-9):
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
    plt.title(
        "Прокладка пути по разрешённым зонам (A*, res = {:.3f})".format(res)
    )
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

    all_x = [start[0]] + [goal[0] for goal in goals] + [
        x for poly in zones_pts for (x, y) in poly
    ]
    all_y = [start[1]] + [goal[1] for goal in goals] + [
        y for poly in zones_pts for (x, y) in poly
    ]
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


if __name__ == "__main__":
    main()
