import heapq
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon as MplPolygon
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import unary_union


def parse_data(text):
    lines = [line.strip() for line in text.strip().splitlines() if line.strip()]
    start = tuple(map(float, lines[0].split()))
    finish = tuple(map(float, lines[1].split()))
    n = int(lines[2])
    zones = []
    for i in range(n):
        parts = list(map(float, lines[3 + i].split()))
        pts = [(parts[j], parts[j + 1]) for j in range(0, len(parts), 2)]
        zones.append(pts)
    return start, finish, zones


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


def edge_cost(a, b, zones_union, cost_normal=12.0, cost_zone=1.0):
    line = LineString([a, b])
    total_len = line.length
    if total_len == 0:
        return 0.0
    inter = line.intersection(zones_union)
    len_in_zone = 0.0
    if not inter.is_empty:
        len_in_zone = inter.length
    len_out = total_len - len_in_zone
    return len_out * cost_normal + len_in_zone * cost_zone


def astar_grid(start, goal, min_x, min_y, res, xs, ys, width, height, zones_union):
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

    open_heap = []
    heapq.heappush(open_heap, (0.0, start_idx))
    g_score = {start_idx: 0.0}
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
            coord = grid_index_to_coord(start_gx, start_gy, min_x, min_y, res)
            path.append(coord)
            path.reverse()
            path[-1] = goal
            path[0] = start
            return path

        for dx, dy in neighbor_offsets:
            nx, ny = current[0] + dx, current[1] + dy
            if not in_bounds(nx, ny):
                continue
            a = grid_index_to_coord(current[0], current[1], min_x, min_y, res)
            b = grid_index_to_coord(nx, ny, min_x, min_y, res)
            tentative_g = g_score[current] + edge_cost(a, b, zones_union)
            neighbor = (nx, ny)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                nb_coord = grid_index_to_coord(nx, ny, min_x, min_y, res)
                heuristic = (
                    math.hypot(goal[0] - nb_coord[0], goal[1] - nb_coord[1]) * 1.0
                )
                f_score = tentative_g + heuristic
                heapq.heappush(open_heap, (f_score, neighbor))
                came_from[neighbor] = current

    return None


def simplify_path(path, zones_union, tol=1e-9):
    if not path or len(path) < 3:
        return path[:]

    simplified = [path[0]]
    last_dx = path[1][0] - path[0][0]
    last_dy = path[1][1] - path[0][1]

    for i in range(2, len(path)):
        cur_dx = path[i][0] - path[i - 1][0]
        cur_dy = path[i][1] - path[i - 1][1]
        if abs(cur_dx - last_dx) > tol or abs(cur_dy - last_dy) > tol:
            simplified.append(path[i - 1])
            last_dx, last_dy = cur_dx, cur_dy
    simplified.append(path[-1])

    i = 0
    final_path = [simplified[0]]
    while i < len(simplified) - 1:
        j = len(simplified) - 1
        merged = False

        while j > i + 1:
            seg = LineString([simplified[i], simplified[j]])
            if zones_union.is_empty or not seg.intersects(zones_union):
                final_path.append(simplified[j])
                i = j
                merged = True
                break
            j -= 1

        if not merged:
            final_path.append(simplified[i + 1])
            i += 1

    compacted = [final_path[0]]
    for pt in final_path[1:]:
        if pt != compacted[-1]:
            compacted.append(pt)
    return compacted


def plot_result(
    start, goal, zones, path, res, min_x, max_x, min_y, max_y, buffered_zones=None
):
    fig, ax = plt.subplots(figsize=(10, 10))

    if buffered_zones and not buffered_zones.is_empty:
        if isinstance(buffered_zones, Polygon):
            polys_to_plot = [buffered_zones]
        elif isinstance(buffered_zones, MultiPolygon):
            polys_to_plot = list(buffered_zones.geoms)
        else:
            polys_to_plot = []

        for poly in polys_to_plot:
            mpl_poly = MplPolygon(
                np.array(poly.exterior.coords),
                closed=True,
                facecolor="yellow",
                alpha=0.3,
                edgecolor="orange",
                linestyle="--",
                zorder=1,
            )
            ax.add_patch(mpl_poly)

    for pts in zones:
        mpl_poly = MplPolygon(
            np.array(pts),
            closed=True,
            facecolor="red",
            alpha=0.4,
            edgecolor="black",
            zorder=2,
        )
        ax.add_patch(mpl_poly)

    if path:
        px, py = zip(*path)
        ax.plot(px, py, marker="o", linewidth=2, markersize=4, label="Путь", zorder=5)
    ax.plot(start[0], start[1], "go", markersize=10, label="Старт", zorder=6)
    ax.plot(goal[0], goal[1], "ro", markersize=10, label="Финиш", zorder=6)

    ax.set_xlim(min_x - res, max_x + res)
    ax.set_ylim(min_y - res, max_y + res)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Прокладка пути (A* на дробной сетке). res = {:.3f}".format(res))
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

    start, goal, zones_pts = parse_data(text)

    res = 0.5
    cost_normal = 1.0
    cost_zone = 5.0
    safety_distance = 0.5

    all_x = [start[0], goal[0]] + [x for poly in zones_pts for (x, y) in poly]
    all_y = [start[1], goal[1]] + [y for poly in zones_pts for (x, y) in poly]
    min_x = min(all_x) - 1.0
    max_x = max(all_x) + 1.0
    min_y = min(all_y) - 1.0
    max_y = max(all_y) + 1.0

    shapely_polys = [Polygon(poly) for poly in zones_pts if len(poly) >= 3]
    if shapely_polys:
        zones_union = unary_union(shapely_polys)
    else:
        zones_union = Polygon()

    buffered_zones_union = zones_union.buffer(safety_distance)

    xs = np.arange(min_x, max_x + 1e-9, res)
    ys = np.arange(min_y, max_y + 1e-9, res)
    width = len(xs)
    height = len(ys)

    path = astar_grid(
        start, goal, min_x, min_y, res, xs, ys, width, height, buffered_zones_union
    )

    if path is None:
        print("Путь не найден. Попробуйте увеличить область или изменить разрешение.")
        plot_result(
            start,
            goal,
            zones_pts,
            None,
            res,
            min_x,
            max_x,
            min_y,
            max_y,
            buffered_zones=buffered_zones_union,
        )
        return

    print("Полный путь (узлы сетки, затем подставленный точный старт/финиш):")
    for p in path:
        print(f"{p[0]:.3f} {p[1]:.3f}")

    simplified = simplify_path(path, zones_union=buffered_zones_union)
    print("\nУпрощённый путь (точки смены направления):")
    for p in simplified:
        print(f"{p[0]:.3f} {p[1]:.3f}")

    plot_result(
        start,
        goal,
        zones_pts,
        simplified,
        res,
        min_x,
        max_x,
        min_y,
        max_y,
        buffered_zones=buffered_zones_union,
    )


if __name__ == "__main__":
    main()
