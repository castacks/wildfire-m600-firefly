import matplotlib.pyplot as plt


def get_vertical_intersection_with_line(x, x1, y1, x2, y2):
    assert x1 != x2
    y = y1 + (y2 - y1) * (x - x1) / (x2 - x1)
    return x, y


def get_vertically_aligned_edge(x, path_xs, path_ys):
    assert x >= path_xs[0]
    assert x <= path_xs[-1]
    for i in range(1, len(path_xs)):
        if path_xs[i] >= x:
            x1 = path_xs[i - 1]
            y1 = path_ys[i - 1]
            x2 = path_xs[i]
            y2 = path_ys[i]
            return x1, y1, x2, y2
    assert False, "Unreachable"


def get_cell_path(floor_xs, floor_ys, ceiling_xs, ceiling_ys, stepover_dist):
    assert floor_xs[0] == ceiling_xs[0]
    assert floor_xs[-1] == ceiling_xs[-1]
    min_x = floor_xs[0]
    max_x = floor_xs[-1]
    path = [(floor_xs[0], floor_ys[0])]
    headed_up = True
    current_x = min_x
    while True:
        x1, y1, x2, y2 = (
            get_vertically_aligned_edge(current_x, ceiling_xs, ceiling_ys)
            if headed_up
            else get_vertically_aligned_edge(current_x, floor_xs, floor_ys)
        )
        if x1 == x2:
            path.append((current_x, max(y1, y2) if headed_up else min(y1, y2)))
        else:
            path.append(get_vertical_intersection_with_line(current_x, x1, y1, x2, y2))

        if current_x >= max_x:
            break

        current_x = min(current_x + stepover_dist, max_x)

        x1, y1, x2, y2 = (
            get_vertically_aligned_edge(current_x, ceiling_xs, ceiling_ys)
            if headed_up
            else get_vertically_aligned_edge(current_x, floor_xs, floor_ys)
        )

        if x1 == x2:
            path.append((current_x, max(y1, y2) if headed_up else min(y1, y2)))
        else:
            path.append(get_vertical_intersection_with_line(current_x, x1, y1, x2, y2))

        headed_up = not headed_up

    return path


def get_idx_leftmost_vertex(vertices):
    leftmost_vertex = None
    vertex_idx = None
    for idx, vertex in enumerate(vertices):
        if leftmost_vertex is None or vertex[0] < leftmost_vertex[0]:
            leftmost_vertex = vertex
            vertex_idx = idx
        elif vertex[0] == leftmost_vertex[0] and vertex[1] < leftmost_vertex[1]:
            leftmost_vertex = vertex
            vertex_idx = idx
    return vertex_idx


def get_polygon_floor_and_ceiling(ccw_vertices):
    starting_idx = get_idx_leftmost_vertex(ccw_vertices)
    adding_to_floor_vertices = True
    floor_vertices = [ccw_vertices[starting_idx]]
    ceiling_vertices = []
    last_vertex = ccw_vertices[starting_idx]
    for i in range(1, len(ccw_vertices)):
        vertex = ccw_vertices[(starting_idx + i) % len(ccw_vertices)]
        if vertex[0] < last_vertex[0] and adding_to_floor_vertices:
            ceiling_vertices.append(last_vertex)
            adding_to_floor_vertices = False
        elif vertex[0] > last_vertex[0] and not adding_to_floor_vertices:
            assert False

        if adding_to_floor_vertices:
            floor_vertices.append(vertex)
        else:
            ceiling_vertices.append(vertex)

        last_vertex = vertex
    if adding_to_floor_vertices:
        ceiling_vertices.append(last_vertex)
    ceiling_vertices.append(ccw_vertices[starting_idx])
    ceiling_vertices.reverse()
    return floor_vertices, ceiling_vertices


def get_polygon_path(ccw_vertices, stepover_dist):
    floor_vertices, ceiling_vertices = get_polygon_floor_and_ceiling(ccw_vertices)
    print(floor_vertices)
    print(ceiling_vertices)
    floor_xs = [floor_vertices[i][0] for i in range(len(floor_vertices))]
    floor_ys = [floor_vertices[i][1] for i in range(len(floor_vertices))]
    ceiling_xs = [ceiling_vertices[i][0] for i in range(len(ceiling_vertices))]
    ceiling_ys = [ceiling_vertices[i][1] for i in range(len(ceiling_vertices))]
    return get_cell_path(floor_xs, floor_ys, ceiling_xs, ceiling_ys, stepover_dist)


if __name__ == "__main__":
    ccw_vertices = [(10, 5), (0, 8), (-10, 5), (-10, -5), (10, -5)]
    ccw_vertices_xs = [ccw_vertices[i][0] for i in range(len(ccw_vertices))]
    ccw_vertices_ys = [ccw_vertices[i][1] for i in range(len(ccw_vertices))]

    path = get_polygon_path(ccw_vertices, stepover_dist=2)
    path_xs = [path[i][0] for i in range(len(path))]
    path_ys = [path[i][1] for i in range(len(path))]

    plt.plot(
        ccw_vertices_xs + [ccw_vertices_xs[0]],
        ccw_vertices_ys + [ccw_vertices_ys[0]],
        linewidth=5.0,
    )
    plt.plot(path_xs, path_ys)

    plt.show()
