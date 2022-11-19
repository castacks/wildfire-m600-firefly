from coverage_planner import *
import matplotlib.pyplot as plt


def plot_cell_traversal(path: List[Trapezoidal_Cell]) -> None:
    assert len(path) > 1
    for i in range(1, len(path)):
        c1 = path[i - 1].get_centroid()
        c2 = path[i].get_centroid()
        plt.plot(
            [c1.x, c2.x],
            [c1.y, c2.y],
            linewidth=2.5,
            color="red",
        )


def rotate_path(xy_path, angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    rotated_xy_path = []
    for (x, y) in xy_path:
        new_x = x * np.cos(angle_rad) - y * np.sin(angle_rad)
        new_y = x * np.sin(angle_rad) + y * np.cos(angle_rad)
        rotated_xy_path.append((new_x, new_y))
    return rotated_xy_path


def rotate_polygon(polygon: List[Point2d], angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    rotated_polygon = []
    for point in polygon:
        new_x = point.x * np.cos(angle_rad) - point.y * np.sin(angle_rad)
        new_y = point.x * np.sin(angle_rad) + point.y * np.cos(angle_rad)
        rotated_polygon.append(Point2d(new_x, new_y))
    return rotated_polygon


def rotate_holey_polygon(
    outer_boundary: List[Point2d], holes: List[List[Point2d]], angle_deg
):
    outer_boundary = rotate_polygon(outer_boundary, angle_deg)
    holes = [rotate_polygon(hole, angle_deg) for hole in holes]
    return outer_boundary, holes


def plot_cell(cell) -> None:
    points = cell.get_vertices()
    plt.fill([p.x for p in points], [p.y for p in points])


def plot_cell_adjancency_edges(cell) -> None:
    centroid = cell.get_centroid()
    for neighbor in cell.neighbors:
        neighbor_centroid = neighbor.get_centroid()
        plt.plot(
            [centroid.x, neighbor_centroid.x],
            [centroid.y, neighbor_centroid.y],
            linewidth=2.5,
            color="white",
        )


if __name__ == "__main__":
    # ccw_vertices = [(10, 5), (0, 8), (-10, 5), (-10, -5), (10, -5)]
    # ccw_vertices_xs = [ccw_vertices[i][0] for i in range(len(ccw_vertices))]
    # ccw_vertices_ys = [ccw_vertices[i][1] for i in range(len(ccw_vertices))]

    # path = get_polygon_path(ccw_vertices, stepover_dist=2)
    # path_xs = [path[i][0] for i in range(len(path))]
    # path_ys = [path[i][1] for i in range(len(path))]

    # plt.plot(
    #     ccw_vertices_xs + [ccw_vertices_xs[0]],
    #     ccw_vertices_ys + [ccw_vertices_ys[0]],
    #     linewidth=5.0,
    # )
    # plt.plot(path_xs, path_ys)
    # ccw_vertices = [(10, 5), (0, 8), (-10, 5), (-10, -5), (10, -5)]
    # ccw_vertices_xs, ccw_vertices_ys = get_polygon_xs_and_ys(ccw_vertices)
    # path = get_polygon_path(ccw_vertices, stepover_dist=2)
    # path_xs, path_ys = get_polygon_xs_and_ys(path)
    # plt.plot(
    #     ccw_vertices_xs + [ccw_vertices_xs[0]],
    #     ccw_vertices_ys + [ccw_vertices_ys[0]],
    #     linewidth=5.0,
    # )
    # plt.plot(path_xs, path_ys)

    # outer_boundary = [
    #     Point2d(10, 5),
    #     Point2d(0, 8),
    #     Point2d(-10, 5),
    #     Point2d(-12, -5),
    #     Point2d(12, -5),
    # ]

    # hole1 = [Point2d(2.5, 3), Point2d(5, 5), Point2d(7, 3)]
    # hole2 = [
    #     Point2d(-6.1, 4.2),
    #     Point2d(-1.12, 0),
    #     Point2d(-6.6, -2),
    #     Point2d(-2.27, 0),
    # ]
    # outer_boundary = [
    #     Point2d(90, 0),
    #     Point2d(70, 50),
    #     Point2d(20, 50),
    #     Point2d(0, 0),
    # ]
    outer_boundary = [
        Point2d(90, 0),
        Point2d(90, 100),
        Point2d(0, 100),
        Point2d(0, 0),
    ]
    outer_boundary = [
        Point2d(0, 100),
        Point2d(0, 0),
        Point2d(150, 0),
        Point2d(150, 150),
        Point2d(0, 150),
        Point2d(0, 110),
        Point2d(90, 110),
        Point2d(90, 100),
    ]

    # hole = [Point2d(30, 20), Point2d(45, 40), Point2d(60, 20)]
    hole1 = [Point2d(30, 30), Point2d(30, 40), Point2d(60, 40), Point2d(60, 30)]
    hole2 = [Point2d(30, 10), Point2d(30, 20), Point2d(60, 20), Point2d(60, 10)]
    hole3 = [Point2d(30, 50), Point2d(30, 60), Point2d(60, 60), Point2d(60, 50)]
    hole4 = [Point2d(30, 70), Point2d(30, 80), Point2d(60, 80), Point2d(60, 70)]
    hole5 = [Point2d(110, 60), Point2d(120, 110), Point2d(130, 50)]

    holes = [hole1, hole2, hole3, hole4, hole5]
    # holes = [hole]
    angle_deg = 0
    outer_boundary, holes = rotate_holey_polygon(outer_boundary, holes, -angle_deg)

    xs = [outer_boundary[i].x for i in range(len(outer_boundary))]
    ys = [outer_boundary[i].y for i in range(len(outer_boundary))]
    plt.plot(xs + [xs[0]], ys + [ys[0]], linewidth=2.5, color="red")

    for hole in holes:
        xs = [hole[i].x for i in range(len(hole))]
        ys = [hole[i].y for i in range(len(hole))]
        plt.plot(xs + [xs[0]], ys + [ys[0]], linewidth=2.5, color="red")

    cells = trapezoidal_decomposition(outer_boundary, holes)
    cell_path = generate_cell_traversal(cells)
    path = get_full_coverage_path(cell_path, 5)
    # path = rotate_path(path, angle_deg)
    # print(path)

    for cell in cells:
        plot_cell(cell)
    #     plot_cell_adjancency_edges(cell)
    # plot_cell_traversal(cell_path)

    path_xs = [path[i][0] for i in range(len(path))]
    path_ys = [path[i][1] for i in range(len(path))]
    plt.plot(path_xs, path_ys, linewidth=2.5, color="black")

    plt.show()
