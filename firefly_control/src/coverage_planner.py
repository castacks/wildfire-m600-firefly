from __future__ import annotations
from typing import Tuple, List
from enum import Enum
import numpy as np


def get_polygon_xs_and_ys(polygon):
    xs = [polygon[i][0] for i in range(len(polygon))]
    ys = [polygon[i][1] for i in range(len(polygon))]
    return xs, ys


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
    floor_xs, floor_ys = get_polygon_xs_and_ys(floor_vertices)
    ceiling_xs, ceiling_ys = get_polygon_xs_and_ys(ceiling_vertices)
    return get_cell_path(floor_xs, floor_ys, ceiling_xs, ceiling_ys, stepover_dist)


class Point2d:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __eq__(self, point: Point2d) -> bool:
        return self.x == point.x and self.y == point.y


class Edge:
    def __init__(self, p1: Point2d, p2: Point2d):
        assert p1.x != p2.x
        self.p1 = p1
        self.p2 = p2

    def __eq__(self, edge: Edge) -> bool:
        return (self.p1 == edge.p1 and self.p2 == edge.p2) or (
            self.p1 == edge.p2 and self.p2 == edge.p1
        )


def get_vertical_intersection_with_edge(x: float, edge: Edge) -> Point2d:
    assert edge.p1.x != edge.p2.x
    y = edge.p1.y + (edge.p2.y - edge.p1.y) * (x - edge.p1.x) / (edge.p2.x - edge.p1.x)
    return Point2d(x, y)


class EventType(Enum):
    IN = 1  # One cell to two cells
    OUT = 2  # Two cells to one cell
    OPEN = 3  # Zero cells to one cell
    CLOSE = 4  # One cell to zero cells
    FLOOR = 5  # One cell to one cell
    CEILING = 6  # One cell to one cell


class Event:
    def __init__(self, vertex: Point2d, prev_vertex: Point2d, next_vertex: Point2d):
        self.x = vertex.x
        self.y = vertex.y

        assert self.x != prev_vertex.x
        assert self.x != next_vertex.x

        self.prev_vertex = prev_vertex
        self.next_vertex = next_vertex
        self.prev_edge = Edge(prev_vertex, self.to_point())
        self.next_edge = Edge(self.to_point(), next_vertex)

        if self.prev_vertex.x < self.x and self.next_vertex.x < self.x:
            if self.prev_vertex.y > self.next_vertex.y:
                self.type = EventType.OUT
            else:
                self.type = EventType.CLOSE
        elif self.prev_vertex.x > self.x and self.next_vertex.x > self.x:
            if self.prev_vertex.y > self.next_vertex.y:
                self.type = EventType.OPEN
            else:
                self.type = EventType.IN
        elif self.prev_vertex.x < self.x and self.next_vertex.x > self.x:
            self.type = EventType.FLOOR
        else:
            self.type = EventType.CEILING

    def to_point(self) -> Point2d:
        return Point2d(self.x, self.y)


def get_events_from_polygon(polygon: List[Point2d]) -> List[Event]:
    events = []
    for i in range(len(polygon)):
        prev = polygon[(i - 1) % len(polygon)]
        next = polygon[(i + 1) % len(polygon)]
        events.append(Event(polygon[i], prev, next))
    return events


class Trapezoidal_Cell:
    def __init__(
        self, floor: Edge, ceiling: Edge, left_x: float, right_x: float, neighbors=[]
    ) -> None:
        assert floor is not None
        assert ceiling is not None
        assert left_x is not None

        self.floor = floor
        self.ceiling = ceiling
        self.left_x = left_x
        self.right_x = right_x

        self.neighbors = neighbors

        self.id = None

    def get_vertices(self) -> List[Point2d]:
        points = []
        points.append(get_vertical_intersection_with_edge(self.left_x, self.floor))
        points.append(get_vertical_intersection_with_edge(self.right_x, self.floor))
        points.append(get_vertical_intersection_with_edge(self.right_x, self.ceiling))
        points.append(get_vertical_intersection_with_edge(self.left_x, self.ceiling))
        return points

    def get_centroid(self) -> Point2d:
        points = self.get_vertices()
        average_x = 0
        average_y = 0
        for point in points:
            average_x += point.x
            average_y += point.y
        average_x /= len(points)
        average_y /= len(points)
        return Point2d(average_x, average_y)

    def plot_adjancency_edges(self) -> None:
        centroid = self.get_centroid()
        for neighbor in self.neighbors:
            neighbor_centroid = neighbor.get_centroid()
            plt.plot(
                [centroid.x, neighbor_centroid.x],
                [centroid.y, neighbor_centroid.y],
                linewidth=2.5,
                color="white",
            )


def get_floor_and_ceiling_edge(event: Event, edges: List[Edge]) -> Tuple[Edge, Edge]:
    floor = None
    ceiling = None
    dist_to_floor = None
    dist_to_ceiling = None
    for edge in edges:
        intersect = get_vertical_intersection_with_edge(event.x, edge)
        signed_dist = intersect.y - event.y

        # if signed_dist == 0:
        #     if edge == event.prev_edge:
        #         floor = event.prev_edge
        #         dist_to_floor = 0
        #     elif edge == event.next_edge:
        #         ceiling = event.next_edge
        #         dist_to_ceiling = 0
        if signed_dist < 0:
            if dist_to_floor is None or abs(signed_dist) < dist_to_floor:
                dist_to_floor = abs(signed_dist)
                floor = edge
        elif signed_dist > 0:
            if dist_to_ceiling is None or signed_dist < dist_to_ceiling:
                dist_to_ceiling = signed_dist
                ceiling = edge

    return floor, ceiling


def trapezoidal_decomposition(
    outer_boundary: List[Point2d], holes: List[List[Point2d]]
) -> List[Trapezoidal_Cell]:
    # TODO: Assert that outer is ccw and holes are cw
    events = []
    events += get_events_from_polygon(outer_boundary)
    for hole in holes:
        events += get_events_from_polygon(hole)
    events.sort(key=lambda e: e.x)

    current_edges = []
    open_cells = []
    closed_cells = []
    for e in events:
        floor, ceiling = get_floor_and_ceiling_edge(e, current_edges)

        if e.type == EventType.IN:
            for i, cell in enumerate(open_cells):
                if floor == cell.floor and ceiling == cell.ceiling:
                    new_bottom_cell = Trapezoidal_Cell(
                        floor, e.prev_edge, e.x, None, [cell]
                    )
                    new_top_cell = Trapezoidal_Cell(
                        e.next_edge, ceiling, e.x, None, [cell]
                    )
                    open_cells.extend([new_bottom_cell, new_top_cell])
                    cell.right_x = e.x
                    cell.neighbors = [
                        new_bottom_cell,
                        new_top_cell,
                    ] + cell.neighbors
                    closed_cells.append(cell)
                    open_cells.pop(i)
                    break
        elif e.type == EventType.OUT:
            upper_cell_idx = None
            lower_cell_idx = None
            for i, cell in enumerate(open_cells):
                if cell.floor == e.prev_edge:
                    upper_cell_idx = i
                    upper_cell = cell
                elif cell.ceiling == e.next_edge:
                    lower_cell_idx = i
                    lower_cell = cell
            assert upper_cell_idx is not None
            assert lower_cell_idx is not None
            assert upper_cell_idx != lower_cell_idx
            new_cell = Trapezoidal_Cell(
                floor, ceiling, e.x, None, [upper_cell, lower_cell]
            )
            open_cells.pop(max(upper_cell_idx, lower_cell_idx))
            open_cells.pop(min(upper_cell_idx, lower_cell_idx))
            upper_cell.right_x = e.x
            lower_cell.right_x = e.x
            upper_cell.neighbors.append(new_cell)
            lower_cell.neighbors.append(new_cell)
            closed_cells.append(upper_cell)
            closed_cells.append(lower_cell)
            open_cells.append(new_cell)
        elif e.type == EventType.OPEN:
            open_cells.append(Trapezoidal_Cell(e.next_edge, e.prev_edge, e.x, None))
        elif e.type == EventType.CLOSE:
            for i, cell in enumerate(open_cells):
                if e.prev_edge == cell.floor and e.next_edge == cell.ceiling:
                    cell.right_x = e.x
                    closed_cells.append(cell)
                    open_cells.pop(i)
                    break
        elif e.type == EventType.FLOOR:
            for i, cell in enumerate(open_cells):
                if e.prev_edge == cell.floor and ceiling == cell.ceiling:
                    new_cell = Trapezoidal_Cell(e.next_edge, ceiling, e.x, None, [cell])
                    cell.right_x = e.x
                    cell.neighbors.append(new_cell)
                    closed_cells.append(cell)
                    open_cells.pop(i)
                    open_cells.append(new_cell)
                    break
        elif e.type == EventType.CEILING:
            for i, cell in enumerate(open_cells):
                if floor == cell.floor and e.next_edge == cell.ceiling:
                    new_cell = Trapezoidal_Cell(floor, e.prev_edge, e.x, None, [cell])
                    cell.right_x = e.x
                    cell.neighbors.append(new_cell)
                    closed_cells.append(cell)
                    open_cells.pop(i)
                    open_cells.append(new_cell)
                    break

        if e.prev_vertex.x < e.x:
            current_edges.remove(e.prev_edge)
        else:
            current_edges.append(e.prev_edge)

        if e.next_vertex.x < e.x:
            current_edges.remove(e.next_edge)
        else:
            current_edges.append(e.next_edge)

    return closed_cells


def get_cell_dist(cell1: Trapezoidal_Cell, cell2: Trapezoidal_Cell) -> float:
    centroid1 = cell1.get_centroid()
    centroid2 = cell2.get_centroid()
    dist = np.sqrt((centroid1.x - centroid2.x) ** 2 + (centroid1.y - centroid2.y) ** 2)
    return dist


def generate_cell_traversal(cells: List[Trapezoidal_Cell]) -> List[Trapezoidal_Cell]:
    assert len(cells) > 0
    for i, cell in enumerate(cells):
        cell.id = i
    unvisited_ids = set(range(len(cells)))
    unvisited_ids.remove(0)
    path_list = [0]

    while len(unvisited_ids) > 0:
        cell = cells[path_list[-1]]
        all_neighbors_visited = True
        potential_next_ids = []
        for neighbor in cell.neighbors:
            if neighbor.id in unvisited_ids:
                potential_next_ids.append(neighbor.id)
                all_neighbors_visited = False

        if all_neighbors_visited:
            potential_next_ids = list(unvisited_ids)

        min_dist = None
        closest_cell_id = None
        for id in potential_next_ids:
            next_cell = cells[id]
            dist = get_cell_dist(cell, next_cell)
            if min_dist is None or dist < min_dist:
                min_dist = dist
                closest_cell_id = id

        unvisited_ids.remove(closest_cell_id)
        path_list.append(closest_cell_id)

    path = [cells[id] for id in path_list]
    return path


def get_full_coverage_path(
    cell_traversal: List[Trapezoidal_Cell], stepover_dist: float
):
    path = []
    for cell in cell_traversal:
        ccw_vertices = [(p.x, p.y) for p in cell.get_vertices()]
        path.extend(get_polygon_path(ccw_vertices, stepover_dist))
    return path
