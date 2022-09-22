import matplotlib.pyplot as plt

def get_vertical_intersection_with_line(x, x1, y1, x2, y2):
    y = y1 + (y2-y1)*(x-x1)/(x2-x1)
    return x, y

def get_vertically_aligned_edge(x, path_xs, path_ys):
    assert x >= path_xs[0]
    assert x <= path_xs[-1]
    for i in range(len(1, path_xs)):
        if path_xs[i] >= x:
            x1 = path_xs[i-1]
            y1 = path_ys[i-1]
            x2 = path_xs[i]
            y2 = path_ys[i]
            return x1, y1, x2, y2
    assert False, "Unreachable"


def get_path(floor_xs, floor_ys, ceiling_xs, ceiling_ys, robot_width) -> None:
    assert floor_xs[0] == ceiling_xs[0]
    assert floor_xs[-1] == ceiling_xs[-1]
    min_x = floor_xs[0]
    max_x = floor_xs[-1]
    path = [(floor_xs[0], floor_ys[0])]
    headed_up =  True
    current_x = min_x
    while True:
        x1, y1, x2, y2 = get_vertically_aligned_edge(current_x, ceiling_xs, ceiling_ys) if headed_up \
                        else get_vertically_aligned_edge(current_x, floor_xs, floor_ys)
        path.append((current_x, get_vertical_intersection_with_line(current_x, x1, y1, x2, y2)))

        if current_x >= max_x:
            break

        current_x = min(current_x + robot_width, max_x)

        x1, y1, x2, y2 = get_vertically_aligned_edge(current_x, ceiling_xs, ceiling_ys) if headed_up \
                        else get_vertically_aligned_edge(current_x, floor_xs, floor_ys)
        path.append((current_x, get_vertical_intersection_with_line(current_x, x1, y1, x2, y2)))

        headed_up = False
    
    return path


if __name__ == "__main__":
    floor_xs = []
    floor_xs = []
    floor_xs = []
    floor_xs = []
    
            
            
            


